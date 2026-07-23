/**
 * Dump hard SUPER optimization problem instances (corner cases) to disk as
 * self-contained, human-readable case directories for offline replay and
 * parameter sweeps.
 *
 * Each case directory contains:
 *   case.yaml             - problem definition + detection reason + opt result
 *   cloud.pcd             - local point cloud used by the SFC search (may be absent)
 *   config_snapshot.yaml  - verbatim copy of the runtime planner config
 */

#ifndef SUPER_CASE_DUMP_HPP
#define SUPER_CASE_DUMP_HPP

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <atomic>

#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <super_core/config.hpp>
#include <data_structure/base/polytope.h>
#include <utils/header/eigen_alias.hpp>
#include <utils/header/type_utils.hpp>

namespace super_planner {
    using namespace super_utils;
    using namespace geometry_utils;

    /* Backup-trajectory problem section: everything needed to re-run the backup
     * optimization offline once the exp trajectory has been replayed. */
    struct BackupCaseSection {
        bool valid{false};
        double t0{0.0};             // braking window start on the exp traj [s]
        double te{0.0};             // braking window end (seed point time) [s]
        double heu_ts{0.0};         // heuristic switch-time guess [s]
        double heu_dur{0.0};        // heuristic braking duration [s]
        Vec3f heu_p{Vec3f::Zero()}; // braking target point [m]
        Polytope sfc;               // backup safe corridor
        bool opt_success{false};
        double opt_time{0.0};       // backup optimizer wall time [s]
    };

    struct ExpOptCaseData {
        std::string reason;                 // comma separated trigger reasons
        std::string stage{"backend"};       // backend | frontend
        double sim_time{0.0};
        Vec3f robot_p{Vec3f::Zero()};
        Vec3f goal_p{Vec3f::Zero()};
        double goal_yaw{0.0};
        bool connected_goal{false};
        StatePVAJ head_pvaj{StatePVAJ::Zero()};
        StatePVAJ tail_pvaj{StatePVAJ::Zero()};
        vec_E<Vec3f> guide_path;
        std::vector<double> guide_t;
        vec_E<Vec3f> pass_wps;
        PolytopeVec sfc;                    // empty for frontend failures
        vec_Vec3f cloud;                    // point cloud used by the SFC search
        // backend optimization outcome
        bool opt_success{false};
        int lbfgs_ret{-9999};
        int iter_num{0};
        double final_cost{0.0};
        double opt_time{0.0};
        VecDf penalty_log;
        // per-stage wall time breakdown [s]
        double frontend_time{0.0};
        double sfc_time{0.0};
        BackupCaseSection backup;
    };

    class CaseDumper {
        static YAML::Node vec3ToNode(const Vec3f &v) {
            YAML::Node n;
            n.push_back(v.x());
            n.push_back(v.y());
            n.push_back(v.z());
            n.SetStyle(YAML::EmitterStyle::Flow);
            return n;
        }

        static YAML::Node pvajToNode(const StatePVAJ &m) {
            YAML::Node n;
            n["pos"] = vec3ToNode(m.col(0));
            n["vel"] = vec3ToNode(m.col(1));
            n["acc"] = vec3ToNode(m.col(2));
            n["jerk"] = vec3ToNode(m.col(3));
            return n;
        }

        static YAML::Node vec3fListToNode(const vec_E<Vec3f> &list) {
            YAML::Node n(YAML::NodeType::Sequence);
            for (const auto &p: list) {
                n.push_back(vec3ToNode(p));
            }
            return n;
        }

        static YAML::Node doubleListToNode(const std::vector<double> &list) {
            YAML::Node n(YAML::NodeType::Sequence);
            n.SetStyle(YAML::EmitterStyle::Flow);
            for (const auto &v: list) {
                n.push_back(v);
            }
            return n;
        }

        static YAML::Node sfcToNode(const PolytopeVec &sfc) {
            YAML::Node n(YAML::NodeType::Sequence);
            for (const auto &poly: sfc) {
                const MatD4f planes = poly.GetPlanes();
                YAML::Node poly_node(YAML::NodeType::Sequence);
                for (int r = 0; r < planes.rows(); r++) {
                    YAML::Node row;
                    row.SetStyle(YAML::EmitterStyle::Flow);
                    for (int c = 0; c < 4; c++) {
                        row.push_back(planes(r, c));
                    }
                    poly_node.push_back(row);
                }
                n.push_back(poly_node);
            }
            return n;
        }

        static bool writeTextFile(const std::string &path, const std::string &content) {
            std::ofstream ofs(path, std::ios::out | std::ios::trunc);
            if (!ofs.is_open()) {
                return false;
            }
            ofs << content;
            return ofs.good();
        }

    public:
        /**
         * Write one corner-case directory and return its case_id (empty on failure).
         *
         * @param[in] cfg        case_dump thresholds and output directory
         * @param[in] cfg_path   runtime planner yaml, copied verbatim into the case dir
         * @param[in] data       problem instance payload
         * @return case id string, empty if the dump failed
         */
        static std::string dump(const CaseDumpConfig &cfg,
                                const std::string &cfg_path,
                                const ExpOptCaseData &data) {
            static std::atomic<int> seq{0};
            const auto now = std::chrono::system_clock::now();
            const auto now_tt = std::chrono::system_clock::to_time_t(now);
            const auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()).count() % 1000;
            std::tm tm_buf{};
            localtime_r(&now_tt, &tm_buf);
            char time_buf[32];
            std::strftime(time_buf, sizeof(time_buf), "%Y%m%d-%H%M%S", &tm_buf);
            std::ostringstream id_ss;
            id_ss << "case-" << time_buf << "-" << std::setw(3) << std::setfill('0') << msec
                  << "-" << seq.fetch_add(1);
            const std::string case_id = id_ss.str();

            std::error_code ec;
            const std::filesystem::path case_dir =
                    std::filesystem::path(cfg.output_dir) / case_id;
            std::filesystem::create_directories(case_dir, ec);
            if (ec) {
                return "";
            }

            YAML::Node root;
            root["case_id"] = case_id;
            root["reason"] = data.reason;
            root["stage"] = data.stage;
            root["sim_time"] = data.sim_time;
            root["robot_p"] = vec3ToNode(data.robot_p);
            root["goal_p"] = vec3ToNode(data.goal_p);
            root["goal_yaw"] = data.goal_yaw;
            root["connected_goal"] = data.connected_goal;
            root["head_pvaj"] = pvajToNode(data.head_pvaj);
            root["tail_pvaj"] = pvajToNode(data.tail_pvaj);
            root["guide_path"] = vec3fListToNode(data.guide_path);
            root["guide_t"] = doubleListToNode(data.guide_t);
            root["pass_wps"] = vec3fListToNode(data.pass_wps);
            root["sfc"] = sfcToNode(data.sfc);

            YAML::Node result;
            result["opt_success"] = data.opt_success;
            result["lbfgs_ret"] = data.lbfgs_ret;
            result["iter_num"] = data.iter_num;
            result["final_cost"] = data.final_cost;
            result["opt_time"] = data.opt_time;
            result["frontend_time"] = data.frontend_time;
            result["sfc_time"] = data.sfc_time;
            if (data.penalty_log.size() > 0) {
                YAML::Node pen(YAML::NodeType::Sequence);
                pen.SetStyle(YAML::EmitterStyle::Flow);
                for (int i = 0; i < data.penalty_log.size(); i++) {
                    pen.push_back(data.penalty_log(i));
                }
                result["penalty_log"] = pen;
            }
            root["result"] = result;

            if (data.backup.valid) {
                YAML::Node backup;
                backup["valid"] = true;
                backup["t0"] = data.backup.t0;
                backup["te"] = data.backup.te;
                backup["heu_ts"] = data.backup.heu_ts;
                backup["heu_dur"] = data.backup.heu_dur;
                backup["heu_p"] = vec3ToNode(data.backup.heu_p);
                backup["opt_success"] = data.backup.opt_success;
                backup["opt_time"] = data.backup.opt_time;
                const MatD4f planes = data.backup.sfc.GetPlanes();
                YAML::Node poly_node(YAML::NodeType::Sequence);
                for (int r = 0; r < planes.rows(); r++) {
                    YAML::Node row;
                    row.SetStyle(YAML::EmitterStyle::Flow);
                    for (int c = 0; c < 4; c++) {
                        row.push_back(planes(r, c));
                    }
                    poly_node.push_back(row);
                }
                backup["sfc"] = poly_node;
                root["backup"] = backup;
            }

            YAML::Emitter out;
            out << root;
            if (!writeTextFile((case_dir / "case.yaml").string(), out.c_str())) {
                return "";
            }

            if (!data.cloud.empty()) {
                pcl::PointCloud<pcl::PointXYZ> cloud;
                cloud.reserve(data.cloud.size());
                for (const auto &p: data.cloud) {
                    cloud.emplace_back(p.x(), p.y(), p.z());
                }
                pcl::io::savePCDFileBinary((case_dir / "cloud.pcd").string(), cloud);
            }

            if (!cfg_path.empty()) {
                std::ifstream ifs(cfg_path);
                std::stringstream ss;
                ss << ifs.rdbuf();
                writeTextFile((case_dir / "config_snapshot.yaml").string(), ss.str());
            }
            return case_id;
        }
    };
}

#endif //SUPER_CASE_DUMP_HPP
