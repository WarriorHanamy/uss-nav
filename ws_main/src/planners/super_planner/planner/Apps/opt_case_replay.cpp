/**
 * Offline replay of one dumped SUPER optimization corner case.
 *
 * Loads a case directory produced by CaseDumper (case.yaml + cloud.pcd +
 * config_snapshot.yaml), optionally regenerates the SFC from the dumped point
 * cloud through ROGMapOffline + CorridorGenerator, then runs ExpTrajOpt with
 * optional parameter overrides and writes a machine-readable result.
 *
 * Usage:
 *   opt_case_replay --case <case_dir> [--override <params.yaml>]...
 *                   [--regen-sfc] [--repeat N] [--out <result.yaml>]
 */

#include <chrono>
#include <filesystem>
#include <iostream>
#include <random>
#include <string>
#include <vector>
#include <unistd.h>

#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>

#include <super_core/config.hpp>
#include <super_core/corridor_generator.h>
#include <traj_opt/exp_traj_optimizer_s4.h>
#include <traj_opt/backup_traj_optimizer_s4.h>
#include <ros_interface/null_interface.hpp>
#include <rog_map/rog_map_offline.h>

using namespace super_utils;
using namespace super_planner;
using namespace geometry_utils;

namespace {

    struct Args {
        std::string case_dir;
        std::vector<std::string> overrides;
        bool regen_sfc{false};
        int repeat{1};
        std::string out_path;
    };

    Args parseArgs(int argc, char **argv) {
        Args args;
        for (int i = 1; i < argc; i++) {
            const std::string a = argv[i];
            auto next = [&]() -> std::string {
                if (i + 1 >= argc) {
                    throw std::runtime_error("missing value for " + a);
                }
                return argv[++i];
            };
            if (a == "--case") args.case_dir = next();
            else if (a == "--override") args.overrides.push_back(next());
            else if (a == "--regen-sfc") args.regen_sfc = true;
            else if (a == "--repeat") args.repeat = std::stoi(next());
            else if (a == "--out") args.out_path = next();
            else throw std::runtime_error("unknown argument: " + a);
        }
        if (args.case_dir.empty()) {
            throw std::runtime_error("--case <case_dir> is required");
        }
        return args;
    }

    Vec3f nodeToVec3(const YAML::Node &n) {
        return Vec3f(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
    }

    StatePVAJ nodeToPvaj(const YAML::Node &n) {
        StatePVAJ m;
        m.col(0) = nodeToVec3(n["pos"]);
        m.col(1) = nodeToVec3(n["vel"]);
        m.col(2) = nodeToVec3(n["acc"]);
        m.col(3) = nodeToVec3(n["jerk"]);
        return m;
    }

    vec_E<Vec3f> nodeToVec3List(const YAML::Node &n) {
        vec_E<Vec3f> out;
        if (!n || !n.IsSequence()) {
            return out;
        }
        for (const auto &p: n) {
            out.push_back(nodeToVec3(p));
        }
        return out;
    }

    std::vector<double> nodeToDoubleList(const YAML::Node &n) {
        std::vector<double> out;
        if (!n || !n.IsSequence()) {
            return out;
        }
        for (const auto &v: n) {
            out.push_back(v.as<double>());
        }
        return out;
    }

    PolytopeVec nodeToSfc(const YAML::Node &n) {
        PolytopeVec sfc;
        if (!n || !n.IsSequence()) {
            return sfc;
        }
        for (const auto &poly_node: n) {
            MatD4f planes(poly_node.size(), 4);
            for (size_t r = 0; r < poly_node.size(); r++) {
                for (int c = 0; c < 4; c++) {
                    planes(r, c) = poly_node[r][c].as<double>();
                }
            }
            sfc.emplace_back(planes);
        }
        return sfc;
    }

    Polytope nodeToPolytope(const YAML::Node &poly_node) {
        MatD4f planes(poly_node.size(), 4);
        for (size_t r = 0; r < poly_node.size(); r++) {
            for (int c = 0; c < 4; c++) {
                planes(r, c) = poly_node[r][c].as<double>();
            }
        }
        return Polytope(planes);
    }

    /* Flatten a yaml map into "a/b/c" -> scalar node pairs. */
    void flattenYaml(const YAML::Node &node, const std::string &prefix,
                     std::vector<std::pair<std::string, YAML::Node>> &out) {
        if (node.IsMap()) {
            for (const auto &kv: node) {
                flattenYaml(kv.second, prefix.empty() ? kv.first.as<std::string>()
                                                      : prefix + "/" + kv.first.as<std::string>(), out);
            }
        } else {
            out.emplace_back(prefix, YAML::Clone(node));
        }
    }

    void setYamlByPathRec(YAML::Node root, const std::vector<std::string> &keys,
                          const size_t idx, const YAML::Node &value) {
        if (idx + 1 == keys.size()) {
            root[keys[idx]] = value;
            return;
        }
        YAML::Node child = root[keys[idx]];
        if (!child || !child.IsMap()) {
            root[keys[idx]] = YAML::Node(YAML::NodeType::Map);
            child = root[keys[idx]];
        }
        setYamlByPathRec(child, keys, idx + 1, value);
    }

    void setYamlByPath(YAML::Node root, const std::string &path, const YAML::Node &value) {
        std::vector<std::string> keys;
        std::stringstream ss(path);
        std::string key;
        while (std::getline(ss, key, '/')) {
            if (!key.empty()) {
                keys.push_back(key);
            }
        }
        if (keys.empty()) {
            return;
        }
        setYamlByPathRec(root, keys, 0, value);
    }

    /* Merge the case config snapshot with override files into a temp config and
     * force the map into offline (no ROS callback) mode. */
    std::string buildMergedConfig(const std::string &case_dir,
                                  const std::vector<std::string> &overrides) {
        const std::string snapshot = case_dir + "/config_snapshot.yaml";
        YAML::Node merged = YAML::LoadFile(snapshot);
        for (const auto &ovr_path: overrides) {
            std::vector<std::pair<std::string, YAML::Node>> flat;
            flattenYaml(YAML::LoadFile(ovr_path), "", flat);
            for (const auto &[path, value]: flat) {
                setYamlByPath(merged, path, value);
            }
        }
        setYamlByPath(merged, "rog_map/ros_callback/enable", YAML::Node(false));
        setYamlByPath(merged, "case_dump/enable", YAML::Node(false));
        /* Container-private /tmp with a random suffix: parallel sweep tasks share
         * the case dir but must never share the merged config file. */
        std::mt19937_64 rng(std::random_device{}());
        const std::string merged_path =
                "/tmp/opt_case_config_merged_" + std::to_string(rng()) + ".yaml";
        std::ofstream ofs(merged_path, std::ios::out | std::ios::trunc);
        ofs << merged;
        ofs.close();
        return merged_path;
    }

} // namespace

int main(int argc, char **argv) {
    Args args;
    try {
        args = parseArgs(argc, argv);
    } catch (const std::exception &e) {
        std::cerr << "[opt_case_replay] " << e.what() << std::endl;
        return 2;
    }

    ros::init(argc, argv, "opt_case_replay", ros::init_options::NoSigintHandler);

    const YAML::Node case_node = YAML::LoadFile(args.case_dir + "/case.yaml");
    const std::string case_id = case_node["case_id"].as<std::string>();
    const std::string merged_cfg = buildMergedConfig(args.case_dir, args.overrides);

    super_planner::Config cfg(merged_cfg);
    auto ros_ptr = std::make_shared<ros_interface::NullRosInterface>();

    const StatePVAJ head_pvaj = nodeToPvaj(case_node["head_pvaj"]);
    const StatePVAJ tail_pvaj = nodeToPvaj(case_node["tail_pvaj"]);
    const vec_E<Vec3f> guide_path = nodeToVec3List(case_node["guide_path"]);
    const std::vector<double> guide_t = nodeToDoubleList(case_node["guide_t"]);
    const vec_E<Vec3f> pass_wps = nodeToVec3List(case_node["pass_wps"]);

    PolytopeVec sfc;
    bool sfc_regenerated{false};
    double sfc_time_s{0.0};
    if (args.regen_sfc) {
        const std::string cloud_path = args.case_dir + "/cloud.pcd";
        pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
        if (pcl::io::loadPCDFile(cloud_path, cloud_xyz) != 0) {
            std::cerr << "[opt_case_replay] failed to load " << cloud_path << std::endl;
            return 3;
        }
        rog_map::PointCloud cloud;
        cloud.reserve(cloud_xyz.size());
        for (const auto &p: cloud_xyz) {
            pcl::PointXYZI pi;
            pi.x = p.x;
            pi.y = p.y;
            pi.z = p.z;
            pi.intensity = 0;
            cloud.push_back(pi);
        }
        auto map_ptr = std::make_shared<rog_map::ROGMapOffline>(merged_cfg);
        const Vec3f robot_p = nodeToVec3(case_node["robot_p"]);
        const super_utils::Pose pose{robot_p, Quatf::Identity()};
        map_ptr->updateMap(cloud, pose);

        const auto &rog_cfg = map_ptr->getMapConfig();
        super_planner::CorridorGenerator cg(ros_ptr, map_ptr,
                                            cfg.corridor_bound_dis,
                                            cfg.corridor_line_max_length,
                                            cfg.resolution,
                                            rog_cfg.virtual_ground_height,
                                            rog_cfg.virtual_ceil_height,
                                            cfg.robot_r,
                                            cfg.obs_skip_num,
                                            cfg.iris_iter_num);
        cg.SetLineNeighborList(cfg.seed_line_neighbour);
        Vec3f shifted_start(9999, 9999, 9999);
        const auto sfc_t0 = std::chrono::steady_clock::now();
        const bool sfc_ok = cg.SearchPolytopeOnPath(guide_path, sfc, shifted_start, false);
        sfc_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - sfc_t0).count();
        if (!sfc_ok) {
            std::cerr << "[opt_case_replay] SFC regeneration failed" << std::endl;
            return 4;
        }
        sfc_regenerated = true;
    } else {
        sfc = nodeToSfc(case_node["sfc"]);
    }
    if (sfc.empty()) {
        std::cerr << "[opt_case_replay] empty SFC, nothing to optimize" << std::endl;
        return 5;
    }

    traj_opt::ExpTrajOpt opt(cfg.exp_traj_cfg, ros_ptr);
    Trajectory traj;
    bool success{false};
    double best_time_s{std::numeric_limits<double>::max()};
    std::vector<double> exp_times;
    for (int r = 0; r < args.repeat; r++) {
        PolytopeVec sfc_run = sfc;
        Trajectory traj_run;
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = opt.optimize(head_pvaj, tail_pvaj, guide_path, guide_t,
                                     sfc_run, traj_run, pass_wps);
        const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        exp_times.push_back(dt);
        best_time_s = std::min(best_time_s, dt);
        if (r == args.repeat - 1) {
            success = ok;
            traj = traj_run;
        }
    }
    const auto &stats = opt.getLastOptStats();

    /* Replay the backup-trajectory problem on the replayed exp trajectory. */
    bool backup_attempted{false};
    bool backup_success{false};
    double backup_best_time_s{0.0};
    std::vector<double> backup_times;
    const YAML::Node backup_node = case_node["backup"];
    if (success && backup_node && backup_node["t0"]) {
        backup_attempted = true;
        const double bk_t0 = backup_node["t0"].as<double>();
        const double bk_te = backup_node["te"].as<double>();
        double bk_heu_ts = backup_node["heu_ts"].as<double>();
        double bk_heu_dur = backup_node["heu_dur"].as<double>();
        const Vec3f bk_heu_p = nodeToVec3(backup_node["heu_p"]);
        Polytope bk_sfc = nodeToPolytope(backup_node["sfc"]);
        traj_opt::BackupTrajOpt back_opt(cfg.back_traj_cfg, ros_ptr);
        backup_best_time_s = std::numeric_limits<double>::max();
        for (int r = 0; r < args.repeat; r++) {
            Trajectory bk_traj;
            double bk_opt_ts = bk_heu_ts;
            double heu_dur_run = bk_heu_dur;
            const auto t0 = std::chrono::steady_clock::now();
            const bool ok = back_opt.optimize(traj, bk_t0, bk_te, bk_heu_ts, bk_heu_p,
                                              heu_dur_run, bk_sfc, bk_traj, bk_opt_ts);
            const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            backup_times.push_back(dt);
            backup_best_time_s = std::min(backup_best_time_s, dt);
            if (r == args.repeat - 1) {
                backup_success = ok;
            }
        }
    }

    YAML::Node result;
    result["case_id"] = case_id;
    result["reason"] = case_node["reason"].as<std::string>("");
    result["ret"] = success ? 0 : -1;
    result["lbfgs_ret"] = stats.lbfgs_ret;
    result["iter_num"] = stats.iter_num;
    result["final_cost"] = stats.final_cost;
    result["opt_time"] = best_time_s;
    result["sfc_count"] = static_cast<int>(sfc.size());
    result["sfc_regenerated"] = sfc_regenerated;
    if (sfc_regenerated) {
        result["sfc_time"] = sfc_time_s;
    }
    result["traj_duration"] = success ? traj.getTotalDuration() : 0.0;
    {
        YAML::Node times(YAML::NodeType::Sequence);
        times.SetStyle(YAML::EmitterStyle::Flow);
        for (const double t: exp_times) {
            times.push_back(t);
        }
        result["exp_opt_times"] = times;
    }
    if (backup_attempted) {
        YAML::Node backup;
        backup["success"] = backup_success;
        backup["opt_time"] = backup_best_time_s;
        YAML::Node btimes(YAML::NodeType::Sequence);
        btimes.SetStyle(YAML::EmitterStyle::Flow);
        for (const double t: backup_times) {
            btimes.push_back(t);
        }
        backup["opt_times"] = btimes;
        result["backup"] = backup;
    }
    if (stats.penalty_log.size() > 0) {
        YAML::Node pen(YAML::NodeType::Sequence);
        pen.SetStyle(YAML::EmitterStyle::Flow);
        for (int i = 0; i < stats.penalty_log.size(); i++) {
            pen.push_back(stats.penalty_log(i));
        }
        result["penalty_log"] = pen;
    }
    if (!args.overrides.empty()) {
        YAML::Node ovr(YAML::NodeType::Sequence);
        for (const auto &o: args.overrides) {
            ovr.push_back(std::filesystem::path(o).filename().string());
        }
        result["overrides"] = ovr;
    }

    YAML::Emitter out;
    out << result;
    if (args.out_path.empty()) {
        std::cout << out.c_str() << std::endl;
    } else {
        std::ofstream ofs(args.out_path, std::ios::out | std::ios::trunc);
        ofs << out.c_str() << std::endl;
        ofs.close();
    }
    std::filesystem::remove(merged_cfg);
    return success ? 0 : 1;
}
