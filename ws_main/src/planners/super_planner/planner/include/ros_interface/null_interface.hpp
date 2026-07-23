/**
 * Headless RosInterface implementation for offline tools (case replay, parameter
 * sweeps). All visualization calls are no-ops; logs go to stdout via fmt; the
 * sim-time interface is backed by the wall clock.
 */

#ifndef SUPER_NULL_INTERFACE_HPP
#define SUPER_NULL_INTERFACE_HPP

#include <chrono>
#include <ros_interface/ros_interface.hpp>

namespace ros_interface {

    class NullRosInterface : public RosInterface {
        std::chrono::steady_clock::time_point t0_ = std::chrono::steady_clock::now();

        double wallNow() const {
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
        }

    public:
        using Ptr = std::shared_ptr<NullRosInterface>;

        void debug(const std::string &msg) override { fmt::print("{}\n", msg); }
        void info(const std::string &msg) override { fmt::print("{}\n", msg); }
        void warn(const std::string &msg) override { fmt::print("{}\n", msg); }
        void error(const std::string &msg) override { fmt::print(stderr, "{}\n", msg); }
        void fatal(const std::string &msg) override { fmt::print(stderr, "{}\n", msg); }

        void setSimTime(const double &) override {}

        double getSimTime() override { return wallNow(); }

        void getSimTime(int32_t &sec, uint32_t &nsec) override {
            const double t = wallNow();
            sec = static_cast<int32_t>(t);
            nsec = static_cast<uint32_t>((t - sec) * 1e9);
        }

        void vizExpTraj(const Trajectory &, const std::string & = "exp_traj") override {}
        void vizBackupTraj(const Trajectory &) override {}
        void vizFrontendPath(const vec_Vec3f &) override {}
        void vizExpSfc(const PolytopeVec &) override {}
        void vizBackupSfc(const Polytope &) override {}
        void vizGoalPath(const vec_Vec3f &) override {}
        void vizCommittedTraj(const Trajectory &, const double &) override {}
        void vizYawTraj(const Trajectory &, const Trajectory &) override {}
        void vizAstarBoundingBox(const Vec3f &, const Vec3f &) override {}
        void vizAstarPoints(const Vec3f &, const Color &, const std::string &,
                            const double & = 0.1, const int & = 0) override {}
        void vizReplanLog(const Trajectory &, const Trajectory &,
                          const Trajectory &, const Trajectory &,
                          const PolytopeVec &, const Polytope &,
                          const vec_Vec3f &, const int &) override {}
        void vizCiriSeedLine(const Vec3f &, const Vec3f &, const double &) override {}
        void vizCiriEllipsoid(const Ellipsoid &) override {}
        void vizCiriInfeasiblePoint(const Vec3f) override {}
        void vizCiriPolytope(const Polytope &, const std::string &) override {}
        void vizCiriPointCloud(const vec_Vec3f &) override {}
    };
}

#endif //SUPER_NULL_INTERFACE_HPP
