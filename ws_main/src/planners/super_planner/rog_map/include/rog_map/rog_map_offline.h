/**
 * Offline ROG-Map variant for headless tools (case replay, parameter sweeps).
 * Identical to the ROS wrapper but without any node handle, topic or timer;
 * the point cloud and robot pose are pushed in directly from a dumped case.
 */

#pragma once

#include <chrono>
#include <rog_map/rog_map.h>

namespace rog_map {

    class ROGMapOffline : public ROGMap {
        const double getSystemWalltimeNow() override {
            return std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
        }

    public:
        typedef std::shared_ptr<ROGMapOffline> Ptr;

        explicit ROGMapOffline(const std::string &cfg_path) {
            cfg_ = rog_map::Config(cfg_path);
            init();
        }

        /**
         * Set the robot pose before pushing the case point cloud.
         *
         * @param[in] pose  robot position [m] and orientation quaternion [-]
         */
        void setRobotStateOffline(const Pose &pose) {
            updateRobotState(pose);
        }
    };
}
