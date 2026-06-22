#ifndef _TRAJ_SERVER_H_
#define _TRAJ_SERVER_H_

#include <iostream>
#include <thread>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/EgoGoalSet.h>
#include <ros/ros.h>
#include <perception_utils/perception_utils.h>
#include <string>

namespace ego_planner
{
    /**
     * 100 Hz trajectory interpolation and command publishing server.
     *
     * Evaluates the optimized polynomial trajectory at 100 Hz, generates
     * PositionCommand messages with position/velocity/acceleration/yaw,
     * and handles yaw planning (normal, look-forward, and panorama modes).
     */
    class TrajServer
    {
    private:

        enum class TrajState {
            IDLE,          // No trajectory loaded
            PRE_YAW,       // Pre-rotation yaw phase before trajectory execution
            EXECUTING_TRAJ // Executing loaded trajectory
          };

        TrajState traj_state_ = TrajState::IDLE;
        double traj_init_yaw_{0.0};        // yaw at trajectory start [rad]
        Eigen::Vector3d traj_init_pos_{Eigen::Vector3d::Zero()}; // position at trajectory start [m]

        ros::NodeHandle node_;
        ros::Publisher pos_cmd_pub_, cmd_vis_pub_;

        shared_ptr<PerceptionUtils> percep_utils_;

        bool receive_traj_{false};
        poly_traj::Trajectory traj_;
        double traj_duration_;  // total trajectory duration [s]
        double start_time_;     // world time at trajectory start [s]
        int traj_id_{0};
        ros::Time heartbeat_time_{0};
        bool do_once_ = true;

        // yaw control
        double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
        double time_forward_;

        double yaw_vel_limit_, yaw_acc_limit_, yaw_vel_low_limit_, yaw_acc_low_limit_; // yaw limits [rad/s], [rad/s^2]
        double yaw_vel_panorama_, yaw_acc_panorama_;                                   // panorama yaw limits [rad/s], [rad/s^2]
        bool panorama_yaw_active_{false};

        struct LAST_POS
        {
            Eigen::Vector3d p;
            bool init{false};
            inline void operator=(const Eigen::Vector3d p_in)
            {
                p = p_in;
                init = true;
            }
        } last_pos_;
        struct YAW_GIVEN
        {
            double yaw;
            bool reach_given_yaw_{true};
            bool look_forward{true};
            uint8_t control_mode{quadrotor_msgs::EgoGoalSet::YAW_MODE_NORMAL};
            uint8_t path_mode{quadrotor_msgs::EgoGoalSet::YAW_PATH_SHORTEST};
            Eigen::Vector3d pos;
        };
        struct TIME_REC
        {
            ros::Time time_last = ros::Time(0);
            bool has_init{false};
        } time_rec_;

    public:
        TrajServer(){};
        ~TrajServer(){};
        
        /**
         * Initialize the trajectory server with ROS node handle.
         *
         * @param[inout] node  ROS node handle
         */
        void initTrajServer(ros::NodeHandle &node);
        /**
         * Set the trajectory to be executed.
         *
         * @param[in] traj        Polynomial trajectory
         * @param[in] start_time  World start time [s]
         */
        void setTrajectory(const poly_traj::Trajectory &traj, double start_time);
        /**
         * Set target yaw for the trajectory.
         *
         * @param[in] des_yaw       Desired yaw [rad]
         * @param[in] cur_yaw       Current yaw [rad]
         * @param[in] pos           Current position [m]
         * @param[in] look_forward  Enable look-forward yaw mode
         * @param[in] control_mode  Yaw control mode
         * @param[in] path_mode     Yaw path mode
         */
        void setYaw(double des_yaw, double cur_yaw, Eigen::Vector3d pos, bool look_forward = true,
                    uint8_t control_mode = quadrotor_msgs::EgoGoalSet::YAW_MODE_NORMAL,
                    uint8_t path_mode = quadrotor_msgs::EgoGoalSet::YAW_PATH_SHORTEST);
        /**
         * Set panorama yaw (continuous 360-degree rotation).
         *
         * @param[in] des_yaw   Desired yaw [rad]
         * @param[in] cur_yaw   Current yaw [rad]
         * @param[in] hold_pos  Position to hold during panorama [m]
         */
        void setPanoramaYaw(double des_yaw, double cur_yaw, const Eigen::Vector3d& hold_pos);
        /**
         * Reset yaw look-forward direction to the given position.
         *
         * @param[in] pos  Position to look toward [m]
         */
        void resetYawLookforward(Eigen::Vector3d pos);
        /**
         * Synchronize internal yaw state from odometry.
         *
         * @param[in] yaw     Odometry yaw [rad]
         * @param[in] source  Source identifier for logging
         */
        void syncYawFromOdom(const double yaw, const std::string& source = "");
        /**
         * Periodic heartbeat callback for trajectory execution.
         */
        void feedDog();
        /**
         * Reset the last commanded position.
         *
         * @param[in] pos  Position to reset to [m]
         */
        void resetLastPos(const Eigen::Vector3d pos);

        YAW_GIVEN yaw_given_;

    private:
        /**
         * Calculate target yaw and yaw rate at the given trajectory time.
         *
         * @param[in]  t_cur  Current trajectory time [s]
         * @param[in]  pos    Current position [m]
         * @param[in]  dt     Time step for derivative [s]
         * @return Pair of (yaw [rad], yaw_rate [rad/s])
         */
        std::pair<double, double> calculate_yaw(double t_cur, const Eigen::Vector3d &pos, double dt);
        /**
         * Publish a position command message.
         *
         * @param[in] p   Position [m]
         * @param[in] v   Velocity [m/s]
         * @param[in] a   Acceleration [m/s^2]
         * @param[in] j   Jerk [m/s^3]
         * @param[in] y   Yaw [rad]
         * @param[in] yd  Yaw rate [rad/s]
         */
        void publish_cmd(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, Eigen::Vector3d j, double y, double yd);
        static void cmdThread(void *obj);
        void cmdFun();
        void drawFOV(const std::vector<Eigen::Vector3d>& list1, const std::vector<Eigen::Vector3d>& list2, ros::Publisher& pub, 
                     double r = 1.0, double g = 0.0, double b = 0.0);
    };
} // namespace ego_planner
#endif
