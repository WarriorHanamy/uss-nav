/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/


#ifdef USE_ROS1

#ifndef SRC_FSM_ROS1_HPP
#define SRC_FSM_ROS1_HPP

#include "fsm/fsm.h"

#include <algorithm>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include "quadrotor_msgs/EgoPlannerResult.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/LocalGoalSet.h"
#include "quadrotor_msgs/WaypointProgress.h"
#include "camera_fov/camera_fov.h"


namespace fsm {
    class FsmRos1 : public Fsm {
        ros::NodeHandle nh_;
        ros::Subscriber goal_sub_;
        ros::Publisher cmd_pub, path_pub_, plan_result_pub_, exec_finish_pub_, wp_progress_pub_;
        ros::Timer execution_timer_, replan_timer_, cmd_timer_;
        quadrotor_msgs::PositionCommand pid_cmd_;
        quadrotor_msgs::LocalGoalSet latest_goal_;
        rog_map::ROGMap::Ptr map_ptr_;
        ego_planner::PerceptionUtils::Ptr percep_utils_;
        quadrotor_msgs::PositionCommand latest_cmd;
        Vec3f latest_attitude_;
        nav_msgs::Path path;
        int16_t plan_count_{0};

        vector<quadrotor_msgs::PositionCommand> cmd_logs_;

        void publishMissionFeedback(bool plan_status, bool modify_status, bool exec_finished) {
            quadrotor_msgs::EgoPlannerResult plan_msg;
            plan_msg.planner_goal.x = latest_goal_.goal[0];
            plan_msg.planner_goal.y = latest_goal_.goal[1];
            plan_msg.planner_goal.z = latest_goal_.goal[2];
            plan_msg.plan_times = plan_count_;
            plan_msg.plan_status = plan_status;
            plan_msg.modify_status = modify_status;
            plan_result_pub_.publish(plan_msg);

            std_msgs::Bool finish_msg;
            finish_msg.data = exec_finished;
            exec_finish_pub_.publish(finish_msg);
            ROS_INFO("[EGOPlanner] super_feedback plan_status=%d modify_status=%d exec_finished=%d goal=[%.3f %.3f %.3f]",
                     static_cast<int>(plan_status), static_cast<int>(modify_status), static_cast<int>(exec_finished),
                     plan_msg.planner_goal.x, plan_msg.planner_goal.y, plan_msg.planner_goal.z);
        }

        void resetVisualizedPath() override {
            path.poses.clear();
        }

        void publishWaypointProgress(bool all_consumed) override {
            quadrotor_msgs::WaypointProgress msg;
            msg.batch_id = gi_.batch_id;
            msg.skipped_mask = gi_.wp_skipped_mask;
            msg.all_consumed = all_consumed;
            if (all_consumed) {
                msg.consumed_count = static_cast<uint8_t>(gi_.wp_window_size);
                msg.active_idx = static_cast<uint8_t>(gi_.wp_window_size);
            } else {
                msg.consumed_count = gi_.wp_active_idx > 0
                                     ? static_cast<uint8_t>(gi_.wp_orig_idx[gi_.wp_active_idx - 1] + 1) : 0;
                msg.active_idx = gi_.wp_active_idx < static_cast<int>(gi_.wp_list.size())
                                 ? static_cast<uint8_t>(gi_.wp_orig_idx[gi_.wp_active_idx])
                                 : static_cast<uint8_t>(gi_.wp_window_size);
            }
            wp_progress_pub_.publish(msg);
            ROS_INFO("[SUPER][Progress] event=wp_progress_pub batch_id=%u consumed=%u active=%u skipped_mask=%u all_consumed=%d",
                     msg.batch_id, msg.consumed_count, msg.active_idx,
                     static_cast<unsigned int>(msg.skipped_mask),
                     static_cast<int>(msg.all_consumed));
        }

        void publishCurPoseToPath() override {
            path.header.frame_id = "world";
            path.header.stamp = ros::Time::now();
            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = robot_state_.p(0);
            pose.pose.position.y = robot_state_.p(1);
            pose.pose.position.z = robot_state_.p(2);
            pose.pose.orientation.x = robot_state_.q.x();
            pose.pose.orientation.y = robot_state_.q.y();
            pose.pose.orientation.z = robot_state_.q.z();
            pose.pose.orientation.w = robot_state_.q.w();
            path.poses.push_back(pose);
            path_pub_.publish(path);
        }

        void publishPolyTraj() override {
            // Disabled for uss-nav integration — MPC polynomial trajectory not needed.
        }

        void getOnePositionCommand(quadrotor_msgs::PositionCommand &pos_cmd, bool &traj_finish) {
            pos_cmd.trajectory_flag = 0;
            StatePVAJ pvaj;
            double yaw, yaw_dot;
            bool on_backup_traj;
            planner_ptr_->getOneCommandFromTraj(pvaj, yaw, yaw_dot, on_backup_traj, traj_finish);
            pos_cmd.header.stamp = ros::Time::now();
            pos_cmd.header.frame_id = "world";
            pos_cmd.position.x = pvaj(0, 0);
            pos_cmd.position.y = pvaj(1, 0);
            pos_cmd.position.z = pvaj(2, 0);
            pos_cmd.velocity.x = pvaj(0, 1);
            pos_cmd.velocity.y = pvaj(1, 1);
            pos_cmd.velocity.z = pvaj(2, 1);
            pos_cmd.acceleration.x = pvaj(0, 2);
            pos_cmd.acceleration.y = pvaj(1, 2);
            pos_cmd.acceleration.z = pvaj(2, 2);
            pos_cmd.jerk.x = pvaj(0, 3);
            pos_cmd.jerk.y = pvaj(1, 3);
            pos_cmd.jerk.z = pvaj(2, 3);
            pos_cmd.yaw = yaw;
            pos_cmd.yaw_dot = yaw_dot;
            pos_cmd.trajectory_flag = on_backup_traj ? 2 : 1;
            Vec3f rpy, omg;
            double aT;
            geometry_utils::convertFlatOutputToAttAndOmg(pvaj.col(0), pvaj.col(1), pvaj.col(2), pvaj.col(3), yaw,
                                                         yaw_dot, rpy, omg, aT);
            latest_attitude_ = rpy;
            latest_cmd = pos_cmd;
            cmd_logs_.push_back(latest_cmd);
        }

    public:
        FsmRos1() = default;

        ~FsmRos1(){
            ros::shutdown();
            saveReplanLogToFile("super_latest_log");
            exit(0);
        };

        typedef std::shared_ptr<FsmRos1> Ptr;

        void saveReplanLogToFile(const string &name = "") {
            // run statistic
            double total_length{0.0};
            int total_replan_num{0};
            double average_compt_t{0.0};
            Vec3f cur_p{0, 0, 0};
            for (auto rp: replan_logs_) {
                if (rp.getRetCode() > 0) {
                    if (cur_p.norm() < 1e-6) {
                        cur_p = rp.getRobotP();
                    } else {
                        total_length += (rp.getRobotP() - cur_p).norm();
                        cur_p = rp.getRobotP();
                    }
                    total_replan_num++;
                    average_compt_t += rp.getTotalCompT();
                }
            }


            fmt::print("Total replan num: {}, total length: {}, average computation time: {} ms\n",
                       total_replan_num, total_length, average_compt_t / (total_replan_num==0?1:total_replan_num) * 1000);


            const std::string save_path = name.empty()
                                          ? LOG_FILE_DIR(
                                                  "replan_logs/" + BinaryFileHandler<int>::getCurrentTimeStr() + ".bin")
                                          : LOG_FILE_DIR("replan_logs/" + name + ".bin");
            const std::string csv_path = name.empty()
                                         ? LOG_FILE_DIR(
                                                 "cmd_logs/" + BinaryFileHandler<int>::getCurrentTimeStr() + ".csv")
                                         : LOG_FILE_DIR("cmd_logs/" + name + ".csv");
            BinaryFileHandler<vector<LogOneReplan>>::save(save_path, replan_logs_);

            std::ofstream csv_writer;
            csv_writer.open(csv_path, std::ios::out | std::ios::trunc);
            csv_writer
                    << "time,posi_x,posi_y,posi_z,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,jerk_x,jerk_y,jerk_z,yaw,yaw_rate,backup"
                    << std::endl;
            csv_writer<<std::fixed<<std::setprecision(15);
            for (const auto &cmd: cmd_logs_) {
                csv_writer << cmd.header.stamp.toSec() - system_start_time_ << "," << cmd.position.x << "," << cmd.position.y << ","
                           << cmd.position.z << ","
                           << cmd.velocity.x << "," << cmd.velocity.y << "," << cmd.velocity.z << ","
                           << cmd.acceleration.x << "," << cmd.acceleration.y << "," << cmd.acceleration.z << ","
                           << cmd.jerk.x << "," << cmd.jerk.y << "," << cmd.jerk.z << ","
                           << cmd.yaw << "," << cmd.yaw_dot << "," << static_cast<int>(cmd.trajectory_flag)
                           << std::endl;
            }
            csv_writer.close();
        }

        bool getPoseFromTraj(super_utils::Pose &pose) {
            if (machine_state_ != FOLLOW_TRAJ) {
                cout << YELLOW << "[Fsm] Not in FOLLOW_TRAJ state, can't get pose from traj." << RESET << endl;
                return false;
            }
            getOnePositionCommand(pid_cmd_, traj_finish_);
            if (traj_finish_) {
                cout << GREEN << " -- [Fsm] Traj finish." << RESET << endl;
                if (closeToGoal(0.1)) {
                    if (!gi_.wp_list.empty()) {
                        publishWaypointProgress(true);
                        gi_.wp_list.clear();
                        gi_.wp_orig_idx.clear();
                    }
                    ChangeState("getPoseFromTraj", WAIT_GOAL);
                } else {
                    ChangeState("getPoseFromTraj", GENERATE_TRAJ);
                }
            }
            pose.first = Vec3f{pid_cmd_.position.x, pid_cmd_.position.y, pid_cmd_.position.z};
            pose.second = eulerToQuaternion(latest_attitude_(0), latest_attitude_(1), latest_attitude_(2));


            /// for checking the trajectory continuty
            static int call_cnt{0};
            call_cnt++;
            double cur_vel_norm = std::sqrt(pid_cmd_.velocity.x * pid_cmd_.velocity.x +
                                            pid_cmd_.velocity.y * pid_cmd_.velocity.y +
                                            pid_cmd_.velocity.z * pid_cmd_.velocity.z);
            static double last_v = cur_vel_norm;
            double delta_v = std::abs(cur_vel_norm - last_v);
            last_v = cur_vel_norm;
            static double max_delta_v{0.0};
            if (delta_v > max_delta_v) {
                max_delta_v = delta_v;
            }
            fmt::print(" -- [Fsm] Cur vel: {}, delta_v: {}, max_delta_v: {}\n", cur_vel_norm, delta_v,
                       max_delta_v);
            cmd_logs_.push_back(latest_cmd);
            return true;
        }

        void goalCallback(const quadrotor_msgs::LocalGoalSetConstPtr &msg) {
            latest_goal_ = *msg;
            ++plan_count_;
            super_utils::Vec3f goal_p = Vec3f{msg->goal[0], msg->goal[1], msg->goal[2]};

            double yaw = msg->yaw;
            int yaw_mode = msg->yaw_mode;
            int yaw_path_mode = msg->yaw_path_mode;

            if (yaw_path_mode == quadrotor_msgs::LocalGoalSet::YAW_PATH_SHORTEST) {
                while (yaw > M_PI) yaw -= 2 * M_PI;
                while (yaw < -M_PI) yaw += 2 * M_PI;
            }

            if (yaw_mode == quadrotor_msgs::LocalGoalSet::YAW_MODE_NORMAL && msg->yaw_low_speed) {
                yaw_mode = quadrotor_msgs::LocalGoalSet::YAW_MODE_LOW_SPEED;
            }

            const bool panorama_source_allowed =
                msg->source_task_id == quadrotor_msgs::LocalGoalSet::SOURCE_TASK_EXPLORATION ||
                msg->source_task_id == quadrotor_msgs::LocalGoalSet::SOURCE_TASK_COUNTING;
            if (yaw_mode == quadrotor_msgs::LocalGoalSet::YAW_MODE_PANORAMA && !panorama_source_allowed) {
                ROS_WARN("[SUPER] Reject panorama mode from source_task_id=%u, fallback to NORMAL + SHORTEST.",
                         static_cast<unsigned int>(msg->source_task_id));
                yaw_mode = quadrotor_msgs::LocalGoalSet::YAW_MODE_NORMAL;
                yaw_path_mode = quadrotor_msgs::LocalGoalSet::YAW_PATH_SHORTEST;
                while (yaw > M_PI) yaw -= 2 * M_PI;
                while (yaw < -M_PI) yaw += 2 * M_PI;
            }

            super_utils::Quatf goal_q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
            publishMissionFeedback(true, true, false);

            /* Waypoint window: waypoints is a flattened xyz array (at most 3 points in
             * travel order). Non-empty => multi-goal batch with SUPER-internal progress. */
            const size_t n_floats = msg->waypoints.size();
            if (n_floats >= 3 && n_floats % 3 == 0) {
                std::vector<Vec3f> raw_wps;
                const size_t n_wp = std::min<size_t>(n_floats / 3, 3);
                raw_wps.reserve(n_wp);
                for (size_t i = 0; i < n_wp; i++) {
                    raw_wps.emplace_back(msg->waypoints[3 * i], msg->waypoints[3 * i + 1],
                                         msg->waypoints[3 * i + 2]);
                }
                if (setGoalWindow(msg->batch_id, raw_wps, goal_q, yaw_mode, yaw_path_mode,
                                  msg->look_forward)) {
                    return;
                }
            }
            /* Legacy single-goal path: no active window. */
            gi_.wp_list.clear();
            gi_.wp_orig_idx.clear();
            planner_ptr_->setWaypointLookahead({});
            setGoalPosiAndYaw(goal_p, goal_q, yaw_mode, yaw_path_mode, msg->look_forward);
        }

        void init(const ros::NodeHandle &nh, const std::string &cfg_path) {
            // 初始化参数读取
            nh_ = nh;
            cfg_ = Config(cfg_path);
            map_ptr_ = std::make_shared<rog_map::ROGMapROS>(nh, cfg_path);
            // 初始化Planner
            ros_ptr_ = std::make_shared<ros_interface::Ros1Interface>(nh_);
            planner_ptr_ = std::make_shared<SuperPlanner>(cfg_path, ros_ptr_, map_ptr_);
            cmd_pub = nh_.advertise<quadrotor_msgs::PositionCommand>(cfg_.cmd_topic, 10);
            path_pub_ = nh_.advertise<nav_msgs::Path>("fsm/path", 100);

            // camera_fov params for FOV visualization
            nh_.setParam("camera_fov/top_angle", 0.6);
            nh_.setParam("camera_fov/left_angle", 0.76);
            nh_.setParam("camera_fov/right_angle", 0.76);
            nh_.setParam("camera_fov/max_dist", 6.0);
            nh_.setParam("camera_fov/vis_dist", 1.0);
            percep_utils_ = std::make_shared<ego_planner::PerceptionUtils>(nh_);
            plan_result_pub_ = nh_.advertise<quadrotor_msgs::EgoPlannerResult>("/planning/ego_plan_result", 10);
            exec_finish_pub_ = nh_.advertise<std_msgs::Bool>("/drone_0_ego_planner_node/exec_finish_trigger", 10);
            wp_progress_pub_ = nh_.advertise<quadrotor_msgs::WaypointProgress>(
                    "/drone_0_ego_planner_node/waypoint_progress", 10);

            int cmd_cnt = 0;

            if (cfg_.click_goal_en) {
                goal_sub_ = nh_.subscribe(cfg_.click_goal_topic, 1, &FsmRos1::goalCallback, this);
                cout << YELLOW << " -- [Fsm] CLICKGOAL ENABLE." << RESET << endl;
                cmd_cnt++;
            }

            if (cmd_cnt != 1) {
                cout << YELLOW << " -- [Fsm] CMD INPUT ERROR." << RESET << endl;
                exit(0);
            }

            if (cfg_.timer_en) {
                execution_timer_ = nh_.createTimer(ros::Duration(0.01), &FsmRos1::mainFsmTimerCallback, this); // 100Hz
                cmd_timer_ = nh_.createTimer(ros::Duration(0.01), &FsmRos1::pubCmdTimerCallback, this); // 100Hz
                replan_timer_ = nh_.createTimer(ros::Duration(1.0 / cfg_.replan_rate), &FsmRos1::replanTimerCallback,
                                                this); // 10Hz
            }

            write_time_.open(DEBUG_FILE_DIR("time_consuming.csv"), std::ios::out | std::ios::trunc);
            log_module_time.resize(9);
            for (int i = 0; i < 9; i++) {
                write_time_ << log_time_str[i];
                if (i != 8) {
                    write_time_ << ",";
                }
            }
            write_time_ << endl;
            machine_state_ = INIT;
            system_start_time_ = ros_ptr_->getSimTime();

            pid_cmd_.kx[0] = 5.7;
            pid_cmd_.kx[1] = 5.7;
            pid_cmd_.kx[2] = 4.2;

            pid_cmd_.kv[0] = 3.4;
            pid_cmd_.kv[1] = 3.4;
            pid_cmd_.kv[2] = 4.0;
        }

        void pubCmdTimerCallback(const ros::TimerEvent &event) {
            if (stop) {
                return;
            }
            if (machine_state_ != FOLLOW_TRAJ && machine_state_ != EMER_STOP) {
                return;
            }

            getOnePositionCommand(pid_cmd_, traj_finish_);
            cmd_pub.publish(pid_cmd_);

            // Draw FOV at current command pose (throttled ~20Hz)
            static int fov_cnt = 0;
            if (++fov_cnt % 5 == 0) {
                Vec3f pos(pid_cmd_.position.x, pid_cmd_.position.y, pid_cmd_.position.z);
                percep_utils_->setPose(Eigen::Vector3d(pos.x(), pos.y(), pos.z()), pid_cmd_.yaw);
                std::vector<Eigen::Vector3d> l1, l2;
                percep_utils_->getFOV(l1, l2);
                auto ros1_ptr = std::dynamic_pointer_cast<ros_interface::Ros1Interface>(ros_ptr_);
                if (ros1_ptr) ros1_ptr->vizFov(l1, l2);
            }

            if (traj_finish_) {
                cout << GREEN << " -- [Fsm] Traj finish." << RESET << endl;
                if (closeToGoal(0.1)) {
                    publishMissionFeedback(true, true, true);
                    if (!gi_.wp_list.empty()) {
                        publishWaypointProgress(true);
                        gi_.wp_list.clear();
                        gi_.wp_orig_idx.clear();
                        ros_ptr_->info(" -- [SUPER][Progress] event=wp_batch_done batch_id={}", gi_.batch_id);
                    }
                    ChangeState("PubCmdCallback", WAIT_GOAL);
                } else {
                    ChangeState("PubCmdCallback", GENERATE_TRAJ);
                }
            }
        }

        void replanTimerCallback(const ros::TimerEvent &event) {
            callReplanOnce();
        }

        void mainFsmTimerCallback(const ros::TimerEvent &event) {
            callMainFsmOnce();
        }

    };
}

#endif //SRC_FSM_ROS1_HPP

#endif
