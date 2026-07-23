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

#include <fsm/fsm.h>
#include <memory>

using namespace super_utils;

namespace fsm {
    Fsm::~Fsm() {
        write_time_.close();
    }

    void Fsm::WriteTimeToLog() {
        write_time_ << (ros_ptr_->getSimTime() - system_start_time_) << ", ";
        for (long unsigned int i = 0; i < log_module_time.size(); i++) {
            write_time_ << log_module_time[i];
            if (i != log_module_time.size() - 1) {
                write_time_ << ", ";
            }
        }
        write_time_ << endl;
    }

    void Fsm::callReplanOnce() {
        if (stop) {
            return;
        }

        if (machine_state_ != FOLLOW_TRAJ) {
            return;
        }

        if (finish_plan) {
            return;
        }

        if (plan_from_rest_) {
            plan_from_rest_ = false;
            return;
        }

        /* Do NOT snap gi_.goal_p here: in-place mutation every replan cycle ratchets the
         * goal away (A* already repairs occupied/out-of-map goals into a local copy). */
        TimeConsuming replan_once_time("replan_once_time", false);

        RET_CODE ret_code = planner_ptr_->ReplanOnce(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
        if (ret_code == FAILED) {
//            cout << YELLOW << " -- [Fsm] ReplanOnce failed." << RESET << endl;
            consecutive_replan_failures_++;
            planner_ptr_->notifyConsecutiveFailures(consecutive_replan_failures_);
            const double dist_to_goal = (robot_state_.p - gi_.goal_p).norm();
            ros_ptr_->warn(" -- [SUPER][Progress] event=replan_failed state={} consecutive_failures={} dist_to_goal={} pos={} goal={} vel_norm={}",
                           MACHINE_STATE_STR[machine_state_], consecutive_replan_failures_, dist_to_goal,
                           robot_state_.p.transpose(), gi_.goal_p.transpose(), robot_state_.v.norm());
        } else {
            consecutive_replan_failures_ = 0;
            cout << GREEN << " -- [Fsm] ReplanOnce succeed." << RESET << endl;
        }

        if (ret_code == EMER) {
            ChangeState("ReplanTimerCallback", EMER_STOP);
        } else if (ret_code == NEW_TRAJ) {
            ChangeState("ReplanTimerCallback", GENERATE_TRAJ);
        } else if (ret_code == SUCCESS || ret_code == FINISH) {
            gi_.new_goal = false;
            publishPolyTraj();
        }

        planner_ptr_->getModuleTimeConsuming(log_module_time);
        log_module_time[log_module_time.size() - 2] = replan_once_time.stop();
        // save on log
        replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
        WriteTimeToLog();
    }

    void Fsm::callMainFsmOnce() {
        if (stop) {
            return;
        }
        static double fsm_start_time = ros_ptr_->getSimTime();
        double cur_t = (ros_ptr_->getSimTime() - fsm_start_time);
        static double last_print_t = 0.0;
        planner_ptr_->getRobotState(robot_state_);


        if (cur_t - last_print_t > 1.0) {
            last_print_t = cur_t;
            if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                return;
            }
            if (!started_) {
                cout << YELLOW << " -- [Fsm] Wait for goal." << RESET << endl;
            }
            cout << std::fixed << std::setprecision(3);
            cout << GREEN << " -- [Fsm " << cur_t << "] Current state: " << MACHINE_STATE_STR[machine_state_]
                 << RESET << endl;
        }

        switch (machine_state_) {
            case INIT: {
                if (!started_) {
                    return;
                }
                if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                    cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                }
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            case WAIT_GOAL: {
                if (!gi_.new_goal) {
                    return;
                } else {
                    ChangeState("MainFsmCallback", GENERATE_TRAJ);
                }
                resetVisualizedPath();
                break;
            }
            case GENERATE_TRAJ: {
                if (closeToGoal(0.3)) {
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    gi_.new_goal = false;
                    finish_plan = true;
                    return;
                }
                int retcode = planner_ptr_->PlanFromRest(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
                if (!planner_ptr_->goalValid()) {
                    cout << YELLOW << " -- [Fsm] Goal is invalid, skip this goal." << RESET << endl;
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    return;
                }
                if (retcode == SUCCESS || retcode == FINISH) {
                    gi_.new_goal = false;
                    plan_from_rest_ = true;
                    finish_plan = false;
                    if (retcode == FINISH) {
                        finish_plan = true;
                    }

                    publishPolyTraj();

                    ChangeState("MainFsmCallback", FOLLOW_TRAJ);
                } else {
                    cout << YELLOW << " -- [Fsm] PlanFromRest failed, try replan." << RESET << endl;
                    // ros::Duration(0.1).sleep();
                }
                replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
                break;
            }
            case FOLLOW_TRAJ: {
                publishCurPoseToPath();
                updateWaypointProgress();
                logNavigationProgress();
                break;
            }
            case EMER_STOP: {
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            default:
                break;
        }
    }

    void Fsm::declareGoalUnreachable(const std::string &reason) {
        const double dist_to_goal = (robot_state_.p - gi_.goal_p).norm();
        ros_ptr_->warn(" -- [SUPER][Progress] event=goal_unreachable reason={} dist_to_goal={} pos={} goal={} "
                       "goal_unfinish_count={} consecutive_replan_failures={}",
                       reason, dist_to_goal, robot_state_.p.transpose(), gi_.goal_p.transpose(),
                       goal_unfinish_count_, consecutive_replan_failures_);
        publishMissionFailure();
        goal_unfinish_count_ = 0;
        consecutive_replan_failures_ = 0;
        gi_.new_goal = false;
        finish_plan = true;
        ChangeState("declareGoalUnreachable", WAIT_GOAL);
    }

    bool Fsm::closeToGoal(const double &thresh_dis) {
        /// The close to goal should consider the the local shift
        /// All goal should be in the known free on inf map.
        /// The intermedia points should be in free space.
        double dis = (robot_state_.p - gi_.goal_p).norm();
        return dis < thresh_dis;
    }

    void Fsm::logNavigationProgress() {
        const double now = ros_ptr_->getSimTime();
        if (last_progress_log_t_ > 0.0 && now - last_progress_log_t_ < 1.0) {
            return;
        }
        last_progress_log_t_ = now;

        const double dist_to_goal = (robot_state_.p - gi_.goal_p).norm();
        double progress = 0.0;
        if (last_progress_dist_ >= 0.0) {
            progress = last_progress_dist_ - dist_to_goal;
        }
        if (last_progress_dist_ < 0.0 || progress > 0.05) {
            last_progress_move_t_ = now;
        }
        last_progress_dist_ = dist_to_goal;

        const double no_progress_duration = last_progress_move_t_ > 0.0 ? now - last_progress_move_t_ : 0.0;
        const bool stuck_suspect = started_ && machine_state_ == FOLLOW_TRAJ &&
                                   dist_to_goal > 0.5 &&
                                   no_progress_duration > 3.0 &&
                                   robot_state_.v.norm() < 0.2;
        ros_ptr_->info(" -- [SUPER][Progress] event=nav_tick state={} dist_to_goal={} progress_1s={} no_progress_duration={} stuck_suspect={} pos={} goal={} vel_norm={} consecutive_replan_failures={} traj_finish={}",
                       MACHINE_STATE_STR[machine_state_], dist_to_goal, progress, no_progress_duration,
                       stuck_suspect, robot_state_.p.transpose(), gi_.goal_p.transpose(),
                       robot_state_.v.norm(), consecutive_replan_failures_, traj_finish_);
        if (stuck_suspect) {
            ros_ptr_->warn(" -- [SUPER][Progress] event=stuck_suspect dist_to_goal={} no_progress_duration={} pos={} goal={} vel_norm={} consecutive_replan_failures={}",
                           dist_to_goal, no_progress_duration, robot_state_.p.transpose(),
                           gi_.goal_p.transpose(), robot_state_.v.norm(), consecutive_replan_failures_);
            if (no_progress_duration > cfg_.goal_unreachable_timeout_s) {
                declareGoalUnreachable("no_progress_timeout");
            }
        }
    }

    void Fsm::setGoalPosiAndYaw(const Vec3f &p, const Quatf &q, int yaw_mode, int yaw_path_mode, bool look_forward) {

        if (planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED, p, gi_.goal_p, 3.0)) {
            cout << GREEN << " -- [Fsm] Get goal at " << RESET << gi_.goal_p.transpose() << endl;
        } else {
            fmt::print(fg(fmt::color::indian_red), "Goal is deeply occupied, skip this goal.\n");
            return;
        }

        if ((robot_state_.p - gi_.goal_p).norm() <
            0.1) {
            //                print(fg(color::gray), " -- [Rviz] Too close to goal, skip this target.\n");
            return;
        }

        gi_.yaw_mode = yaw_mode;
        gi_.yaw_path_mode = yaw_path_mode;
        gi_.look_forward = look_forward;

        if (cfg_.yaw_mode == fsm::Config::YAW_TO_VEL) {
            gi_.goal_yaw = NAN;
            ros_ptr_->info(" -- [SUPER] [Fsm] yaw_mode=YAW_TO_VEL goal yaw free (velocity heading) goal=[{:.3f},{:.3f},{:.3f}]",
                           gi_.goal_p.x(), gi_.goal_p.y(), gi_.goal_p.z());
        } else if (cfg_.yaw_mode == fsm::Config::YAW_TO_GOAL) {
            if (look_forward || !cfg_.click_yaw_en) {
                gi_.goal_yaw = NAN;
                ros_ptr_->info(" -- [SUPER] [Fsm] yaw_mode=YAW_TO_GOAL look_forward={} goal yaw free goal=[{:.3f},{:.3f},{:.3f}]",
                               static_cast<int>(look_forward),
                               gi_.goal_p.x(), gi_.goal_p.y(), gi_.goal_p.z());
            } else {
                if (isnan(q.w()) || isnan(q.x()) || isnan(q.y()) || isnan(q.z())) {
                    gi_.goal_yaw = NAN;
                    ros_ptr_->info(" -- [SUPER] [Fsm] goal yaw disabled (NaN quat) goal=[{:.3f},{:.3f},{:.3f}]",
                                   gi_.goal_p.x(), gi_.goal_p.y(), gi_.goal_p.z());
                } else {
                    gi_.goal_yaw = geometry_utils::get_yaw_from_quaternion(q);
                    ros_ptr_->info(" -- [SUPER] [Fsm] goal yaw={:.2f} deg goal=[{:.3f},{:.3f},{:.3f}]",
                                   gi_.goal_yaw * 57.3,
                                   gi_.goal_p.x(), gi_.goal_p.y(), gi_.goal_p.z());
                }
            }
        }

        planner_ptr_->getRobotState(robot_state_);
        if (robot_state_.rcv) {
            planner_ptr_->getMap()->clearUnknownAroundOnFirstGoal(robot_state_.p);
        }

        started_ = true;
        gi_.new_goal = true;
        last_progress_dist_ = -1.0;
        last_progress_move_t_ = ros_ptr_->getSimTime();
        consecutive_replan_failures_ = 0;
        goal_unfinish_count_ = 0;
    }

    bool Fsm::setGoalWindow(const uint32_t batch_id, const std::vector<Vec3f> &raw_wps,
                            const Quatf &q, const int yaw_mode, const int yaw_path_mode,
                            const bool look_forward) {
        gi_.wp_list.clear();
        gi_.wp_orig_idx.clear();
        gi_.wp_skipped_mask = 0;
        gi_.wp_window_size = static_cast<int>(raw_wps.size());
        /* Reproject every waypoint onto the nearest non-occupied cell on the inflate map;
         * deeply occupied ones are skipped and reported through skipped_mask. */
        for (size_t i = 0; i < raw_wps.size(); i++) {
            Vec3f reproj;
            if (planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED, raw_wps[i], reproj, 3.0)) {
                gi_.wp_list.push_back(reproj);
                gi_.wp_orig_idx.push_back(static_cast<int>(i));
            } else {
                gi_.wp_skipped_mask |= static_cast<uint8_t>(1u << i);
                ros_ptr_->warn(" -- [SUPER][Progress] event=wp_reproject_fail batch_id={} wp_idx={} wp={}",
                               batch_id, i, raw_wps[i].transpose());
            }
        }
        if (gi_.wp_list.empty()) {
            ros_ptr_->warn(" -- [SUPER][Progress] event=wp_batch_rejected batch_id={} reason=all_reproject_failed",
                           batch_id);
            return false;
        }
        gi_.batch_id = batch_id;
        gi_.wp_active_idx = 0;
        ros_ptr_->info(" -- [SUPER][Progress] event=wp_batch_recv batch_id={} size={} valid={} skipped_mask={}",
                       batch_id, raw_wps.size(), gi_.wp_list.size(),
                       static_cast<int>(gi_.wp_skipped_mask));
        /* If the robot is already on top of the first waypoint, consume it immediately. */
        planner_ptr_->getRobotState(robot_state_);
        while (gi_.wp_list.size() > 1 &&
               (robot_state_.p - gi_.wp_list.front()).norm() < cfg_.wp_reach_radius) {
            gi_.wp_list.erase(gi_.wp_list.begin());
            gi_.wp_orig_idx.erase(gi_.wp_orig_idx.begin());
        }
        setGoalPosiAndYaw(gi_.wp_list.front(), q, yaw_mode, yaw_path_mode, look_forward);
        /* Yaw only applies when the batch-final waypoint becomes the active target. */
        gi_.pending_goal_yaw = gi_.goal_yaw;
        if (gi_.wp_list.size() > 1) {
            gi_.goal_yaw = NAN;
        }
        pushWaypointLookaheadToPlanner();
        return true;
    }

    void Fsm::pushWaypointLookaheadToPlanner() {
        vec_E<Vec3f> la;
        for (size_t i = gi_.wp_active_idx + 1; i < gi_.wp_list.size(); i++) {
            la.push_back(gi_.wp_list[i]);
        }
        planner_ptr_->setWaypointLookahead(la);
    }

    void Fsm::updateWaypointProgress() {
        if (!started_ || gi_.wp_list.empty() ||
            gi_.wp_active_idx >= static_cast<int>(gi_.wp_list.size())) {
            return;
        }
        const Vec3f tgt = gi_.wp_list[gi_.wp_active_idx];
        const double dist = (robot_state_.p - tgt).norm();
        bool reached = dist < cfg_.wp_reach_radius;
        if (!reached && gi_.wp_active_idx + 1 < static_cast<int>(gi_.wp_list.size())) {
            /* Pass-by: the robot crossed the waypoint's plane along the travel direction
             * with bounded lateral error (the soft pass cost may legitimately deviate). */
            const Vec3f dir = (gi_.wp_list[gi_.wp_active_idx + 1] - tgt).normalized();
            const Vec3f rel = robot_state_.p - tgt;
            const double along = rel.dot(dir);
            const double lateral = (rel - along * dir).norm();
            reached = along > 0.0 && lateral < 2.0;
        }
        if (!reached) {
            return;
        }
        ros_ptr_->info(" -- [SUPER][Progress] event=wp_consumed batch_id={} wp_idx={} orig_idx={} dist={}",
                       gi_.batch_id, gi_.wp_active_idx, gi_.wp_orig_idx[gi_.wp_active_idx], dist);
        gi_.wp_active_idx++;
        if (gi_.wp_active_idx < static_cast<int>(gi_.wp_list.size())) {
            gi_.goal_p = gi_.wp_list[gi_.wp_active_idx];
            if (gi_.wp_active_idx == static_cast<int>(gi_.wp_list.size()) - 1) {
                gi_.goal_yaw = gi_.pending_goal_yaw;
            }
            gi_.new_goal = true;
            pushWaypointLookaheadToPlanner();
        } else {
            /* The batch-final arrival is still reported through the exec_finish path
             * (traj_finish + closeToGoal); only clear the lookahead here. */
            planner_ptr_->setWaypointLookahead({});
        }
        publishWaypointProgress(false);
    }

    void Fsm::ChangeState(const string &call_func, const MACHINE_STATE &new_state) {
        fmt::print(fg(fmt::color::green), " -- [Fsm]: [{}] change state from [{}] to [{}].\n", call_func,
                   MACHINE_STATE_STR[int(machine_state_)], MACHINE_STATE_STR[int(new_state)]);
        machine_state_ = new_state;
    }
}
