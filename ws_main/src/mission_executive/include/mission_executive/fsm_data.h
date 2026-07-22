#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <sys/types.h>
#include <string>
#include <vector>
#include <quadrotor_msgs/PerceptionMsg.h>
#include <quadrotor_msgs/Instruction.h>
#include <scene_graph/scene_graph.h>

using std::vector;
using Eigen::Vector3d;

namespace mission_executive {

/**
 * Enum for target type classification.
 */
enum TARGET_TYPE
{
  MANUAL_TARGET  = 1,
  EXPLORE_TARGET = 2,
  PRESET_TARGET  = 3,
  REFENCE_PATH   = 4
};

/**
 * Runtime FSM data shared across mission states.
 */
struct FSMData
{
  bool                    trigger_, have_odom_, static_state_;
  ros::Time               last_pub_time_;
  ros::Time               warmup_start_time_;

  Eigen::Vector3d         odom_pos_, odom_vel_;
  Eigen::Quaterniond      odom_orient_;
  double                  odom_yaw_;
  double                  odom_yaw_rate_;

  Eigen::Vector3d         start_pt_, start_vel_, start_acc_, start_yaw_;

  Eigen::Vector3d         home_pos_;
  bool                    has_home_path_ = false;

  vector<Eigen::Vector3d> path_res_;
  int                     path_inx_;
  Eigen::Vector3d         aim_pos_, aim_vel_;
  Eigen::Vector3d         local_aim_pos_;
  double                  aim_yaw_;
  bool                    has_rotated_;
  bool                    is_lookforward_;
  Eigen::Vector3d         track_pos_;
  bool                    track_trigger_;
  bool                    track_init_;
  bool                    track_finish_candidate_active_{false};
  bool                    track_finish_sent_{false};
  ros::Time               track_finish_candidate_start_time_;
  Eigen::Vector3d         track_finish_last_pos_;
  double                  track_finish_last_yaw_{0.0};
  double                  track_finish_move_acc_{0.0};
  double                  track_finish_yaw_acc_{0.0};
  bool                    directly_connect_to_goal;
  bool                    instruct_directly_to_goal;

  Eigen::Vector3d         ego_local_goal_;
  int                     ego_plan_times_;
  bool                    ego_plan_status_;
  bool                    ego_modify_status_;
  int                     goal_replan_times_;
  bool                    ego_exec_finished_;
  double                  target_yaw_;

  std::unordered_map<int, bool>                              perception_data_get_response_;
  unordered_map<unsigned int, quadrotor_msgs::PerceptionMsg> map_merge_database_;
  quadrotor_msgs::PerceptionMsg                              local_perception_data_;

  Eigen::Vector3d next_given_goal_;
  Eigen::Vector3d waypoint_target_;
  double          waypoint_target_yaw_;

  std::string target_cmd_, prior_knowledge_;
  int object_target_id_;
  u_int8_t go_object_process_phase{0};
  u_int8_t go_waypoint_process_phase{0};

  double  stuck_begin_time_{-1.0};
  int     stuck_force_advance_count_{0};
  bool    stuck_force_advance_triggered_{false};

  quadrotor_msgs::Instruction stored_object_id_nav_instruction_;
  bool has_stored_object_id_nav_instruction_{false};
  double object_id_nav_replan_stuck_begin_time_{-1.0};
  bool object_id_nav_replan_topic_triggered_{false};
  int  object_id_nav_replan_stuck_count_{0};
  double object_id_nav_finish_hold_begin_time_{-1.0};
  bool new_topo_need_predict_immediately_{false};
  bool find_terminate_target_mode_{false};
  u_int8_t llm_plan_explore_counter_{0};

  u_int8_t df_demo_phase_{0};
  int      df_demo_target_id_{-100};
  bool     df_demo_mode_{false};
};

/**
 * FSM parameter configuration loaded from ROS params.
 */
struct FSMParam
{
  double                  replan_dis_thresh_;
  double                  replan_thresh2_;
  double                  replan_thresh3_;
  double                  replan_time_;
  double                  arrive_dis_thr_;
  double                  battery_thr_;
  bool                    flag_realworld_exp_;
  bool                    enable_area_prediction_{false};
  bool                    auto_init_scene_graph_{true};
  bool                    auto_load_scene_graph_{false};
  std::string             scene_graph_data_path_;
  std::string             scene_graph_load_name_;
  double                  auto_init_delay_sec_{2.0};
  double                  scene_graph_init_forward_dist_{1.8};
  double                  track_finish_hold_time_{3.0};
  double                  track_finish_move_thresh_{0.2};
  double                  track_finish_yaw_thresh_{0.2};
  std::string             tracking_backend_{"ego"};
  std::string             tracking_target_odom_topic_{"/target_ekf_odom"};
  double                  track_aim_dist_{1.5};
  double                  track_replan_dist_{0.8};
  double                  track_yaw_thr_{0.2};
  double                  track_turn_yaw_dist_{1.5};
  double                  track_fly_yaw_thr_{0.5};
  double                  track_detect_error_{0.5};
  double                  radius_close_{0.8};
  std::string             planner_cmd_mux_ego_mode_{"ego"};

  bool   stuck_force_advance_enable_{true};
  double stuck_force_advance_vel_thresh_{0.1};
  double stuck_force_advance_yaw_rate_thresh_{0.1};
  double stuck_force_advance_duration_{3.0};
  int    stuck_force_advance_max_consecutive_{2};

  bool   object_id_nav_replan_enable_{false};
  int    object_id_nav_replan_mode_{0};
  double object_id_nav_replan_stuck_vel_thresh_{0.1};
  double object_id_nav_replan_stuck_yaw_rate_thresh_{0.1};
  double object_id_nav_replan_stuck_duration_{3.0};
  int    object_id_nav_replan_stuck_max_consecutive_{0};
  double object_id_nav_replan_mode2_stuck_fallback_delay_{10.0};
  bool   object_id_nav_require_final_yaw_{true};
  double object_id_nav_finish_dwell_sec_{2.0};
  double object_id_nav_finish_vel_thresh_{0.3};
};

}  // namespace mission_executive

#endif
