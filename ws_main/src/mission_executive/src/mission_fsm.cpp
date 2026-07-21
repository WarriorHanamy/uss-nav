#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <mission_executive/mission_data.h>
#include <global_belief/map_interface.hpp>
#include <ostream>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <scene_graph/PromptMsg.h>
#include <scene_graph/data_structure.h>
#include <scene_graph/scene_graph.h>
#include <scene_graph/skeleton_generation.h>
#include <std_msgs/Bool.h>
#include <string>
#include <mission_executive/mission_fsm.h>
#include <mission_executive/fsm_data.h>
#include <plan_env/grid_map.h>
#include <memory>
#include <unistd.h>
#include <visualization_msgs/MarkerArray.h>

#define CALL_EVERY_N_TIMES(func, n)         \
    do {                                    \
        static int counter = 0;             \
        ++counter;                          \
        if (counter >= (n)) {               \
            func();                         \
            counter = 0;                    \
        }                                   \
    } while (0)
using Eigen::Vector4d;

namespace mission_executive {
namespace {
/**
 * Normalize an angle to the range [-pi, pi].
 *
 * @param[in] angle  Input angle [rad]
 * @return Normalized angle [rad]
 */
double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

/**
 * Escape special characters (backslash and quote) in a JSON string value.
 *
 * @param[in] value  Raw string
 * @return Escaped string safe for JSON insertion
 */
std::string jsonEscape(const std::string& value) {
  std::string out;
  out.reserve(value.size());
  for (char c : value) {
    if (c == '\\' || c == '"') out.push_back('\\');
    out.push_back(c);
  }
  return out;
}

std::string formatTopoPath(const vector<Eigen::Vector3d>& path) {
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < path.size(); ++i) {
    if (i > 0) oss << ";";
    oss << i << ":(" << path[i].x() << "," << path[i].y() << "," << path[i].z() << ")";
  }
  oss << "]";
  return oss.str();
}
}  // namespace

void MissionFSM::init(ros::NodeHandle& nh, const global_belief::MapInterface::Ptr& map)
{
  md_ = std::make_shared<MissionData>();
  fp_ = std::make_shared<FSMParam>();
  fd_ = std::make_shared<FSMData>();
  node_ = nh;
  /*  Fsm param  */
  nh.param("fsm/drone_id", md_->drone_id_, 0);
  md_->is_initialized_=false;//swarm info callback

  /* The SUPER backend owns waypoint progress; the EGO backend keeps the legacy
   * single-goal protocol with distance-based advancement. */
  bool enable_ego_replan = true;
  nh.param("fsm/enable_ego_replan", enable_ego_replan, true);
  use_super_backend_ = !enable_ego_replan;
  ROS_INFO("[MissionFSM] planner_backend=%s", use_super_backend_ ? "super" : "ego");

  fd_->target_cmd_ = "None";
  fd_->prior_knowledge_ = "Toilet is derictly connected to living room";
  fd_->target_cmd_ = nh.param<std::string>("fsm/target_cmd",    "None");

  nh.param("fsm/thresh_replan1",             fp_->replan_dis_thresh_, -1.0);
  nh.param("fsm/thresh_replan2",             fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3",             fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time",                fp_->replan_time_, -1.0);
  nh.param("fsm/arrive_dis_thr",             fp_->arrive_dis_thr_, 0.1);
  nh.param("fsm/battery_thr",                fp_->battery_thr_,  19.0);
  nh.param("fsm/realworld_experiment",       fp_->flag_realworld_exp_, true);
  nh.param("fsm/enable_area_prediction",     fp_->enable_area_prediction_, true);
  nh.param("fsm/auto_init_scene_graph",      fp_->auto_init_scene_graph_, true);
  nh.param("fsm/auto_load_scene_graph",     fp_->auto_load_scene_graph_, false);
  nh.param("fsm/scene_graph_data_path",     fp_->scene_graph_data_path_, std::string(""));
  nh.param("fsm/scene_graph_load_name",     fp_->scene_graph_load_name_, std::string(""));
  nh.param("fsm/auto_init_delay_sec",        fp_->auto_init_delay_sec_, 2.0);
  nh.param("fsm/scene_graph_init_forward_dist", fp_->scene_graph_init_forward_dist_, 1.8);
  nh.param("fsm/enable_yaw_scan", enable_yaw_scan_, false);
  nh.param("fsm/enable_scene_graph_update_after_load", enable_scene_graph_update_after_load_, true);
  double panorama_max_step_deg = 120.0;
  double panorama_extend_angle_deg = 40.0;
  nh.param("fsm/panorama_max_step", panorama_max_step_deg, 120.0);
  nh.param("fsm/panorama_extend_angle", panorama_extend_angle_deg, 40.0);
  panorama_max_step_ = std::max(1.0, panorama_max_step_deg) * M_PI / 180.0;
  panorama_extend_angle_ = std::max(0.0, panorama_extend_angle_deg) * M_PI / 180.0;
  if (panorama_extend_angle_ >= panorama_max_step_)
  {
    ROS_WARN("[Panorama] fsm/panorama_extend_angle must be smaller than panorama_max_step, clamp it.");
    panorama_extend_angle_ = 0.5 * panorama_max_step_;
  }
  nh.param("tracking/finish_hold_time",       fp_->track_finish_hold_time_, 3.0);
  nh.param("tracking/finish_move_thresh",     fp_->track_finish_move_thresh_, 0.2);
  nh.param("tracking/finish_yaw_thresh",      fp_->track_finish_yaw_thresh_, 0.2);
  nh.param("tracking/backend",                 fp_->tracking_backend_, std::string("ego"));
  nh.param("tracking/target_odom_topic",       fp_->tracking_target_odom_topic_, std::string("/target_ekf_odom"));
  nh.param("tracking/aim_dist",                fp_->track_aim_dist_, 1.5);
  nh.param("tracking/replan_dist",             fp_->track_replan_dist_, 0.8);
  nh.param("tracking/yaw_thr",                 fp_->track_yaw_thr_, 0.2);
  nh.param("tracking/turn_yaw_dist",           fp_->track_turn_yaw_dist_, 1.5);
  nh.param("tracking/fly_yaw_thr",             fp_->track_fly_yaw_thr_, 0.5);
  nh.param("tracking/detect_error",            fp_->track_detect_error_, 0.5);
  nh.param("tracking/radius_close",            fp_->radius_close_, 0.8);
  nh.param("planner_cmd_mux/input_timeout",    planner_cmd_mux_input_timeout_, 0.5);
  nh.param("planner_cmd_mux/ego_mode",         fp_->planner_cmd_mux_ego_mode_, std::string("ego"));
  // 卡死强制推进参数
  nh.param("topo_block/stuck_force_advance_enable",          fp_->stuck_force_advance_enable_, true);
  nh.param("topo_block/stuck_force_advance_vel_thresh",      fp_->stuck_force_advance_vel_thresh_, 0.1);
  nh.param("topo_block/stuck_force_advance_yaw_rate_thresh", fp_->stuck_force_advance_yaw_rate_thresh_, 0.1);
  nh.param("topo_block/stuck_force_advance_duration",        fp_->stuck_force_advance_duration_, 3.0);
  nh.param("topo_block/stuck_force_advance_max_consecutive", fp_->stuck_force_advance_max_consecutive_, 2);
  // object-id-nav replan 参数
  nh.param("object_id_nav_replan/enable",               fp_->object_id_nav_replan_enable_, false);
  nh.param("object_id_nav_replan/mode",                 fp_->object_id_nav_replan_mode_, 0);
  nh.param("object_id_nav_replan/stuck_vel_thresh",     fp_->object_id_nav_replan_stuck_vel_thresh_, 0.1);
  nh.param("object_id_nav_replan/stuck_yaw_rate_thresh",fp_->object_id_nav_replan_stuck_yaw_rate_thresh_, 0.1);
  nh.param("object_id_nav_replan/stuck_duration",       fp_->object_id_nav_replan_stuck_duration_, 3.0);
  nh.param("object_id_nav_replan/stuck_max_consecutive", fp_->object_id_nav_replan_stuck_max_consecutive_, 0);
  nh.param("object_id_nav_replan/mode2_stuck_fallback_delay", fp_->object_id_nav_replan_mode2_stuck_fallback_delay_, 10.0);
  nh.param("object_id_nav/require_final_yaw",             fp_->object_id_nav_require_final_yaw_, true);
  nh.param("object_id_nav/autostart_enable", object_id_nav_autostart_enable_, false);
  nh.param("object_id_nav/autostart_target_id", object_id_nav_autostart_target_id_, 2);
  nh.param("object_id_nav/autostart_delay_sec", object_id_nav_autostart_delay_sec_, 1.0);
  object_id_nav_autostart_delay_sec_ = std::max(0.0, object_id_nav_autostart_delay_sec_);
  nh.param("vla_search/enable",                  vla_search_enabled_, false);
  nh.param("vla_search/result_topic",            vla_search_result_topic_, std::string("/planning/vla_search_result"));
  nh.param("vla_search/bbox_topic",              vla_search_bbox_topic_, std::string("/vla_search/bbox"));
  nh.param("vla_search/target_topic",            vla_search_target_topic_, std::string("/vla_search/target"));
  nh.param(
      "vla_search/camera_topic", vla_search_camera_topic_,
      std::string("/drone_") + std::to_string(md_->drone_id_) +
          "/camera/color/image/compressed");
  nh.param(
      "vla_search/observation_topic", vla_search_observation_topic_,
      std::string("/vla_search/observation"));
  nh.param("vla_search/prompt_timeout",          vla_search_prompt_timeout_, 20.0);
  nh.param("vla_search/target_timeout",          vla_search_target_timeout_, 10.0);
  nh.param("vla_search/ego_plan_timeout",        vla_search_ego_plan_timeout_, 5.0);
  nh.param("vla_search/ego_exec_timeout",        vla_search_ego_exec_timeout_, 30.0);
  nh.param("vla_search/max_plan_retries",        vla_search_max_plan_retries_, 2);
  nh.param("vla_search/max_target_retries",      vla_search_max_target_retries_, 2);
  nh.param("vla_search/max_exploration_rounds",  vla_search_max_exploration_rounds_, 6);
  nh.param("vla_search/enable_room_description", vla_search_enable_room_description_, false);
  nh.param("vla_search/waypoint_distance",       vla_search_waypoint_distance_, 2.0);
  nh.param("vla_search/goal_tolerance",          vla_search_goal_tolerance_, 0.5);
  nh.param("vla_search/flight_height",           vla_search_flight_height_, 1.0);
  nh.param("vla_search/map_update_period",        vla_search_map_update_period_, 1.0);
  nh.param(
      "vla_search/scan_yaw_tolerance", vla_search_scan_yaw_tolerance_, 0.08);
  nh.param(
      "vla_search/scan_settle_time", vla_search_scan_settle_time_, 0.4);
  nh.param(
      "vla_search/scan_timeout", vla_search_scan_timeout_, 8.0);
  // 优先使用显式角度列表；若未设置则根据扫描步长自动生成。
  std::vector<double> scan_yaw_offsets_deg;
  nh.getParam("vla_search/scan_yaw_offsets_deg", scan_yaw_offsets_deg);
  nh.param("vla_search/scan_yaw_step_deg", vla_search_scan_yaw_step_deg_, 90.0);
  vla_search_scan_yaw_step_deg_ = std::max(1.0, std::min(180.0, vla_search_scan_yaw_step_deg_));

  if (scan_yaw_offsets_deg.empty()) {
    // 先直行方向(0)，左转一次(-step)，然后持续右转累加直至覆盖近360°，最后归零。
    scan_yaw_offsets_deg.push_back(0.0);
    scan_yaw_offsets_deg.push_back(-vla_search_scan_yaw_step_deg_);
    double angle = vla_search_scan_yaw_step_deg_;
    while (angle < 360.0 - 1e-6) {
      scan_yaw_offsets_deg.push_back(angle);
      angle += vla_search_scan_yaw_step_deg_;
    }
    scan_yaw_offsets_deg.push_back(0.0);
  }

  vla_search_scan_yaw_offsets_.clear();
  for (const double offset_deg : scan_yaw_offsets_deg) {
    vla_search_scan_yaw_offsets_.push_back(offset_deg * M_PI / 180.0);
  }
  if (vla_search_scan_yaw_offsets_.empty()) {
    vla_search_scan_yaw_offsets_.push_back(0.0);
  } else if (vla_search_scan_yaw_offsets_.size() > 24) {
    ROS_WARN(
        "[VLA_SEARCH] scan_yaw_offsets_deg supports at most 24 observations; "
        "extra entries are ignored.");
    vla_search_scan_yaw_offsets_.resize(24);
  }
  vla_search_waypoint_distance_ = std::max(0.2, vla_search_waypoint_distance_);
  vla_search_goal_tolerance_ = std::max(0.1, vla_search_goal_tolerance_);
  vla_search_map_update_period_ = std::max(0.2, vla_search_map_update_period_);
  vla_search_scan_yaw_tolerance_ =
      std::max(0.01, vla_search_scan_yaw_tolerance_);
  vla_search_scan_settle_time_ =
      std::max(0.0, vla_search_scan_settle_time_);
  vla_search_scan_timeout_ = std::max(1.0, vla_search_scan_timeout_);

  std::cout << "\n***** Target Cmd : " << fd_->target_cmd_ << "\n" << std::endl;
  std::cout << "ALL Main FSM Params loaded successfully ..." << std::endl;


  fd_->home_pos_ << 0.0, 0.0, 1.0; // TODO
  fd_->ego_exec_finished_ = true;

  /* Initialize main modules */
  map_ = map;
  scene_graph_        = std::make_shared<SceneGraph>(nh, map_);
  vla_search_map_      = std::make_shared<VLASearchMap>(nh, map_);
  counting_scene_graph_ = std::make_shared<CountingSceneGraph>(nh);
  traj_visualizer_    = std::make_shared<TrajectoryVisualizer>(nh);

  scene_graph_->setTargetAndPriorKnowledge(fd_->target_cmd_, fd_->prior_knowledge_);

  md_->mission_state_ = MISSION_FSM_STATE::INIT;
  md_->is_leader_     = false;
  md_->is_follower_   = false;

  md_->state_str_[MISSION_FSM_STATE::INIT]              = "INIT";
  md_->state_str_[MISSION_FSM_STATE::WARM_UP]           = "WARM_UP";
  md_->state_str_[MISSION_FSM_STATE::WAIT_TRIGGER]      = "WAIT_TRIGGER";
  md_->state_str_[MISSION_FSM_STATE::PLAN_EXPLORE]      = "PLAN_REGULAR_EXPLORE";
  md_->state_str_[MISSION_FSM_STATE::LLM_PLAN_EXPLORE]  = "PLAN_LLM_EXPLORE";
  md_->state_str_[MISSION_FSM_STATE::PLAN_TRACK]        = "PLAN_TRACK";
  md_->state_str_[MISSION_FSM_STATE::APPROACH_TRACK]    = "APPROACH_TRACK";
  md_->state_str_[MISSION_FSM_STATE::THINKING]          = "THINKING";
  md_->state_str_[MISSION_FSM_STATE::YAW_HANDLE]        = "YAW_HANDLE";
  md_->state_str_[MISSION_FSM_STATE::APPROACH_EXPLORE]  = "APPROACH_EXPLORE";
  md_->state_str_[MISSION_FSM_STATE::STOP]              = "STOP";
  md_->state_str_[MISSION_FSM_STATE::UNKONWN]           = "UNKNOWN";
  md_->state_str_[MISSION_FSM_STATE::GO_TARGET_OBJECT]  = "GO_TARGET_OBJECT";
  md_->state_str_[MISSION_FSM_STATE::GO_TARGET_WITH_WAYPOINT] = "GO_TARGET_WITH_WAYPOINT";
  md_->state_str_[MISSION_FSM_STATE::FIND_TERMINATE_TARGET] = "FIND_TERMINATE_TARGET";
  md_->state_str_[MISSION_FSM_STATE::FINISH]            = "FINISH";
  md_->state_str_[MISSION_FSM_STATE::DF_DEMO]           = "DF_DEMO";
  md_->state_str_[MISSION_FSM_STATE::VLA_SEARCH_PLAN_LOCAL] = "VLA_SEARCH_PLAN_LOCAL";
  md_->state_str_[MISSION_FSM_STATE::VLA_SEARCH_WAIT_LLM] = "VLA_SEARCH_WAIT_LLM";
  md_->state_str_[MISSION_FSM_STATE::VLA_SEARCH_WAIT_TARGET] = "VLA_SEARCH_WAIT_TARGET";
  md_->state_str_[MISSION_FSM_STATE::VLA_SEARCH_APPROACH] = "VLA_SEARCH_APPROACH";
  md_->state_str_[MISSION_FSM_STATE::VLA_SEARCH_YAW_HANDLE] = "VLA_SEARCH_YAW_HANDLE";
  md_->state_str_[MISSION_FSM_STATE::VLA_SEARCH_RECOVERY] = "VLA_SEARCH_RECOVERY";
  md_->state_str_[MISSION_FSM_STATE::VLA_SEARCH_FINISH] = "VLA_SEARCH_FINISH";

  /* Initialize FSM data */
  fd_->have_odom_    = false;
  fd_->odom_pos_.setZero();
  fd_->odom_vel_.setZero();
  fd_->odom_yaw_ = 0.0;
  fd_->static_state_ = true;
  fd_->trigger_      = false;
  fd_->track_trigger_ = false;
  fd_->track_init_ = false;
  fd_->track_pos_.setZero();
  resetTrackingFinishCandidate();
  fd_->track_finish_sent_ = false;
  fd_->waypoint_target_.setZero();
  fd_->waypoint_target_yaw_ = 0.0;
  fd_->goal_replan_times_ = 0;
  fd_->warmup_start_time_ = ros::Time(0);
  object_id_nav_autostart_triggered_ = false;
  object_id_nav_autostart_ready_time_ = ros::Time(0);

  /* Ros sub, pub and timer */
  exec_timer_      = nh.createTimer(ros::Duration(0.1), &MissionFSM::FSMCallback, this);
  vla_search_map_timer_ = nh.createTimer(
      ros::Duration(vla_search_map_update_period_),
      &MissionFSM::vlaSearchMapCallback, this);

  // vis_timer_       = nh.createTimer(ros::Duration(0.2), &MissionFSM::visualize, this); // [gwq] has thread problem! Don't turn on!

  instruction_sub_ = nh.subscribe("/bridge/Instruct", 10, &MissionFSM::instructionCallback, this, ros::TransportHints().tcpNoDelay());
  odom_sub_        = nh.subscribe("odom_world",  1, &MissionFSM::odometryCallback, this, ros::TransportHints().tcpNoDelay());
  battery_sub_     = nh.subscribe("/mavros/battery", 10, &MissionFSM::batteryCallBack, this, ros::TransportHints().tcpNoDelay());
  ego_plan_res_sub_= nh.subscribe("/planning/ego_plan_result", 100, &MissionFSM::egoPlanResCallback, this, ros::TransportHints().tcpNoDelay());
  trigger_sub_     = nh.subscribe("/move_base_simple/goal", 2, &MissionFSM::triggerCallback, this, ros::TransportHints().tcpNoDelay());
  ego_exec_finish_sub_ = nh.subscribe("exec_finish_trigger", 10, &MissionFSM::egoExecFinishCallback, this, ros::TransportHints().tcpNoDelay());
  wp_progress_sub_ = nh.subscribe("/drone_0_ego_planner_node/waypoint_progress", 10, &MissionFSM::waypointProgressCallback, this, ros::TransportHints().tcpNoDelay());
  track_command_sub_ = nh.subscribe("/planning/track_command", 2, &MissionFSM::trackCommandCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  target_sub_ = nh.subscribe("/tracking_target", 2, &MissionFSM::targetCallbackReal, this,
                             ros::TransportHints().tcpNoDelay());
  emergency_stop_sub_ = nh.subscribe("/command/emergency_stop", 10,
                                     &MissionFSM::emergencyStopCallback, this,
                                     ros::TransportHints().tcpNoDelay());
  vla_search_target_sub_ = nh.subscribe(
      vla_search_target_topic_, 10,
      &MissionFSM::vlaSearchTargetCallback, this,
      ros::TransportHints().tcpNoDelay());
  vla_search_camera_sub_ = nh.subscribe(
      vla_search_camera_topic_, 2,
      &MissionFSM::vlaSearchCameraCallback, this,
      ros::TransportHints().tcpNoDelay());
  vla_search_ego_state_trigger_sub_ = nh.subscribe(
      "/planning/ego_state_trigger", 10,
      &MissionFSM::vlaSearchEgoStateTriggerCallback, this,
      ros::TransportHints().tcpNoDelay());
  object_id_nav_replan_sub_ = nh.subscribe("/object_id_nav_replan", 10,
      &MissionFSM::objectIdNavReplanCallback, this,
      ros::TransportHints().tcpNoDelay());

  ego_goal_pub_         = nh.advertise<quadrotor_msgs::LocalGoalSet>("local_goal", 10);

  vis_marker_pub_       = nh.advertise<visualization_msgs::Marker>("planning/fsm_vis", 10);
  vis_path_pub_         = nh.advertise<visualization_msgs::MarkerArray>("planning/fsm_path", 10);
  perception_data_pub_  = nh.advertise<quadrotor_msgs::PerceptionMsg>("/perception_data_to_bridge", 10);
  instruction_resp_pub_ = nh.advertise<quadrotor_msgs::InstructionResMsg>("/Instruct_res", 10);

  fsm_state_pub_        = nh.advertise<std_msgs::String>("/planner/fsm_state", 10);
  tracking_finish_pub_  = nh.advertise<std_msgs::Bool>("/tracking_finish", 10);
  tracking_target_odom_pub_ = nh.advertise<nav_msgs::Odometry>(fp_->tracking_target_odom_topic_, 10);
  position_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 20);
  ego_position_cmd_sub_ = nh.subscribe("ego_position_cmd", 20, &MissionFSM::egoPositionCmdCallback, this,
                                       ros::TransportHints().tcpNoDelay());
  vla_search_result_pub_ = nh.advertise<std_msgs::String>(vla_search_result_topic_, 10);
  vla_search_bbox_pub_ =
      nh.advertise<quadrotor_msgs::VLASearchBBox>(vla_search_bbox_topic_, 10);
  vla_search_observation_pub_ =
      nh.advertise<scene_graph::VLASearchObservation>(
          vla_search_observation_topic_, 10);

  switchPlannerCmdMuxToEgo("fsm_init");
}



bool MissionFSM::isVlaSearchState(MISSION_FSM_STATE state) const
{
  return state == MISSION_FSM_STATE::VLA_SEARCH_PLAN_LOCAL ||
         state == MISSION_FSM_STATE::VLA_SEARCH_WAIT_LLM ||
         state == MISSION_FSM_STATE::VLA_SEARCH_WAIT_TARGET ||
         state == MISSION_FSM_STATE::VLA_SEARCH_APPROACH ||
         state == MISSION_FSM_STATE::VLA_SEARCH_YAW_HANDLE ||
         state == MISSION_FSM_STATE::VLA_SEARCH_RECOVERY ||
         state == MISSION_FSM_STATE::VLA_SEARCH_FINISH;
}

void MissionFSM::resetVlaSearchContext()
{
  if (vla_search_prompt_pending_ && scene_graph_ != nullptr) {
    scene_graph_->clearPromptData(vla_search_prompt_id_);
  }
  vla_search_active_ = false;
  vla_search_result_published_ = false;
  vla_search_success_ = false;
  vla_search_session_id_ = 0;
  vla_search_command_.clear();
  vla_search_finish_reason_.clear();
  vla_search_finish_detail_.clear();
  vla_search_prompt_pending_ = false;
  vla_search_place_checked_ = false;
  vla_search_explore_area_id_ = -1;
  vla_search_prompt_id_ = 0;
  vla_search_prompt_type_ = 0;
  vla_search_observation_batch_id_ = 0;
  vla_search_target_request_id_ = 0;
  vla_search_prompt_start_time_ = ros::Time();
  vla_search_target_start_time_ = ros::Time();
  vla_search_observation_stamp_ = ros::Time();
  vla_search_scan_index_ = 0;
  vla_search_scan_base_yaw_ = 0.0;
  vla_search_scan_target_yaw_ = 0.0;
  vla_search_scan_hold_position_.setZero();
  vla_search_scan_command_time_ = ros::Time();
  vla_search_scan_yaw_reached_time_ = ros::Time();
  vla_search_scan_initialized_ = false;
  vla_search_scan_command_published_ = false;
  vla_search_target_pending_ = false;
  vla_search_target_received_ = false;
  vla_search_target_success_ = false;
  vla_search_target_observation_index_ = 0;
  vla_search_target_source_ = quadrotor_msgs::VLASearchTarget::SOURCE_UNKNOWN;
  vla_search_target_position_.setZero();
  vla_search_target_error_.clear();
  vla_search_path_.clear();
  vla_search_waypoint_publish_time_ = ros::Time();
  vla_search_path_reaches_task_target_ = false;
  vla_search_waypoint_published_ = false;
  vla_search_waypoint_is_final_ = false;
  vla_search_plan_feedback_received_ = false;
  vla_search_plan_feedback_success_ = false;
  vla_search_waypoint_retry_count_ = 0;
  vla_search_ego_stable_ = false;
  vla_search_exploration_round_ = 0;
  vla_search_aa_done_ = false;
  vla_search_key_action_history_.clear();
  // room_descriptions_ 不在此清理 —— 同一 session 内跨轮复用
}

void MissionFSM::publishVlaSearchResult(bool success, const std::string& reason,
                                               const std::string& detail)
{
  if (!vla_search_active_ || vla_search_result_published_) {
    return;
  }

  std_msgs::String msg;
  std::ostringstream ss;
  ss << "{"
     << "\"task_session_id\":" << vla_search_session_id_ << ","
     << "\"finished\":true,"
     << "\"success\":" << (success ? "true" : "false") << ","
     << "\"reason\":\"" << jsonEscape(reason) << "\","
     << "\"detail\":\"" << jsonEscape(detail) << "\","
     << "\"command\":\"" << jsonEscape(vla_search_command_) << "\","
     << "\"state\":\"" << md_->state_str_[md_->mission_state_] << "\""
     << "}";
  msg.data = ss.str();
  vla_search_result_pub_.publish(msg);
  vla_search_result_published_ = true;
}

void MissionFSM::startVlaSearchTask(const quadrotor_msgs::InstructionConstPtr& msg)
{
  resetVlaSearchContext();
  vla_search_room_descriptions_.clear();
  vla_search_active_ = true;
  vla_search_session_id_ = msg->task_session_id;
  vla_search_observation_batch_id_ = 1;
  vla_search_command_ = msg->command;
  active_instruction_task_id_ = msg->source_task_id;
  active_instruction_session_id_ = msg->task_session_id;
  fd_->target_cmd_ = msg->command;

  switchPlannerCmdMuxToEgo("startVlaSearchTask");
  stopMotion();
  transitState(MISSION_FSM_STATE::VLA_SEARCH_PLAN_LOCAL, "startVlaSearchTask");
}

bool MissionFSM::startVlaSearchTargetRequest(
    const nlohmann::json& payload)
{
  if (!payload.contains("bounding_box") ||
      !payload["bounding_box"].is_array() ||
      payload["bounding_box"].size() != 4) {
    vla_search_finish_detail_ =
        "Visual target prompt requires bounding_box=[x0,y0,x1,y1]";
    return false;
  }

  int observation_index = 0;
  switch (vla_search_prompt_type_) {
    case scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A1:
    case scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B1:
      observation_index = 1;
      break;
    case scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A2:
    case scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B2:
      observation_index = 2;
      break;
    case scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A3:
    case scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B3:
      observation_index = 3;
      break;
    default:
      if (payload.contains("observation_index") &&
          payload["observation_index"].is_number_integer()) {
        observation_index = payload["observation_index"].get<int>();
      }
      break;
  }
  if (observation_index < 0 || observation_index > 3) {
    vla_search_finish_detail_ = "observation_index must be in [0,3]";
    return false;
  }

  quadrotor_msgs::VLASearchBBox request;
  request.header.stamp =
      vla_search_observation_stamp_.isZero()
          ? vla_search_prompt_start_time_
          : vla_search_observation_stamp_;
  request.header.frame_id = "world";
  request.task_session_id = vla_search_session_id_;
  request.observation_batch_id = vla_search_observation_batch_id_;
  request.request_id = ++vla_search_target_request_id_;
  request.observation_index = static_cast<uint8_t>(observation_index);
  try {
    for (size_t index = 0; index < 4; ++index) {
      request.bbox_xyxy[index] =
          payload["bounding_box"][index].get<int>();
    }
  } catch (const std::exception& error) {
    vla_search_finish_detail_ = error.what();
    return false;
  }
  if (request.bbox_xyxy[2] <= request.bbox_xyxy[0] ||
      request.bbox_xyxy[3] <= request.bbox_xyxy[1]) {
    vla_search_finish_detail_ = "bounding_box has non-positive width or height";
    return false;
  }

  vla_search_target_observation_index_ = request.observation_index;
  vla_search_target_pending_ = true;
  vla_search_target_received_ = false;
  vla_search_target_success_ = false;
  vla_search_target_error_.clear();
  vla_search_target_start_time_ = ros::Time::now();
  vla_search_bbox_pub_.publish(request);
  transitState(
      MISSION_FSM_STATE::VLA_SEARCH_WAIT_TARGET,
      "VLA_Search bbox target request sent");
  return true;
}

bool MissionFSM::prepareVlaSearchPath(
    const Eigen::Vector3d& requested_goal, bool reaches_task_target,
    int door_id)
{
  if (!requested_goal.allFinite()) {
    vla_search_finish_reason_ = "invalid_navigation_goal";
    vla_search_finish_detail_ = "Navigation goal contains non-finite values";
    return false;
  }

  Eigen::Vector3d navigation_goal = requested_goal;
  std::vector<Eigen::Vector3d> raw_path;
  if (door_id >= 0) {
    navigation_goal.z() = vla_search_flight_height_;
    if (vla_search_map_ == nullptr ||
        !vla_search_map_->planDoorPath(
            fd_->odom_pos_, door_id, navigation_goal.z(),
            vla_search_waypoint_distance_, raw_path)) {
      vla_search_finish_reason_ = "small_map_path_unreachable";
      vla_search_finish_detail_ =
          "SmallMap A* cannot reach door id=" + std::to_string(door_id);
      return false;
    }

    // SmallMap 负责生成二维门路径，现有三维 A* 负责确认终点确实可由飞行空间到达。
    std::vector<Eigen::Vector3d> verification_path;
    if (!map_->searchPath(
            fd_->odom_pos_, navigation_goal, verification_path, 0.2)) {
      vla_search_finish_reason_ = "door_path_unreachable_3d";
      vla_search_finish_detail_ =
          "3D occupancy map rejects door id=" + std::to_string(door_id);
      return false;
    }
  } else {
    // 视觉或场景图目标可能位于物体占据栅格内，沿机器人方向寻找最近可执行停靠点。
    Eigen::Vector3d direction_to_robot = Eigen::Vector3d::UnitX();
    if ((fd_->odom_pos_ - requested_goal).norm() > 1e-6) {
      direction_to_robot =
          (fd_->odom_pos_ - requested_goal).normalized();
    }
    bool goal_is_free = false;
    for (double offset = 0.0; offset <= 2.0; offset += 0.2) {
      Eigen::Vector3d candidate =
          requested_goal + direction_to_robot * offset;
      if (map_->getInflateOccupancy(candidate) != global_belief::MapInterface::OCCUPIED &&
          map_->getOccupancy(candidate) != global_belief::MapInterface::UNKNOWN) {
        navigation_goal = candidate;
        goal_is_free = true;
        break;
      }
    }
    if (!goal_is_free ||
        !map_->searchPath(
            fd_->odom_pos_, navigation_goal, raw_path, 0.2)) {
      vla_search_finish_reason_ = "target_path_unreachable";
      vla_search_finish_detail_ =
          "No collision-free 3D path exists for the selected target";
      return false;
    }
  }

  if (raw_path.size() < 2) {
    vla_search_finish_reason_ = "path_generation_failed";
    vla_search_finish_detail_ =
        "Path generator returned fewer than two points";
    return false;
  }

  // 对三维 A* 的密集输出再次按配置距离采样；SmallMap 路径已采样，但也通过
  // 同一逻辑确保不同来源的 waypoint 间距一致。
  std::vector<Eigen::Vector3d> sampled_path;
  sampled_path.push_back(raw_path.front());
  double distance_since_last_sample = 0.0;
  for (size_t index = 1; index < raw_path.size(); ++index) {
    Eigen::Vector3d segment_start = raw_path[index - 1];
    const Eigen::Vector3d segment_end = raw_path[index];
    Eigen::Vector3d segment = segment_end - segment_start;
    double segment_length = segment.norm();
    while (segment_length > 1e-6 &&
           distance_since_last_sample + segment_length >=
               vla_search_waypoint_distance_) {
      const double step =
          vla_search_waypoint_distance_ - distance_since_last_sample;
      segment_start += segment * (step / segment_length);
      sampled_path.push_back(segment_start);
      segment = segment_end - segment_start;
      segment_length = segment.norm();
      distance_since_last_sample = 0.0;
    }
    distance_since_last_sample += segment_length;
  }
  if ((sampled_path.back() - raw_path.back()).norm() > 1e-3) {
    sampled_path.push_back(raw_path.back());
  }

  vla_search_path_ = std::move(sampled_path);
  vla_search_path_reaches_task_target_ = reaches_task_target;
  vla_search_waypoint_published_ = false;
  vla_search_waypoint_is_final_ = false;
  vla_search_plan_feedback_received_ = false;
  vla_search_plan_feedback_success_ = false;
  vla_search_waypoint_retry_count_ = 0;
  fd_->path_inx_ = 0;
  vla_search_finish_reason_.clear();
  vla_search_finish_detail_.clear();
  transitState(
      MISSION_FSM_STATE::VLA_SEARCH_APPROACH,
      door_id >= 0 ? "VLA_Search door path ready"
                   : "VLA_Search target path ready");
  return true;
}

bool MissionFSM::publishNextVlaSearchWaypoint()
{
  if (vla_search_path_.size() < 2) {
    return false;
  }

  vla_search_plan_feedback_received_ = false;
  vla_search_plan_feedback_success_ = false;
  vla_search_waypoint_retry_count_ = 0;
  if (!getAndPublishNextAim(vla_search_path_, true, fd_->odom_yaw_)) {
    return false;
  }
  vla_search_waypoint_published_ = true;
  vla_search_waypoint_is_final_ =
      vla_search_path_.size() <= 2 ||
      fd_->path_inx_ >= static_cast<int>(vla_search_path_.size()) - 1;
  vla_search_waypoint_publish_time_ = ros::Time::now();
  return true;
}

void MissionFSM::retryVlaSearchWaypoint(
    const std::string& failure_reason)
{
  if (vla_search_waypoint_retry_count_ >=
      std::max(0, vla_search_max_plan_retries_)) {
    vla_search_finish_reason_ = failure_reason;
    vla_search_finish_detail_ =
        "EGO failed waypoint after " +
        std::to_string(vla_search_waypoint_retry_count_) + " retries";
    transitState(
        MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
        "VLA_Search EGO retry exhausted");
    return;
  }

  ++vla_search_waypoint_retry_count_;
  vla_search_plan_feedback_received_ = false;
  vla_search_plan_feedback_success_ = false;
  pubLocalGoal(fd_->local_aim_pos_, fd_->odom_yaw_, true);
  vla_search_waypoint_publish_time_ = ros::Time::now();
  ROS_WARN_STREAM(
      "[VLA_SEARCH] Retry waypoint " << vla_search_waypoint_retry_count_
      << "/" << vla_search_max_plan_retries_
      << ", reason=" << failure_reason);
}

void MissionFSM::cancelVlaSearchTask(const std::string& reason, const std::string& detail)
{
  if (!vla_search_active_) {
    return;
  }
  stopMotion();
  vla_search_success_ = false;
  vla_search_finish_reason_ = reason;
  vla_search_finish_detail_ = detail;
  transitState(MISSION_FSM_STATE::VLA_SEARCH_FINISH, "cancelVlaSearchTask");
  handleVlaSearchFinish();
}

void MissionFSM::handleVlaSearchPlanLocal()
{
  if (!vla_search_active_) {
    transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "VLA_Search inactive");
    return;
  }

  if (vla_search_prompt_pending_) {
    transitState(MISSION_FSM_STATE::VLA_SEARCH_WAIT_LLM, "VLA_Search prompt already pending");
    return;
  }

  // AA 阶段：全局评估，参照原始 VLA_Swarm 的 AA→A→B→C→TASK_OVER 链路。
  // 在每轮 PLACE 之前询问 LLM 是否继续探索、有无新发现。
  if (!vla_search_aa_done_) {
    nlohmann::json aa_context;
    aa_context["exploration_round"] = vla_search_exploration_round_;
    aa_context["key_action_history"] = vla_search_key_action_history_;
    aa_context["room_descriptions"] = vla_search_room_descriptions_;

    vla_search_prompt_type_ =
        scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_AA;
    vla_search_prompt_id_ = scene_graph_->getCurPromptIdAndPlusOne();
    std::string prompt;
    if (!scene_graph_->vlaSearchPromptGen(
            vla_search_prompt_type_, vla_search_command_,
            vla_search_session_id_, vla_search_observation_batch_id_,
            aa_context, prompt)) {
      vla_search_finish_reason_ = "aa_prompt_generation_failed";
      vla_search_finish_detail_ = "Failed to generate AA prompt";
      transitState(MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
                   "VLA_Search AA prompt generation failed");
      return;
    }
    const int aa_timeout =
        std::max(1, static_cast<int>(std::ceil(vla_search_prompt_timeout_)));
    scene_graph_->sendPrompt(vla_search_prompt_id_, vla_search_prompt_type_,
                             prompt, std::chrono::seconds(aa_timeout), 1);
    vla_search_prompt_pending_ = true;
    vla_search_prompt_start_time_ = ros::Time::now();
    vla_search_aa_done_ = true;
    transitState(MISSION_FSM_STATE::VLA_SEARCH_WAIT_LLM,
                 "VLA_Search AA prompt sent");
    return;
  }

  if (vla_search_map_ == nullptr ||
      (!vla_search_map_->ready() && !vla_search_map_->update(fd_->odom_pos_))) {
    vla_search_finish_reason_ = "small_map_not_ready";
    vla_search_finish_detail_ = "SmallMap cannot be generated from the current occupancy map";
    transitState(MISSION_FSM_STATE::VLA_SEARCH_RECOVERY, "VLA_Search SmallMap unavailable");
    return;
  }

  nlohmann::json semantic_context =
      vla_search_map_->promptContext(*scene_graph_, fd_->odom_pos_);
  // 注入跨轮记忆与房间描述，传递给 PLACE/LOCAL_PLAN prompt。
  semantic_context["key_action_history"] = vla_search_key_action_history_;
  semantic_context["exploration_round"] = vla_search_exploration_round_;
  if (!vla_search_room_descriptions_.empty()) {
    semantic_context["room_descriptions"] = vla_search_room_descriptions_;
  }

  vla_search_prompt_type_ =
      vla_search_place_checked_
          ? scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION
          : scene_graph::PromptMsg::PROMPT_TYPE_PLACE_PREDICTION;
  if (vla_search_place_checked_ &&
      semantic_context["candidate_ids"].empty()) {
    vla_search_finish_reason_ = "no_exploration_candidate";
    vla_search_finish_detail_ = "SmallMap contains no valid door or frontier candidate";
    transitState(MISSION_FSM_STATE::VLA_SEARCH_RECOVERY, "VLA_Search has no map candidate");
    return;
  }

  vla_search_prompt_id_ = scene_graph_->getCurPromptIdAndPlusOne();
  std::string prompt;
  if (!scene_graph_->vlaSearchPromptGen(
          vla_search_prompt_type_,
          vla_search_command_,
          vla_search_session_id_,
          vla_search_observation_batch_id_,
          semantic_context,
          prompt)) {
    vla_search_finish_reason_ = "prompt_generation_failed";
    vla_search_finish_detail_ = "Failed to generate VLA_Swarm prompt from SmallMap context";
    transitState(MISSION_FSM_STATE::VLA_SEARCH_RECOVERY, "VLA_Search prompt generation failed");
    return;
  }

  const int timeout_seconds = std::max(1, static_cast<int>(std::ceil(vla_search_prompt_timeout_)));
  const int max_retries = std::max(1, vla_search_max_plan_retries_);
  scene_graph_->sendPrompt(
      vla_search_prompt_id_,
      vla_search_prompt_type_,
      prompt,
      std::chrono::seconds(timeout_seconds),
      max_retries);
  vla_search_prompt_pending_ = true;
  vla_search_prompt_start_time_ = ros::Time::now();
  vla_search_observation_stamp_ = vla_search_prompt_start_time_;
  transitState(
      MISSION_FSM_STATE::VLA_SEARCH_WAIT_LLM,
      vla_search_place_checked_
          ? "VLA_Search LOCAL_PLAN prompt sent"
          : "VLA_Search PLACE prompt sent");
}

void MissionFSM::handleVlaSearchWaitLLM()
{
  if (!vla_search_active_ || !vla_search_prompt_pending_) {
    vla_search_finish_reason_ = "prompt_state_invalid";
    vla_search_finish_detail_ = "VLA_SEARCH_WAIT_LLM has no active prompt";
    transitState(MISSION_FSM_STATE::VLA_SEARCH_RECOVERY, "VLA_Search missing active prompt");
    return;
  }

  if (!scene_graph_->hasPromptAnswer(vla_search_prompt_id_)) {
    const double retry_window =
        std::max(1.0, vla_search_prompt_timeout_) * std::max(1, vla_search_max_plan_retries_) + 1.0;
    if ((ros::Time::now() - vla_search_prompt_start_time_).toSec() <= retry_window) {
      return;
    }
    scene_graph_->clearPromptData(vla_search_prompt_id_);
    vla_search_prompt_pending_ = false;
    vla_search_finish_reason_ = "prompt_timeout";
    vla_search_finish_detail_ = "VLA_Search prompt did not return within the configured retry window";
    transitState(MISSION_FSM_STATE::VLA_SEARCH_RECOVERY, "VLA_Search prompt timeout");
    return;
  }

  const VLASearchPromptResult result =
      scene_graph_->parseVlaSearchPromptResult(vla_search_prompt_id_, vla_search_prompt_type_);
  scene_graph_->clearPromptData(vla_search_prompt_id_);
  vla_search_prompt_pending_ = false;

  if (!result.valid) {
    vla_search_finish_reason_ = result.error.empty() ? "invalid_prompt_result" : result.error;
    vla_search_finish_detail_ = result.detail;
    transitState(MISSION_FSM_STATE::VLA_SEARCH_RECOVERY, "VLA_Search invalid prompt JSON");
    return;
  }
  if (!result.success) {
    if (result.error == "observation_not_ready") {
      // 图像快照尚未被处理端接收时，留在当前方向重新固化一帧。
      vla_search_scan_command_published_ = false;
      transitState(
          MISSION_FSM_STATE::VLA_SEARCH_YAW_HANDLE,
          "VLA_Search observation cache not ready");
      return;
    }
    vla_search_finish_reason_ = result.error.empty() ? "prompt_error" : result.error;
    vla_search_finish_detail_ = result.detail;
    transitState(MISSION_FSM_STATE::VLA_SEARCH_RECOVERY, "VLA_Search prompt processor error");
    return;
  }

  auto parseIntegerField = [&](const char* field_name, int& value) {
    if (!result.payload.contains(field_name)) {
      vla_search_finish_detail_ =
          std::string("Prompt answer requires field: ") + field_name;
      return false;
    }
    try {
      const auto& field = result.payload[field_name];
      if (field.is_number_integer()) {
        value = field.get<int>();
        return true;
      }
      if (field.is_string()) {
        const std::string raw_value = field.get<std::string>();
        size_t consumed = 0;
        value = std::stoi(raw_value, &consumed);
        if (consumed == raw_value.size()) {
          return true;
        }
      }
    } catch (const std::exception& e) {
      vla_search_finish_detail_ = e.what();
      return false;
    }
    vla_search_finish_detail_ =
        std::string(field_name) + " must be an integer or integer string";
    return false;
  };

  // AA 阶段响应路由：LLM 基于全局历史判断是否继续探索。
  if (vla_search_prompt_type_ ==
      scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_AA) {
    bool aa_found = false;
    if (result.payload.contains("found") &&
        result.payload["found"].is_boolean()) {
      aa_found = result.payload["found"].get<bool>();
    }
    std::string aa_action = "continue";
    if (result.payload.contains("action") &&
        result.payload["action"].is_string()) {
      aa_action = result.payload["action"].get<std::string>();
    }

    if (aa_action == "stop") {
      vla_search_success_ = false;
      vla_search_finish_reason_ = "task_over_by_llm";
      vla_search_finish_detail_ = "AA stage: LLM decided exploration is complete";
      transitState(MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
                   "VLA_Search AA says stop");
      return;
    }

    if (aa_found) {
      nlohmann::json entry;
      entry["round"] = vla_search_exploration_round_;
      entry["action"] = "AA: found potential target, continuing exploration";
      vla_search_key_action_history_.push_back(entry);
    }

    transitState(MISSION_FSM_STATE::VLA_SEARCH_PLAN_LOCAL,
                 "VLA_Search AA complete, continue to PLACE");
    return;
  }

  if (vla_search_prompt_type_ ==
      scene_graph::PromptMsg::PROMPT_TYPE_PLACE_PREDICTION) {
    int action = 0;
    if (!parseIntegerField("action", action) ||
        (action != -1 && action != 1 && action != 2)) {
      vla_search_finish_reason_ = "invalid_prompt_schema";
      if (vla_search_finish_detail_.empty()) {
        vla_search_finish_detail_ = "PLACE action must be -1, 1 or 2";
      }
      transitState(MISSION_FSM_STATE::VLA_SEARCH_RECOVERY, "VLA_Search invalid PLACE result");
      return;
    }

    if (action == -1) {
      // 当前语义没有直接目标时，转入 SmallMap 视觉局部规划。
      vla_search_place_checked_ = true;
      transitState(MISSION_FSM_STATE::VLA_SEARCH_PLAN_LOCAL, "VLA_Search PLACE continues to LOCAL_PLAN");
      return;
    }

    int selected_id = -1;
    if (!parseIntegerField("id", selected_id)) {
      vla_search_finish_reason_ = "invalid_prompt_schema";
      transitState(MISSION_FSM_STATE::VLA_SEARCH_RECOVERY, "VLA_Search invalid PLACE id");
      return;
    }

    Eigen::Vector3d selected_goal = Eigen::Vector3d::Zero();
    bool reaches_task_target = false;
    bool selected_goal_found = false;
    if (action == 1) {
      for (const auto& room : vla_search_map_->rooms()) {
        if (room.id == selected_id) {
          selected_goal =
              Eigen::Vector3d(
                  room.center.x(), room.center.y(),
                  vla_search_flight_height_);
          selected_goal_found = true;
          break;
        }
      }
    } else if (scene_graph_->object_factory_ != nullptr) {
      const auto object_iterator =
          scene_graph_->object_factory_->object_map_.find(selected_id);
      if (object_iterator !=
              scene_graph_->object_factory_->object_map_.end() &&
          object_iterator->second != nullptr) {
        selected_goal = object_iterator->second->pos;
        reaches_task_target = true;
        selected_goal_found = true;
      }
    }
    if (!selected_goal_found) {
      vla_search_finish_reason_ = "place_target_not_found";
      vla_search_finish_detail_ =
          "PLACE selected unknown id=" + std::to_string(selected_id);
      transitState(
          MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
          "VLA_Search PLACE target missing");
      return;
    }
    if (!prepareVlaSearchPath(
            selected_goal, reaches_task_target)) {
      transitState(
          MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
          "VLA_Search PLACE target path failed");
    }
    return;
  }

  if (vla_search_prompt_type_ ==
      scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION) {
    int explore_area_id = -1;
    if (!parseIntegerField("explore_area_id", explore_area_id)) {
      vla_search_finish_reason_ = "invalid_prompt_schema";
      transitState(MISSION_FSM_STATE::VLA_SEARCH_RECOVERY, "VLA_Search invalid LOCAL_PLAN result");
      return;
    }

    bool candidate_exists = false;
    for (const auto& door : vla_search_map_->doors()) {
      if (door.id == explore_area_id) {
        candidate_exists = true;
        break;
      }
    }
    if (!candidate_exists) {
      vla_search_finish_reason_ = "invalid_exploration_candidate";
      vla_search_finish_detail_ =
          "LOCAL_PLAN explore_area_id is not present in the current SmallMap";
      transitState(MISSION_FSM_STATE::VLA_SEARCH_RECOVERY, "VLA_Search candidate not found");
      return;
    }

    vla_search_explore_area_id_ = explore_area_id;
    VLASearchDoor selected_door;
    if (!vla_search_map_->findDoor(
            vla_search_explore_area_id_, selected_door)) {
      vla_search_finish_reason_ = "invalid_exploration_candidate";
      vla_search_finish_detail_ =
          "Selected door disappeared before path generation";
      transitState(
          MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
          "VLA_Search selected door expired");
      return;
    }
    const Eigen::Vector3d door_goal(
        selected_door.position.x(), selected_door.position.y(),
        vla_search_flight_height_);
    if (!prepareVlaSearchPath(
            door_goal, false, vla_search_explore_area_id_)) {
      transitState(
          MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
          "VLA_Search door path failed");
    }
    return;
  }

  const bool is_visual_target_prompt =
      vla_search_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A ||
      vla_search_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A1 ||
      vla_search_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A2 ||
      vla_search_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A3 ||
      vla_search_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B ||
      vla_search_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B1 ||
      vla_search_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B2 ||
      vla_search_prompt_type_ ==
          scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B3;
  if (is_visual_target_prompt) {
    bool found = result.payload.contains("bounding_box");
    if (result.payload.contains("found") &&
        result.payload["found"].is_boolean()) {
      found = result.payload["found"].get<bool>();
    }
    if (!found) {
      ++vla_search_scan_index_;
      vla_search_scan_command_published_ = false;
      vla_search_scan_yaw_reached_time_ = ros::Time();
      transitState(
          MISSION_FSM_STATE::VLA_SEARCH_YAW_HANDLE,
          "VLA_Search target absent in current observation");
      return;
    }
    if (!startVlaSearchTargetRequest(result.payload)) {
      vla_search_finish_reason_ = "invalid_bbox_result";
      transitState(
          MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
          "VLA_Search invalid visual target result");
    }
    return;
  }

  vla_search_finish_reason_ = "invalid_prompt_type";
  vla_search_finish_detail_ = "Unexpected VLA_Swarm prompt type in WAIT_LLM";
  transitState(MISSION_FSM_STATE::VLA_SEARCH_RECOVERY, "VLA_Search unexpected prompt type");
}

void MissionFSM::handleVlaSearchWaitTarget()
{
  if (!vla_search_active_ || !vla_search_target_pending_) {
    vla_search_finish_reason_ = "target_state_invalid";
    vla_search_finish_detail_ = "VLA_SEARCH_WAIT_TARGET has no active bbox request";
    transitState(
        MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
        "VLA_Search target request missing");
    return;
  }

  if (!vla_search_target_received_) {
    if ((ros::Time::now() - vla_search_target_start_time_).toSec() <=
        std::max(1.0, vla_search_target_timeout_)) {
      return;
    }
    vla_search_target_pending_ = false;
    vla_search_finish_reason_ = "target_timeout";
    vla_search_finish_detail_ =
        "Neither LiDAR nor MoGe returned a target within target_timeout";
    transitState(
        MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
        "VLA_Search target localization timeout");
    return;
  }

  vla_search_target_pending_ = false;
  if (!vla_search_target_success_) {
    vla_search_finish_reason_ = "target_localization_failed";
    vla_search_finish_detail_ = vla_search_target_error_;
    transitState(
        MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
        "VLA_Search target localization failed");
    return;
  }

  if (!prepareVlaSearchPath(
          vla_search_target_position_, true)) {
    transitState(
        MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
        "VLA_Search localized target path failed");
  }
}

void MissionFSM::handleVlaSearchApproach()
{
  if (!vla_search_active_ || vla_search_path_.size() < 2) {
    vla_search_finish_reason_ = "approach_state_invalid";
    vla_search_finish_detail_ =
        "VLA_SEARCH_APPROACH has no active path";
    transitState(
        MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
        "VLA_Search approach path missing");
    return;
  }

  if (!vla_search_waypoint_published_) {
    // 等待 ego_state_trigger 确保无人机已停止运动后再发布导航指令。
    if (!vla_search_ego_stable_) {
      return;
    }
    vla_search_ego_stable_ = false;
    if (!publishNextVlaSearchWaypoint()) {
      vla_search_finish_reason_ = "waypoint_generation_failed";
      vla_search_finish_detail_ =
          "No valid local waypoint can be selected from the path";
      transitState(
          MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
          "VLA_Search waypoint unavailable");
    }
    return;
  }

  const double elapsed =
      (ros::Time::now() - vla_search_waypoint_publish_time_).toSec();
  if (!vla_search_plan_feedback_received_) {
    if (elapsed > std::max(0.5, vla_search_ego_plan_timeout_)) {
      retryVlaSearchWaypoint("ego_plan_timeout");
    }
    return;
  }
  if (!vla_search_plan_feedback_success_) {
    retryVlaSearchWaypoint("ego_plan_failed");
    return;
  }

  const double distance_to_waypoint =
      (fd_->odom_pos_ - fd_->local_aim_pos_).norm();
  const bool waypoint_reached =
      distance_to_waypoint <= vla_search_goal_tolerance_ ||
      (fd_->ego_exec_finished_ &&
       distance_to_waypoint <=
           std::max(0.75, 2.0 * vla_search_goal_tolerance_));
  if (!waypoint_reached) {
    if (elapsed > std::max(
                      vla_search_ego_plan_timeout_ + 0.5,
                      vla_search_ego_exec_timeout_)) {
      retryVlaSearchWaypoint("ego_execution_timeout");
    }
    return;
  }

  if (!vla_search_waypoint_is_final_) {
    vla_search_waypoint_published_ = false;
    if (!publishNextVlaSearchWaypoint()) {
      vla_search_finish_reason_ = "waypoint_advance_failed";
      vla_search_finish_detail_ =
          "Path ended before the final waypoint was reached";
      transitState(
          MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
          "VLA_Search waypoint advance failed");
    }
    return;
  }

  if (vla_search_path_reaches_task_target_) {
    vla_search_success_ = true;
    vla_search_finish_reason_ = "target_reached";
    vla_search_finish_detail_ =
        "EGO completed the VLA_Swarm target path";
    transitState(
        MISSION_FSM_STATE::VLA_SEARCH_FINISH,
        "VLA_Search target reached");
    return;
  }

  // 到达房间或门后，优先沿路径末端方向观察。对门路径而言，该方向通常指向门后新空间。
  vla_search_scan_base_yaw_ = fd_->odom_yaw_;
  if (vla_search_path_.size() >= 2) {
    const Eigen::Vector3d terminal_direction =
        vla_search_path_.back() -
        vla_search_path_[vla_search_path_.size() - 2];
    if (terminal_direction.head<2>().norm() > 1e-3) {
      vla_search_scan_base_yaw_ =
          std::atan2(terminal_direction.y(), terminal_direction.x());
    }
  }

  // 记录本轮到达的门/区域，作为 AA 阶段的跨轮记忆输入。
  {
    nlohmann::json entry;
    entry["round"] = vla_search_exploration_round_ + 1;
    entry["action"] =
        "Arrived at door " + std::to_string(vla_search_explore_area_id_);
    vla_search_key_action_history_.push_back(entry);
  }

  // 到达房间或门仅表示完成一轮局部探索，刷新批次并执行正面优先的分时观察。
  ++vla_search_observation_batch_id_;
  vla_search_place_checked_ = false;
  vla_search_path_.clear();
  vla_search_waypoint_published_ = false;
  vla_search_scan_initialized_ = false;
  transitState(
      MISSION_FSM_STATE::VLA_SEARCH_YAW_HANDLE,
      "VLA_Search exploration waypoint reached, start visual scan");
}

void MissionFSM::handleVlaSearchYaw()
{
  if (!vla_search_active_ || vla_search_scan_yaw_offsets_.empty()) {
    vla_search_finish_reason_ = "yaw_state_invalid";
    vla_search_finish_detail_ =
        "VLA_SEARCH_YAW_HANDLE has no active task or scan direction";
    transitState(
        MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
        "VLA_Search yaw state invalid");
    return;
  }

  if (!vla_search_scan_initialized_) {
    vla_search_scan_index_ = 0;
    vla_search_scan_command_published_ = false;
    vla_search_scan_yaw_reached_time_ = ros::Time();
    vla_search_scan_initialized_ = true;
  }

  if (vla_search_scan_index_ >= vla_search_scan_yaw_offsets_.size()) {
    // 当前门前所有方向均已扫描且未发现目标。
    vla_search_scan_initialized_ = false;
    vla_search_explore_area_id_ = -1;
    ++vla_search_exploration_round_;

    // 达到最大探索轮次时终止，交由上层 agent_run 决定是否重试。
    if (vla_search_exploration_round_ > vla_search_max_exploration_rounds_) {
      vla_search_success_ = false;
      vla_search_finish_reason_ = "max_exploration_rounds";
      vla_search_finish_detail_ =
          "Exceeded maximum exploration rounds (" +
          std::to_string(vla_search_max_exploration_rounds_) +
          ") without finding target";
      transitState(
          MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
          "VLA_Search max exploration rounds reached");
      return;
    }

    // 累积跨轮记忆，参照原始 VLA_Swarm 的 key_action_history 机制。
    {
      nlohmann::json entry;
      entry["round"] = vla_search_exploration_round_;
      entry["action"] =
          "Scanned " + std::to_string(vla_search_scan_yaw_offsets_.size()) +
          " directions at door " + std::to_string(vla_search_explore_area_id_) +
          ", no target found";
      vla_search_key_action_history_.push_back(entry);
    }
    // 重置 AA 标记，下一轮重新评估全局状态。
    vla_search_aa_done_ = false;

    transitState(
        MISSION_FSM_STATE::VLA_SEARCH_PLAN_LOCAL,
        "VLA_Search visual scan completed without target");
    return;
  }

  if (!vla_search_scan_command_published_) {
    // 等待 ego_state_trigger 确保无人机已停止运动后再发布旋转指令。
    if (!vla_search_ego_stable_) {
      return;
    }
    vla_search_ego_stable_ = false;
    // 在确定无人机停稳后记录悬停位置，避免捕获到惯性滑行中的偏移坐标。
    vla_search_scan_hold_position_ = fd_->odom_pos_;
    vla_search_scan_target_yaw_ = normalizeAngle(
        vla_search_scan_base_yaw_ +
        vla_search_scan_yaw_offsets_[vla_search_scan_index_]);
    pubLocalGoal(
        vla_search_scan_hold_position_, vla_search_scan_target_yaw_, false,
        quadrotor_msgs::LocalGoalSet::YAW_MODE_LOW_SPEED);
    vla_search_scan_command_time_ = ros::Time::now();
    vla_search_scan_yaw_reached_time_ = ros::Time();
    vla_search_scan_command_published_ = true;
    ROS_INFO_STREAM(
        "[VLA_SEARCH] Scan observation=" << vla_search_scan_index_
        << ", target_yaw=" << vla_search_scan_target_yaw_);
    return;
  }

  if ((ros::Time::now() - vla_search_scan_command_time_).toSec() >
      vla_search_scan_timeout_) {
    vla_search_finish_reason_ = "yaw_scan_timeout";
    vla_search_finish_detail_ =
        "Yaw did not reach the requested observation direction";
    transitState(
        MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
        "VLA_Search yaw scan timeout");
    return;
  }

  const double yaw_error =
      std::abs(normalizeAngle(vla_search_scan_target_yaw_ - fd_->odom_yaw_));
  if (yaw_error > vla_search_scan_yaw_tolerance_) {
    vla_search_scan_yaw_reached_time_ = ros::Time();
    return;
  }
  if (vla_search_scan_yaw_reached_time_.isZero()) {
    vla_search_scan_yaw_reached_time_ = ros::Time::now();
    return;
  }
  if ((ros::Time::now() - vla_search_scan_yaw_reached_time_).toSec() <
      vla_search_scan_settle_time_) {
    return;
  }

  sensor_msgs::CompressedImageConstPtr camera_image;
  ros::Time camera_receive_time;
  {
    std::lock_guard<std::mutex> lock(vla_search_camera_mutex_);
    camera_image = vla_search_latest_camera_image_;
    camera_receive_time = vla_search_latest_camera_receive_time_;
  }
  if (camera_image == nullptr) {
    return;
  }
  const ros::Time image_stamp =
      camera_image->header.stamp.isZero()
          ? camera_receive_time
          : camera_image->header.stamp;
  if (image_stamp <= vla_search_scan_yaw_reached_time_) {
    return;
  }

  scene_graph::VLASearchObservation observation;
  observation.header.stamp = image_stamp;
  observation.header.frame_id =
      camera_image->header.frame_id.empty()
          ? std::string("camera")
          : camera_image->header.frame_id;
  observation.task_session_id = vla_search_session_id_;
  observation.observation_batch_id = vla_search_observation_batch_id_;
  observation.observation_index =
      static_cast<uint8_t>(vla_search_scan_index_);
  observation.body_yaw = fd_->odom_yaw_;
  observation.odom_pose.position.x = fd_->odom_pos_.x();
  observation.odom_pose.position.y = fd_->odom_pos_.y();
  observation.odom_pose.position.z = fd_->odom_pos_.z();
  observation.odom_pose.orientation.w = fd_->odom_orient_.w();
  observation.odom_pose.orientation.x = fd_->odom_orient_.x();
  observation.odom_pose.orientation.y = fd_->odom_orient_.y();
  observation.odom_pose.orientation.z = fd_->odom_orient_.z();
  observation.image = *camera_image;
  observation.image.header.stamp = image_stamp;
  vla_search_observation_pub_.publish(observation);
  vla_search_observation_stamp_ = observation.header.stamp;

  static const std::array<uint8_t, 4> prompt_types{{
      scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A,
      scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A1,
      scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A2,
      scene_graph::PromptMsg::PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A3,
  }};
  if (vla_search_scan_index_ >= vla_search_scan_yaw_offsets_.size()) {
    vla_search_finish_reason_ = "yaw_scan_configuration_invalid";
    vla_search_finish_detail_ =
        "Scan index exceeds configured observation count";
    transitState(
        MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
        "VLA_Search scan configuration invalid");
    return;
  }

  vla_search_prompt_type_ =
      prompt_types[vla_search_scan_index_ % prompt_types.size()];
  vla_search_prompt_id_ = scene_graph_->getCurPromptIdAndPlusOne();
  nlohmann::json visual_context;
  visual_context["observation_index"] = vla_search_scan_index_;
  visual_context["preferred_door_id"] = vla_search_explore_area_id_;
  std::string prompt;
  if (!scene_graph_->vlaSearchPromptGen(
          vla_search_prompt_type_, vla_search_command_,
          vla_search_session_id_, vla_search_observation_batch_id_,
          visual_context, prompt)) {
    vla_search_finish_reason_ = "prompt_generation_failed";
    vla_search_finish_detail_ =
        "Failed to generate visual observation prompt";
    transitState(
        MISSION_FSM_STATE::VLA_SEARCH_RECOVERY,
        "VLA_Search visual prompt generation failed");
    return;
  }

  const int timeout_seconds =
      std::max(1, static_cast<int>(std::ceil(vla_search_prompt_timeout_)));
  scene_graph_->sendPrompt(
      vla_search_prompt_id_, vla_search_prompt_type_, prompt,
      std::chrono::seconds(timeout_seconds),
      std::max(1, vla_search_max_plan_retries_));
  vla_search_prompt_pending_ = true;
  vla_search_prompt_start_time_ = ros::Time::now();
  transitState(
      MISSION_FSM_STATE::VLA_SEARCH_WAIT_LLM,
      "VLA_Search visual observation prompt sent");
}

void MissionFSM::handleVlaSearchRecovery()
{
  if (vla_search_finish_reason_.empty()) {
    vla_search_finish_reason_ = "internal_error";
    vla_search_finish_detail_ = "VLA_Search recovery has no failure reason";
  }
  stopMotion();
  transitState(MISSION_FSM_STATE::VLA_SEARCH_FINISH, "VLA_Search recovery");
}

void MissionFSM::handleVlaSearchFinish()
{
  if (!vla_search_active_) {
    transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "VLA_Search finish inactive");
    return;
  }

  publishVlaSearchResult(vla_search_success_, vla_search_finish_reason_, vla_search_finish_detail_);
  vla_search_active_ = false;
  transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "VLA_Search finish");
}

void MissionFSM::emergencyStopCallback(const std_msgs::Empty::ConstPtr&)
{
  if (!vla_search_active_) {
    return;
  }
  cancelVlaSearchTask("task_cancelled", "received /command/emergency_stop");
}

void MissionFSM::triggerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  fd_->trigger_ = true;
}

void MissionFSM::handleGoalInstruction(const std::vector<geometry_msgs::Point>& goals,
                                               const std::vector<float>& yaws,
                                               bool look_forward,
                                               const std::string& source) {
  switchPlannerCmdMuxToEgo(source + ":goal");

  if (goals.empty()) {
    ROS_WARN_STREAM("[GOAL] Ignore empty goal instruction from " << source);
    return;
  }

  const auto& first_goal = goals.front();
  const double goal_z = std::isfinite(first_goal.z) ? static_cast<double>(first_goal.z) : 1.0;
  const double yaw = yaws.empty() ? fd_->odom_yaw_ : static_cast<double>(yaws.front());

  {
    std::unique_lock<std::mutex> lck(mtx_);
    fd_->track_trigger_ = false;
    fd_->track_init_ = false;
    resetTrackingFinishCandidate();
    fd_->track_finish_sent_ = false;
    map_->setTarget(fd_->track_pos_, false);
    if (md_->mission_state_ == MISSION_FSM_STATE::PLAN_TRACK ||
        md_->mission_state_ == MISSION_FSM_STATE::APPROACH_TRACK) {
      stopMotion();
    }
    transitState(MISSION_FSM_STATE::WAIT_TRIGGER, source);
  }

  pubLocalGoal(
      Eigen::Vector3d(first_goal.x, first_goal.y, goal_z),
      yaw,
      look_forward,
      quadrotor_msgs::LocalGoalSet::YAW_MODE_NORMAL);
}

void MissionFSM::handleTrackingTarget(const std::vector<geometry_msgs::Point>& global_poses,
                                              const std::string& source,
                                              const ros::Time& stamp,
                                              const std::string& frame_id) {
  (void)stamp;
  (void)frame_id;
  if (global_poses.empty()) return;

  const auto& first_pose = global_poses.front();
  if (first_pose.x == -1 && first_pose.y == -1 && first_pose.z == -1) return;

  std::unique_lock<std::mutex> lck(mtx_);
  if (!fd_->track_trigger_) {
    ROS_WARN_STREAM_THROTTLE(2.0, "Wait for track command, ignore tracking target.");
    return;
  }

  double min_dist = std::numeric_limits<double>::max();
  int min_index = -1;
  for (int i = 0; i < static_cast<int>(global_poses.size()); ++i) {
    const auto& pose = global_poses[i];
    if (pose.x == -1 && pose.y == -1 && pose.z == -1) continue;

    const Eigen::Vector3d candidate = geoPt2Vec3d(pose);
    const double dist = (candidate - fd_->track_pos_).norm();
    if (dist < min_dist) {
      min_dist = dist;
      min_index = i;
    }
  }

  if (min_index < 0) return;

  const Eigen::Vector3d candidate = geoPt2Vec3d(global_poses[min_index]);
  const bool was_track_init = fd_->track_init_;
  if (!fd_->track_init_ || min_dist < fp_->track_detect_error_) {
    fd_->track_pos_ = candidate;
    fd_->track_init_ = true;
    if (!was_track_init) {
      resetTrackingFinishCandidate();
    }
    ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] Update target: " << fd_->track_pos_.transpose());
  } else {
    ROS_WARN_STREAM_THROTTLE(1.0, "[TRACK] Ignore target jump, candidate: " << candidate.transpose()
                             << " previous: " << fd_->track_pos_.transpose());
    return;
  }

  if (map_->isInited()) {
    map_->setTarget(fd_->track_pos_, fd_->track_init_);
  }

  if (md_->mission_state_ != MISSION_FSM_STATE::PLAN_TRACK &&
      md_->mission_state_ != MISSION_FSM_STATE::APPROACH_TRACK) {
    transitState(MISSION_FSM_STATE::PLAN_TRACK, source);
  }
}

void MissionFSM::egoExecFinishCallback(const std_msgs::Bool::ConstPtr &msg) {
  fd_->ego_exec_finished_ = msg->data;
  INFO_MSG_GREEN("--------- [FSM] EGO-Planner Execution Finished -----------");
}

void MissionFSM::trackCommandCallback(const quadrotor_msgs::TrackCommand::ConstPtr& msg) {
  if (msg->robot_id != md_->drone_id_) return;

  std::unique_lock<std::mutex> lck(mtx_);
  if (!msg->enable)
  {
    fd_->track_trigger_ = false;
    fd_->track_init_ = false;
    resetTrackingFinishCandidate();
    fd_->track_finish_sent_ = false;
    switchPlannerCmdMuxToEgo("trackCommand:disable");
    map_->setTarget(fd_->track_pos_, false);
    if (md_->mission_state_ == MISSION_FSM_STATE::PLAN_TRACK ||
        md_->mission_state_ == MISSION_FSM_STATE::APPROACH_TRACK)
    {
      stopMotion();
      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "trackCommand:disable");
    }
    return;
  }

  const bool was_track_trigger = fd_->track_trigger_;
  fd_->track_trigger_ = true;
  if (!was_track_trigger) {
    resetTrackingFinishCandidate();
    fd_->track_finish_sent_ = false;
  }
  if (msg->has_target_position)
  {
    fd_->track_pos_ = geoPt2Vec3d(msg->target_position);
  }

  switchPlannerCmdMuxToEgo("trackCommand:ego_tracking_enable");
  map_->setTarget(fd_->track_pos_, false);
  transitState(MISSION_FSM_STATE::PLAN_TRACK, "trackCommand:enable");
}





void MissionFSM::targetCallbackReal(const quadrotor_msgs::DetectOut::ConstPtr& msg)
{
  handleTrackingTarget(msg->global_poses, "trackTargetUpdate", msg->header.stamp, msg->header.frame_id);
}

void MissionFSM::vlaSearchTargetCallback(
    const quadrotor_msgs::VLASearchTarget::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!vla_search_active_ || !vla_search_target_pending_) {
    return;
  }
  if (msg->task_session_id != vla_search_session_id_ ||
      msg->observation_batch_id != vla_search_observation_batch_id_ ||
      msg->request_id != vla_search_target_request_id_ ||
      msg->observation_index != vla_search_target_observation_index_) {
    ROS_WARN_STREAM_THROTTLE(
        1.0,
        "[VLA_SEARCH] Ignore stale target result: session="
            << static_cast<int>(msg->task_session_id)
            << ", batch=" << msg->observation_batch_id
            << ", request=" << msg->request_id
            << ", observation=" << static_cast<int>(msg->observation_index));
    return;
  }

  vla_search_target_received_ = true;
  vla_search_target_success_ = msg->success;
  vla_search_target_source_ = msg->source;
  vla_search_target_error_ = msg->error;
  if (msg->success) {
    vla_search_target_position_ = Eigen::Vector3d(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z);
  }
}

void MissionFSM::vlaSearchCameraCallback(
    const sensor_msgs::CompressedImageConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(vla_search_camera_mutex_);
  vla_search_latest_camera_image_ = msg;
  vla_search_latest_camera_receive_time_ = ros::Time::now();
}

void MissionFSM::vlaSearchEgoStateTriggerCallback(
    const quadrotor_msgs::EgoStateTrigger::ConstPtr& msg)
{
  vla_search_ego_stable_ = msg->data;
}

bool MissionFSM::getSceneGraphInitSeed(Eigen::Vector3d& init_seed, std::string* reason) const {
  init_seed = fd_->odom_pos_;
  init_seed.x() += fp_->scene_graph_init_forward_dist_ * std::cos(fd_->odom_yaw_);
  init_seed.y() += fp_->scene_graph_init_forward_dist_ * std::sin(fd_->odom_yaw_);

  if (!map_->isInGlobalMap(init_seed)) {
    if (reason != nullptr) *reason = "seed out of global map";
    return false;
  }

  if (!map_->isInLocalMap(init_seed)) {
    if (reason != nullptr) *reason = "seed out of local map buffer";
    return false;
  }

  if (map_->getInflateOccupancy(init_seed) == global_belief::MapInterface::OCCUPIED) {
    if (reason != nullptr) *reason = "seed in occupied region";
    return false;
  }

  return true;
}









void MissionFSM::resetTrackingFinishCandidate() {
  fd_->track_finish_candidate_active_ = false;
  fd_->track_finish_candidate_start_time_ = ros::Time(0);
  fd_->track_finish_last_pos_ = fd_->odom_pos_;
  fd_->track_finish_last_yaw_ = fd_->odom_yaw_;
  fd_->track_finish_move_acc_ = 0.0;
  fd_->track_finish_yaw_acc_ = 0.0;
}

bool MissionFSM::updateTrackingFinishCandidate(double dis_2_aim, double angle_2_aim) {
  const bool near_aim = dis_2_aim < fp_->arrive_dis_thr_;
  const bool yaw_ok = angle_2_aim < fp_->track_yaw_thr_;
  if (!near_aim || !yaw_ok || fd_->track_finish_sent_) {
    resetTrackingFinishCandidate();
    return false;
  }

  const ros::Time now = ros::Time::now();
  if (!fd_->track_finish_candidate_active_) {
    fd_->track_finish_candidate_active_ = true;
    fd_->track_finish_candidate_start_time_ = now;
    fd_->track_finish_last_pos_ = fd_->odom_pos_;
    fd_->track_finish_last_yaw_ = fd_->odom_yaw_;
    fd_->track_finish_move_acc_ = 0.0;
    fd_->track_finish_yaw_acc_ = 0.0;
    ROS_INFO_STREAM("[TRACK] finish candidate start, pos=" << fd_->odom_pos_.transpose()
                    << ", yaw=" << fd_->odom_yaw_);
    return false;
  }

  fd_->track_finish_move_acc_ += (fd_->odom_pos_ - fd_->track_finish_last_pos_).norm();
  fd_->track_finish_yaw_acc_ += std::fabs(normalizeAngle(fd_->odom_yaw_ - fd_->track_finish_last_yaw_));
  fd_->track_finish_last_pos_ = fd_->odom_pos_;
  fd_->track_finish_last_yaw_ = fd_->odom_yaw_;

  if (fd_->track_finish_move_acc_ > fp_->track_finish_move_thresh_ ||
      fd_->track_finish_yaw_acc_ > fp_->track_finish_yaw_thresh_) {
    ROS_INFO_STREAM("[TRACK] finish candidate reset: move_acc=" << fd_->track_finish_move_acc_
                    << ", yaw_acc=" << fd_->track_finish_yaw_acc_);
    resetTrackingFinishCandidate();
    return false;
  }

  const double hold_time = (now - fd_->track_finish_candidate_start_time_).toSec();
  if (hold_time < fp_->track_finish_hold_time_) {
    ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] finish candidate holding: time=" << hold_time
                             << "/" << fp_->track_finish_hold_time_
                             << ", move_acc=" << fd_->track_finish_move_acc_
                             << ", yaw_acc=" << fd_->track_finish_yaw_acc_);
    return false;
  }

  return true;
}

void MissionFSM::publishTrackingFinish() {
  if (fd_->track_finish_sent_) {
    return;
  }
  std_msgs::Bool msg;
  msg.data = true;
  tracking_finish_pub_.publish(msg);
  fd_->track_finish_sent_ = true;
  resetTrackingFinishCandidate();
  ROS_INFO("[TRACK] publish /tracking_finish=true");
}


void MissionFSM::switchPlannerCmdMuxToEgo(const std::string& source) {
  const std::string next_mode = fp_->planner_cmd_mux_ego_mode_;
  if (next_mode == planner_cmd_mux_active_mode_) return;
  planner_cmd_mux_active_mode_ = next_mode;
  ROS_INFO_STREAM("[planner_cmd_mux] switch mode to " << planner_cmd_mux_active_mode_ << " from " << source);
  publishLastForMode(source);
}


bool MissionFSM::isFresh(const ros::Time& stamp) const {
  if (planner_cmd_mux_input_timeout_ <= 0.0) return true;
  if (stamp.isZero()) return false;
  return (ros::Time::now() - stamp).toSec() <= planner_cmd_mux_input_timeout_;
}


void MissionFSM::publishIfActive(const quadrotor_msgs::PositionCommand& cmd,
                                          const std::string& source_mode) {
  if (planner_cmd_mux_active_mode_ != source_mode) return;
  position_cmd_pub_.publish(cmd);
}

void MissionFSM::publishLastForMode(const std::string& source) {
  if (planner_cmd_mux_active_mode_ == fp_->planner_cmd_mux_ego_mode_) {
    if (has_ego_cmd_ && isFresh(ego_cmd_stamp_)) {
      position_cmd_pub_.publish(last_ego_cmd_);
    } else {
      ROS_WARN_STREAM_THROTTLE(1.0, "[planner_cmd_mux] no fresh ego cmd after switch from " << source);
    }
  }
}

void MissionFSM::egoPositionCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
  last_ego_cmd_ = *msg;
  ego_cmd_stamp_ = ros::Time::now();
  has_ego_cmd_ = true;
  publishIfActive(last_ego_cmd_, fp_->planner_cmd_mux_ego_mode_);
}





void MissionFSM::planTrack() {
  ROS_INFO("\033[1;31mPlan TRACK!\033[0m");

  if (!fd_->track_trigger_) {
    resetTrackingFinishCandidate();
    transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "planTrack:no trigger");
    return;
  }

  if (!fd_->track_init_) {
    resetTrackingFinishCandidate();
    ROS_WARN_THROTTLE(1.0, "[TRACK] Wait tracking target initialization.");
    return;
  }

  const Eigen::Vector3d target_vec = fd_->track_pos_ - fd_->odom_pos_;
  if (target_vec.norm() < 1e-3) {
    resetTrackingFinishCandidate();
    ROS_WARN_THROTTLE(1.0, "[TRACK] Target too close to current position, skip planning.");
    return;
  }

  fd_->path_res_.clear();
  fd_->aim_pos_ = fd_->track_pos_ - target_vec.normalized() * fp_->track_aim_dist_;
  fd_->aim_pos_.z() = 1.0;
  fd_->aim_yaw_ = atan2(target_vec.y(), target_vec.x());

  ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] track pos: " << fd_->track_pos_.transpose()
                           << " aim pos: " << fd_->aim_pos_.transpose()
                           << " odom pos: " << fd_->odom_pos_.transpose()
                           << " aim yaw: " << fd_->aim_yaw_);

  const double pos_err = (fd_->aim_pos_ - fd_->odom_pos_).norm();
  const double yaw_err = std::fabs(normalizeAngle(fd_->aim_yaw_ - fd_->odom_yaw_));
  if (pos_err < fp_->track_replan_dist_ &&
      yaw_err < fp_->track_yaw_thr_) {
    if (updateTrackingFinishCandidate(pos_err, yaw_err)) {
      publishTrackingFinish();
      return;
    }
    ROS_WARN_THROTTLE(1.0, "[TRACK] Already close to tracking aim, skip planning.");
    fd_->local_aim_pos_ = fd_->aim_pos_;
    fd_->has_rotated_ = true;
    fd_->last_pub_time_ = ros::Time::now();
    transitState(MISSION_FSM_STATE::APPROACH_TRACK, "planTrack:already_close_hold");
    return;
  }

  const int res = callTrackPlanner(fd_->aim_pos_, fd_->aim_vel_, fd_->aim_yaw_, fd_->path_res_);
  if (res != 0) {
    resetTrackingFinishCandidate();
    ROS_WARN_THROTTLE(1.0, "[TRACK] Tracking target not directly reachable yet.");
    return;
  }

  fd_->path_inx_ = 0;
  fd_->local_aim_pos_ = fd_->aim_pos_;

  const double dis_2_aim_2d = (fd_->aim_pos_ - fd_->odom_pos_).head(2).norm();
  // 远距离 yaw 偏差过大时提前按目标方向边飞边转；近距离 yaw-lock 保持快速转向以尽快找回目标视野。
  const bool near_yaw_lock = dis_2_aim_2d < fp_->track_turn_yaw_dist_;
  const bool far_yaw_align = !near_yaw_lock && yaw_err > fp_->track_fly_yaw_thr_;
  const bool look_forward = !(near_yaw_lock || far_yaw_align);
  const bool yaw_low_speed = far_yaw_align;

  pubLocalGoal(fd_->path_res_.back(), fd_->aim_yaw_, look_forward,
               yaw_low_speed ? quadrotor_msgs::LocalGoalSet::YAW_MODE_LOW_SPEED
                             : quadrotor_msgs::LocalGoalSet::YAW_MODE_NORMAL);
  resetTrackingFinishCandidate();
  INFO_MSG_GREEN("[TRACK] [look_forward = " << look_forward
                 << ", yaw_low_speed = " << yaw_low_speed
                 << ", yaw_err = " << yaw_err << "] aim: "
                 << fd_->path_res_.back().transpose() << ", yaw: " << fd_->aim_yaw_);

  fd_->has_rotated_ = near_yaw_lock;
  fd_->last_pub_time_ = ros::Time::now();
  transitState(MISSION_FSM_STATE::APPROACH_TRACK, "planTrack");
}

void MissionFSM::approachTrack() {
  ROS_INFO_STREAM_THROTTLE(0.5, "\033[1;33mApproach TRACK...\033[0m");

  if (!fd_->track_trigger_) {
    resetTrackingFinishCandidate();
    transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "approachTrack:disable");
    return;
  }

  if (!fd_->track_init_) {
    resetTrackingFinishCandidate();
    transitState(MISSION_FSM_STATE::PLAN_TRACK, "approachTrack:wait target");
    return;
  }

  const double dis_2_aim = (fd_->aim_pos_ - fd_->odom_pos_).norm();
  const double dis_2_aim_2d = (fd_->aim_pos_ - fd_->odom_pos_).head(2).norm();
  const double dis_2_local_aim = (fd_->local_aim_pos_ - fd_->odom_pos_).norm();
  const double angle_2_aim = std::fabs(normalizeAngle(fd_->aim_yaw_ - fd_->odom_yaw_));
  const double t_cur = (ros::Time::now() - fd_->last_pub_time_).toSec();

  ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] Dis to aim: " << dis_2_aim_2d
                           << " local aim: " << dis_2_local_aim
                           << " yaw err: " << angle_2_aim
                           << " t_cur: " << t_cur);

  if (updateTrackingFinishCandidate(dis_2_aim, angle_2_aim)) {
    publishTrackingFinish();
    return;
  }

  if (t_cur > fp_->replan_thresh3_) {
    resetTrackingFinishCandidate();
    transitState(MISSION_FSM_STATE::PLAN_TRACK, "approachTrack:periodic");
    return;
  }

  const Eigen::Vector3d target_vec = fd_->track_pos_ - fd_->odom_pos_;
  if (target_vec.norm() < 1e-3) {
    resetTrackingFinishCandidate();
    transitState(MISSION_FSM_STATE::PLAN_TRACK, "approachTrack:target too close");
    return;
  }

  Eigen::Vector3d aim_pos_new = fd_->track_pos_ - target_vec.normalized() * fp_->track_aim_dist_;
  aim_pos_new.z() = 1.0;
  if ((fd_->aim_pos_ - aim_pos_new).norm() > fp_->track_replan_dist_) {
    INFO_MSG_GREEN("[TRACK] aim_pos_old: " << fd_->aim_pos_.transpose()
                   << " aim_pos_new: " << aim_pos_new.transpose());
    resetTrackingFinishCandidate();
    transitState(MISSION_FSM_STATE::PLAN_TRACK, "approachTrack:moved");
    return;
  }

  const double current_dir = atan2(target_vec.y(), target_vec.x());
  if (!fd_->has_rotated_ && dis_2_aim_2d < fp_->track_turn_yaw_dist_) {
    fd_->has_rotated_ = true;
    fd_->aim_yaw_ = current_dir;
    resetTrackingFinishCandidate();
    pubLocalGoal(fd_->aim_pos_, fd_->aim_yaw_, false,
                 quadrotor_msgs::LocalGoalSet::YAW_MODE_NORMAL);
    INFO_MSG_GREEN("[TRACK] Switch to yaw-lock, aim: " << fd_->aim_pos_.transpose()
                   << ", yaw: " << fd_->aim_yaw_);
    return;
  }

  if (fd_->has_rotated_ &&
      std::fabs(normalizeAngle(current_dir - fd_->aim_yaw_)) > fp_->track_yaw_thr_) {
    fd_->aim_yaw_ = current_dir;
    resetTrackingFinishCandidate();
    pubLocalGoal(fd_->aim_pos_, fd_->aim_yaw_, false,
                 quadrotor_msgs::LocalGoalSet::YAW_MODE_NORMAL);
    INFO_MSG_GREEN("[TRACK] Update yaw-lock, aim: " << fd_->aim_pos_.transpose()
                   << ", yaw: " << fd_->aim_yaw_);
  }
}

void MissionFSM::startPanoramaRotation() {
  // 全景旋转固定沿yaw正方向执行，odometry增量负责记录真实累计转角。
  panorama_last_odom_yaw_ = fd_->odom_yaw_;
  panorama_start_yaw_ = fd_->odom_yaw_;
  panorama_unwrapped_yaw_ = fd_->odom_yaw_;
  panorama_accumulated_yaw_ = 0.0;
  panorama_command_target_yaw_ = panorama_unwrapped_yaw_;
  panorama_hold_pos_ = fd_->odom_pos_;
  panorama_command_active_ = false;
  need_panorama_ = true;
  need_rotate_yaw_ = false;
  ROS_WARN("[FSM] Start panorama 360° rotation, start_yaw=%.2f°",
           panorama_unwrapped_yaw_ * 180.0 / M_PI);
}

bool MissionFSM::waitForFreshMapAfterReset() {
  if (!wait_fresh_map_after_reset_)
    return false;

  const uint64_t current_update_seq = map_->getOccupancyUpdateSeq();
  if (current_update_seq <= map_reset_update_seq_) {
    ROS_WARN_THROTTLE(1.0, "[FSM] Waiting for the first occupancy update after map reset.");
    return true;
  }

  wait_fresh_map_after_reset_ = false;
  ROS_WARN("[FSM] Fresh occupancy received after map reset, start panorama rotation.");
  startPanoramaRotation();
  return false;
}

void MissionFSM::handlePanoramaYaw() {
  constexpr double TOTAL_ANGLE = 2.0 * M_PI;
  constexpr double FINISH_TOLERANCE = 2.0 * M_PI / 180.0;
  constexpr double TARGET_SETTLE_TOLERANCE = 1.0 * M_PI / 180.0;

  const double remaining = std::max(0.0, TOTAL_ANGLE - panorama_accumulated_yaw_);
  const double final_target_error =
      fabs((panorama_start_yaw_ + TOTAL_ANGLE) - panorama_unwrapped_yaw_);
  if (remaining <= FINISH_TOLERANCE && final_target_error <= TARGET_SETTLE_TOLERANCE) {
    ROS_WARN("[FSM] Panorama 360° done, accumulated=%.2f°",
             panorama_accumulated_yaw_ * 180.0 / M_PI);
    panorama_command_active_ = false;
    need_panorama_ = false;
    return;
  }

  if (panorama_command_active_)
  {
    const double angle_to_command = panorama_command_target_yaw_ - panorama_unwrapped_yaw_;
    const double commanded_angle = panorama_command_target_yaw_ - panorama_start_yaw_;
    const bool final_target_published = commanded_angle >= TOTAL_ANGLE - 1e-6;
    if (final_target_published || angle_to_command > panorama_extend_angle_)
      return;
  }

  // 每次最多向前延长配置角度，最终连续目标严格封顶为起始yaw+360°。
  const double commanded_angle =
      std::max(0.0, panorama_command_target_yaw_ - panorama_start_yaw_);
  const double next_commanded_angle =
      std::min(TOTAL_ANGLE, commanded_angle + panorama_max_step_);
  panorama_command_target_yaw_ = panorama_start_yaw_ + next_commanded_angle;
  ROS_INFO("[Panorama] publish target=%.1f°, accumulated=%.1f°, remaining=%.1f°",
           panorama_command_target_yaw_ * 180.0 / M_PI,
           panorama_accumulated_yaw_ * 180.0 / M_PI,
           remaining * 180.0 / M_PI);
  pubLocalGoal(panorama_hold_pos_, panorama_command_target_yaw_, false,
               quadrotor_msgs::LocalGoalSet::YAW_MODE_PANORAMA,
               quadrotor_msgs::LocalGoalSet::YAW_PATH_KEEP_DIRECTION);
  panorama_command_active_ = true;
}

void MissionFSM::handleYawChange() {
  // 依次转向原始朝向+45°、-45°，最终回到原始朝向。
  if (!enable_yaw_scan_) {
    need_rotate_yaw_ = false;
    transitState(stash_state_, "Yaw Scan Disabled");
    return;
  }

  if (!fd_->ego_exec_finished_) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[Handleyaw] : ego exec not finished, skip yaw handle ...");
    return ;
  }

  if (!yawhandle_left_ok) {
    if (abs(fd_->odom_yaw_ - yawhandle_yaw_target_left) < 0.05) yawhandle_left_ok = true;
    if (!yawhandle_left_published) {
      INFO_MSG("[HandleYaw] | Turn Left ...");
      yawhandle_left_published = true;
      pubLocalGoal(fd_->odom_pos_, yawhandle_yaw_target_left, false,
                   quadrotor_msgs::LocalGoalSet::YAW_MODE_LOW_SPEED);
    }
    return ;
  }

  if (!yawhandle_right_ok) {
    if (abs(fd_->odom_yaw_ - yawhandle_yaw_target_right) < 0.05) yawhandle_right_ok = true;
    if (!yawhandle_right_published) {
      ros::Duration(0.5).sleep();
      INFO_MSG("[HandleYaw] | Turn Right ...");
      yawhandle_right_published = true;
      pubLocalGoal(fd_->odom_pos_, yawhandle_yaw_target_right, false,
                   quadrotor_msgs::LocalGoalSet::YAW_MODE_LOW_SPEED);
    }
    return ;
  }

  if (!yawhandle_back_ok) {
    if (abs(fd_->odom_yaw_ - yawhandle_yaw_raw) < 0.05) yawhandle_back_ok = true;
    if (!yawhandle_back_published) {
      ros::Duration(0.5).sleep();
      INFO_MSG("[HandleYaw] | Back to raw ...");
      yawhandle_back_published = true;
      pubLocalGoal(fd_->odom_pos_, yawhandle_yaw_raw, false,
                   quadrotor_msgs::LocalGoalSet::YAW_MODE_LOW_SPEED);
    }
    return ;
  }

  yawhandle_left_published  = yawhandle_right_published = yawhandle_back_published = false;
  yawhandle_left_ok         = yawhandle_right_ok        = yawhandle_back_ok        = false;
  need_rotate_yaw_          = false;
  transitState(stash_state_, "Yaw Handle Done");
}

void MissionFSM::FSMCallback(const ros::TimerEvent& e)
{
  // ROS_INFO_STREAM_THROTTLE(10.0, "** [EXP-FSM]: state: " << md_->state_str_[md_->mission_state_]);
  CALL_EVERY_N_TIMES(displayMissionState, 5);

  std_msgs::String fsm_state_str;
  if (md_->mission_state_ == FINISH || md_->mission_state_ == WAIT_TRIGGER) fsm_state_str.data = "FINISH";
  else fsm_state_str.data = "EXECING";
  fsm_state_pub_.publish(fsm_state_str);

  traj_visualizer_->addPoint(fd_->odom_pos_, fd_->odom_vel_.norm());

  switch (md_->mission_state_)
  {
    case INIT:
    {
      // Wait for odometry ready
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(2.0, "no odom.");
        return;
      }
      if (!map_->isInited()){
        ROS_WARN_THROTTLE(2.0, "no map.");
        return;
      }
      // Go to wait trigger when odom is ok
      fd_->warmup_start_time_ = ros::Time::now();
      transitState(MISSION_FSM_STATE::WARM_UP, "FSM");
      break;
    }

    // warm up 10sec -> init skeleton -> start explore
    case WARM_UP:
    {
      if (fp_->auto_load_scene_graph_ && !fp_->scene_graph_load_name_.empty()) {
        bool ok = scene_graph_->loadMap(fp_->scene_graph_load_name_,
                                         fp_->scene_graph_data_path_);
        if (!ok) {
          ROS_FATAL("[MissionFSM] Failed to load offline scene graph '%s/%s'",
                    fp_->scene_graph_data_path_.c_str(),
                    fp_->scene_graph_load_name_.c_str());
          ros::shutdown();
          return;
        }
        ROS_INFO("[MissionFSM] Loaded offline scene graph: %s/%s",
                 fp_->scene_graph_data_path_.c_str(),
                 fp_->scene_graph_load_name_.c_str());
        scene_graph_->object_factory_->runThisModule();
        scene_graph_->refreshLoadedMapVisualization();
        transitState(WAIT_TRIGGER, "auto_load_scene_graph");
        break;
      }

      if (fp_->auto_init_scene_graph_ && !fd_->trigger_) {
        const double warmup_elapsed = (ros::Time::now() - fd_->warmup_start_time_).toSec();
        if (warmup_elapsed >= fp_->auto_init_delay_sec_) {
          fd_->trigger_ = true;
        } else {
          ROS_INFO_THROTTLE(2.0, "Wait auto init delay before skeleton expand ... ");
          return;
        }
      }

      if (!fd_->trigger_) {
        ROS_INFO_THROTTLE(10.0, "Wait Trigger For Skeleton Expand ... ");
        return;
      }
      bool new_topo = false;
      fd_->trigger_ = false;
      Eigen::Vector3d init_seed;
      std::string init_block_reason;
      if (!getSceneGraphInitSeed(init_seed, &init_block_reason)) {
        ROS_WARN_STREAM_THROTTLE(2.0, "[EXP-FSM] Scene graph init seed is not ready: " << init_block_reason);
        return;
      }

      ROS_INFO_STREAM("[EXP-FSM] Init scene graph from forward seed: " << init_seed.transpose()
                      << " (yaw=" << fd_->odom_yaw_ << ")");
      scene_graph_->initSceneGraph(init_seed, fd_->odom_yaw_);
      scene_graph_->history_visited_area_ids_.push_back(0);

      if (scene_graph_->skeleton_gen_->ready()) {
        scene_graph_->object_factory_->runThisModule();
      }else {
        transitState(MISSION_FSM_STATE::INIT, "FSM-WARMUP");
        fd_->trigger_ = false;
        return;
      }

      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER:
    {
      if (object_id_nav_autostart_enable_ && !object_id_nav_autostart_triggered_) {
        if (object_id_nav_autostart_ready_time_.isZero()) {
          object_id_nav_autostart_ready_time_ = ros::Time::now();
        }
        const double ready_sec = (ros::Time::now() - object_id_nav_autostart_ready_time_).toSec();
        if (ready_sec >= object_id_nav_autostart_delay_sec_) {
          if (scene_graph_ == nullptr || scene_graph_->object_factory_ == nullptr) {
            ROS_WARN_THROTTLE(2.0, "[ObjIdNavAuto] SceneGraph object factory is not ready.");
          } else if (scene_graph_->object_factory_->object_map_.find(object_id_nav_autostart_target_id_) ==
                     scene_graph_->object_factory_->object_map_.end()) {
            ROS_WARN_THROTTLE(2.0, "[ObjIdNavAuto] target object id %d is not registered yet.",
                              object_id_nav_autostart_target_id_);
          } else {
            object_id_nav_autostart_triggered_ = true;
            ROS_WARN("[ObjIdNavAuto] start object-id navigation to target id %d.",
                     object_id_nav_autostart_target_id_);
            startObjectIdNav(object_id_nav_autostart_target_id_,
                             quadrotor_msgs::Instruction::SOURCE_TASK_EXPLORATION,
                             0, "object_id_nav_autostart");
          }
        }
      }
      if (fd_->trigger_) {
        fd_->trigger_ = false;
      }
      break;
    }

    case GO_TARGET_OBJECT: {
      goTargetObject();
      break;
    }

    case GO_TARGET_WITH_WAYPOINT: {
      goTargetWithWaypoint();
      break;
    }

    case FINISH: {
      ROS_INFO_STREAM_THROTTLE(10.0, "\033[1;31mFinish!\033[0m");  //红
      break;
    }

    case YAW_HANDLE: {
      handleYawChange();
      break;
    }

    case THINKING: {
      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "thinking_removed");
      break;
    }

    case LLM_PLAN_EXPLORE: {
      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "llm_plan_explore_removed");
      break;
    }

    case PLAN_EXPLORE: {
      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "plan_explore_removed");
      break;
    }

    case PLAN_TRACK: {
      planTrack();
      break;
    }

    case APPROACH_EXPLORE: {
      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "approach_explore_removed");
      break;
    }

    case APPROACH_TRACK: {
      approachTrack();
      break;
    }

    case FIND_TERMINATE_TARGET:{
      findTerminateTarget();
      break;
    }

    case DF_DEMO: {
      execDFDemo();
      break;
    }

    case VLA_SEARCH_PLAN_LOCAL: {
      handleVlaSearchPlanLocal();
      break;
    }

    case VLA_SEARCH_WAIT_LLM: {
      handleVlaSearchWaitLLM();
      break;
    }

    case VLA_SEARCH_WAIT_TARGET: {
      handleVlaSearchWaitTarget();
      break;
    }

    case VLA_SEARCH_APPROACH: {
      handleVlaSearchApproach();
      break;
    }

    case VLA_SEARCH_YAW_HANDLE: {
      handleVlaSearchYaw();
      break;
    }

    case VLA_SEARCH_RECOVERY: {
      handleVlaSearchRecovery();
      break;
    }

    case VLA_SEARCH_FINISH: {
      handleVlaSearchFinish();
      break;
    }

    default:{
      break;
    }

  }
}

void MissionFSM::goTargetObject() {
  if (fd_->go_object_process_phase == 0) {
    scene_graph_->mountCurPoly(fd_->odom_pos_, fd_->odom_yaw_);
    ROS_INFO_STREAM("[MissionFSM] object_path_request target_obj_id=" << fd_->object_target_id_
                    << " source_task_id=" << static_cast<int>(active_instruction_task_id_)
                    << " task_session_id=" << static_cast<int>(active_instruction_session_id_)
                    << " odom_pos=" << fd_->odom_pos_.transpose()
                    << " odom_yaw=" << fd_->odom_yaw_);
    if (scene_graph_->getPathToObjectWithId(fd_->object_target_id_, fd_->path_res_, fd_->aim_pos_, fd_->aim_yaw_)) {
      ROS_INFO_STREAM("[MissionFSM] object_path_result success=1 target_obj_id=" << fd_->object_target_id_
                      << " path_size=" << fd_->path_res_.size()
                      << " aim_pos=" << fd_->aim_pos_.transpose()
                      << " aim_yaw=" << fd_->aim_yaw_);
      ROS_INFO_STREAM("[MissionFSM] object_path_topo_sequence target_obj_id=" << fd_->object_target_id_
                      << " source_task_id=" << static_cast<int>(active_instruction_task_id_)
                      << " task_session_id=" << static_cast<int>(active_instruction_session_id_)
                      << " path_size=" << fd_->path_res_.size()
                      << " topo_sequence=" << formatTopoPath(fd_->path_res_));
      INFO_MSG_GREEN("[Targ Obj] | find path to object success, size: " << fd_->path_res_.size());

      fd_->has_rotated_     = false;
      fd_->stuck_begin_time_ = -1.0;                  // 新路径生成时重置卡死计时
      fd_->stuck_force_advance_count_ = 0;             // 新路径生成时重置强制推进计数
      fd_->stuck_force_advance_triggered_ = false;
      fd_->last_pub_time_   = ros::Time::now();
      INFO_MSG_CYAN("[Targ Obj] | PubNxtLocalAim, aim: " << fd_->local_aim_pos_ << ", global aim: " << fd_->aim_pos_);
      getAndPublishNextAim(fd_->path_res_, true, 0.0f);

      displayPath();
      fd_->go_object_process_phase ++;
    }else {
      ROS_WARN_STREAM("[MissionFSM] object_path_result success=0 target_obj_id=" << fd_->object_target_id_
                      << " reason=scene_graph_path_failed");
      fd_->go_object_process_phase = 0;
      if(fd_->find_terminate_target_mode_) {
        transitState(FINISH, "** FIND TERMINATE TARGET PATH FAILED **");
      }
      else transitState(WAIT_TRIGGER, "** FIND OBJECT PATH FAILED **");
    }
  }

  if (fd_->go_object_process_phase == 1) {
    double dis_2_aim_2d    = (fd_->aim_pos_       - fd_->odom_pos_).head(2).norm();
    double dis_2_local_aim = (fd_->local_aim_pos_ - fd_->odom_pos_).norm();
    double dis_yaw         = abs(fd_->aim_yaw_ - fd_->odom_yaw_);
    double t_cur = (ros::Time::now() - fd_->last_pub_time_).toSec();
    std::string ego_plan_status_str_   = fd_->ego_plan_status_ ? "True" : "False";
    std::string ego_modify_status_str_ = fd_->ego_modify_status_ ? "True" : "False";
    ROS_INFO_STREAM_THROTTLE(0.5, "\033[1;33mApproach Object...\033[0m \n"
                                  "   * Dis to Aim: " << dis_2_aim_2d << "\n"
                                  "   * Dis to LocalAim: " << dis_2_local_aim << "\n"
                                  "   * Dis to yaw: " << dis_yaw);  // 黄
    ROS_INFO_STREAM_THROTTLE(0.5, "[Targ Obj] : ego local goal -> (" << fd_->ego_local_goal_.transpose() << ")");
    ROS_INFO_STREAM_THROTTLE(0.5, "[Targ Obj] : ego plan times: " << fd_->ego_plan_times_
                                                                  << "  ego plan statue: " << ego_plan_status_str_
                                                                  << "  ego modify status: " << ego_modify_status_str_);
    ROS_INFO_STREAM_THROTTLE(1.0, "[MissionFSM] object_topo_progress target_obj_id=" << fd_->object_target_id_
                             << " source_task_id=" << static_cast<int>(active_instruction_task_id_)
                             << " task_session_id=" << static_cast<int>(active_instruction_session_id_)
                             << " path_index=" << fd_->path_inx_
                             << " path_size=" << fd_->path_res_.size()
                             << " is_final_path_index="
                             << (fd_->path_res_.empty() ? 0 : (fd_->path_inx_ >= static_cast<int>(fd_->path_res_.size()) - 1))
                             << " dis_2_local_aim=" << dis_2_local_aim
                             << " dis_2_aim_2d=" << dis_2_aim_2d
                             << " dis_yaw=" << dis_yaw
                             << " ego_exec_finished=" << fd_->ego_exec_finished_
                             << " ego_plan_status=" << fd_->ego_plan_status_
                             << " ego_modify_status=" << fd_->ego_modify_status_
                             << " odom_pos=" << fd_->odom_pos_.transpose()
                             << " local_aim=" << fd_->local_aim_pos_.transpose()
                             << " aim_pos=" << fd_->aim_pos_.transpose());

    displayLocalAim();  // 橙色marker标记当前导航点

    bool pos_finish = (dis_2_aim_2d < fp_->replan_dis_thresh_);
    bool yaw_finish = !fp_->object_id_nav_require_final_yaw_ ||
                      (fabs(fd_->odom_yaw_ - fd_->aim_yaw_) / M_PI * 180.0 < 5.0);
    bool direct_path = fd_->path_res_.size() <= 2;
    bool final_path_index = !fd_->path_res_.empty() &&
                            fd_->path_inx_ >= static_cast<int>(fd_->path_res_.size()) - 1;
    bool final_topo_stage = direct_path || final_path_index;
    bool finish_ready = pos_finish && yaw_finish && final_topo_stage && fd_->ego_exec_finished_;
    if (pos_finish && yaw_finish && !finish_ready) {
      ROS_INFO_STREAM_THROTTLE(0.5, "[MissionFSM] object_id_nav_finish_hold target_obj_id=" << fd_->object_target_id_
                               << " source_task_id=" << static_cast<int>(active_instruction_task_id_)
                               << " task_session_id=" << static_cast<int>(active_instruction_session_id_)
                               << " path_index=" << fd_->path_inx_
                               << " path_size=" << fd_->path_res_.size()
                               << " direct_path=" << direct_path
                               << " final_path_index=" << final_path_index
                               << " final_topo_stage=" << final_topo_stage
                               << " pos_finish=" << pos_finish
                               << " yaw_finish=" << yaw_finish
                               << " ego_exec_finished=" << fd_->ego_exec_finished_
                               << " dis_2_aim_2d=" << dis_2_aim_2d
                               << " dis_2_local_aim=" << dis_2_local_aim
                               << " dis_yaw=" << dis_yaw
                               << " reason=wait_final_topo_or_exec_finish");
    }
    if (finish_ready) {
      ROS_INFO_STREAM("[MissionFSM] object_id_nav_finish target_obj_id=" << fd_->object_target_id_
                      << " source_task_id=" << static_cast<int>(active_instruction_task_id_)
                      << " task_session_id=" << static_cast<int>(active_instruction_session_id_)
                      << " path_index=" << fd_->path_inx_
                      << " path_size=" << fd_->path_res_.size()
                      << " direct_path=" << direct_path
                      << " final_path_index=" << final_path_index
                      << " final_topo_stage=" << final_topo_stage
                      << " pos_finish=" << pos_finish
                      << " yaw_finish=" << yaw_finish
                      << " ego_exec_finished=" << fd_->ego_exec_finished_
                      << " ego_plan_status=" << fd_->ego_plan_status_
                      << " ego_modify_status=" << fd_->ego_modify_status_
                      << " dis_2_aim_2d=" << dis_2_aim_2d
                      << " dis_2_local_aim=" << dis_2_local_aim
                      << " dis_yaw=" << dis_yaw
                      << " replan_dis_thresh=" << fp_->replan_dis_thresh_
                      << " odom_pos=" << fd_->odom_pos_.transpose()
                      << " local_aim=" << fd_->local_aim_pos_.transpose()
                      << " aim_pos=" << fd_->aim_pos_.transpose());
      ROS_WARN("-------------> Finish: [Reach Aim] <-------------");
      ROS_INFO_STREAM("t_cur: " << t_cur);
      fd_->go_object_process_phase = 0;
      if (fd_->find_terminate_target_mode_) {
        transitState(FINISH, "Find Terminate Target Finish");
      }
      else transitState(WAIT_TRIGGER, "Go Target Object Finish");
      return;
    }

    // crash recovery 1: 末尾 yaw 旋转不到位时重新下发旋转指令
    if (fp_->object_id_nav_require_final_yaw_ &&
        fd_->ego_exec_finished_ && fd_->ego_modify_status_
         && (dis_2_local_aim > 1.0 || dis_yaw > 10.0f / 180.0f * M_PI)
         && fd_->path_inx_ == fd_->path_res_.size() - 1){
      fd_->last_pub_time_ = ros::Time::now();
      ROS_WARN("-------------> RePublish LocalGoal: crash recovery, forcely rotate yaw<----------------");
      pubLocalGoal(fd_->odom_pos_, fd_->aim_yaw_, false,
                   quadrotor_msgs::LocalGoalSet::YAW_MODE_NORMAL);
      // INFO_MSG_GREEN("[Targ Obj] [PubNxtLocalAim] aim: " << fd_->aim_pos_.transpose() << ", local_aim: " << fd_->local_aim_pos_.transpose());
    }

    // Replan after some time
    if (t_cur > fp_->replan_thresh3_ && fd_->odom_vel_.norm() <= 0.1) {
      ROS_WARN_STREAM("[MissionFSM] object_id_nav_replan_needed reason=periodic_still target_obj_id="
                      << fd_->object_target_id_ << " elapsed_sec=" << t_cur
                      << " odom_vel_norm=" << fd_->odom_vel_.norm());
      ROS_WARN("-------------> Replan: periodic call <-------------");
      ROS_WARN("t_cur: %f s", t_cur);
      fd_->go_object_process_phase = 0;
      transitState(WAIT_TRIGGER, "Go Target Object Replan");
      return;
    }

    // ========== 先replan后topo-block: replan耗尽或mode2超时后fallthrough到topo-block ==========
    bool fallthrough_to_topo = false;
    if (fp_->object_id_nav_replan_enable_) {

      // 统一卡死检测(所有mode共用)
      double vel_norm = fd_->odom_vel_.norm();
      double yaw_rate = fabs(fd_->odom_yaw_rate_);
      bool is_stuck = (vel_norm < fp_->object_id_nav_replan_stuck_vel_thresh_ &&
                       yaw_rate < fp_->object_id_nav_replan_stuck_yaw_rate_thresh_);
      if (is_stuck) {
        if (fd_->object_id_nav_replan_stuck_begin_time_ < 0.0) {
          fd_->object_id_nav_replan_stuck_begin_time_ = ros::Time::now().toSec();
        }
      } else {
        fd_->object_id_nav_replan_stuck_begin_time_ = -1.0;  // 有运动, 重置
      }
      double stuck_sec = (fd_->object_id_nav_replan_stuck_begin_time_ >= 0.0)
                         ? ros::Time::now().toSec() - fd_->object_id_nav_replan_stuck_begin_time_
                         : -1.0;

      // ---- Mode 0/1: 卡死自动replan ----
      if (fp_->object_id_nav_replan_mode_ == 0 || fp_->object_id_nav_replan_mode_ == 1) {
        if (stuck_sec > fp_->object_id_nav_replan_stuck_duration_) {
          int max_cnt = fp_->object_id_nav_replan_stuck_max_consecutive_;
          if (max_cnt > 0 && fd_->object_id_nav_replan_stuck_count_ >= max_cnt) {
            ROS_WARN_STREAM("[MissionFSM] object_id_nav_replan_exhausted fallback=topo_block reason=stuck_max_reached"
                            << " target_obj_id=" << fd_->object_target_id_
                            << " stuck_sec=" << stuck_sec
                            << " count=" << fd_->object_id_nav_replan_stuck_count_
                            << " max_count=" << max_cnt);
            ROS_WARN("[ObjIdNavReplan] Stuck replan max reached (count=%d, max=%d), fallback to topo-block",
                     fd_->object_id_nav_replan_stuck_count_, max_cnt);
            fallthrough_to_topo = true;
          } else {
            ROS_WARN_STREAM("[MissionFSM] object_id_nav_replan_needed reason=stuck_detected target_obj_id="
                            << fd_->object_target_id_
                            << " stuck_sec=" << stuck_sec
                            << " count=" << fd_->object_id_nav_replan_stuck_count_);
            ROS_WARN("[ObjIdNavReplan] Stuck detected (%.1fs), triggering replan", stuck_sec);
            triggerObjectIdNavReplan("stuck_detected");
            return;
          }
        }
      }

      // ---- Mode 0/2: 话题手动replan ----
      if ((fp_->object_id_nav_replan_mode_ == 0 || fp_->object_id_nav_replan_mode_ == 2) &&
          fd_->object_id_nav_replan_topic_triggered_) {
        if (!fd_->has_stored_object_id_nav_instruction_) {
          ROS_WARN("[ObjIdNavReplan] Topic trigger ignored: no stored instruction");
          fd_->object_id_nav_replan_topic_triggered_ = false;
        } else {
          ROS_WARN_STREAM("[MissionFSM] object_id_nav_replan_needed reason=topic_triggered target_obj_id="
                          << fd_->object_target_id_);
          ROS_WARN("[ObjIdNavReplan] Topic trigger received, triggering replan");
          triggerObjectIdNavReplan("topic_triggered");
          return;
        }
      }

      // ---- Mode 2: 卡死超时fallback到topo-block ----
      if (fp_->object_id_nav_replan_mode_ == 2 &&
          stuck_sec > fp_->object_id_nav_replan_mode2_stuck_fallback_delay_) {
        ROS_WARN_STREAM("[MissionFSM] object_id_nav_replan_exhausted fallback=topo_block reason=mode2_stuck_timeout"
                        << " target_obj_id=" << fd_->object_target_id_
                        << " stuck_sec=" << stuck_sec
                        << " fallback_delay=" << fp_->object_id_nav_replan_mode2_stuck_fallback_delay_);
        ROS_WARN("[ObjIdNavReplan] Mode2 stuck %.1fs > fallback delay %.1fs, fallback to topo-block",
                 stuck_sec, fp_->object_id_nav_replan_mode2_stuck_fallback_delay_);
        fallthrough_to_topo = true;
      }

    }

    // ---- topo-block 卡死强制推进(兜底逻辑) ----
    // 仅当 replan 未启用 或 replan 已耗尽(fallthrough) 时执行
    // 分层策略: tier1=强制重规划topo路径(清除blocked), tier2=逐点强制推进path_inx++
    bool run_topo = !fp_->object_id_nav_replan_enable_ || fallthrough_to_topo;
    if (run_topo && fp_->stuck_force_advance_enable_) {
      double vel_norm = fd_->odom_vel_.norm();
      double yaw_rate = fabs(fd_->odom_yaw_rate_);
      if (vel_norm < fp_->stuck_force_advance_vel_thresh_ &&
          yaw_rate < fp_->stuck_force_advance_yaw_rate_thresh_) {
        if (fd_->stuck_begin_time_ < 0.0) {
          fd_->stuck_begin_time_ = ros::Time::now().toSec();
        }
        double stuck_duration = ros::Time::now().toSec() - fd_->stuck_begin_time_;
        if (stuck_duration > fp_->stuck_force_advance_duration_ &&
            !fd_->stuck_force_advance_triggered_ &&
            fd_->stuck_force_advance_count_ < fp_->stuck_force_advance_max_consecutive_) {

          // Tier1: 首次卡死 → 清除blocked标记后内联重规划topo路径(类似强制重启任务)
          if (fd_->stuck_force_advance_count_ == 0) {
            ROS_WARN_STREAM("[MissionFSM] topo_block_fallback tier=tier1_replan target_obj_id="
                            << fd_->object_target_id_ << " stuck_duration=" << stuck_duration);
            ROS_WARN("[Targ Obj] Stuck tier1: force topo replan (clear blocked + regenerate path)");
            scene_graph_->clearAllBlocked();
            scene_graph_->mountCurPoly(fd_->odom_pos_, fd_->odom_yaw_);
            if (scene_graph_->getPathToObjectWithId(fd_->object_target_id_,
                    fd_->path_res_, fd_->aim_pos_, fd_->aim_yaw_)) {
              ROS_INFO_STREAM("[MissionFSM] topo_block_fallback_result success=1 tier=tier1_replan path_size="
                              << fd_->path_res_.size());
              ROS_INFO_STREAM("[MissionFSM] object_path_topo_sequence target_obj_id=" << fd_->object_target_id_
                              << " source_task_id=" << static_cast<int>(active_instruction_task_id_)
                              << " task_session_id=" << static_cast<int>(active_instruction_session_id_)
                              << " reason=topo_block_fallback_tier1_replan"
                              << " path_size=" << fd_->path_res_.size()
                              << " topo_sequence=" << formatTopoPath(fd_->path_res_));
              INFO_MSG_GREEN("[Targ Obj] Stuck tier1: new path found, size: " << fd_->path_res_.size());
              fd_->path_inx_ = 0;
              getAndPublishNextAim(fd_->path_res_, true, fd_->aim_yaw_);
              displayPath();
              fd_->stuck_force_advance_count_++;
              fd_->stuck_force_advance_triggered_ = true;
              fd_->stuck_begin_time_ = -1.0;
              fd_->last_pub_time_ = ros::Time::now();
            } else {
              ROS_WARN("[MissionFSM] topo_block_fallback_result success=0 tier=tier1_replan fallback=tier2_force_advance");
              // tier1 重规划失败 → 跳过tier1直接进入tier2逻辑
              ROS_WARN("[Targ Obj] Stuck tier1 failed (no path), fallback to tier2");
              fd_->stuck_force_advance_count_ = 1;  // 直接标记为已消耗tier1配额
              // 不设置triggered, 让下一轮立即进入tier2判定
            }
          } else {
            // Tier2: 二次卡死 → 逐点强制推进(当前逻辑)
            if (fd_->path_inx_ < (int)fd_->path_res_.size() - 1) {
              fd_->path_inx_++;
              fd_->stuck_force_advance_count_++;
              fd_->stuck_force_advance_triggered_ = true;
              fd_->stuck_begin_time_ = -1.0;
              ROS_WARN_STREAM("[MissionFSM] object_topo_waypoint_forced_advance target_obj_id="
                              << fd_->object_target_id_
                              << " source_task_id=" << static_cast<int>(active_instruction_task_id_)
                              << " task_session_id=" << static_cast<int>(active_instruction_session_id_)
                              << " path_index=" << fd_->path_inx_
                              << " path_size=" << fd_->path_res_.size()
                              << " reason=topo_block_fallback_tier2"
                              << " count=" << fd_->stuck_force_advance_count_
                              << " odom_pos=" << fd_->odom_pos_.transpose());
              getAndPublishNextAim(fd_->path_res_, true, fd_->aim_yaw_);
              fd_->last_pub_time_ = ros::Time::now();
              ROS_WARN_STREAM("[MissionFSM] topo_block_fallback tier=tier2_force_advance path_index="
                              << fd_->path_inx_ << " count=" << fd_->stuck_force_advance_count_);
              ROS_WARN("[Targ Obj] Stuck tier2: force advance path_inx=%d, count=%d",
                       fd_->path_inx_, fd_->stuck_force_advance_count_);
            } else {
              // 已是最后一个点无法再推进 → 走全局重规划
              ROS_WARN("[Targ Obj] Stuck at final waypoint, forced replan");
              fd_->go_object_process_phase = 0;
              transitState(WAIT_TRIGGER, "stuck at final waypoint, forced replan");
              return;
            }
          }
        }
      } else {
        fd_->stuck_begin_time_ = -1.0;             // 有运动, 重置计时器
        fd_->stuck_force_advance_triggered_ = false; // 运动表示脱离原卡死状态, 允许下次再触发
      }
    }

    // Close to aim, rotate yaw (仅 require_final_yaw_=true 时执行)
    if (fp_->object_id_nav_require_final_yaw_ &&
        (fd_->path_inx_ >= fd_->path_res_.size() - 1 || fd_->path_res_.size() == 2) &&
        dis_2_aim_2d < fp_->radius_close_ && !fd_->has_rotated_ && fd_->ego_exec_finished_){
      INFO_MSG_GREEN("[TARG Obj] [Rotate Yaw] yaw: " << fd_->odom_yaw_ << ", target yaw: " << fd_->aim_yaw_
          << ", err : " << (fd_->odom_yaw_ - fd_->aim_yaw_) / 3.14 * 180.0f << "deg");

      auto cur_obj = scene_graph_->object_factory_->object_map_[fd_->object_target_id_];
      Eigen::Vector3d target_pos = fd_->path_res_.back();
      target_pos[2] =  adjustTerminateHeightFindingObject(cur_obj, fd_->aim_pos_, true);
      pubLocalGoal(target_pos, fd_->aim_yaw_, false,
                   quadrotor_msgs::LocalGoalSet::YAW_MODE_LOW_SPEED);
      fd_->has_rotated_ = true;
      return;
    }

    // Local goal. With the SUPER backend, waypoint progress is determined inside SUPER
    // and fed back via waypoint_progress; the EGO backend keeps distance-based advance.
    if (fd_->path_res_.size() > 2 && dis_2_local_aim < 1.5){
      if (fd_->path_inx_ == fd_->path_res_.size() - 1 && dis_2_local_aim < 1.0
          && fd_->ego_exec_finished_ && fd_->ego_modify_status_) {
        INFO_MSG_YELLOW("[TARG Obj] Force Replan, because local goal can't reach!");
        fd_->go_object_process_phase = 0;
        transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "can't reach local goal");
        return ;
      }
      if (!use_super_backend_) {
        ROS_INFO_STREAM("[MissionFSM] object_topo_waypoint_reached target_obj_id=" << fd_->object_target_id_
                        << " source_task_id=" << static_cast<int>(active_instruction_task_id_)
                        << " task_session_id=" << static_cast<int>(active_instruction_session_id_)
                        << " reached_path_index=" << fd_->path_inx_
                        << " path_size=" << fd_->path_res_.size()
                        << " is_final_path_index="
                        << (fd_->path_res_.empty() ? 0 : (fd_->path_inx_ >= static_cast<int>(fd_->path_res_.size()) - 1))
                        << " dis_2_local_aim=" << dis_2_local_aim
                        << " reach_thresh=1.5"
                        << " ego_exec_finished=" << fd_->ego_exec_finished_
                        << " ego_plan_status=" << fd_->ego_plan_status_
                        << " ego_modify_status=" << fd_->ego_modify_status_
                        << " odom_pos=" << fd_->odom_pos_.transpose()
                        << " local_aim=" << fd_->local_aim_pos_.transpose());
        getAndPublishNextAim(fd_->path_res_, true, fd_->aim_yaw_);
        fd_->stuck_force_advance_count_ = 0;       // 正常推进时重置卡死强制推进计数
        fd_->stuck_force_advance_triggered_ = false;
        fd_->object_id_nav_replan_stuck_begin_time_ = -1.0;  // 正常推进时重置新replan卡死计时
        fd_->object_id_nav_replan_stuck_count_ = 0;           // 正常推进时重置replan计数
        fd_->last_pub_time_ = ros::Time::now();
      }
    }
  }
}


void MissionFSM::goTargetWithWaypoint() {
  if (fd_->go_waypoint_process_phase == 0) {
    scene_graph_->mountCurPoly(fd_->odom_pos_, fd_->odom_yaw_);

    if (scene_graph_->getCurPoly() == nullptr) {
      fd_->go_waypoint_process_phase = 0;
      transitState(WAIT_TRIGGER, "** FIND WAYPOINT TOPO FAILED: CUR POLY NULL **");
      return;
    }

    auto target_poly = scene_graph_->skeleton_gen_->mountCurTopoPoint(fd_->waypoint_target_, true);
    if (target_poly == nullptr) {
      fd_->go_waypoint_process_phase = 0;
      transitState(WAIT_TRIGGER, "** FIND WAYPOINT TOPO FAILED: TARGET POLY NULL **");
      return;
    }

    fd_->path_res_.clear();
    const double topo_dis =
        scene_graph_->skeleton_gen_->astarSearch(scene_graph_->getCurPoly(), target_poly, fd_->path_res_);
    if (topo_dis >= 99999.0 || fd_->path_res_.empty()) {
      fd_->go_waypoint_process_phase = 0;
      transitState(WAIT_TRIGGER, "** FIND WAYPOINT TOPO PATH FAILED **");
      return;
    }

    if ((fd_->path_res_.back() - fd_->waypoint_target_).norm() > 1e-3) {
      fd_->path_res_.push_back(fd_->waypoint_target_);
    }

    fd_->aim_pos_ = fd_->waypoint_target_;
    fd_->aim_yaw_ = fd_->waypoint_target_yaw_;

    INFO_MSG_GREEN("[Targ Wpt] | find path to waypoint success, size: " << fd_->path_res_.size());
    getAndPublishNextAim(fd_->path_res_, true, 0.0f);
    fd_->path_inx_      = 0;
    fd_->has_rotated_   = false;
    fd_->last_pub_time_ = ros::Time::now();
    INFO_MSG("[Targ Wpt] | PubNxtLocalAim, aim: " << fd_->local_aim_pos_ << ", global aim: " << fd_->aim_pos_);

    displayPath();
    fd_->go_waypoint_process_phase++;
  }

  if (fd_->go_waypoint_process_phase == 1) {
    double dis_2_aim_2d    = (fd_->aim_pos_       - fd_->odom_pos_).head(2).norm();
    double dis_2_local_aim = (fd_->local_aim_pos_ - fd_->odom_pos_).norm();
    double dis_yaw         = abs(fd_->aim_yaw_ - fd_->odom_yaw_);
    double t_cur = (ros::Time::now() - fd_->last_pub_time_).toSec();
    std::string ego_plan_status_str_   = fd_->ego_plan_status_ ? "True" : "False";
    std::string ego_modify_status_str_ = fd_->ego_modify_status_ ? "True" : "False";
    ROS_INFO_STREAM_THROTTLE(0.5, "\033[1;33mApproach Waypoint...\033[0m \n"
                                  "   * Dis to Aim: " << dis_2_aim_2d << "\n"
                                  "   * Dis to LocalAim: " << dis_2_local_aim << "\n"
                                  "   * Dis to yaw: " << dis_yaw);
    ROS_INFO_STREAM_THROTTLE(0.5, "[Targ Wpt] : ego local goal -> (" << fd_->ego_local_goal_.transpose() << ")");
    ROS_INFO_STREAM_THROTTLE(0.5, "[Targ Wpt] : ego plan times: " << fd_->ego_plan_times_
                                                                  << "  ego plan statue: " << ego_plan_status_str_
                                                                  << "  ego modify status: " << ego_modify_status_str_);

    if (dis_2_aim_2d < fp_->replan_dis_thresh_ && fabs(fd_->odom_yaw_ - fd_->aim_yaw_) / 3.14 * 180.0f < 5.0) {
      ROS_WARN("-------------> Finish: [Reach Both Pos&Yaw Aim] <-------------");
      ROS_INFO_STREAM("t_cur: " << t_cur);
      fd_->go_waypoint_process_phase = 0;
      transitState(WAIT_TRIGGER, "Go Target Waypoint Finish");
      return;
    }

    if (fd_->ego_exec_finished_ && fd_->ego_modify_status_
         && (dis_2_local_aim > 1.0 || dis_yaw > 10.0f / 180.0f * M_PI)
         && fd_->path_inx_ == fd_->path_res_.size() - 1) {
      fd_->last_pub_time_ = ros::Time::now();
      ROS_WARN("-------------> RePublish LocalGoal: crash recovery, forcely rotate yaw<----------------");
      pubLocalGoal(fd_->odom_pos_, fd_->aim_yaw_, false,
                   quadrotor_msgs::LocalGoalSet::YAW_MODE_NORMAL);
    }

    if (t_cur > fp_->replan_thresh3_ && fd_->odom_vel_.norm() <= 0.1) {
      ROS_WARN("-------------> Replan: periodic call <-------------");
      ROS_WARN("t_cur: %f s", t_cur);
      fd_->go_waypoint_process_phase = 0;
      transitState(WAIT_TRIGGER, "Go Target Waypoint Replan");
      return;
    }

    if ((fd_->path_inx_ >= fd_->path_res_.size() - 1 || fd_->path_res_.size() == 2) &&
        dis_2_aim_2d < fp_->radius_close_ && !fd_->has_rotated_ && fd_->ego_exec_finished_) {
      INFO_MSG_GREEN("[TARG Wpt] [Rotate Yaw] yaw: " << fd_->odom_yaw_ << ", target yaw: " << fd_->aim_yaw_
          << ", err : " << (fd_->odom_yaw_ - fd_->aim_yaw_) / 3.14 * 180.0f << "deg");

      pubLocalGoal(fd_->aim_pos_, fd_->aim_yaw_, false,
                   quadrotor_msgs::LocalGoalSet::YAW_MODE_LOW_SPEED);
      fd_->has_rotated_ = true;
      return;
    }

    if (fd_->path_res_.size() > 2 && dis_2_local_aim < 2.0) {
      if (fd_->path_inx_ == fd_->path_res_.size() - 1 && dis_2_local_aim < 1.0
          && fd_->ego_exec_finished_ && fd_->ego_modify_status_) {
        INFO_MSG_YELLOW("[TARG Wpt] Force Replan, because local goal can't reach!");
        fd_->go_waypoint_process_phase = 0;
        transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "can't reach local goal");
        return;
      }
      // SUPER backend advances via waypoint_progress; EGO backend advances by distance.
      if (!use_super_backend_) {
        getAndPublishNextAim(fd_->path_res_, true, fd_->aim_yaw_);
        fd_->last_pub_time_ = ros::Time::now();
      }
    }
  }
}

void MissionFSM::execDFDemo() {
  if(fd_->df_demo_phase_ == 0){

    if (fd_->df_demo_target_id_ >= 0){
      fd_->df_demo_phase_ = 1;
      fd_->df_demo_mode_  = false;
      fd_->object_target_id_ = fd_->df_demo_target_id_;
      transitState(MISSION_FSM_STATE::GO_TARGET_OBJECT, "DF Demo[2] go target obj");
      return;
    }

    if (fd_->df_demo_target_id_ == -100){
      std::string prompt;
      scene_graph_->DFDemoPromptGen(prompt);
      scene_graph_->sendPrompt(scene_graph_->getCurPromptIdAndPlusOne(),
                               scene_graph::PromptMsg::PROMPT_TYPE_DF_DEMO,
                               prompt, std::chrono::seconds(10), 1);
      think_start_time_ = ros::Time::now().toSec();
      think_duration_limit_ = 10.0 * 1.0;
      transitState(MISSION_FSM_STATE::THINKING, "DF Demo[1] llm call");
      return ;
    }

    if (fd_->df_demo_target_id_ == -1){

      int cur_area_id = scene_graph_->cur_poly_->area_id_;
      if (scene_graph_->skeleton_gen_->area_handler_->areas_need_predict_[cur_area_id]){
        INFO_MSG_CYAN("[DF Demo] | current area need llm predict, reset");
        fd_->df_demo_target_id_ = -100;
        return ;
      }

      transitState(MISSION_FSM_STATE::PLAN_EXPLORE, "DF Demo[1] plan explore");
      return ;
    }
  }
}

void MissionFSM::findTerminateTarget(){
  fd_->go_object_process_phase    = 0;
  fd_->find_terminate_target_mode_ = true;

  std::string prompt;
  scene_graph_->chooseTerminateObjIdPromptGen(prompt);
  scene_graph_->sendPrompt(scene_graph_->getCurPromptIdAndPlusOne(),
                           scene_graph::PromptMsg::PROMPT_TYPE_TERMINATE_OBJ_ID,
                           prompt, std::chrono::seconds(10), 1);
  stashCurStateAndTransit(MISSION_FSM_STATE::THINKING, "llm terminate obj plan !");
  think_start_time_ = ros::Time::now().toSec();
  think_duration_limit_ = 10.0 * 1.0;
  return ;
}

double MissionFSM::adjustTerminateHeightFindingObject(ObjectNode::Ptr target_obj, Eigen::Vector3d init_pos, bool final_point){
  // 根据物体的高度，飞机观测物体的xy坐标以及理想观测角度来确定终止高度，并通过安全性检查对高度进行上下微调
  double obj_height             = target_obj->pos.z();
  double observe_angle          = 0.0f / 180.0f * M_PI; // radians
  double observe_xy_distance    = (target_obj->edge.polyhedron_father->center_.head<2>() - target_obj->pos.head<2>()).norm();
  double ideal_terminate_height = obj_height + tan(observe_angle) * observe_xy_distance; // 2.0m away in xy plane
  double ideal_poly_height      = init_pos.z();

  double adjusted_height = ideal_terminate_height;
  // 安全性检查与调整, 在当前位置不安全时尝试向上调整高度
  double height_step = 0.2; // meters
  int max_adjust_steps = 5; // 最大调整步数

  for (int i = 0; i < max_adjust_steps; ++i) {
    Eigen::Vector3d check_pos = init_pos;
    check_pos.z() = adjusted_height;
    if (map_->isInLocalMap(check_pos) &&
        map_->isVisible(fd_->odom_pos_, check_pos) &&
        map_->getInflateOccupancy(check_pos) == global_belief::MapInterface::FREE) {
      Eigen::Vector3d check_floor = check_pos;
      check_floor.z() -= 0.5;
      if(map_->getInflateOccupancy(check_floor) == global_belief::MapInterface::FREE){
        INFO_MSG_CYAN("[FSM] Adjust Terminate Height Finding Object: from " << ideal_terminate_height
          << " to " << adjusted_height << " m");
        return adjusted_height;
      }
    }
    adjusted_height += height_step; // 向上调整高度
  }

  // 如果无法找到安全高度，返回理想高度
  if(final_point){
    INFO_MSG_RED("[FSM] Cna't find safe height for terminate point, use poly height: " << ideal_poly_height << " m");
    return target_obj->edge.polyhedron_father->center_.z();
  }else{
    return ideal_poly_height;
  }
}

double MissionFSM::adjustTerminateHeightNormal(const Eigen::Vector3d& next_aim_raw){
  double ideal_terminate_height = fd_->odom_pos_.z();
  double adjusted_height        = ideal_terminate_height;
  double height_step            = 0.2;
  int max_adjust_steps          = 5;
  // TODO [gwq] height adjust for normal waypoint not finished
  return ideal_terminate_height;
}

/**
 * Select and publish the next local aim point from the global path with shortcut optimization.
 *
 * @param[in]     path_res     Global path points [m]
 * @param[in]     look_forward Whether to face the final goal yaw
 * @param[in]     aim_yaw      Target yaw when look_forward is false [rad]
 * @return True if a valid aim point was published
 */
bool MissionFSM::getAndPublishNextAim(vector<Eigen::Vector3d>& path_res,
                                              const bool look_forward, const double aim_yaw) {
  auto getLocalAim = [&](vector<Eigen::Vector3d>& path_res, int& path_inx, Eigen::Vector3d& local_goal) -> bool
  {
    for(int i = path_res.size()-1; i > path_inx; i--)
    {
      if (map_->isInLocalMap(path_res[i]) &&
          map_->isVisible(fd_->odom_pos_, path_res[i]))
      {
        Eigen::Vector3d cand = path_res[i];
        // inflate 守卫: 候选点落入膨胀层时, 先尝试 C0 投影到最近 inflate-free 且可达点;
        // 投影趋向下一个路径点(避免回飞); 投影失败才判为真障碍, 去抖标记不可达并跳过(严格不进膨胀层)
        if (map_->getInflateOccupancy(cand) == global_belief::MapInterface::OCCUPIED)
        {
          ROS_WARN_STREAM("[MissionFSM] local_occ_candidate_blocked target_obj_id=" << fd_->object_target_id_
                          << " path_index=" << i
                          << " candidate=" << cand.transpose()
                          << " odom_pos=" << fd_->odom_pos_.transpose());
          // 前向参考: 路径上更靠近目标的下一个点(末点则取自身, 退化为无方向)
          Eigen::Vector3d toward = (i + 1 < (int)path_res.size()) ? path_res[i + 1] : path_res.back();
          Eigen::Vector3d repaired;
          if (scene_graph_->projectToInflateFree(cand, toward, repaired) &&
              map_->isVisible(fd_->odom_pos_, repaired))
          {
            ROS_INFO_STREAM("[MissionFSM] local_occ_candidate_repaired target_obj_id=" << fd_->object_target_id_
                            << " path_index=" << i
                            << " original=" << path_res[i].transpose()
                            << " repaired=" << repaired.transpose()
                            << " toward=" << toward.transpose());
            cand = repaired;
            // 模式2: 修复点到toward不可直线可见时, 尝试球交会生成中间点
            if (scene_graph_->getRepairVisMode() == 2 && !map_->isVisible(repaired, toward))
            {
              Eigen::Vector3d mid;
              if (scene_graph_->findIntersectionMidpoint(repaired, toward,
                      scene_graph_->getRepairVisSphereRadius(), mid))
              {
                ROS_INFO_STREAM("[MissionFSM] local_occ_repair_midpoint target_obj_id=" << fd_->object_target_id_
                                << " path_index=" << i
                                << " midpoint=" << mid.transpose());
                // 插入中间点到路径中 repaired 和 toward 之间
                path_res.insert(path_res.begin() + i + 1, mid);
                INFO_MSG_GREEN("[EXP-FSM] :[getAndPubNextAim] mode2 insert intersection mid pt");
                { visualization_msgs::Marker mp;
                  mp.header.frame_id = "world"; mp.header.stamp = ros::Time::now();
                  mp.ns = "intersection_mid"; mp.id = 0;
                  mp.type = visualization_msgs::Marker::SPHERE;
                  mp.action = visualization_msgs::Marker::ADD;
                  mp.scale.x = mp.scale.y = mp.scale.z = 0.4;
                  mp.color.r = 0.0f; mp.color.g = 0.5f; mp.color.b = 1.0f; mp.color.a = 0.9f;
                  mp.pose.position.x = mid(0); mp.pose.position.y = mid(1); mp.pose.position.z = mid(2);
                  mp.pose.orientation.w = 1.0;
                  vis_marker_pub_.publish(mp); }
              }
              else
              {
                ROS_WARN_STREAM("[MissionFSM] local_occ_candidate_rejected reason=repair_midpoint_failed target_obj_id="
                                << fd_->object_target_id_ << " path_index=" << i
                                << " candidate=" << path_res[i].transpose());
                // 球交会失败 → 标记不可达, 跳过此点
                scene_graph_->markPolyhedronBlocked(path_res[i]);
                continue;
              }
            }
            // 发布红色方块标记修复点
            { visualization_msgs::Marker rp;
              rp.header.frame_id = "world"; rp.header.stamp = ros::Time::now();
              rp.ns = "repair_point"; rp.id = 0;
              rp.type = visualization_msgs::Marker::CUBE;
              rp.action = visualization_msgs::Marker::ADD;
              rp.scale.x = rp.scale.y = rp.scale.z = 0.3;
              rp.color.r = 1.0f; rp.color.g = 0.0f; rp.color.b = 0.0f; rp.color.a = 1.0f;
              rp.pose.position.x = repaired(0); rp.pose.position.y = repaired(1); rp.pose.position.z = repaired(2);
              rp.pose.orientation.w = 1.0;
              vis_marker_pub_.publish(rp); }
            // 插入模式: 将修复点编入拓扑图(旧节点永久丢弃, 新节点连接可见邻居后永久可用)
            scene_graph_->insertReplacementNode(path_res[i], repaired);
          }
          else
          {
            ROS_WARN_STREAM("[MissionFSM] local_occ_candidate_rejected reason=project_to_inflate_free_failed target_obj_id="
                            << fd_->object_target_id_ << " path_index=" << i
                            << " candidate=" << path_res[i].transpose());
            scene_graph_->markPolyhedronBlocked(path_res[i]);
            continue;
          }
        }
        path_inx = i;
        local_goal = cand;
        ROS_INFO_STREAM("[MissionFSM] local_goal_selected mode=visible_shortcut target_obj_id="
                        << fd_->object_target_id_ << " path_index=" << path_inx
                        << " path_size=" << path_res.size()
                        << " local_goal=" << local_goal.transpose()
                        << " odom_pos=" << fd_->odom_pos_.transpose());
        INFO_MSG_GREEN("[EXP-FSM] :[getAndPubNextAim] direct aim to local_goal");
        return true;
      }
    }
    // path_inx 记录了当前路径执行的进度
    path_inx++;
    int idx = path_inx;
    if (path_inx >= path_res.size())
    {
      path_inx -- ;
      ROS_WARN_STREAM("[MissionFSM] local_goal_selection_failed reason=path_finished path_size="
                      << path_res.size() << " path_index=" << path_inx);
      ROS_WARN_THROTTLE(1.0, "[EXP-FSM] :[getAndPubNextAim] Path exec finished");
      return false;
    }
    else
    {
      local_goal = path_res[idx];
      // 查找是否有已经接近的点，并向前搜索，保证ego planner获得的点足够远
      while ((fd_->odom_pos_ - local_goal).norm() < 0.1)
      {
        path_inx++;
        if (path_inx >= path_res.size())
        {
          ROS_WARN_STREAM("[MissionFSM] local_goal_selection_failed reason=all_remaining_points_too_close path_size="
                          << path_res.size() << " path_index=" << path_inx);
          return false;
        }
        idx = path_inx;
        local_goal = path_res[idx];
      }
      ROS_INFO_STREAM("[MissionFSM] local_goal_selected mode=next_path_point target_obj_id="
                      << fd_->object_target_id_ << " path_index=" << path_inx
                      << " path_size=" << path_res.size()
                      << " local_goal=" << local_goal.transpose()
                      << " odom_pos=" << fd_->odom_pos_.transpose());
      return true;
    }
  };

  // Choose local goal
  INFO_MSG("path_res size: " << path_res.size() << ", path_inx_: " << fd_->path_inx_);
  if(path_res.size() <= 2)   // directly aim to the svp
  {
    fd_->local_aim_pos_ = path_res.back();
    fd_->aim_pos_       = path_res.back();
    if(md_->mission_state_ == MISSION_FSM_STATE::GO_TARGET_OBJECT &&
        scene_graph_->object_factory_->object_map_.find(fd_->object_target_id_) != scene_graph_->object_factory_->object_map_.end()){

      auto cur_obj = scene_graph_->object_factory_->object_map_[fd_->object_target_id_];
      fd_->local_aim_pos_[2] =  adjustTerminateHeightFindingObject(cur_obj, fd_->local_aim_pos_, true);
      fd_->aim_pos_[2] = fd_->local_aim_pos_[2];
    }
    ROS_INFO_STREAM("[MissionFSM] local_goal_selected mode=direct_final target_obj_id="
                    << fd_->object_target_id_ << " path_size=" << path_res.size()
                    << " local_goal=" << fd_->local_aim_pos_.transpose()
                    << " aim_yaw=" << aim_yaw
                    << " look_forward=" << look_forward);
    if (use_super_backend_) {
      pubLocalGoalWindow({fd_->local_aim_pos_}, aim_yaw, look_forward);
    } else {
      pubLocalGoal(fd_->local_aim_pos_, aim_yaw, look_forward);
    }
    if (md_->mission_state_ == MISSION_FSM_STATE::GO_TARGET_OBJECT) {
      ROS_INFO_STREAM("[MissionFSM] object_topo_waypoint_command target_obj_id=" << fd_->object_target_id_
                      << " source_task_id=" << static_cast<int>(active_instruction_task_id_)
                      << " task_session_id=" << static_cast<int>(active_instruction_session_id_)
                      << " path_index=" << (path_res.empty() ? -1 : static_cast<int>(path_res.size()) - 1)
                      << " path_size=" << path_res.size()
                      << " is_final_path_index=1"
                      << " mode=direct_final"
                      << " local_goal=" << fd_->local_aim_pos_.transpose()
                      << " aim_pos=" << fd_->aim_pos_.transpose()
                      << " aim_yaw=" << aim_yaw
                      << " look_forward=" << look_forward);
    }
    std::cout << "[EXP-FM][getAndPubNextAim][look_forward = "<< look_forward << "] Pub aim:" << path_res.back().transpose() << ", yaw: " << aim_yaw << std::endl;
    return true;
  }
  else
  {
    if (getLocalAim(path_res, fd_->path_inx_, fd_->local_aim_pos_))
    {
      if(md_->mission_state_ == MISSION_FSM_STATE::GO_TARGET_OBJECT &&
        scene_graph_->object_factory_->object_map_.find(fd_->object_target_id_) != scene_graph_->object_factory_->object_map_.end()){

        auto cur_obj = scene_graph_->object_factory_->object_map_[fd_->object_target_id_];
        if(fd_->path_inx_ == path_res.size() -1){
          fd_->local_aim_pos_[2] =  adjustTerminateHeightFindingObject(cur_obj, fd_->local_aim_pos_, true);
          fd_->aim_pos_[2] = fd_->local_aim_pos_[2];
        }
      }
      if (use_super_backend_) {
        // Waypoint window: current aim + up to 2 following path points; SUPER plans A*
        // and the trajectory through all of them and reports consumption feedback.
        std::vector<Eigen::Vector3d> window;
        window.push_back(fd_->local_aim_pos_);
        for (int k = 1; k <= 2 && fd_->path_inx_ + k < static_cast<int>(path_res.size()); k++) {
          window.push_back(path_res[fd_->path_inx_ + k]);
        }
        pubLocalGoalWindow(window, aim_yaw, true);
      } else {
        pubLocalGoal(fd_->local_aim_pos_, aim_yaw, true);
      }
      if (md_->mission_state_ == MISSION_FSM_STATE::GO_TARGET_OBJECT) {
        ROS_INFO_STREAM("[MissionFSM] object_topo_waypoint_command target_obj_id=" << fd_->object_target_id_
                        << " source_task_id=" << static_cast<int>(active_instruction_task_id_)
                        << " task_session_id=" << static_cast<int>(active_instruction_session_id_)
                        << " path_index=" << fd_->path_inx_
                        << " path_size=" << path_res.size()
                        << " is_final_path_index="
                        << (path_res.empty() ? 0 : (fd_->path_inx_ >= static_cast<int>(path_res.size()) - 1))
                        << " mode=path_point"
                        << " local_goal=" << fd_->local_aim_pos_.transpose()
                        << " aim_pos=" << fd_->aim_pos_.transpose()
                        << " aim_yaw=" << aim_yaw
                        << " look_forward=1");
      }
      std::cout << "[EXP-FM][getAndPubNextAim][look_forward = 1]" << " Pub local aim: " << fd_->local_aim_pos_.transpose() << std::endl;
      return true;
    }
    return false;
  }
}

void MissionFSM::pubLocalGoal(const Eigen::Vector3d local_goal, const double yaw,
                                      const bool look_forward, const uint8_t yaw_mode,
                                      const uint8_t yaw_path_mode)
{
  // yaw-only全景命令不占用EGO位置轨迹完成标志，目标续接由odometry角度驱动。
  if (yaw_mode != quadrotor_msgs::LocalGoalSet::YAW_MODE_PANORAMA)
    fd_->ego_exec_finished_ = false;
  wp_window_active_ = false;

  quadrotor_msgs::LocalGoalSet msg;
  msg.drone_id = md_->drone_id_;
  msg.source_task_id = active_instruction_task_id_;
  msg.goal[0] = static_cast<float>(local_goal.x());
  msg.goal[1] = static_cast<float>(local_goal.y());
  msg.goal[2] = static_cast<float>(local_goal.z());
  msg.look_forward = look_forward;
  msg.yaw = yaw;
  msg.yaw_low_speed = yaw_mode == quadrotor_msgs::LocalGoalSet::YAW_MODE_LOW_SPEED;
  msg.yaw_mode = yaw_mode;
  msg.yaw_path_mode = yaw_path_mode;
  ego_goal_pub_.publish(msg);
  ROS_INFO_STREAM("[MissionFSM] local_goal_published source_task_id=" << static_cast<int>(active_instruction_task_id_)
                  << " task_session_id=" << static_cast<int>(active_instruction_session_id_)
                  << " goal=" << local_goal.transpose()
                  << " yaw=" << yaw
                  << " look_forward=" << look_forward
                  << " yaw_mode=" << static_cast<unsigned int>(yaw_mode)
                  << " yaw_path_mode=" << static_cast<unsigned int>(yaw_path_mode));
}

void MissionFSM::pubLocalGoalWindow(const std::vector<Eigen::Vector3d>& window, const double yaw,
                                    const bool look_forward, const uint8_t yaw_mode,
                                    const uint8_t yaw_path_mode)
{
  if (window.empty()) return;
  fd_->ego_exec_finished_ = false;

  quadrotor_msgs::LocalGoalSet msg;
  msg.drone_id = md_->drone_id_;
  msg.source_task_id = active_instruction_task_id_;
  const Eigen::Vector3d& final_wp = window.back();
  msg.goal[0] = static_cast<float>(final_wp.x());
  msg.goal[1] = static_cast<float>(final_wp.y());
  msg.goal[2] = static_cast<float>(final_wp.z());
  msg.look_forward = look_forward;
  msg.yaw = yaw;
  msg.yaw_low_speed = yaw_mode == quadrotor_msgs::LocalGoalSet::YAW_MODE_LOW_SPEED;
  msg.yaw_mode = yaw_mode;
  msg.yaw_path_mode = yaw_path_mode;
  msg.batch_id = ++wp_batch_seq_;
  msg.waypoints.reserve(window.size() * 3);
  for (const auto& wp : window) {
    msg.waypoints.push_back(static_cast<float>(wp.x()));
    msg.waypoints.push_back(static_cast<float>(wp.y()));
    msg.waypoints.push_back(static_cast<float>(wp.z()));
  }
  ego_goal_pub_.publish(msg);
  wp_batch_start_inx_ = fd_->path_inx_;
  wp_window_active_ = true;
  ROS_INFO_STREAM("[MissionFSM] local_goal_window_published batch_id=" << wp_batch_seq_
                  << " source_task_id=" << static_cast<int>(active_instruction_task_id_)
                  << " task_session_id=" << static_cast<int>(active_instruction_session_id_)
                  << " window_size=" << window.size()
                  << " start_path_index=" << wp_batch_start_inx_
                  << " final_goal=" << final_wp.transpose()
                  << " yaw=" << yaw
                  << " look_forward=" << look_forward);
}

void MissionFSM::waypointProgressCallback(const quadrotor_msgs::WaypointProgressConstPtr& msg)
{
  if (!wp_window_active_ || msg->batch_id != wp_batch_seq_) return;
  ROS_INFO_STREAM("[MissionFSM] waypoint_progress_received batch_id=" << msg->batch_id
                  << " consumed_count=" << static_cast<int>(msg->consumed_count)
                  << " active_idx=" << static_cast<int>(msg->active_idx)
                  << " skipped_mask=" << static_cast<int>(msg->skipped_mask)
                  << " all_consumed=" << static_cast<int>(msg->all_consumed));
  if (msg->all_consumed) {
    wp_window_active_ = false;
    return;
  }
  const int new_inx = wp_batch_start_inx_ + static_cast<int>(msg->consumed_count);
  if (new_inx > fd_->path_inx_ && new_inx < static_cast<int>(fd_->path_res_.size())) {
    fd_->path_inx_ = new_inx;
    ROS_INFO_STREAM("[MissionFSM] waypoint_progress_advance batch_id=" << msg->batch_id
                    << " new_path_index=" << fd_->path_inx_
                    << " path_size=" << fd_->path_res_.size());
    getAndPublishNextAim(fd_->path_res_, true, fd_->aim_yaw_);
    fd_->last_pub_time_ = ros::Time::now();
    fd_->stuck_force_advance_count_ = 0;       // 正常推进时重置卡死强制推进计数
    fd_->stuck_force_advance_triggered_ = false;
    fd_->object_id_nav_replan_stuck_begin_time_ = -1.0;  // 正常推进时重置新replan卡死计时
    fd_->object_id_nav_replan_stuck_count_ = 0;           // 正常推进时重置replan计数
  }
}



int MissionFSM::callTrackPlanner(Eigen::Vector3d& aim_pose, Eigen::Vector3d& aim_vel,
                                         double& aim_yaw, vector<Eigen::Vector3d>& path_res)
{
  (void)aim_vel;
  (void)aim_yaw;
  map_->Lock();
  int res = map_->searchPath(fd_->odom_pos_, aim_pose, path_res, 0.2) ? 0 : -1;
  map_->Unlock();
  return res;
}

void MissionFSM::vlaSearchMapCallback(const ros::TimerEvent&)
{
  if (!vla_search_enabled_ || !fd_->have_odom_ || vla_search_map_ == nullptr) {
    return;
  }

  // SmallMap 独立于单次任务持续更新，使 PLACE 和 LOCAL_PLAN 使用同一帧地图语义。
  if (!vla_search_map_->update(fd_->odom_pos_)) {
    ROS_WARN_THROTTLE(5.0, "[VLA_SEARCH] SmallMap is waiting for an initialized occupancy map.");
  }
}



void MissionFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_yaw_rate_ = msg->twist.twist.angular.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  if (need_panorama_)
  {
    double yaw_delta = fd_->odom_yaw_ - panorama_last_odom_yaw_;
    while (yaw_delta > M_PI) yaw_delta -= 2 * M_PI;
    while (yaw_delta < -M_PI) yaw_delta += 2 * M_PI;

    panorama_unwrapped_yaw_ += yaw_delta;
    // 累计量表示相对起始朝向沿正方向的净变化，反向扰动会增加后续剩余角。
    panorama_accumulated_yaw_ = std::max(0.0, panorama_accumulated_yaw_ + yaw_delta);
    panorama_last_odom_yaw_ = fd_->odom_yaw_;
  }

  fd_->have_odom_ = true;
}

void MissionFSM::egoPlanResCallback(const quadrotor_msgs::EgoPlannerResultConstPtr &msg) {
  fd_->ego_local_goal_.x() = msg->planner_goal.x;
  fd_->ego_local_goal_.y() = msg->planner_goal.y;
  fd_->ego_local_goal_.z() = msg->planner_goal.z;
  fd_->ego_plan_times_     = msg->plan_times;
  fd_->ego_plan_status_    = msg->plan_status;
  fd_->ego_modify_status_  = msg->modify_status;

  // EGO 结果消息没有会话字段，只在 VLA 当前局部目标坐标匹配时消费，
  // 避免前一任务或 stopMotion 的迟到回调推进本次路径。
  if (vla_search_active_ &&
      md_->mission_state_ == MISSION_FSM_STATE::VLA_SEARCH_APPROACH &&
      vla_search_waypoint_published_ &&
      (fd_->ego_local_goal_ - fd_->local_aim_pos_).norm() <= 0.25) {
    vla_search_plan_feedback_received_ = true;
    vla_search_plan_feedback_success_ = msg->plan_status;
    if (msg->plan_status) {
      fd_->ego_exec_finished_ = false;
    }
  }
}

void MissionFSM::instructionCallback(const quadrotor_msgs::InstructionConstPtr& msg)
{
  if (msg->robot_id == md_->drone_id_) {
    ROS_INFO_STREAM("[MissionFSM] instruction_received instruction_type=" << static_cast<int>(msg->instruction_type)
                    << " source_task_id=" << static_cast<int>(msg->source_task_id)
                    << " task_session_id=" << static_cast<int>(msg->task_session_id)
                    << " target_obj_id=" << msg->target_obj_id
                    << " command=" << msg->command
                    << " state=" << md_->state_str_[md_->mission_state_]);
  }
  if (msg->robot_id != md_->drone_id_) return;
  // check recv time frequncy
  static bool ic_first_recv_flag = true;
  static ros::Time ic_last_recv_time;
  const bool bypass_freq_limit =
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_GOAL ||
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_WAYPOINT_NAV ||
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_TRACKING ||
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_OBJECT_NAV ||
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_REGULAR_EXPLORATION ||
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_VLA_SEARCH ||
      msg->instruction_type == quadrotor_msgs::Instruction::REQUEST_ALL_AREA_AND_OBJS;
  if (ic_first_recv_flag){
    ic_first_recv_flag = false;
    ic_last_recv_time = ros::Time::now();
  }else if (!bypass_freq_limit && !ic_first_recv_flag &&
            (ros::Time::now() - ic_last_recv_time).toSec() < 0.8){
    ic_last_recv_time = ros::Time::now();
    ROS_WARN_STREAM("[MissionFSM] instruction_rejected reason=frequency_limit instruction_type="
                    << static_cast<int>(msg->instruction_type)
                    << " source_task_id=" << static_cast<int>(msg->source_task_id)
                    << " task_session_id=" << static_cast<int>(msg->task_session_id));
    std::cout << "[InstructionCallback] : recv too frequent, skip once! instruction_type="
              << static_cast<int>(msg->instruction_type)
              << ", command=" << msg->command << std::endl;
    return;
  }else
    ic_last_recv_time = ros::Time::now();

  if (vla_search_active_) {
    const bool same_vla_search_session =
        msg->instruction_type == quadrotor_msgs::Instruction::TURN_VLA_SEARCH &&
        msg->source_task_id == quadrotor_msgs::Instruction::SOURCE_TASK_VLA_SEARCH &&
        msg->task_session_id == vla_search_session_id_;
    if (same_vla_search_session) {
      ROS_WARN_STREAM("[MissionFSM] instruction_rejected reason=duplicate_vla_search_session task_session_id="
                      << static_cast<int>(msg->task_session_id));
      ROS_WARN_STREAM("[VLA_SEARCH] Ignore duplicated Instruction for active session="
                      << vla_search_session_id_);
      return;
    }
    cancelVlaSearchTask("replaced_by_new_task", "received a new Instruction");
  }

  md_->instruction_ = msg->instruction_type;
  if (counting_scene_graph_ != nullptr && counting_scene_graph_->active()) {
    counting_scene_graph_->cancelSession();
  }
  active_instruction_task_id_ = msg->source_task_id;
  active_instruction_session_id_ = msg->task_session_id;
  const bool source_requires_panorama =
      msg->source_task_id == quadrotor_msgs::Instruction::SOURCE_TASK_EXPLORATION ||
      msg->source_task_id == quadrotor_msgs::Instruction::SOURCE_TASK_COUNTING;
  // 每条新Instruction先终止旧调度状态，只有下方匹配的task来源和探索指令可重新开启。
  need_panorama_ = false;
  panorama_command_active_ = false;
  wait_fresh_map_after_reset_ = false;
  fd_->instruct_directly_to_goal = false; // [gwq] 防止从turn_ego_plan状态切出的时候其他状态依旧使用强制ego规划
  if (msg->instruction_type != quadrotor_msgs::Instruction::TURN_TRACKING) {
    switchPlannerCmdMuxToEgo("instructionCallback:non_tracking");
    std::unique_lock<std::mutex> lck(mtx_);
    fd_->track_trigger_ = false;
    fd_->track_init_ = false;
    resetTrackingFinishCandidate();
    fd_->track_finish_sent_ = false;
    map_->setTarget(fd_->track_pos_, false);
  }

  // load map !
  if (msg->instruction_type == quadrotor_msgs::Instruction::TURN_LOAD_SCENE_GRAPH){
    // stopMotion();
    const bool load_ok = scene_graph_->loadMap(msg->map_folder);
    if (load_ok) {
      fd_->path_res_.clear();
      fd_->path_inx_ = 0;
      fd_->trigger_ = false;
      //fd_->regular_explore_ = false;
      fd_->df_demo_mode_ = false;
      fd_->find_terminate_target_mode_ = false;
      fd_->new_topo_need_predict_immediately_ = false;
      fd_->llm_plan_explore_counter_ = 0;
      has_made_area_decision_ = false;
      need_rotate_yaw_ = false;
      map_->resetGlobalBox();
      scene_graph_->object_factory_->runThisModule();
      scene_graph_->refreshLoadedMapVisualization();
      // 根据配置决定是否冻结场景图增量更新
      if (!enable_scene_graph_update_after_load_) {
        scene_graph_->freezeUpdate();
        INFO_MSG_GREEN("[InstructionCallback] Scene graph update FROZEN after load.");
      }
      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "instructionCallback(load scene graph)");
      INFO_MSG_GREEN("[InstructionCallback] Load scene graph snapshot succeeded.");
    } else {
      INFO_MSG_RED("[InstructionCallback] Load scene graph snapshot failed.");
    }
    INFO_MSG("\n\n");
    return ;
  }else if(msg->instruction_type == quadrotor_msgs::Instruction::REQUEST_ALL_AREA_AND_OBJS){
    INFO_MSG_CYAN("[InstructionCallback] Request all area and object info, and publish to CoPaw!");
    string scene_graph_json_str;
    scene_graph_->DFDemoPromptGen(scene_graph_json_str);
    scene_graph_->sendSceneGraphJson(scene_graph_json_str);
  }

  if (md_->mission_state_ == INIT || md_->mission_state_ == WARM_UP) return;
  vector<int> target_drone_ids, source_drone_ids;

  switch (msg->instruction_type)
  {
    case quadrotor_msgs::Instruction::TURN_VLA_SEARCH:
      resetVlaSearchContext();
      vla_search_active_ = true;
      vla_search_session_id_ = msg->task_session_id;
      vla_search_command_ = msg->command;
      if (!vla_search_enabled_) {
        vla_search_finish_reason_ = "disabled";
        vla_search_finish_detail_ = "vla_search/enable is false";
        transitState(MISSION_FSM_STATE::VLA_SEARCH_FINISH, "instructionCallback:vla_search_disabled");
      } else if (msg->source_task_id != quadrotor_msgs::Instruction::SOURCE_TASK_VLA_SEARCH) {
        vla_search_finish_reason_ = "invalid_source_task";
        vla_search_finish_detail_ = "TURN_VLA_SEARCH requires SOURCE_TASK_VLA_SEARCH";
        transitState(MISSION_FSM_STATE::VLA_SEARCH_FINISH, "instructionCallback:vla_search_invalid_source");
      } else if (msg->task_session_id == 0) {
        vla_search_finish_reason_ = "invalid_session";
        vla_search_finish_detail_ = "TURN_VLA_SEARCH requires non-zero task_session_id";
        transitState(MISSION_FSM_STATE::VLA_SEARCH_FINISH, "instructionCallback:vla_search_invalid_session");
      } else if (msg->command.empty()) {
        vla_search_finish_reason_ = "invalid_command";
        vla_search_finish_detail_ = "TURN_VLA_SEARCH requires a non-empty command";
        transitState(MISSION_FSM_STATE::VLA_SEARCH_FINISH, "instructionCallback:vla_search_invalid_command");
      } else {
        startVlaSearchTask(msg);
      }
      break;

    case quadrotor_msgs::Instruction::TURN_OBJECT_ID_NAV:
      object_id_nav_autostart_triggered_ = true;
      startObjectIdNav(msg->target_obj_id, msg->source_task_id, msg->task_session_id,
                       "instructionCallback", msg.get());
      break;

    case quadrotor_msgs::Instruction::TURN_WAYPOINT_NAV: {
      if (msg->nav_waypoint.empty()) {
        INFO_MSG_RED("[InstructionCallback]: TURN_WAYPOINT_NAV has empty nav_waypoint, switch to WAIT_TRIGGER");
        transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "instructionCallback:empty_waypoint");
        break;
      }

      const auto& nav_waypoint = msg->nav_waypoint.front();
      const double waypoint_z = std::isfinite(nav_waypoint.z) ? static_cast<double>(nav_waypoint.z) : 1.0;
      fd_->waypoint_target_ = Eigen::Vector3d(nav_waypoint.x, nav_waypoint.y, waypoint_z);
      fd_->waypoint_target_yaw_ =
          msg->nav_yaw.empty() ? fd_->odom_yaw_ : static_cast<double>(msg->nav_yaw.front());
      fd_->go_waypoint_process_phase = 0;
      fd_->find_terminate_target_mode_ = false;
      transitState(MISSION_FSM_STATE::GO_TARGET_WITH_WAYPOINT, "instructionCallback");
      break;
    }

    case quadrotor_msgs::Instruction::TURN_OBJECT_NAV:
      //fd_->regular_explore_ = false;
      if (msg->source_task_id == quadrotor_msgs::Instruction::SOURCE_TASK_COUNTING &&
          msg->task_session_id > 0) {
        counting_scene_graph_->startSession(msg->task_session_id, fd_->odom_pos_);
      }
      if (source_requires_panorama && msg->clear_local_map) {
        stopMotion();
        map_->resetOccupancyToUnknown();
        map_->resetGlobalBox();
        map_reset_update_seq_ = map_->getOccupancyUpdateSeq();
        wait_fresh_map_after_reset_ = true;
      } else if (source_requires_panorama) {
        startPanoramaRotation();
      }
      fd_->find_terminate_target_mode_ = false;
      fd_->new_topo_need_predict_immediately_ = true;
      fd_->df_demo_mode_ = false;
      fd_->target_cmd_ = msg->command;
      scene_graph_->setTargetAndPriorKnowledge(fd_->target_cmd_, fd_->prior_knowledge_);
      transitState(MISSION_FSM_STATE::LLM_PLAN_EXPLORE, "instructionCallback");
      break;

    case quadrotor_msgs::Instruction::TURN_REGULAR_EXPLORATION:
      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "instructionCallback:regular_exploration_removed");
      break;

    case quadrotor_msgs::Instruction::TURN_DF_DEMO:
      fd_->df_demo_mode_ = true;
      fd_->df_demo_phase_ = 0;
      //fd_->explore_count_ = 0;
      fd_->df_demo_target_id_ = -100;
      for (auto& area_iter : scene_graph_->skeleton_gen_->area_handler_->areas_need_predict_)
        area_iter.second = true;
      fd_->target_cmd_ = msg->command;
      scene_graph_->setTargetAndPriorKnowledge(fd_->target_cmd_, fd_->prior_knowledge_);
      transitState(MISSION_FSM_STATE::DF_DEMO, "instructionCallback");
      break;

    case quadrotor_msgs::Instruction::TURN_GOAL:
      handleGoalInstruction(msg->goal, msg->yaw, msg->look_forward, "instructionCallback:goal");
      break;

    case quadrotor_msgs::Instruction::TURN_TRACKING:
      if (!msg->enable)
      {
        std::unique_lock<std::mutex> lck(mtx_);
        fd_->track_trigger_ = false;
        fd_->track_init_ = false;
        resetTrackingFinishCandidate();
        fd_->track_finish_sent_ = false;
        switchPlannerCmdMuxToEgo("instructionCallback:tracking_disable");
        map_->setTarget(fd_->track_pos_, false);
        if (md_->mission_state_ == MISSION_FSM_STATE::PLAN_TRACK ||
            md_->mission_state_ == MISSION_FSM_STATE::APPROACH_TRACK)
        {
          stopMotion();
          transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "instructionCallback:tracking_disable");
        }
        break;
      }

      {
        std::unique_lock<std::mutex> lck(mtx_);
        const bool was_track_trigger = fd_->track_trigger_;
        fd_->track_trigger_ = true;
        if (!was_track_trigger) {
          resetTrackingFinishCandidate();
          fd_->track_finish_sent_ = false;
        }
        if (msg->has_target_position)
        {
          fd_->track_pos_ = geoPt2Vec3d(msg->target_position);
        }
        switchPlannerCmdMuxToEgo("instructionCallback:ego_tracking_enable");
        map_->setTarget(fd_->track_pos_, false);
        if (md_->mission_state_ != MISSION_FSM_STATE::PLAN_TRACK &&
            md_->mission_state_ != MISSION_FSM_STATE::APPROACH_TRACK)
        {
          transitState(MISSION_FSM_STATE::PLAN_TRACK, "instructionCallback:tracking_enable");
        }
      }

      if (!msg->global_poses.empty())
      {
        handleTrackingTarget(msg->global_poses, "instructionCallback:tracking_target",
                             msg->header.stamp, msg->header.frame_id);
      }
      break;

    case quadrotor_msgs::Instruction::TURN_SAVE_SCENE_GRAPH: {
      const bool save_ok = scene_graph_->saveMap(msg->map_folder);
      if (save_ok) {
        INFO_MSG_GREEN("[InstructionCallback] Save scene graph snapshot succeeded.");
      } else {
        INFO_MSG_RED("[InstructionCallback] Save scene graph snapshot failed.");
      }
      break;
    }

    default:
      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "instructionCallback");
      INFO_MSG_RED("[InstructionCallback]: No Valid Instruction! please check, switch to WAIT_TRIGGER");
      break;
  }
}

void MissionFSM::startObjectIdNav(int target_obj_id, uint8_t source_task_id,
                                  uint32_t session_id, const std::string& reason,
                                  const quadrotor_msgs::Instruction* source_msg,
                                  bool reset_replan_count) {
  if (source_msg != nullptr) {
    fd_->stored_object_id_nav_instruction_ = *source_msg;
  } else {
    quadrotor_msgs::Instruction stored;
    stored.header.stamp = ros::Time::now();
    stored.robot_id = md_->drone_id_;
    stored.instruction_type = quadrotor_msgs::Instruction::TURN_OBJECT_ID_NAV;
    stored.source_task_id = source_task_id;
    stored.task_session_id = session_id;
    stored.target_obj_id = static_cast<uint16_t>(std::max(0, target_obj_id));
    fd_->stored_object_id_nav_instruction_ = stored;
  }

  fd_->has_stored_object_id_nav_instruction_ = true;
  fd_->object_id_nav_replan_stuck_begin_time_ = -1.0;
  fd_->object_id_nav_replan_topic_triggered_ = false;
  if (reset_replan_count) {
    fd_->object_id_nav_replan_stuck_count_ = 0;
  }

  active_instruction_task_id_ = source_task_id;
  active_instruction_session_id_ = session_id;
  fd_->object_target_id_ = target_obj_id;
  fd_->path_inx_ = 0;
  fd_->go_object_process_phase = 0;
  fd_->find_terminate_target_mode_ = false;
  scene_graph_->clearAllBlocked();
  ROS_INFO_STREAM("[MissionFSM] object_id_nav_start target_obj_id=" << target_obj_id
                  << " source_task_id=" << static_cast<int>(source_task_id)
                  << " task_session_id=" << static_cast<int>(session_id)
                  << " reason=" << reason
                  << " reset_replan_count=" << reset_replan_count);
  switchPlannerCmdMuxToEgo("startObjectIdNav:" + reason);
  transitState(MISSION_FSM_STATE::GO_TARGET_OBJECT, reason);
}

void MissionFSM::triggerObjectIdNavReplan(const std::string& reason) {
  if (!fd_->has_stored_object_id_nav_instruction_) {
    ROS_WARN("[ObjIdNavReplan] No stored instruction, cannot replan");
    transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "replan_no_stored_instruction");
    return;
  }

  const auto& msg = fd_->stored_object_id_nav_instruction_;
  fd_->object_id_nav_replan_stuck_count_++;
  ROS_WARN_STREAM("[MissionFSM] object_id_nav_replan target_obj_id=" << msg.target_obj_id
                  << " source_task_id=" << static_cast<int>(msg.source_task_id)
                  << " task_session_id=" << static_cast<int>(msg.task_session_id)
                  << " reason=" << reason
                  << " count=" << fd_->object_id_nav_replan_stuck_count_);
  ROS_WARN("[ObjIdNavReplan] Replanning (target_obj_id=%d, reason=%s, count=%d)",
           msg.target_obj_id, reason.c_str(), fd_->object_id_nav_replan_stuck_count_);

  startObjectIdNav(msg.target_obj_id, msg.source_task_id, msg.task_session_id,
                   "object_id_nav_replan:" + reason, &msg, false);
}

void MissionFSM::objectIdNavReplanCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!fp_->object_id_nav_replan_enable_) {
    return;  // 功能未启用, 忽略
  }
  if (msg->data) {
    ROS_INFO("[ObjIdNavReplan] Received /object_id_nav_replan = true");
    fd_->object_id_nav_replan_topic_triggered_ = true;
  }
}

void MissionFSM::batteryCallBack(const sensor_msgs::BatteryState msg) {
  static int trigger_time = 0;
  ROS_INFO_STREAM_THROTTLE(2.0, "[FSM] voltage: " << msg.voltage);
  if (msg.voltage < fp_->battery_thr_)
  {
    // transitMode(MISSION_MODE::HOME, "batteryCallBack");
    // transitState(MISSION_FSM_STATE::GOHOME, "batteryCallBack");
    ROS_ERROR_THROTTLE(1.0, "\n========================\n***** Battery Low *****\n========================\n");
    trigger_time++;
  }
  return;
}

void MissionFSM::stashCurStateAndTransit(MISSION_FSM_STATE new_state, string who_called) {
  stash_state_ = md_->mission_state_;
  transitState(new_state, who_called);
}

void MissionFSM::transitState(MISSION_FSM_STATE new_state, string pos_call)
{
  MISSION_FSM_STATE pre_s = md_->mission_state_;
  md_->mission_state_ = new_state;

  ROS_INFO_STREAM("[MissionFSM] state_transition from=" << md_->state_str_[pre_s]
                  << " to=" << md_->state_str_[md_->mission_state_]
                  << " reason=" << pos_call
                  << " source_task_id=" << static_cast<int>(active_instruction_task_id_)
                  << " task_session_id=" << static_cast<int>(active_instruction_session_id_));
  ROS_INFO_STREAM("\033[1;36m" << "[" << pos_call << "]: from " << md_->state_str_[pre_s]
                    << " to " << md_->state_str_[md_->mission_state_] << "\033[0m"); // 青色
}

void MissionFSM::displayPath() {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "global_path";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  for (int i = 0; i < fd_->path_res_.size(); i++) {
    geometry_msgs::Point p;
    p.x = fd_->path_res_[i](0);
    p.y = fd_->path_res_[i](1);
    p.z = fd_->path_res_[i](2);
    marker.points.push_back(p);
  }
  marker_array.markers.push_back(marker);
  vis_path_pub_.publish(marker_array);
}

void MissionFSM::displayLocalAim() {
  // 橙色SPHERE标记当前local_aim导航点, 尺寸大于路径marker(0.5m > 0.3m)
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "local_aim";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.5;
  marker.color.r = 1.0f;   // 橙色
  marker.color.g = 0.5f;
  marker.color.b = 0.0f;
  marker.color.a = 0.9f;
  marker.pose.position.x = fd_->local_aim_pos_(0);
  marker.pose.position.y = fd_->local_aim_pos_(1);
  marker.pose.position.z = fd_->local_aim_pos_(2);
  marker.pose.orientation.w = 1.0;
  vis_marker_pub_.publish(marker);
}

void MissionFSM::displayMissionState()
{
  std::string text;
  text = "[S] ";
  switch (md_->mission_state_) {
    case INIT: text += "Init"; break;
    case PLAN_EXPLORE: text += "PExplore"; break;
    case LLM_PLAN_EXPLORE: text += "LLMExplore"; break;
    case PLAN_TRACK: text += "PTrack"; break;
    case WAIT_TRIGGER: text += "WTrigger"; break;
    case WARM_UP : text += "WarmUp"; break;
    case THINKING: text += "Thinking"; break;
    case YAW_HANDLE: text += "YawHandle"; break;
    case FINISH: text += "Finish"; break;
    case APPROACH_EXPLORE: text+="ApproExplore"; break;
    case APPROACH_TRACK: text+="ApproTrack"; break;
    case GO_TARGET_OBJECT: text+="Go-Obj"; break;
    case GO_TARGET_WITH_WAYPOINT: text+="Go-Wpt"; break;
    case DF_DEMO: text+="DFDemo"; break;
    case VLA_SEARCH_PLAN_LOCAL: text+="VSwarm-Plan"; break;
    case VLA_SEARCH_WAIT_LLM: text+="VSwarm-LLM"; break;
    case VLA_SEARCH_WAIT_TARGET: text+="VSwarm-Target"; break;
    case VLA_SEARCH_APPROACH: text+="VSwarm-Approach"; break;
    case VLA_SEARCH_YAW_HANDLE: text+="VSwarm-Yaw"; break;
    case VLA_SEARCH_RECOVERY: text+="VSwarm-Recovery"; break;
    case VLA_SEARCH_FINISH: text+="VSwarm-Finish"; break;
    default: text += "Unknown"; break;
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "mission_status";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.z = 0.5;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  marker.text = text;

  marker.pose.position.x = fd_->odom_pos_(0) + 0.5;
  marker.pose.position.y = fd_->odom_pos_(1) + 0.5;
  marker.pose.position.z = fd_->odom_pos_(2) + 1.0;

  vis_marker_pub_.publish(marker);
}

void MissionFSM::visualize(const ros::TimerEvent& e)
{
  (void)e;
  displayMissionState();
}

void MissionFSM::stopMotion()
{
  pubLocalGoal(fd_->odom_pos_, fd_->odom_yaw_, true);
}

inline void MissionFSM::geoPt2Vec3d(const geometry_msgs::Point &p_in, Eigen::Vector3d &p_out) {
  p_out.x() = p_in.x; p_out.y() = p_in.y; p_out.z() = p_in.z;
}
inline void MissionFSM::vec3d2GeoPt(const Eigen::Vector3d &p_in, geometry_msgs::Point &p_out) {
  p_out.x = p_in.x(); p_out.y = p_in.y(); p_out.z = p_in.z();
}
inline geometry_msgs::Point MissionFSM::vec3d2GeoPt(const Eigen::Vector3d &p_in) {
  geometry_msgs::Point p_out;
  p_out.x = p_in.x(); p_out.y = p_in.y(); p_out.z = p_in.z();
  return p_out;
}
inline Eigen::Vector3d MissionFSM::geoPt2Vec3d(const geometry_msgs::Point &p_in) {
  Eigen::Vector3d p_out;
  p_out.x() = p_in.x; p_out.y() = p_in.y; p_out.z() = p_in.z;
  return p_out;
}
}  // namespace mission_executive
