#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <exploration_manager/mission_data.h>
#include <map_interface/map_interface.hpp>
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
#include <traj_utils/planning_visualization.h>
#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
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

namespace ego_planner {
namespace {
double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

std::string jsonEscape(const std::string& value) {
  std::string out;
  out.reserve(value.size());
  for (char c : value) {
    if (c == '\\' || c == '"') out.push_back('\\');
    out.push_back(c);
  }
  return out;
}
}  // namespace

void FastExplorationFSM::init(ros::NodeHandle& nh, const MapInterface::Ptr& map) 
{
  md_ = std::make_shared<MissionData>();
  fp_ = std::make_shared<FSMParam>();
  fd_ = std::make_shared<FSMData>();
  node_ = nh;
  /*  Fsm param  */
  nh.param("fsm/drone_id", md_->drone_id_, 0);
  md_->is_initialized_=false;//swarm info callback

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
  nh.param("fsm/auto_init_delay_sec",        fp_->auto_init_delay_sec_, 2.0);
  nh.param("fsm/scene_graph_init_forward_dist", fp_->scene_graph_init_forward_dist_, 1.8);
  nh.param("fsm/frontier_update_dt",          fp_->frontier_update_dt_, 0.5);
  if (fp_->frontier_update_dt_ < 0.05) {
    ROS_WARN("[ExploreFSM] fsm/frontier_update_dt is too small, clamp to 0.05s.");
    fp_->frontier_update_dt_ = 0.05;
  }
  nh.param("fsm/enable_yaw_scan", enable_yaw_scan_, false);
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
  nh.param("planner_cmd_mux/mode_topic",       fp_->planner_cmd_mux_mode_topic_, std::string("/planner_mux/mode"));
  nh.param("planner_cmd_mux/ego_mode",         fp_->planner_cmd_mux_ego_mode_, std::string("ego"));
  nh.param("planner_cmd_mux/elastic_mode",     fp_->planner_cmd_mux_elastic_mode_, std::string("elastic"));
  nh.param("elastic_tracker/trigger_topic",    fp_->elastic_tracker_trigger_topic_, std::string("/triger"));
  nh.param("elastic_tracker/finish_topic",     fp_->elastic_tracker_finish_topic_, std::string("/elastic_tracker/tracking_finish"));
  nh.param("elastic_tracker/stop_topic",       fp_->elastic_tracker_stop_topic_, std::string("/elastic_tracker/stop"));

  std::cout << "\n***** Target Cmd : " << fd_->target_cmd_ << "\n" << std::endl;
  std::cout << "ALL Main FSM Params loaded successfully ..." << std::endl;


  fd_->home_pos_ << 0.0, 0.0, 1.0; // TODO
  fd_->ego_exec_finished_ = true;

  /* Initialize main modules */
  map_ = map;
  visualization_      = std::make_shared<PlanningVisualization>(nh);
  scene_graph_        = std::make_shared<SceneGraph>(nh, map_);
  counting_scene_graph_ = std::make_shared<CountingSceneGraph>(nh);
  expl_manager_       = std::make_shared<FrontierManager>(nh, map, scene_graph_);
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

  /* Ros sub, pub and timer */
  exec_timer_      = nh.createTimer(ros::Duration(0.1), &FastExplorationFSM::FSMCallback, this);
  frontier_timer_  = nh.createTimer(ros::Duration(fp_->frontier_update_dt_), &FastExplorationFSM::frontierCallback, this);

  // vis_timer_       = nh.createTimer(ros::Duration(0.2), &FastExplorationFSM::visualize, this); // [gwq] has thread problem! Don't turn on!

  instruction_sub_ = nh.subscribe("/bridge/Instruct", 10, &FastExplorationFSM::instructionCallback, this, ros::TransportHints().tcpNoDelay());
  odom_sub_        = nh.subscribe("odom_world",  1, &FastExplorationFSM::odometryCallback, this, ros::TransportHints().tcpNoDelay());
  battery_sub_     = nh.subscribe("/mavros/battery", 10, &FastExplorationFSM::batteryCallBack, this, ros::TransportHints().tcpNoDelay());
  ego_plan_res_sub_= nh.subscribe("/planning/ego_plan_result", 100, &FastExplorationFSM::egoPlanResCallback, this, ros::TransportHints().tcpNoDelay());
  trigger_sub_     = nh.subscribe("/move_base_simple/goal", 2, &FastExplorationFSM::triggerCallback, this, ros::TransportHints().tcpNoDelay());
  egoplanner_goal_sub_ = nh.subscribe("/goal_with_id_from_station", 2, &FastExplorationFSM::egoPlannerGoalCallback, this, ros::TransportHints().tcpNoDelay());
  ego_exec_finish_sub_ = nh.subscribe("exec_finish_trigger", 10, &FastExplorationFSM::egoExecFinishCallback, this, ros::TransportHints().tcpNoDelay());
  track_command_sub_ = nh.subscribe("/planning/track_command", 2, &FastExplorationFSM::trackCommandCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  target_sub_ = nh.subscribe("/tracking_target", 2, &FastExplorationFSM::targetCallbackReal, this,
                             ros::TransportHints().tcpNoDelay());
  elastic_tracking_finish_sub_ = nh.subscribe(fp_->elastic_tracker_finish_topic_, 10,
                                              &FastExplorationFSM::elasticTrackingFinishCallback, this,
                                              ros::TransportHints().tcpNoDelay());

  ego_goal_pub_         = nh.advertise<quadrotor_msgs::EgoGoalSet>("local_goal", 10);
  goal_from_station_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/goal_with_id_from_station", 10);
  vis_marker_pub_       = nh.advertise<visualization_msgs::Marker>("planning/fsm_vis", 10);
  vis_path_pub_         = nh.advertise<visualization_msgs::MarkerArray>("planning/fsm_path", 10);
  perception_data_pub_  = nh.advertise<quadrotor_msgs::PerceptionMsg>("/perception_data_to_bridge", 10);
  instruction_resp_pub_ = nh.advertise<quadrotor_msgs::InstructionResMsg>("/Instruct_res", 10);

  fsm_state_pub_        = nh.advertise<std_msgs::String>("/planner/fsm_state", 10);
  tracking_finish_pub_  = nh.advertise<std_msgs::Bool>("/tracking_finish", 10);
  tracking_target_odom_pub_ = nh.advertise<nav_msgs::Odometry>(fp_->tracking_target_odom_topic_, 10);
  planner_cmd_mux_mode_pub_ = nh.advertise<std_msgs::String>(fp_->planner_cmd_mux_mode_topic_, 10, true);
  elastic_tracker_trigger_pub_ = nh.advertise<geometry_msgs::PoseStamped>(fp_->elastic_tracker_trigger_topic_, 10);
  elastic_tracker_stop_pub_ = nh.advertise<std_msgs::Empty>(fp_->elastic_tracker_stop_topic_, 10);
  exploration_result_pub_ = nh.advertise<std_msgs::String>("/planning/exploration_result", 10);

  switchPlannerCmdMuxToEgo("fsm_init");
}

void FastExplorationFSM::applyExplorationRegionFromInstruction(const quadrotor_msgs::InstructionConstPtr& msg)
{
  std::vector<Eigen::Vector3d> polygon;
  if (msg->has_exploration_region && msg->exploration_region.size() >= 3) {
    polygon.reserve(msg->exploration_region.size());
    for (const auto& point : msg->exploration_region) {
      polygon.emplace_back(point.x, point.y, point.z);
    }
    expl_manager_->setExplorationRegion(polygon, true);
    return;
  }
  expl_manager_->setExplorationRegion(polygon, false);
}

void FastExplorationFSM::publishExplorationResult(bool success, const std::string& reason,
                                                  const std::string& message)
{
  // Counting 专用对象图必须先冻结并发布，确保上层收到 exploration finished 时
  // 对应 session 的 JSON 已经进入 ROS 发布队列。
  if (counting_scene_graph_ != nullptr && counting_scene_graph_->active()) {
    counting_scene_graph_->finishSessionAndPublish();
  }

  std_msgs::String msg;
  std::ostringstream ss;
  ss << "{"
     << "\"finished\":true,"
     << "\"success\":" << (success ? "true" : "false") << ","
     << "\"reason\":\"" << jsonEscape(reason) << "\","
     << "\"message\":\"" << jsonEscape(message) << "\","
     << "\"instruction_type\":" << static_cast<int>(md_->instruction_) << ","
     << "\"task_session_id\":" << active_instruction_session_id_ << ","
     << "\"command\":\"" << jsonEscape(fd_->target_cmd_) << "\","
     << "\"has_region\":" << (expl_manager_->hasExplorationRegion() ? "true" : "false") << ","
     << "\"state\":\"" << md_->state_str_[md_->mission_state_] << "\""
     << "}";
  msg.data = ss.str();
  exploration_result_pub_.publish(msg);
}

void FastExplorationFSM::triggerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  fd_->trigger_ = true;
}

void FastExplorationFSM::handleGoalInstruction(const std::vector<geometry_msgs::Point>& goals,
                                               const std::vector<float>& yaws,
                                               bool look_forward,
                                               const std::string& source) {
  switchPlannerCmdMuxToEgo(source + ":goal");
  stopElasticTracker(source + ":goal");

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
    if (!useElasticTrackerBackend()) {
      map_->setTarget(fd_->track_pos_, false);
    }
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
      quadrotor_msgs::EgoGoalSet::YAW_MODE_NORMAL);
}

void FastExplorationFSM::handleTrackingTarget(const std::vector<geometry_msgs::Point>& global_poses,
                                              const std::string& source,
                                              const ros::Time& stamp,
                                              const std::string& frame_id) {
  expl_manager_->vis_ptr_->visualize_a_ball(fd_->track_pos_, 0.35, "track_pos", visualization::Color::blue);
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
  if (!fd_->track_init_ || min_dist < expl_manager_->ep_->track_detect_error_) {
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

  if (useElasticTrackerBackend()) {
    publishTrackingTargetOdom(fd_->track_pos_, stamp, frame_id);
    switchPlannerCmdMuxToElastic("tracking_target_update");
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

void FastExplorationFSM::egoPlannerGoalCallback(const quadrotor_msgs::GoalSet::ConstPtr &msg) {
  handleGoalInstruction(msg->goal, msg->yaw, msg->look_forward, "goalFromStation");
}

void FastExplorationFSM::egoExecFinishCallback(const std_msgs::Bool::ConstPtr &msg) {
  fd_->ego_exec_finished_ = msg->data;
  INFO_MSG_GREEN("--------- [FSM] EGO-Planner Execution Finished -----------");
}

void FastExplorationFSM::trackCommandCallback(const quadrotor_msgs::TrackCommand::ConstPtr& msg) {
  if (msg->robot_id != md_->drone_id_) return;

  std::unique_lock<std::mutex> lck(mtx_);
  if (!msg->enable)
  {
    fd_->track_trigger_ = false;
    fd_->track_init_ = false;
    resetTrackingFinishCandidate();
    fd_->track_finish_sent_ = false;
    switchPlannerCmdMuxToEgo("trackCommand:disable");
    stopElasticTracker("trackCommand:disable");
    if (!useElasticTrackerBackend()) {
      map_->setTarget(fd_->track_pos_, false);
    }
    if (md_->mission_state_ == MISSION_FSM_STATE::PLAN_TRACK ||
        md_->mission_state_ == MISSION_FSM_STATE::APPROACH_TRACK)
    {
      stopMotion();
      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "trackCommand:disable");
    }
    return;
  }

  const bool was_track_trigger = fd_->track_trigger_;
  if (useElasticTrackerBackend() && fd_->track_finish_sent_ && !was_track_trigger) {
    ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] ignore residual Elastic-Tracker enable after finish; waiting for disable/reset.");
    return;
  }
  fd_->track_trigger_ = true;
  if (!was_track_trigger) {
    resetTrackingFinishCandidate();
    fd_->track_finish_sent_ = false;
  }
  if (msg->has_target_position)
  {
    fd_->track_pos_ = geoPt2Vec3d(msg->target_position);
  }
  if (useElasticTrackerBackend()) {
    if (msg->has_target_position) {
      fd_->track_init_ = true;
      publishTrackingTargetOdom(fd_->track_pos_, msg->header.stamp, msg->header.frame_id);
    }
    // Elastic-Tracker 的 /triger 表示新会话开始；持续重发会清空 traj_server 缓存并重置 hover 完成计时。
    if (!was_track_trigger) {
      publishElasticTrackerTrigger(msg->header.stamp, msg->header.frame_id);
    }
    switchPlannerCmdMuxToElastic("trackCommand:elastic_tracker_enable");
    if (md_->mission_state_ != MISSION_FSM_STATE::WAIT_TRIGGER) {
      stopMotion();
      transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "trackCommand:elastic_tracker_enable");
    }
    return;
  }

  switchPlannerCmdMuxToEgo("trackCommand:ego_tracking_enable");
  map_->setTarget(fd_->track_pos_, false);
  transitState(MISSION_FSM_STATE::PLAN_TRACK, "trackCommand:enable");
}

void FastExplorationFSM::elasticTrackingFinishCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!msg->data || !useElasticTrackerBackend()) {
    return;
  }

  std::unique_lock<std::mutex> lck(mtx_);
  if (!fd_->track_trigger_) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[TRACK] Ignore Elastic-Tracker finish without active tracking session.");
    return;
  }

  fd_->track_trigger_ = false;
  fd_->track_init_ = false;
  resetTrackingFinishCandidate();
  stopMotion();
  switchPlannerCmdMuxToEgo("elasticTrackingFinish");
  publishTrackingFinish();
  if (md_->mission_state_ != MISSION_FSM_STATE::INIT &&
      md_->mission_state_ != MISSION_FSM_STATE::WARM_UP) {
    transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "elasticTrackingFinish");
  }
}

void FastExplorationFSM::targetCallbackReal(const quadrotor_msgs::DetectOut::ConstPtr& msg)
{
  handleTrackingTarget(msg->global_poses, "trackTargetUpdate", msg->header.stamp, msg->header.frame_id);
}

bool FastExplorationFSM::getSceneGraphInitSeed(Eigen::Vector3d& init_seed, std::string* reason) const {
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

  if (map_->getInflateOccupancy(init_seed) == MapInterface::OCCUPIED) {
    if (reason != nullptr) *reason = "seed in occupied region";
    return false;
  }

  return true;
}

void FastExplorationFSM::handelThingkingProcess() {
  double t_cur = ros::Time::now().toSec() - think_start_time_;
  if (stash_state_ == MISSION_FSM_STATE::THINKING && md_->mission_state_ != MISSION_FSM_STATE::WAIT_TRIGGER && !fd_->df_demo_mode_) {
    transitState(LLM_PLAN_EXPLORE, "** fsm RECOVER !!!!!!!");
    stash_state_ = LLM_PLAN_EXPLORE;
  }

  if (t_cur > think_duration_limit_) {
    ROS_ERROR("thinking overtime ! skip this step ...");
    transitState(stash_state_, "thinking overtime ! skip this step ...");
    return ;
  }

  expl_manager_->visualize(fd_->odom_pos_);
  unsigned cur_prompt_id = scene_graph_->wait_recv_id_;

  if (scene_graph_->llm_ans_str_poll_.find(cur_prompt_id) != scene_graph_->llm_ans_str_poll_.end()) {
    INFO_MSG_GREEN("LLM ANS \n" << scene_graph_->llm_ans_str_poll_[cur_prompt_id]);

    if (scene_graph_->llm_prompts_[cur_prompt_id].prompt_type == scene_graph::PromptMsg::PROMPT_TYPE_ROOM_PREDICTION) {
      scene_graph_->handleRoomPredictionResult(cur_prompt_id);

    }else if (scene_graph_->llm_prompts_[cur_prompt_id].prompt_type == scene_graph::PromptMsg::PROMPT_TYPE_EXPL_PREDICTION) {
      expl_area_id_ = scene_graph_->handelExplorationResult(cur_prompt_id);
      if (expl_area_id_ == -100) {
        transitState(MISSION_FSM_STATE::FIND_TERMINATE_TARGET, "Have found correct area !");
        return ;
      }
      if (scene_graph_->skeleton_gen_->area_handler_->area_map_.find(expl_area_id_) != scene_graph_->skeleton_gen_->area_handler_->area_map_.end()) {
        has_made_area_decision_ = (expl_area_id_ != -1);
      }

    }else if(scene_graph_->llm_prompts_[cur_prompt_id].prompt_type == scene_graph::PromptMsg::PROMPT_TYPE_TERMINATE_OBJ_ID){
      fd_->object_target_id_ = scene_graph_->handelTerminateObjIdResult(cur_prompt_id);
      if (fd_->object_target_id_ >= 0) {
        transitState(MISSION_FSM_STATE::GO_TARGET_OBJECT, "Recv Terminate Object ID !");
        return ;
      }else {
        publishExplorationResult(false, "target_not_found", "terminate object id result is invalid");
        transitState(MISSION_FSM_STATE::FINISH, "Recv Terminate Object ID Failed !");
        return ;
      }

    }else if(scene_graph_->llm_prompts_[cur_prompt_id].prompt_type == scene_graph::PromptMsg::PROMPT_TYPE_DF_DEMO){
      fd_->df_demo_target_id_ = scene_graph_->handelDFDemoResult(cur_prompt_id);
      transitState(MISSION_FSM_STATE::DF_DEMO, "Recv DF Demo Result !");
      return ;
    }

    transitState(stash_state_, "THINKING :D");
  }
}

void FastExplorationFSM::planLLMExplore() {
  ROS_INFO("\033[1;31m\n\n ============== [Plan LLM Explore] ==============\033[0m");

  if (waitForFreshMapAfterReset())
    return;

  // 全景旋转优先：task_id=2/8 启动阶段先360°旋转一圈再探索
  if (need_panorama_) {
    handlePanoramaYaw();
    return;
  }

  if (need_rotate_yaw_ && !enable_yaw_scan_)
    need_rotate_yaw_ = false;

  if (need_rotate_yaw_) {
    auto yawLimit = [] (double yaw) {
      while (yaw >= M_PI) yaw -= 2 * M_PI;
      while (yaw < -M_PI) yaw += 2 * M_PI;
      return yaw;
    };
    yawhandle_yaw_raw          = yawLimit(fd_->odom_yaw_);
    yawhandle_yaw_target_left  = yawLimit(yawhandle_yaw_raw + 45 / 180.0 * M_PI);
    yawhandle_yaw_target_right = yawLimit(yawhandle_yaw_raw - 45 / 180.0 * M_PI);
    yawhandle_left_ok = yawhandle_right_ok = yawhandle_back_ok = false;
    yawhandle_left_published = yawhandle_right_published = yawhandle_back_published = false;
    stashCurStateAndTransit(MISSION_FSM_STATE::YAW_HANDLE, "rotate yaw before LLM explore");
    INFO_MSG_YELLOW("[FSM] Plan LLM Explore : No Yaw Handle ... start yaw handle process");
    return ;
  }

  scene_graph_->mountCurPoly(fd_->odom_pos_, fd_->odom_yaw_);

  if (scene_graph_->needAreaPrediction()) {
    expl_manager_->frontier_finder_->updateSceneGraphWithFtr();
    INFO_MSG_YELLOW("[FSM] Plan LLM Explore : Area Need Prediction ... start area predict process");
    std::string llm_prompt_str;
    scene_graph_->newAreaPredictionPromptGen(llm_prompt_str);
    cur_prompt_id_ = scene_graph_->getCurPromptId();
    scene_graph_->sendPrompt(scene_graph_->getCurPromptIdAndPlusOne(),
                             scene_graph::PromptMsg::PROMPT_TYPE_ROOM_PREDICTION,
                             llm_prompt_str, std::chrono::seconds(20), 1);
    stashCurStateAndTransit(MISSION_FSM_STATE::THINKING, "frontierCallback");
    think_start_time_     = ros::Time::now().toSec();
    think_duration_limit_ = 20.0 * 1.0;

    has_made_area_decision_ = false;
    return ;
  }

  if (!has_made_area_decision_) {
    INFO_MSG_GREEN("[FSM]: check if current area need more exploration ...");
    if (scene_graph_->cur_poly_ != nullptr) {
      // step1 current area need more exploration
      auto it = scene_graph_->skeleton_gen_->area_handler_->area_map_.find(scene_graph_->cur_poly_->area_id_);

      if (it != scene_graph_->skeleton_gen_->area_handler_->area_map_.end() && it->second->room_label_ == "Unknown" && it->second->num_ftrs_ > 0) {
        INFO_MSG_YELLOW("\n ********************************************************************************** ");
        INFO_MSG_YELLOW(" [Plan LLM Expl]: Current Area Need More Exploration, Go to Current Area [" << it->second->id_ << "] ");
        INFO_MSG_YELLOW(" **********************************************************************************\n");
        expl_area_id_ = it->second->id_;
        has_made_area_decision_ = true;

      } else {
        // step2 check nbr area need more exploration
        if (!it->second->nbr_area_.empty()) {
          for (auto nbr_area_id : it->second->nbr_area_) {
            auto nbr_area = scene_graph_->skeleton_gen_->area_handler_->area_map_.find(nbr_area_id.first);
            if(nbr_area != scene_graph_->skeleton_gen_->area_handler_->area_map_.end() && nbr_area->second->room_label_ == "Unknown" && nbr_area->second->num_ftrs_ > 0) {
              INFO_MSG_YELLOW("[Plan LLM Expl]: Found Nbr Area with Unknown Room Label, Go to Nbr Area [" << nbr_area->second->id_ << "] ");
              expl_area_id_ = nbr_area->second->id_;
              has_made_area_decision_ = true;
              break;
            }
          }
        }
      }
    }
  }

  if(fd_->llm_plan_explore_counter_ == 1){
    has_made_area_decision_ = true;
    auto it = scene_graph_->skeleton_gen_->area_handler_->area_map_.find(scene_graph_->cur_poly_->area_id_);
    expl_area_id_ = it->second->id_;
    INFO_MSG_YELLOW("*** [FSM] Plan LLM Explore : Regular explore ...");
  }

  if (!has_made_area_decision_) {
    INFO_MSG_YELLOW("[FSM] Plan LLM Explore : No Area Decision Made ... start llm-exploration process");
    scene_graph_->history_visited_area_ids_.push_back(scene_graph_->cur_poly_->area_id_);

    std::string prompt;
    scene_graph_->chooseAreaToGoPromptGen(prompt);
    scene_graph_->sendPrompt(scene_graph_->getCurPromptIdAndPlusOne(),
                             scene_graph::PromptMsg::PROMPT_TYPE_EXPL_PREDICTION,
                             prompt, std::chrono::seconds(10), 1);
    stashCurStateAndTransit(MISSION_FSM_STATE::THINKING, "llm explore plan!");
    think_start_time_     = ros::Time::now().toSec();
    think_duration_limit_ = 10.0 * 1.0;
    return ;
  }

  fd_->path_res_.clear();
  int res = callExplorationLLMPlanner(fd_->aim_pos_, fd_->aim_vel_, fd_->aim_yaw_, fd_->path_res_);

  fd_->llm_plan_explore_counter_ ++;
  if(fd_->llm_plan_explore_counter_ >= 2){
    has_made_area_decision_ = false;
    fd_->llm_plan_explore_counter_ = 0;
  }

  if (res == NO_FRONTIER) {
    has_made_area_decision_ = false;
    publishExplorationResult(false, "target_not_found", "target area has no frontier");
    transitState(FINISH, "LLM Plan No Frontier");
    return;

  } else if (res == FAIL) {
    has_made_area_decision_ = false;
    planRegularExplore();
    INFO_MSG_RED(" !!!!!!!!!!!!!!!!!!!! LLM Plan Explore Failed !!!!!!!!!!!!!!!!!!!!!!");
    INFO_MSG_RED("                     PLan Regular Explore Once ");
    return ;

  }else {
    has_made_area_decision_ = false;
    double dis_2_aim       = (fd_->aim_pos_ - fd_->odom_pos_).norm();
    double dis_2_aim_2d    = (fd_->aim_pos_ - fd_->odom_pos_).head(2).norm();
    bool look_forward      = dis_2_aim >= expl_manager_->ep_->radius_close_;
    fd_->path_inx_ = 0;

    getAndPublishNextAim(fd_->path_res_, look_forward, fd_->aim_yaw_);
    fd_->is_lookforward_ = look_forward;
    fd_->has_rotated_ = !look_forward;
    fd_->last_pub_time_ = ros::Time::now();
    INFO_MSG_GREEN("[EXP-FSM] [look_forward = " << look_forward << "] aim: " << fd_->aim_pos_.transpose() << ", local_aim: " << fd_->local_aim_pos_.transpose());
    transitState(APPROACH_EXPLORE, "LLM Plan Success");
  }
}

void FastExplorationFSM::planRegularExplore() {
  ROS_INFO("\033[1;31mPlan Regular Explore!\033[0m");  //红

  if (waitForFreshMapAfterReset())
    return;

  // 全景旋转优先：task_id=2/8 启动阶段先360°旋转一圈再探索
  if (need_panorama_) {
    handlePanoramaYaw();
    return;
  }

  if (need_rotate_yaw_ && !enable_yaw_scan_)
    need_rotate_yaw_ = false;

  if (need_rotate_yaw_ && fd_->df_demo_mode_) {
    auto yawLimit = [] (double yaw) {
      while (yaw >= M_PI) yaw -= 2 * M_PI;
      while (yaw < -M_PI) yaw += 2 * M_PI;
      return yaw;
    };
    yawhandle_yaw_raw          = yawLimit(fd_->odom_yaw_);
    yawhandle_yaw_target_left  = yawLimit(yawhandle_yaw_raw + 45.0 / 180.0 * M_PI);
    yawhandle_yaw_target_right = yawLimit(yawhandle_yaw_raw - 45.0 / 180.0 * M_PI);
    yawhandle_left_ok = yawhandle_right_ok = yawhandle_back_ok = false;
    yawhandle_left_published = yawhandle_right_published = yawhandle_back_published = false;
    stashCurStateAndTransit(MISSION_FSM_STATE::YAW_HANDLE, "rotate yaw before LLM explore");
    INFO_MSG_YELLOW("[FSM] Plan Regular Explore : No Yaw Handle ... start yaw handle process");
    need_rotate_yaw_ = false;
    return ;
  }

  fd_->path_res_.clear();
  int res = callExplorationPlanner(fd_->aim_pos_, fd_->aim_vel_, fd_->aim_yaw_, fd_->path_res_);

  if (fd_->df_demo_mode_) 
    need_rotate_yaw_ = enable_yaw_scan_;

  if (res == SUCCEED)
  {
    double dis_2_aim       = (fd_->aim_pos_ - fd_->odom_pos_).norm();
    double dis_2_aim_2d    = (fd_->aim_pos_ - fd_->odom_pos_).head(2).norm();
    bool look_forward = dis_2_aim >= expl_manager_->ep_->radius_close_;
    fd_->path_inx_ = 0;

    getAndPublishNextAim(fd_->path_res_, look_forward, fd_->aim_yaw_);
    fd_->is_lookforward_ = look_forward;
    fd_->has_rotated_ = !look_forward;
    INFO_MSG_GREEN("[EXP-FSM] [look_forward = " << look_forward << "] aim: " << fd_->aim_pos_.transpose() << ", local_aim: " << fd_->local_aim_pos_.transpose());
    fd_->last_pub_time_ = ros::Time::now();

    transitState(APPROACH_EXPLORE, "FSM");
    need_rotate_yaw_ = enable_yaw_scan_;
    // ROS_ERROR_STREAM("[Plan_Traj] start_pos: " << fd_->start_pt_.transpose());
    // ROS_ERROR_STREAM("[Plan_Traj] odom_pos: " << fd_->odom_pos_.transpose());
    // ROS_ERROR_STREAM("[Plan_Traj] start_vel: " << fd_->start_vel_.transpose());
    // ROS_ERROR_STREAM("[Plan_Traj] odom_vel: " << fd_->odom_vel_.transpose());
  }
  else if (res == NO_FRONTIER)
  {
    const std::string reason = expl_manager_->hasExplorationRegion() ? "region_explored" : "global_explored";
    publishExplorationResult(true, reason, "no frontier remains");
    transitState(FINISH, "FSM");
    // clearVisMarker();
  }
  else if (res == FAIL)
  {
    // Still in PLAN_TRAJ state, keep replanning
    ROS_ERROR("\n\n ======================== [FSM] Plan fail! ==========================");
    INFO_MSG_RED("================================================================================");
    INFO_MSG_RED("[EXPL-FSM] : Can't reach frontier, delete this frontier and add it to blacklist!");
    INFO_MSG_RED("================================================================================");
    // expl_manager_->frontier_finder_->addFtrBlacklist(expl_manager_->ed_->frontier_to_explore_.average_);
    INFO_MSG_RED("add to blacklist: " << expl_manager_->ed_->frontier_to_explore_.average_.transpose());
    expl_manager_->forceDeleteFrontier(expl_manager_->ed_->frontier_to_explore_);
  }
  expl_manager_->visualize(fd_->odom_pos_);
}

void FastExplorationFSM::approachRegularExplore() {

  double dis_2_aim       = (fd_->aim_pos_       - fd_->odom_pos_).norm();
  double dis_2_aim_2d    = (fd_->aim_pos_       - fd_->odom_pos_).head(2).norm();
  double dis_2_local_aim = (fd_->local_aim_pos_ - fd_->odom_pos_).norm();
  double dis_yaw         = fd_->aim_yaw_ - fd_->odom_yaw_;

  double t_cur = (ros::Time::now() - fd_->last_pub_time_).toSec();
  std::string ego_plan_status_str_   = fd_->ego_plan_status_ ? "True" : "False";
  std::string ego_modify_status_str_ = fd_->ego_modify_status_ ? "True" : "False";

  ROS_INFO_STREAM_THROTTLE(0.5, "\033[1;33mApproach EXPLORE...\033[0m \n"
                                "   * Dis to Aim: " << dis_2_aim_2d << "\n"
                                "   * Dis to LocalAim: " << dis_2_local_aim << "\n"
                                "   * Dis to yaw: " << dis_yaw);  // 黄
  ROS_INFO_STREAM_THROTTLE(0.5, "[EXPL-FSM] : ego local goal -> (" << fd_->ego_local_goal_.transpose() << ")");
  ROS_INFO_STREAM_THROTTLE(0.5, "[EXPL-FSM] : ego plan times: " << fd_->ego_plan_times_
                                                                << "  ego plan statue: " << ego_plan_status_str_
                                                                << "  ego modify status: " << ego_modify_status_str_);
  // ! bad frontier delete
  bool bad_frontier = false;
  Eigen::Vector3d cur_viewpoint = fd_->path_res_.back();
  if (fd_->ego_plan_times_ > 40) {
    INFO_MSG_RED("[EXPL-FSM] : replan time out, delete this frontier and add it to blacklist, replan!");
    bad_frontier = true;
  }

  if (fd_->ego_modify_status_ && fd_->ego_exec_finished_ 
      && (fd_->odom_pos_ - fd_->ego_local_goal_).norm() < 0.1
      && map_->isInLocalMap(cur_viewpoint) && t_cur > 8.0) {
    INFO_MSG_RED("[EXPL-FSM] : ego modify status, delete this frontier and add it to blacklist, replan!");
    bad_frontier = true;
  }

  // replan judgement
  MISSION_FSM_STATE replan_target_state = 
        fd_->regular_explore_ ? MISSION_FSM_STATE::PLAN_EXPLORE : MISSION_FSM_STATE::LLM_PLAN_EXPLORE;

  if (fd_->df_demo_mode_) 
    replan_target_state = MISSION_FSM_STATE::DF_DEMO;

  if (bad_frontier){
    INFO_MSG_RED("===========================================================================================");
    INFO_MSG_RED("[EXPL-FSM] : Can't reach frontier, delete this frontier and add it to blacklist, replan!");
    INFO_MSG_RED("===========================================================================================");
    // expl_manager_->frontier_finder_->addFtrBlacklist(expl_manager_->ed_->frontier_to_explore_.average_);
    INFO_MSG_RED("Ftr [" << expl_manager_->ed_->frontier_to_explore_.id_ << "] pos : "<< expl_manager_->ed_->frontier_to_explore_.average_.transpose());
    expl_manager_->forceDeleteFrontier(expl_manager_->ed_->frontier_to_explore_);
    transitState(replan_target_state, "FSM");
  }

  if (dis_2_aim_2d < fp_->replan_dis_thresh_ && fabs(fd_->odom_yaw_ - fd_->aim_yaw_) < 10.0 / 180.0f * M_PI) {
    ROS_WARN("\n-------------> Replan: [Reach Both Pos&Yaw Aim] <-------------\n");
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM("t_cur: " << t_cur);
    transitState(replan_target_state, "FSM");
    return;
  }

  // Replan after some time
  if (t_cur > fp_->replan_thresh3_ && fd_->odom_vel_.norm() <= 0.1) {
    ROS_WARN("\n-------------> Replan: periodic call <-------------\n");
    ROS_WARN("t_cur: %f s", t_cur);
    transitState(replan_target_state, "FSM");
    return;
  }

  // Close to aim, rotate yaw
  if ((fd_->path_inx_ == fd_->path_res_.size() - 1 || fd_->path_res_.size() == 2) &&
      dis_2_aim_2d < expl_manager_->ep_->radius_close_ && !fd_->has_rotated_ && fd_->ego_exec_finished_){
    INFO_MSG_CYAN("\n[Approach EXPLORE] Close to Aim Position, Rotate Yaw to Aim Yaw!\n");
    pubLocalGoal(fd_->aim_pos_, fd_->aim_yaw_, false,
                 quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED);
    fd_->has_rotated_ = true;
    INFO_MSG_GREEN("[EXP-FSM] [Rotate Yaw] aim: " << fd_->aim_pos_.transpose() << ", local_aim: " << fd_->local_aim_pos_.transpose());
    return;
  }

  // Local goal
  if (fd_->path_res_.size() > 2 && dis_2_local_aim < 2.0){
    if (fd_->path_inx_ >= fd_->path_res_.size() - 1 && 
    fd_->ego_exec_finished_ && fd_->ego_modify_status_) {
      INFO_MSG_RED("\n[Approach EXPLORE] Force Replan, because local goal can't reach!\n");
      transitState(MISSION_FSM_STATE::PLAN_EXPLORE, "can't reach local goal");
      return ;
    }
    getAndPublishNextAim(fd_->path_res_, true);
    fd_->last_pub_time_ = ros::Time::now();
    INFO_MSG_GREEN("[EXP-FSM] [PubNxtLocalAim] aim: " << fd_->aim_pos_.transpose() << ", local_aim: " << fd_->local_aim_pos_.transpose());
  }
}

void FastExplorationFSM::resetTrackingFinishCandidate() {
  fd_->track_finish_candidate_active_ = false;
  fd_->track_finish_candidate_start_time_ = ros::Time(0);
  fd_->track_finish_last_pos_ = fd_->odom_pos_;
  fd_->track_finish_last_yaw_ = fd_->odom_yaw_;
  fd_->track_finish_move_acc_ = 0.0;
  fd_->track_finish_yaw_acc_ = 0.0;
}

bool FastExplorationFSM::updateTrackingFinishCandidate(double dis_2_aim, double angle_2_aim) {
  const bool near_aim = dis_2_aim < fp_->arrive_dis_thr_;
  const bool yaw_ok = angle_2_aim < expl_manager_->ep_->track_yaw_thr_;
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

void FastExplorationFSM::publishTrackingFinish() {
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

bool FastExplorationFSM::useElasticTrackerBackend() const {
  return fp_->tracking_backend_ == "elastic_tracker" ||
         fp_->tracking_backend_ == "tracker";
}

void FastExplorationFSM::publishPlannerCmdMuxMode(const std::string& mode,
                                                  const std::string& source) {
  std_msgs::String msg;
  msg.data = mode;
  planner_cmd_mux_mode_pub_.publish(msg);
  ROS_INFO_STREAM_THROTTLE(0.5, "[planner_cmd_mux] request mode=" << mode
                           << " from " << source);
}

void FastExplorationFSM::switchPlannerCmdMuxToEgo(const std::string& source) {
  publishPlannerCmdMuxMode(fp_->planner_cmd_mux_ego_mode_, source);
}

void FastExplorationFSM::switchPlannerCmdMuxToElastic(const std::string& source) {
  publishPlannerCmdMuxMode(fp_->planner_cmd_mux_elastic_mode_, source);
}

void FastExplorationFSM::publishElasticTrackerTrigger(const ros::Time& stamp,
                                                      const std::string& frame_id) {
  geometry_msgs::PoseStamped trigger;
  trigger.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
  trigger.header.frame_id = frame_id.empty() ? "world" : frame_id;
  trigger.pose.position.x = 0.0;
  trigger.pose.position.y = 0.0;
  trigger.pose.position.z = 0.0;
  trigger.pose.orientation.w = 1.0;
  elastic_tracker_trigger_pub_.publish(trigger);
  ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] publish Elastic-Tracker trigger -> "
                           << fp_->elastic_tracker_trigger_topic_);
}

void FastExplorationFSM::stopElasticTracker(const std::string& source) {
  if (!useElasticTrackerBackend()) {
    return;
  }
  std_msgs::Empty msg;
  elastic_tracker_stop_pub_.publish(msg);
  ROS_INFO_STREAM("[TRACK] publish Elastic-Tracker stop -> "
                  << fp_->elastic_tracker_stop_topic_ << " from " << source);
}

void FastExplorationFSM::publishTrackingTargetOdom(const Eigen::Vector3d& target_pos,
                                                   const ros::Time& stamp,
                                                   const std::string& frame_id) {
  nav_msgs::Odometry target_odom;
  target_odom.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
  target_odom.header.frame_id = frame_id.empty() ? "world" : frame_id;
  target_odom.child_frame_id = "target";
  target_odom.pose.pose.position.x = target_pos.x();
  target_odom.pose.pose.position.y = target_pos.y();
  target_odom.pose.pose.position.z = target_pos.z();
  target_odom.pose.pose.orientation.w = 1.0;
  target_odom.twist.twist.linear.x = 0.0;
  target_odom.twist.twist.linear.y = 0.0;
  target_odom.twist.twist.linear.z = 0.0;
  tracking_target_odom_pub_.publish(target_odom);
  ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] publish target odom for Elastic-Tracker: "
                           << target_pos.transpose() << " -> "
                           << fp_->tracking_target_odom_topic_);
}

void FastExplorationFSM::planTrack() {
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
  fd_->aim_pos_ = fd_->track_pos_ - target_vec.normalized() * expl_manager_->ep_->track_dist_;
  fd_->aim_pos_.z() = 1.0;
  fd_->aim_yaw_ = atan2(target_vec.y(), target_vec.x());

  ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] track pos: " << fd_->track_pos_.transpose()
                           << " aim pos: " << fd_->aim_pos_.transpose()
                           << " odom pos: " << fd_->odom_pos_.transpose()
                           << " aim yaw: " << fd_->aim_yaw_);

  const double pos_err = (fd_->aim_pos_ - fd_->odom_pos_).norm();
  const double yaw_err = std::fabs(normalizeAngle(fd_->aim_yaw_ - fd_->odom_yaw_));
  if (pos_err < expl_manager_->ep_->track_replan_dist_ &&
      yaw_err < expl_manager_->ep_->track_yaw_thr_) {
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
  if (res != SUCCEED) {
    resetTrackingFinishCandidate();
    ROS_WARN_THROTTLE(1.0, "[TRACK] Tracking target not directly reachable yet.");
    return;
  }

  fd_->path_inx_ = 0;
  fd_->local_aim_pos_ = fd_->aim_pos_;

  const double dis_2_aim_2d = (fd_->aim_pos_ - fd_->odom_pos_).head(2).norm();
  // 远距离 yaw 偏差过大时提前按目标方向边飞边转；近距离 yaw-lock 保持快速转向以尽快找回目标视野。
  const bool near_yaw_lock = dis_2_aim_2d < expl_manager_->ep_->track_turn_yaw_dist_;
  const bool far_yaw_align = !near_yaw_lock && yaw_err > expl_manager_->ep_->track_fly_yaw_thr_;
  const bool look_forward = !(near_yaw_lock || far_yaw_align);
  const bool yaw_low_speed = far_yaw_align;

  pubLocalGoal(fd_->path_res_.back(), fd_->aim_yaw_, look_forward,
               yaw_low_speed ? quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED
                             : quadrotor_msgs::EgoGoalSet::YAW_MODE_NORMAL);
  resetTrackingFinishCandidate();
  INFO_MSG_GREEN("[TRACK] [look_forward = " << look_forward
                 << ", yaw_low_speed = " << yaw_low_speed
                 << ", yaw_err = " << yaw_err << "] aim: "
                 << fd_->path_res_.back().transpose() << ", yaw: " << fd_->aim_yaw_);

  fd_->has_rotated_ = near_yaw_lock;
  fd_->last_pub_time_ = ros::Time::now();
  transitState(MISSION_FSM_STATE::APPROACH_TRACK, "planTrack");
}

void FastExplorationFSM::approachTrack() {
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

  Eigen::Vector3d aim_pos_new = fd_->track_pos_ - target_vec.normalized() * expl_manager_->ep_->track_dist_;
  aim_pos_new.z() = 1.0;
  if ((fd_->aim_pos_ - aim_pos_new).norm() > expl_manager_->ep_->track_replan_dist_) {
    INFO_MSG_GREEN("[TRACK] aim_pos_old: " << fd_->aim_pos_.transpose()
                   << " aim_pos_new: " << aim_pos_new.transpose());
    resetTrackingFinishCandidate();
    transitState(MISSION_FSM_STATE::PLAN_TRACK, "approachTrack:moved");
    return;
  }

  const double current_dir = atan2(target_vec.y(), target_vec.x());
  if (!fd_->has_rotated_ && dis_2_aim_2d < expl_manager_->ep_->track_turn_yaw_dist_) {
    fd_->has_rotated_ = true;
    fd_->aim_yaw_ = current_dir;
    resetTrackingFinishCandidate();
    pubLocalGoal(fd_->aim_pos_, fd_->aim_yaw_, false,
                 quadrotor_msgs::EgoGoalSet::YAW_MODE_NORMAL);
    INFO_MSG_GREEN("[TRACK] Switch to yaw-lock, aim: " << fd_->aim_pos_.transpose()
                   << ", yaw: " << fd_->aim_yaw_);
    return;
  }

  if (fd_->has_rotated_ &&
      std::fabs(normalizeAngle(current_dir - fd_->aim_yaw_)) > expl_manager_->ep_->track_yaw_thr_) {
    fd_->aim_yaw_ = current_dir;
    resetTrackingFinishCandidate();
    pubLocalGoal(fd_->aim_pos_, fd_->aim_yaw_, false,
                 quadrotor_msgs::EgoGoalSet::YAW_MODE_NORMAL);
    INFO_MSG_GREEN("[TRACK] Update yaw-lock, aim: " << fd_->aim_pos_.transpose()
                   << ", yaw: " << fd_->aim_yaw_);
  }
}

void FastExplorationFSM::startPanoramaRotation() {
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

bool FastExplorationFSM::waitForFreshMapAfterReset() {
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

void FastExplorationFSM::handlePanoramaYaw() {
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
               quadrotor_msgs::EgoGoalSet::YAW_MODE_PANORAMA,
               quadrotor_msgs::EgoGoalSet::YAW_PATH_KEEP_DIRECTION);
  panorama_command_active_ = true;
}

void FastExplorationFSM::handleYawChange() {
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
                   quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED);
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
                   quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED);
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
                   quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED);
    }
    return ;
  }

  yawhandle_left_published  = yawhandle_right_published = yawhandle_back_published = false;
  yawhandle_left_ok         = yawhandle_right_ok        = yawhandle_back_ok        = false;
  need_rotate_yaw_          = false;
  transitState(stash_state_, "Yaw Handle Done");
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) 
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
      expl_manager_->visualize(fd_->odom_pos_);
      break;
    }

    case WAIT_TRIGGER: 
    {
      // Do nothing but wait for trigger
      if (!fd_->trigger_) {
        ROS_WARN_THROTTLE(10.0, "[EXP-FSM] Wait Trigger For Plan Explore ... ");
        return ;
      }
      fd_->trigger_ = false;
      transitState(MISSION_FSM_STATE::LLM_PLAN_EXPLORE, "FSM");
      expl_manager_->visualize(fd_->odom_pos_);
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
      ROS_INFO_STREAM_THROTTLE(0.5, "\033[1;31mTHINKING!\033[0m");  //red
      handelThingkingProcess();
      break;
    }

    case LLM_PLAN_EXPLORE: {
      if (need_panorama_ || fd_->ego_exec_finished_)
        planLLMExplore();
      break;
    }

    case PLAN_EXPLORE: {
      planRegularExplore();
      break;
    }

    case PLAN_TRACK: {
      planTrack();
      break;
    }

    case APPROACH_EXPLORE: {
      approachRegularExplore();
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

    default:{
      break;
    }

  }
}

void FastExplorationFSM::goTargetObject() {
  if (fd_->go_object_process_phase == 0) {
    scene_graph_->mountCurPoly(fd_->odom_pos_, fd_->odom_yaw_);
    if (scene_graph_->getPathToObjectWithId(fd_->object_target_id_, fd_->path_res_, fd_->aim_pos_, fd_->aim_yaw_)) {
      INFO_MSG_GREEN("[Targ Obj] | find path to object success, size: " << fd_->path_res_.size());
      getAndPublishNextAim(fd_->path_res_, true, 0.0f);
      fd_->path_inx_        = 0;
      fd_->has_rotated_     = false;
      fd_->last_pub_time_   = ros::Time::now();
      INFO_MSG("[Targ Obj] | PubNxtLocalAim, aim: " << fd_->local_aim_pos_ << ", global aim: " << fd_->aim_pos_);

      displayPath();
      fd_->go_object_process_phase ++;
    }else {
      fd_->go_object_process_phase = 0;
      if(fd_->find_terminate_target_mode_) {
        publishExplorationResult(false, "target_path_failed", "failed to plan path to target object");
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

    if (dis_2_aim_2d < fp_->replan_dis_thresh_ && fabs(fd_->odom_yaw_ - fd_->aim_yaw_) / 3.14 * 180.0f < 5.0) {
      ROS_WARN("-------------> Finish: [Reach Both Pos&Yaw Aim] <-------------");
      ROS_INFO_STREAM("t_cur: " << t_cur);
      fd_->go_object_process_phase = 0;
      if (fd_->find_terminate_target_mode_) {
        publishExplorationResult(true, "target_found", "reached target object");
        transitState(FINISH, "Find Terminate Target Finish");
      }
      else transitState(WAIT_TRIGGER, "Go Target Object Finish");
      return;
    }

    // crash recovery 1
    if (fd_->ego_exec_finished_ && fd_->ego_modify_status_ 
         && (dis_2_local_aim > 1.0 || dis_yaw > 10.0f / 180.0f * M_PI) 
         && fd_->path_inx_ == fd_->path_res_.size() - 1){
      fd_->last_pub_time_ = ros::Time::now();
      ROS_WARN("-------------> RePublish LocalGoal: crash recovery, forcely rotate yaw<----------------");
      pubLocalGoal(fd_->odom_pos_, fd_->aim_yaw_, false,
                   quadrotor_msgs::EgoGoalSet::YAW_MODE_NORMAL);
      // INFO_MSG_GREEN("[Targ Obj] [PubNxtLocalAim] aim: " << fd_->aim_pos_.transpose() << ", local_aim: " << fd_->local_aim_pos_.transpose());
    }

    // Replan after some time
    if (t_cur > fp_->replan_thresh3_ && fd_->odom_vel_.norm() <= 0.1) {
      ROS_WARN("-------------> Replan: periodic call <-------------");
      ROS_WARN("t_cur: %f s", t_cur);
      fd_->go_object_process_phase = 0;
      transitState(WAIT_TRIGGER, "Go Target Object Replan");
      return;
    }

    // Close to aim, rotate yaw
    if ((fd_->path_inx_ >= fd_->path_res_.size() - 1 || fd_->path_res_.size() == 2) &&
        dis_2_aim_2d < expl_manager_->ep_->radius_close_ && !fd_->has_rotated_ && fd_->ego_exec_finished_){
      INFO_MSG_GREEN("[TARG Obj] [Rotate Yaw] yaw: " << fd_->odom_yaw_ << ", target yaw: " << fd_->aim_yaw_
          << ", err : " << (fd_->odom_yaw_ - fd_->aim_yaw_) / 3.14 * 180.0f << "deg");
      
      auto cur_obj = scene_graph_->object_factory_->object_map_[fd_->object_target_id_];
      Eigen::Vector3d target_pos = fd_->path_res_.back();
      target_pos[2] =  adjustTerminateHeightFindingObject(cur_obj, fd_->aim_pos_, true);
      pubLocalGoal(target_pos, fd_->aim_yaw_, false,
                   quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED);
      fd_->has_rotated_ = true;
      return;
    }

    // Local goal
    if (fd_->path_res_.size() > 2 && dis_2_local_aim < 2.0){
      if (fd_->path_inx_ == fd_->path_res_.size() - 1 && dis_2_local_aim < 1.0 
          && fd_->ego_exec_finished_ && fd_->ego_modify_status_) {  
        INFO_MSG_YELLOW("[TARG Obj] Force Replan, because local goal can't reach!");
        fd_->go_object_process_phase = 0;
        transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "can't reach local goal");
        return ;
      }
      getAndPublishNextAim(fd_->path_res_, true, fd_->aim_yaw_);
      fd_->last_pub_time_ = ros::Time::now();
      // INFO_MSG_GREEN("[TARG Obj] [PubNxtLocalAim] aim: " << fd_->aim_pos_.transpose() << ", local_aim: " << fd_->local_aim_pos_.transpose());
    }
  }
}


void FastExplorationFSM::goTargetWithWaypoint() {
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
                   quadrotor_msgs::EgoGoalSet::YAW_MODE_NORMAL);
    }

    if (t_cur > fp_->replan_thresh3_ && fd_->odom_vel_.norm() <= 0.1) {
      ROS_WARN("-------------> Replan: periodic call <-------------");
      ROS_WARN("t_cur: %f s", t_cur);
      fd_->go_waypoint_process_phase = 0;
      transitState(WAIT_TRIGGER, "Go Target Waypoint Replan");
      return;
    }

    if ((fd_->path_inx_ >= fd_->path_res_.size() - 1 || fd_->path_res_.size() == 2) &&
        dis_2_aim_2d < expl_manager_->ep_->radius_close_ && !fd_->has_rotated_ && fd_->ego_exec_finished_) {
      INFO_MSG_GREEN("[TARG Wpt] [Rotate Yaw] yaw: " << fd_->odom_yaw_ << ", target yaw: " << fd_->aim_yaw_
          << ", err : " << (fd_->odom_yaw_ - fd_->aim_yaw_) / 3.14 * 180.0f << "deg");

      pubLocalGoal(fd_->aim_pos_, fd_->aim_yaw_, false,
                   quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED);
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
      getAndPublishNextAim(fd_->path_res_, true, fd_->aim_yaw_);
      fd_->last_pub_time_ = ros::Time::now();
    }
  }
}

void FastExplorationFSM::execDFDemo() {
  if(fd_->df_demo_phase_ == 0){

    if (fd_->df_demo_target_id_ >= 0){
      fd_->explore_count_ = 0;
      fd_->df_demo_phase_ = 1;
      fd_->df_demo_mode_  = false;
      fd_->object_target_id_ = fd_->df_demo_target_id_;
      transitState(MISSION_FSM_STATE::GO_TARGET_OBJECT, "DF Demo[2] go target obj");
      return;
    }

    if (fd_->explore_count_ == 0 && fd_->df_demo_target_id_ == -100){
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

    if(fd_->explore_count_ > 2){
      fd_->explore_count_     = 0;
      fd_->df_demo_target_id_ = -100;
      return ;
    }
    
    if (fd_->df_demo_target_id_ == -1){

      int cur_area_id = scene_graph_->cur_poly_->area_id_;
      if (scene_graph_->skeleton_gen_->area_handler_->areas_need_predict_[cur_area_id]){
        INFO_MSG_CYAN("[DF Demo] | current area need llm predict, reset explore count");
        fd_->explore_count_ = 0;
        fd_->df_demo_target_id_ = -100;
        return ;
      } 

      transitState(MISSION_FSM_STATE::PLAN_EXPLORE, "DF Demo[1] plan explore");
      INFO_MSG_CYAN("Do Regular Explore iter [" << fd_->explore_count_ << "]");
      fd_->explore_count_ ++;
      return ;
    }
  }
}

void FastExplorationFSM::findTerminateTarget(){
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

double FastExplorationFSM::adjustTerminateHeightFindingObject(ObjectNode::Ptr target_obj, Eigen::Vector3d init_pos, bool final_point){
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
        map_->getInflateOccupancy(check_pos) == MapInterface::FREE) {
      Eigen::Vector3d check_floor = check_pos;
      check_floor.z() -= 0.5; 
      if(map_->getInflateOccupancy(check_floor) == MapInterface::FREE){
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

double FastExplorationFSM::adjustTerminateHeightNormal(const Eigen::Vector3d& next_aim_raw){
  double ideal_terminate_height = fd_->odom_pos_.z();
  double adjusted_height        = ideal_terminate_height;
  double height_step            = 0.2; 
  int max_adjust_steps          = 5; 
  // TODO [gwq] height adjust for normal waypoint not finished
  return ideal_terminate_height;
}

/**
* @brief 从全局路径中智能地选取并发布一个最优的局部目标点，以供下层运动规划器执行。
* * 该函数的核心优化策略有两点：
* 1. 捷径优化：通过回溯检查路径，寻找当前位置可以直接无障碍到达的最远路径点，以跳过不必要的中间点。
* 2. 前瞻距离保证：如果未找到捷径，则顺序选取下一个路径点，并确保该点与机器人当前位置有足够的安全距离，以保证运动规划的平滑性。
* * @param[in]  path_res     机器人需要跟随的全局路径点向量。
* @param[in]  look_forward 一个布尔标志，指示机器人是否应朝向最终目标点的姿态。在长路径导航中，此参数会被内部逻辑覆盖为true，强制朝向局部目标点。
* @param[in]  aim_yaw      当look_forward为false时，指定的最终目标偏航角。
* @return     bool         如果成功找到并发布了一个有效的局部目标点，则返回true；如果路径已执行完毕，则返回false。
*/
// get local aim from the path, path_inx++, if exceed path size, do nothing
bool FastExplorationFSM::getAndPublishNextAim(vector<Eigen::Vector3d>& path_res,
                                              const bool look_forward, const double aim_yaw) {
  auto getLocalAim = [&](vector<Eigen::Vector3d>& path_res, int& path_inx, Eigen::Vector3d& local_goal) -> bool
  {
    for(int i = path_res.size()-1; i > path_inx; i--)
    {
      if (map_->isInLocalMap(path_res[i]) &&
          map_->isVisible(fd_->odom_pos_, path_res[i]))
      {
        path_inx = i;
        local_goal = path_res[i];
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
          return false;
        }
        idx = path_inx;
        local_goal = path_res[idx];
      }
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
    pubLocalGoal(fd_->local_aim_pos_, aim_yaw, look_forward);
    cout << "[EXP-FM][getAndPubNextAim][look_forward = "<< look_forward << "] Pub aim:" << path_res.back().transpose() << ", yaw: " << aim_yaw << endl;
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
      pubLocalGoal(fd_->local_aim_pos_, aim_yaw, true);
      cout << "[EXP-FM][getAndPubNextAim][look_forward = 1]" << " Pub local aim: " << fd_->local_aim_pos_.transpose() << endl;
      return true;
    }
    return false;
  }
}

void FastExplorationFSM::pubLocalGoal(const Eigen::Vector3d local_goal, const double yaw,
                                      const bool look_forward, const uint8_t yaw_mode,
                                      const uint8_t yaw_path_mode)
{
  // yaw-only全景命令不占用EGO位置轨迹完成标志，目标续接由odometry角度驱动。
  if (yaw_mode != quadrotor_msgs::EgoGoalSet::YAW_MODE_PANORAMA)
    fd_->ego_exec_finished_ = false;

  quadrotor_msgs::EgoGoalSet msg;
  msg.drone_id = md_->drone_id_;
  msg.source_task_id = active_instruction_task_id_;
  msg.goal[0] = static_cast<float>(local_goal.x());
  msg.goal[1] = static_cast<float>(local_goal.y());
  msg.goal[2] = static_cast<float>(local_goal.z());
  msg.look_forward = look_forward;
  msg.yaw = yaw;
  msg.yaw_low_speed = yaw_mode == quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED;
  msg.yaw_mode = yaw_mode;
  msg.yaw_path_mode = yaw_path_mode;
  ego_goal_pub_.publish(msg);
}

// return aim_pose aim_vel, aim_yaw and path_res
int FastExplorationFSM::callExplorationPlanner(Eigen::Vector3d& aim_pose, Eigen::Vector3d& aim_vel, double& aim_yaw, vector<Eigen::Vector3d>& path_res)
{
  // mode 1在TSP前同步消费累计雷达更新盒，完成新frontier生成。
  expl_manager_->updateFrontiersForPlanning(fd_->odom_pos_, fd_->odom_yaw_);

  map_->Lock();
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

  int res = expl_manager_->planExploreTSP(fd_->odom_pos_, fd_->odom_vel_, fd_->odom_yaw_,
                                          aim_pose, aim_vel, aim_yaw, path_res);

  map_->Unlock();
  return res;
}

int FastExplorationFSM::callExplorationLLMPlanner(Eigen::Vector3d &aim_pose, Eigen::Vector3d &aim_vel, double &aim_yaw, vector<Eigen::Vector3d> &path_res) {
  int res = expl_manager_->planLLMExploration(expl_area_id_, fd_->odom_pos_, fd_->odom_vel_, fd_->odom_yaw_,
                                              scene_graph_->getCurPoly(), aim_pose, aim_yaw, aim_vel, path_res);
  return res;
}

int FastExplorationFSM::callTrackPlanner(Eigen::Vector3d& aim_pose, Eigen::Vector3d& aim_vel,
                                         double& aim_yaw, vector<Eigen::Vector3d>& path_res)
{
  (void)aim_vel;
  (void)aim_yaw;
  map_->Lock();
  int res = expl_manager_->planTrackGoal(fd_->odom_pos_, fd_->odom_vel_, aim_pose, path_res);
  map_->Unlock();
  return res;
}

void FastExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  static int delay = 0;
  if (!scene_graph_->skeleton_gen_->ready()) {
    ROS_WARN_THROTTLE(2.0, "[ExploreFSM] | skeleton has not been generated, skip once!");
    return ;
  }

  if (++delay < 5) return;
  if (md_->mission_state_ == INIT) return;
  if (waitForFreshMapAfterReset()) return;

  ros::Time t1 = ros::Time::now();
  auto ft = expl_manager_->frontier_finder_;
  auto ed = expl_manager_->ed_;

  bool new_topo = false;
  scene_graph_->updateSceneGraph(fd_->odom_pos_, fd_->odom_yaw_, new_topo);

  if (new_topo && fp_->enable_area_prediction_ && fd_->new_topo_need_predict_immediately_) {
    std::string llm_prompt_str;
    scene_graph_->newAreaPredictionPromptGen(llm_prompt_str);
    cur_prompt_id_ = scene_graph_->getCurPromptId();
    scene_graph_->sendPrompt(scene_graph_->getCurPromptIdAndPlusOne(),
                             scene_graph::PromptMsg::PROMPT_TYPE_ROOM_PREDICTION,
                             llm_prompt_str, std::chrono::seconds(10), 1);
    if (scene_graph_->skeleton_gen_->cur_iter_first_poly_ != nullptr) {
      // double aim_direction = atan2(fd_->odom_pos_.y() - scene_graph_->skeleton_gen_->cur_iter_first_poly_->center_.y(),
      //                         fd_->odom_pos_.x() - scene_graph_->skeleton_gen_->cur_iter_first_poly_->center_.x()) + M_PI;
      
      double aim_direction = fd_->odom_yaw_;

      if (aim_direction > M_PI)
        aim_direction -= 2 * M_PI;
      if (aim_direction < -M_PI)
        aim_direction += 2 * M_PI;

      Eigen::Vector3d aim_pos = scene_graph_->skeleton_gen_->cur_iter_first_poly_->center_;
      pubLocalGoal(aim_pos, aim_direction, false,
                   quadrotor_msgs::EgoGoalSet::YAW_MODE_LOW_SPEED);
      // pubLocalGoal(fd_->odom_pos_, fd_->odom_yaw_, false, false);
      INFO_MSG_GREEN("Get New skeleton info, stop motion & predict new area");
    }
    if (md_->mission_state_ == MISSION_FSM_STATE::WAIT_TRIGGER)
      return ;
    transitState(MISSION_FSM_STATE::LLM_PLAN_EXPLORE, "ftr_callback -> New Topo Found -> Replan");
    stashCurStateAndTransit(MISSION_FSM_STATE::THINKING, "frontierCallback -> New Topo Found -> Predict!");
    think_start_time_        = ros::Time::now().toSec();
    think_duration_limit_    = 10.0;
    has_made_area_decision_  = false;
    need_rotate_yaw_         = enable_yaw_scan_;
  }

  expl_manager_->setCurrentTopoNode(scene_graph_->skeleton_gen_->mountCurTopoPoint(fd_->odom_pos_, fd_->odom_yaw_));
  scene_graph_->mountCurPoly(fd_->odom_pos_, fd_->odom_yaw_);
  if (new_topo) {
    ft->reCalculateAllFtrTopo(fd_->odom_pos_);
  }
  ft->searchFrontiers(fd_->odom_pos_, fd_->odom_yaw_,
                      expl_manager_->ep_->frontier_tsp_mode_ == 1);
  ft->computeFrontiersToVisit(fd_->odom_pos_);
  ft->updateFrontierCostMatrix();
  ft->updateSceneGraphWithFtr();

  //! Update HGrid
  expl_manager_->updateHgrid();

  // int area_id = scene_graph_->getAreaFromPoly(scene_graph_->getCurPoly());
  // expl_manager_->planLLMExploration(area_id, fd_->odom_pos_, fd_->odom_vel_, fd_->odom_yaw_,
  //                                   scene_graph_->getCurPoly(), fd_->aim_pos_, fd_->aim_yaw_, fd_->aim_vel_, fd_->path_res_);

  // ft->vp_handler_->updateVPWellObserved(fd_->odom_pos_, fd_->odom_yaw_, 2.0);
  // ft->vp_handler_->updateCostMatrix(scene_graph_->cur_areas_[area_id]);
  // Eigen::MatrixXd cost_mat;
  // std::vector<Viewpoint::Ptr> vp_list;
  // ft->vp_handler_->calCurrentAreaCostMatrix(scene_graph_->cur_areas_[area_id], fd_->odom_pos_, fd_->odom_yaw_, fd_->odom_vel_, cost_mat);
  // expl_manager_->findVPGlobalTour(ft->vp_handler_->vps_candidate_, cost_mat, fd_->odom_pos_, fd_->odom_yaw_, fd_->odom_vel_);

  ros::Time t2 = ros::Time::now();
  expl_manager_->visualize(fd_->odom_pos_);
  scene_graph_->visualizeSceneGraph();
  ros::Time t3 = ros::Time::now();

  double vis_time        = (t3 - t2).toSec() * 1e3;
  double perception_time = (t2 - t1).toSec() * 1e3;
  if (perception_time > 20.0 || vis_time > 10.0 )
    ROS_WARN_STREAM("[FtrCallback] : Update time: " << perception_time << " + Vis spend time : " << vis_time << "ms");
}

void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

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

void FastExplorationFSM::egoPlanResCallback(const quadrotor_msgs::EgoPlannerResultConstPtr &msg) {
  fd_->ego_local_goal_.x() = msg->planner_goal.x;
  fd_->ego_local_goal_.y() = msg->planner_goal.y;
  fd_->ego_local_goal_.z() = msg->planner_goal.z;
  fd_->ego_plan_times_     = msg->plan_times;
  fd_->ego_plan_status_    = msg->plan_status;
  fd_->ego_modify_status_  = msg->modify_status;
}

void FastExplorationFSM::instructionCallback(const quadrotor_msgs::InstructionConstPtr& msg)
{
  if (msg->robot_id != md_->drone_id_) return;
  // check recv time frequncy
  static bool ic_first_recv_flag = true;
  static ros::Time ic_last_recv_time;
  const bool bypass_freq_limit =
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_GOAL ||
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_WAYPOINT_NAV ||
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_TRACKING ||
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_OBJECT_NAV ||
      msg->instruction_type == quadrotor_msgs::Instruction::TURN_REGULAR_EXPLORATION;
  if (ic_first_recv_flag){
    ic_first_recv_flag = false;
    ic_last_recv_time = ros::Time::now();
  }else if (!bypass_freq_limit && !ic_first_recv_flag &&
            (ros::Time::now() - ic_last_recv_time).toSec() < 0.8){
    ic_last_recv_time = ros::Time::now();
    std::cout << "[InstructionCallback] : recv too frequent, skip once! instruction_type="
              << static_cast<int>(msg->instruction_type)
              << ", command=" << msg->command << std::endl;
    return;
  }else
    ic_last_recv_time = ros::Time::now();

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
    stopElasticTracker("instructionCallback:non_tracking");
    std::unique_lock<std::mutex> lck(mtx_);
    fd_->track_trigger_ = false;
    fd_->track_init_ = false;
    resetTrackingFinishCandidate();
    fd_->track_finish_sent_ = false;
    if (!useElasticTrackerBackend()) {
      map_->setTarget(fd_->track_pos_, false);
    }
  }

  // load map !
  if (msg->instruction_type == quadrotor_msgs::Instruction::TURN_LOAD_SCENE_GRAPH){
    // stopMotion();
    const bool load_ok = scene_graph_->loadMap(msg->map_folder);
    if (load_ok) {
      fd_->path_res_.clear();
      fd_->path_inx_ = 0;
      fd_->trigger_ = false;
      fd_->regular_explore_ = false;
      fd_->df_demo_mode_ = false;
      fd_->find_terminate_target_mode_ = false;
      fd_->new_topo_need_predict_immediately_ = false;
      fd_->llm_plan_explore_counter_ = 0;
      has_made_area_decision_ = false;
      need_rotate_yaw_ = false;
      expl_area_id_ = -1;
      hardResetExploreArea(false, false);
      scene_graph_->object_factory_->runThisModule();
      scene_graph_->refreshLoadedMapVisualization();
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
    case quadrotor_msgs::Instruction::TURN_OBJECT_ID_NAV:
      expl_manager_->setExplorationRegion(std::vector<Eigen::Vector3d>(), false);
      fd_->object_target_id_ = msg->target_obj_id;
      fd_->go_object_process_phase = 0;
      fd_->find_terminate_target_mode_ = false;
      transitState(MISSION_FSM_STATE::GO_TARGET_OBJECT, "instructionCallback");
      break;

    case quadrotor_msgs::Instruction::TURN_WAYPOINT_NAV: {
      expl_manager_->setExplorationRegion(std::vector<Eigen::Vector3d>(), false);
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
      applyExplorationRegionFromInstruction(msg);
      fd_->regular_explore_ = false;
      if (msg->source_task_id == quadrotor_msgs::Instruction::SOURCE_TASK_COUNTING &&
          msg->task_session_id > 0) {
        counting_scene_graph_->startSession(msg->task_session_id, fd_->odom_pos_);
      }
      if (source_requires_panorama && msg->clear_local_map) {
        stopMotion();
        hardResetExploreArea(true, false);
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
      applyExplorationRegionFromInstruction(msg);
      fd_->regular_explore_ = true;
      if (msg->source_task_id == quadrotor_msgs::Instruction::SOURCE_TASK_COUNTING &&
          msg->task_session_id > 0) {
        counting_scene_graph_->startSession(msg->task_session_id, fd_->odom_pos_);
      }
      if (source_requires_panorama && msg->clear_local_map) {
        stopMotion();
        hardResetExploreArea(true, false);
        map_reset_update_seq_ = map_->getOccupancyUpdateSeq();
        wait_fresh_map_after_reset_ = true;
      } else if (source_requires_panorama) {
        startPanoramaRotation();
      }
      fd_->df_demo_mode_    = false;
      fd_->find_terminate_target_mode_ = false;
      transitState(MISSION_FSM_STATE::PLAN_EXPLORE, "instructionCallback");
      break;
    
    case quadrotor_msgs::Instruction::TURN_DF_DEMO:
      expl_manager_->setExplorationRegion(std::vector<Eigen::Vector3d>(), false);
      fd_->df_demo_mode_ = true;
      fd_->df_demo_phase_ = 0;
      fd_->explore_count_ = 0;
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
        stopElasticTracker("instructionCallback:tracking_disable");
        if (!useElasticTrackerBackend()) {
          map_->setTarget(fd_->track_pos_, false);
        }
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
        if (useElasticTrackerBackend() && fd_->track_finish_sent_ && !was_track_trigger) {
          ROS_INFO_STREAM_THROTTLE(0.5, "[TRACK] ignore residual Elastic-Tracker instruction enable after finish; waiting for disable/reset.");
          break;
        }
        fd_->track_trigger_ = true;
        if (!was_track_trigger) {
          resetTrackingFinishCandidate();
          fd_->track_finish_sent_ = false;
        }
        if (msg->has_target_position)
        {
          fd_->track_pos_ = geoPt2Vec3d(msg->target_position);
        }
        if (useElasticTrackerBackend()) {
          if (msg->has_target_position) {
            fd_->track_init_ = true;
            publishTrackingTargetOdom(fd_->track_pos_, msg->header.stamp, msg->header.frame_id);
          }
          // Elastic-Tracker 的 /triger 只用于打开新 session，后续目标更新只走 target odom。
          if (!was_track_trigger) {
            publishElasticTrackerTrigger(msg->header.stamp, msg->header.frame_id);
          }
          switchPlannerCmdMuxToElastic("instructionCallback:elastic_tracker_enable");
          if (md_->mission_state_ != MISSION_FSM_STATE::WAIT_TRIGGER) {
            stopMotion();
            transitState(MISSION_FSM_STATE::WAIT_TRIGGER, "instructionCallback:elastic_tracker_enable");
          }
        } else {
          switchPlannerCmdMuxToEgo("instructionCallback:ego_tracking_enable");
          map_->setTarget(fd_->track_pos_, false);
          if (md_->mission_state_ != MISSION_FSM_STATE::PLAN_TRACK &&
              md_->mission_state_ != MISSION_FSM_STATE::APPROACH_TRACK)
          {
            transitState(MISSION_FSM_STATE::PLAN_TRACK, "instructionCallback:tracking_enable");
          }
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

void FastExplorationFSM::batteryCallBack(const sensor_msgs::BatteryState msg) {
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

void FastExplorationFSM::stashCurStateAndTransit(MISSION_FSM_STATE new_state, string who_called) {
  stash_state_ = md_->mission_state_;
  transitState(new_state, who_called);
}

void FastExplorationFSM::transitState(MISSION_FSM_STATE new_state, string pos_call) 
{
  MISSION_FSM_STATE pre_s = md_->mission_state_;
  md_->mission_state_ = new_state;

  ROS_INFO_STREAM("\033[1;36m" << "[" << pos_call << "]: from " << md_->state_str_[pre_s]
                      << " to " << md_->state_str_[md_->mission_state_] << "\033[0m"); // 青色
}

void FastExplorationFSM::displayPath() {
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

void FastExplorationFSM::displayMissionState()
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

void FastExplorationFSM::visualize(const ros::TimerEvent& e)
{
  displayMissionState();
  expl_manager_->visHgrid(fd_->odom_pos_);
}

void FastExplorationFSM::stopMotion()
{
  pubLocalGoal(fd_->odom_pos_, fd_->odom_yaw_, true);
}

void FastExplorationFSM::hardResetExploreArea(bool clear_occupancy, bool clear_posegraph) {
  (void)clear_posegraph;
  if (clear_occupancy)
    map_->resetOccupancyToUnknown();

  // 重置探索边界及所有由地图派生的探索状态，保留scene graph和topo map。
  map_->resetGlobalBox();
  Eigen::Vector3d global_box_min, global_box_max;
  map_->getGlobalBox(global_box_min, global_box_max);
  expl_manager_->frontier_finder_->frontierForceDeleteAll();
  expl_manager_->hgrid_->init(global_box_min, global_box_max);
  expl_manager_->ed_->frontiers_.clear();
  expl_manager_->ed_->frontiers_with_info_.clear();
  expl_manager_->ed_->dead_frontiers_.clear();
  expl_manager_->ed_->frontier_boxes_.clear();
  expl_manager_->ed_->global_tour_.clear();
  expl_manager_->ed_->global_tour_map_.clear();
  expl_manager_->ed_->path_next_goal_.clear();
  expl_manager_->ed_->last_grid_ids_.clear();
  expl_manager_->ed_->last_frontiers_with_info_.clear();
  expl_manager_->ed_->last_indices_.clear();
  expl_manager_->ed_->refined_ids_.clear();
  expl_manager_->ed_->n_points_.clear();
  expl_manager_->ed_->unrefined_points_.clear();
  expl_manager_->ed_->refined_points_.clear();
  expl_manager_->ed_->refined_views_.clear();
  expl_manager_->ed_->refined_views1_.clear();
  expl_manager_->ed_->refined_views2_.clear();
  expl_manager_->ed_->refined_tour_.clear();

  fd_->path_res_.clear();
  fd_->path_inx_ = 0;
  fd_->llm_plan_explore_counter_ = 0;
  fd_->explore_count_ = 0;
  has_made_area_decision_ = false;
  expl_area_id_ = -1;
  INFO_MSG_GREEN("=================================");
  INFO_MSG_GREEN("[FSM] : Explore Area Reset Done .");
  INFO_MSG_GREEN("=================================");
}

inline void FastExplorationFSM::geoPt2Vec3d(const geometry_msgs::Point &p_in, Eigen::Vector3d &p_out) {
  p_out.x() = p_in.x; p_out.y() = p_in.y; p_out.z() = p_in.z;
}
inline void FastExplorationFSM::vec3d2GeoPt(const Eigen::Vector3d &p_in, geometry_msgs::Point &p_out) {
  p_out.x = p_in.x(); p_out.y = p_in.y(); p_out.z = p_in.z();
}
inline geometry_msgs::Point FastExplorationFSM::vec3d2GeoPt(const Eigen::Vector3d &p_in) {
  geometry_msgs::Point p_out;
  p_out.x = p_in.x(); p_out.y = p_in.y(); p_out.z = p_in.z();
  return p_out;
}
inline Eigen::Vector3d FastExplorationFSM::geoPt2Vec3d(const geometry_msgs::Point &p_in) {
  Eigen::Vector3d p_out;
  p_out.x() = p_in.x; p_out.y() = p_in.y; p_out.z() = p_in.z;
  return p_out;
}
}  // namespace fast_planner
