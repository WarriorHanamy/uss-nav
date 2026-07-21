#ifndef _MISSION_FSM_H_
#define _MISSION_FSM_H_

#include <Eigen/Eigen>

#include <Eigen/src/Core/Matrix.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/CompressedImage.h>

#include <quadrotor_msgs/Instruction.h>
#include <quadrotor_msgs/InstructionResMsg.h>
#include <quadrotor_msgs/EgoPlannerResult.h>
#include <quadrotor_msgs/MultiPoseGraph.h>
#include <quadrotor_msgs/HgridMsg.h>
#include <quadrotor_msgs/PerceptionMsg.h>
#include <quadrotor_msgs/LocalGoalSet.h>
#include <quadrotor_msgs/WaypointProgress.h>
#include <quadrotor_msgs/EgoStateTrigger.h>
#include <quadrotor_msgs/DetectOut.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TrackCommand.h>
#include <quadrotor_msgs/VLASearchBBox.h>
#include <quadrotor_msgs/VLASearchTarget.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>
#include <deque>
#include <mutex>
#include <global_belief/map_interface.hpp>
#include <mission_executive/mission_data.h>
#include <mission_executive/vla_search_map.h>
#include <scene_graph/object_factory.h>
#include <scene_graph/counting_scene_graph.h>
#include <scene_graph/scene_graph.h>
#include <scene_graph/traj_visualizer.h>
#include <scene_graph/VLASearchObservation.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::string;

namespace mission_executive {
class Tabv;
struct FSMParam;
struct FSMData;
class EkfEstimator;
class PerceptionDataMsgFactory;


class MissionFSM {
private:
  /* planning utils */
  global_belief::MapInterface::Ptr                                 map_;
  shared_ptr<SceneGraph>                            scene_graph_;
  CountingSceneGraph::Ptr                           counting_scene_graph_;
  shared_ptr<TrajectoryVisualizer>                  traj_visualizer_;
  VLASearchMap::Ptr                                  vla_search_map_;
  shared_ptr<FSMParam>                              fp_;
  shared_ptr<FSMData>                               fd_;
  shared_ptr<MissionData>                           md_;
  double                                            scale_;

  std::mutex mtx_; 

  bool classic_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, vla_search_map_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, ego_exec_finish_sub_, wp_progress_sub_;
  ros::Subscriber track_command_sub_, target_sub_;
  ros::Subscriber instruction_sub_, ego_plan_res_sub_, battery_sub_, perception_data_sub_, emergency_stop_sub_;
  ros::Subscriber vla_search_target_sub_, vla_search_camera_sub_;
  ros::Subscriber vla_search_ego_state_trigger_sub_;
  ros::Subscriber object_id_nav_replan_sub_;    // 订阅 /object_id_nav_replan
  // Planner command mux (integrated)
  ros::Subscriber ego_position_cmd_sub_;
  ros::Publisher position_cmd_pub_;
  std::string planner_cmd_mux_active_mode_{"ego"};
  double planner_cmd_mux_input_timeout_{0.5};
  bool has_ego_cmd_{false};
  ros::Time ego_cmd_stamp_;
  quadrotor_msgs::PositionCommand last_ego_cmd_;

  ros::Publisher ego_goal_pub_, perception_data_pub_, instruction_resp_pub_;
  ros::Publisher vis_marker_pub_, vis_path_pub_;
  // SUPER waypoint window state (multi-goal batch protocol)
  bool use_super_backend_{false};
  uint32_t wp_batch_seq_{0};
  int wp_batch_start_inx_{0};
  bool wp_window_active_{false};
  ros::Publisher fsm_state_pub_;
  ros::Publisher tracking_finish_pub_;
  ros::Publisher tracking_target_odom_pub_;
  ros::Publisher vla_search_result_pub_;
  ros::Publisher vla_search_bbox_pub_;
  ros::Publisher vla_search_observation_pub_;

  // LLM related
  MISSION_FSM_STATE stash_state_{MISSION_FSM_STATE::UNKONWN};
  unsigned int cur_prompt_id_{0};
  bool has_made_area_decision_{false}, need_rotate_yaw_{false};    // only used for llm plan
  bool enable_yaw_scan_{false};                                   // 是否执行+45°/-45°/回正扫描
  bool enable_scene_graph_update_after_load_{true};               // 载入预存地图后是否继续增量更新场景图
  // 仅由source_task_id=EXPLORATION/COUNTING开启的360度全景旋转状态
  bool need_panorama_{false};
  bool panorama_command_active_{false};
  uint8_t active_instruction_task_id_{0};
  uint32_t active_instruction_session_id_{0};
  double panorama_last_odom_yaw_{0.0};
  double panorama_start_yaw_{0.0};
  double panorama_unwrapped_yaw_{0.0};
  double panorama_accumulated_yaw_{0.0};
  double panorama_command_target_yaw_{0.0};
  Eigen::Vector3d panorama_hold_pos_{Eigen::Vector3d::Zero()};
  double panorama_max_step_{2.0943951023931953};
  double panorama_extend_angle_{0.6981317007977318};
  bool wait_fresh_map_after_reset_{false};
  uint64_t map_reset_update_seq_{0};
  double think_duration_limit_;
  double think_start_time_;

  // VLA_Swarm 独立任务上下文。
  bool vla_search_enabled_{false};
  bool vla_search_active_{false};
  bool vla_search_result_published_{false};
  bool vla_search_success_{false};
  uint32_t vla_search_session_id_{0};
  std::string vla_search_command_;
  std::string vla_search_finish_reason_;
  std::string vla_search_finish_detail_;
  std::string vla_search_result_topic_{"/planning/vla_search_result"};
  std::string vla_search_bbox_topic_{"/vla_search/bbox"};
  std::string vla_search_target_topic_{"/vla_search/target"};
  std::string vla_search_camera_topic_;
  std::string vla_search_observation_topic_{"/vla_search/observation"};
  bool vla_search_prompt_pending_{false};
  bool vla_search_place_checked_{false};
  int vla_search_explore_area_id_{-1};
  unsigned int vla_search_prompt_id_{0};
  uint8_t vla_search_prompt_type_{0};
  uint32_t vla_search_observation_batch_id_{0};
  uint32_t vla_search_target_request_id_{0};
  ros::Time vla_search_prompt_start_time_;
  ros::Time vla_search_target_start_time_;
  ros::Time vla_search_observation_stamp_;
  sensor_msgs::CompressedImageConstPtr vla_search_latest_camera_image_;
  ros::Time vla_search_latest_camera_receive_time_;
  std::mutex vla_search_camera_mutex_;
  std::vector<double> vla_search_scan_yaw_offsets_;
  size_t vla_search_scan_index_{0};
  double vla_search_scan_base_yaw_{0.0};
  double vla_search_scan_target_yaw_{0.0};
  Eigen::Vector3d vla_search_scan_hold_position_{Eigen::Vector3d::Zero()};
  ros::Time vla_search_scan_command_time_;
  ros::Time vla_search_scan_yaw_reached_time_;
  bool vla_search_scan_initialized_{false};
  bool vla_search_scan_command_published_{false};
  bool vla_search_target_pending_{false};
  bool vla_search_target_received_{false};
  bool vla_search_target_success_{false};
  uint8_t vla_search_target_observation_index_{0};
  uint8_t vla_search_target_source_{0};
  Eigen::Vector3d vla_search_target_position_{Eigen::Vector3d::Zero()};
  std::string vla_search_target_error_;
  std::vector<Eigen::Vector3d> vla_search_path_;
  ros::Time vla_search_waypoint_publish_time_;
  bool vla_search_path_reaches_task_target_{false};
  bool vla_search_waypoint_published_{false};
  bool vla_search_waypoint_is_final_{false};
  bool vla_search_plan_feedback_received_{false};
  bool vla_search_plan_feedback_success_{false};
  int vla_search_waypoint_retry_count_{0};
  double vla_search_prompt_timeout_{20.0};
  double vla_search_target_timeout_{10.0};
  double vla_search_ego_plan_timeout_{5.0};
  double vla_search_ego_exec_timeout_{30.0};
  int vla_search_max_plan_retries_{2};
  int vla_search_max_target_retries_{2};
  double vla_search_waypoint_distance_{2.0};
  double vla_search_goal_tolerance_{0.5};
  double vla_search_flight_height_{1.0};
  double vla_search_map_update_period_{1.0};
  double vla_search_scan_yaw_tolerance_{0.08};
  double vla_search_scan_settle_time_{0.4};
  double vla_search_scan_timeout_{8.0};
  double vla_search_scan_yaw_step_deg_{90.0};
  bool vla_search_ego_stable_{true};
  int vla_search_exploration_round_{0};
  int vla_search_max_exploration_rounds_{6};
  // AA 阶段：全局评估与跨轮记忆，参照原始 VLA_Swarm 的 AA→A→B→C→TASK_OVER 链路
  bool vla_search_aa_done_{false};
  nlohmann::json vla_search_key_action_history_;
  std::map<int, std::string> vla_search_room_descriptions_;
  bool vla_search_enable_room_description_{false};

  bool object_id_nav_autostart_enable_{false};
  bool object_id_nav_autostart_triggered_{false};
  int object_id_nav_autostart_target_id_{2};
  double object_id_nav_autostart_delay_sec_{1.0};
  ros::Time object_id_nav_autostart_ready_time_;

 private:
  /* helper functions */
  /**
   * Dispatch to tracking planner for target following.
   *
   * @param[out] aim_pose  Target position [m]
   * @param[out] aim_vel   Target velocity [m/s]
   * @param[out] aim_yaw   Target yaw [rad]
   * @param[out] path_res  Planned global path points
   * @return 0 on success, non-zero on failure
   */
  int callTrackPlanner(Eigen::Vector3d& aim_pose, Eigen::Vector3d& aim_vel, double& aim_yaw,
                       vector<Eigen::Vector3d>& path_res);
  /**
   * Reset the tracking finish candidate state.
   */
  void resetTrackingFinishCandidate();
  /**
   * Update tracking finish candidate with current distance and angle to aim.
   *
   * @param[in] dis_2_aim    Distance to aim point [m]
   * @param[in] angle_2_aim  Angle to aim point [rad]
   * @return True if tracking finish condition is met
   */
  bool updateTrackingFinishCandidate(double dis_2_aim, double angle_2_aim);
  /**
   * Publish tracking finish message.
   */
  void publishTrackingFinish();
  /**
   * Switch planner command mux to EGO planner mode.
   *
   * @param[in] source  Source identifier for logging
   */
  void switchPlannerCmdMuxToEgo(const std::string& source);
  /**
   * Check whether the given FSM state belongs to VLA swarm states.
   *
   * @param[in] state  FSM state to check
   * @return True if state is a VLA swarm substate
   */
  bool isVlaSearchState(MISSION_FSM_STATE state) const;
  /**
   * Reset all VLA swarm task context variables.
   */
  void resetVlaSearchContext();
  /**
   * Start a new VLA swarm task from an instruction message.
   *
   * @param[in] msg  Instruction message with VLA swarm command
   */
  void startVlaSearchTask(const quadrotor_msgs::InstructionConstPtr& msg);
  /**
   * Cancel the current VLA swarm task.
   *
   * @param[in] reason  Cancellation reason
   * @param[in] detail  Detail description
   */
  void cancelVlaSearchTask(const std::string& reason, const std::string& detail);
  /**
   * Publish VLA swarm task result.
   *
   * @param[in] success Whether the task succeeded
   * @param[in] reason  Reason string
   * @param[in] detail  Detail description
   */
  void publishVlaSearchResult(bool success, const std::string& reason,
                             const std::string& detail = "");
  /**
   * Start a VLA swarm target request via LLM.
   *
   * @param[in] payload  JSON payload for the LLM request
   * @return True if request was sent successfully
   */
  bool startVlaSearchTargetRequest(const nlohmann::json& payload);
  /**
   * Prepare a global path to the requested VLA swarm goal.
   *
   * @param[in] requested_goal     Target position [m]
   * @param[in] reaches_task_target Whether this goal completes the task
   * @param[in] door_id            Door ID to traverse, -1 for direct path
   * @return True if path was prepared successfully
   */
  bool prepareVlaSearchPath(const Eigen::Vector3d& requested_goal,
                           bool reaches_task_target, int door_id = -1);
  /**
   * Publish the next waypoint along the prepared VLA swarm path.
   *
   * @return True if a waypoint was published
   */
  bool publishNextVlaSearchWaypoint();
  /**
   * Retry the current VLA swarm waypoint due to planning failure.
   *
   * @param[in] failure_reason  Reason for the failure
   */
  void retryVlaSearchWaypoint(const std::string& failure_reason);
  /**
   * Handle VLA swarm PLAN_LOCAL FSM state.
   */
  void handleVlaSearchPlanLocal();
  /**
   * Handle VLA swarm WAIT_LLM FSM state.
   */
  void handleVlaSearchWaitLLM();
  /**
   * Handle VLA swarm WAIT_TARGET FSM state.
   */
  void handleVlaSearchWaitTarget();
  /**
   * Handle VLA swarm APPROACH FSM state.
   */
  void handleVlaSearchApproach();
  /**
   * Handle VLA swarm YAW FSM state (yaw scanning).
   */
  void handleVlaSearchYaw();
  /**
   * Handle VLA swarm RECOVERY FSM state.
   */
  void handleVlaSearchRecovery();
  /**
   * Handle VLA swarm FINISH FSM state.
   */
  void handleVlaSearchFinish();
  
  /**
   * Transition the FSM to a new state.
   *
   * @param[in] new_state  Target FSM state
   * @param[in] pos_call   Caller identifier for logging
   */
  void transitState(MISSION_FSM_STATE new_state, string pos_call);
  /**
   * Stash current state and transition to a temporary state.
   *
   * @param[in] new_state  Temporary FSM state
   * @param[in] who_called Caller identifier for logging
   */
  void stashCurStateAndTransit(MISSION_FSM_STATE new_state, string who_called);
  /**
   * Trigger object-id navigation replan.
   *
   * @param[in] reason  Replan reason
   */
  void triggerObjectIdNavReplan(const std::string& reason);
  /**
   * Start object-id navigation through the common MissionFSM path.
   *
   * @param[in] target_obj_id  SceneGraph object ID [--]
   * @param[in] source_task_id Instruction source task ID [--]
   * @param[in] session_id     Instruction session ID [--]
   * @param[in] reason         Transition/log reason
   * @param[in] source_msg     Optional original instruction to preserve for replan
   * @param[in] reset_replan_count Whether to reset object-id replan counter
   */
  void startObjectIdNav(int target_obj_id, uint8_t source_task_id,
                        uint32_t session_id, const std::string& reason,
                        const quadrotor_msgs::Instruction* source_msg = nullptr,
                        bool reset_replan_count = true);
  /**
   * Publish a command only if the mode matches the active mux mode.
   *
   * @param[in] cmd          Position command
   * @param[in] source_mode  Source mode identifier
   */
  void publishIfActive(const quadrotor_msgs::PositionCommand& cmd,
                       const std::string& source_mode);
  /**
   * Publish the last cached command for the currently active mux mode.
   *
   * @param[in] source  Caller identifier for logging
   */
  void publishLastForMode(const std::string& source);
  /**
   * Check whether a timestamp is within the input timeout window.
   *
   * @param[in] stamp  Message timestamp [s]
   * @return True if the message is still fresh
   */
  bool isFresh(const ros::Time& stamp) const;
  /**
   * Get the initial seed position for scene graph initialization.
   *
   * @param[out] init_seed  Seed position [m]
   * @param[out] reason     Optional failure reason
   * @return True if seed position is valid
   */
  bool getSceneGraphInitSeed(Eigen::Vector3d& init_seed, std::string* reason = nullptr) const;

  /* ROS functions */
  /**
   * Main FSM execution timer callback — drives state transitions each cycle.
   *
   * @param[in] e  Timer event
   */
  void FSMCallback(const ros::TimerEvent& e);
  /**
   * Periodic VLA swarm map update timer callback.
   *
   * @param[in] e  Timer event
   */
  void vlaSearchMapCallback(const ros::TimerEvent& e);
  /**
   * Callback for external trigger pose (RViz click or station).
   *
   * @param[in] msg  Trigger pose
   */
  void triggerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  /**
   * Callback for EGO planner execution finish signal.
   *
   * @param[in] msg  Bool indicating execution complete
   */
  void egoExecFinishCallback(const std_msgs::Bool::ConstPtr& msg);
  /**
   * Callback for target tracking command.
   *
   * @param[in] msg  Track command message
   */
  void trackCommandCallback(const quadrotor_msgs::TrackCommand::ConstPtr& msg);
  /**
   * Callback for real target detection output.
   *
   * @param[in] msg  Detection output message
   */
  void targetCallbackReal(const quadrotor_msgs::DetectOut::ConstPtr& msg);
  /**
   * Callback for VLA swarm target waypoint from LLM.
   *
   * @param[in] msg  VLA swarm target message
   */
  void vlaSearchTargetCallback(
      const quadrotor_msgs::VLASearchTarget::ConstPtr& msg);
  /**
   * Callback for VLA swarm camera image.
   *
   * @param[in] msg  Compressed camera image
   */
  void vlaSearchCameraCallback(
      const sensor_msgs::CompressedImageConstPtr& msg);
  /**
   * Callback for VLA swarm ego state trigger.
   *
   * @param[in] msg  Ego state trigger message
   */
  void vlaSearchEgoStateTriggerCallback(
      const quadrotor_msgs::EgoStateTrigger::ConstPtr& msg);
  /**
   * Callback for object-id navigation replan trigger.
   *
   * @param[in] msg  Bool message
   */
  void objectIdNavReplanCallback(const std_msgs::Bool::ConstPtr& msg);
  /**
   * Callback for EGO planner position commands (mux input A).
   *
   * @param[in] msg  Position command from EGO planner
   */
  void egoPositionCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
  /**
   * Handle goal instruction for exploration or object search.
   *
   * @param[in] goals        Goal positions [m]
   * @param[in] yaws         Goal yaws [rad]
   * @param[in] look_forward Whether to face forward toward goals
   * @param[in] source       Source identifier
   */
  void handleGoalInstruction(const std::vector<geometry_msgs::Point>& goals, const std::vector<float>& yaws,
                             bool look_forward, const std::string& source);
  /**
   * Handle tracking target command with global poses.
   *
   * @param[in] global_poses  Target positions in global frame [m]
   * @param[in] source        Source identifier
   * @param[in] stamp         Timestamp [s]
   * @param[in] frame_id      Reference frame
   */
  void handleTrackingTarget(const std::vector<geometry_msgs::Point>& global_poses,
                            const std::string& source,
                            const ros::Time& stamp = ros::Time(),
                            const std::string& frame_id = "world");
  /**
   * Callback for incoming instruction messages.
   *
   * @param[in] msg  Instruction message
   */
  void instructionCallback(const quadrotor_msgs::InstructionConstPtr& msg);
  /**
   * Callback for emergency stop signal.
   *
   * @param[in] msg  Empty message
   */
  void emergencyStopCallback(const std_msgs::Empty::ConstPtr& msg);
  /**
   * Callback for battery state updates.
   *
   * @param[in] msg  Battery state message
   */
  void batteryCallBack(const sensor_msgs::BatteryState msg);
  /**
   * Callback for odometry updates.
   *
   * @param[in] msg  Odometry message
   */
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  /**
   * Callback for EGO planner result feedback.
   *
   * @param[in] msg  Planner result message
   */
  void egoPlanResCallback(const quadrotor_msgs::EgoPlannerResultConstPtr& msg);
  /**
   * Select and publish the next local aim point from the global path with shortcut optimization.
   *
   * @param[in]     path_res     Global path points [m]
   * @param[in]     look_forward Whether to face the final goal yaw
   * @param[in]     aim_yaw      Target yaw when look_forward is false [rad]
   * @return True if a valid aim point was published
   */
  bool getAndPublishNextAim(vector<Eigen::Vector3d>& path_res,
                              const bool look_forward = true, const double aim_yaw = 0.0);
  /**
   * Publish a local goal command to the low-level motion planner.
   *
   * @param[in] local_goal   Goal position [m]
   * @param[in] yaw          Yaw angle [rad]
   * @param[in] look_forward Whether to face forward
   * @param[in] yaw_mode     Yaw mode enum
   * @param[in] yaw_path_mode Yaw path mode enum
   */
  void pubLocalGoal(
      const Eigen::Vector3d local_goal, const double yaw = 0.0, const bool look_forward = true,
      uint8_t yaw_mode = quadrotor_msgs::LocalGoalSet::YAW_MODE_NORMAL,
      uint8_t yaw_path_mode = quadrotor_msgs::LocalGoalSet::YAW_PATH_SHORTEST);
  /**
   * Publish a waypoint window (up to 3 waypoints in travel order) to SUPER.
   *
   * Waypoint progress is determined inside SUPER; each consumption is reported back via
   * /drone_0_ego_planner_node/waypoint_progress and advances path_inx_ here.
   *
   * @param[in] window       Waypoints in travel order (1-3 points) [m]
   * @param[in] yaw          Yaw applied at the batch-final waypoint [rad]
   * @param[in] look_forward Whether to face forward
   * @param[in] yaw_mode     Yaw mode enum
   * @param[in] yaw_path_mode Yaw path mode enum
   */
  void pubLocalGoalWindow(
      const std::vector<Eigen::Vector3d>& window, const double yaw = 0.0,
      const bool look_forward = true,
      uint8_t yaw_mode = quadrotor_msgs::LocalGoalSet::YAW_MODE_NORMAL,
      uint8_t yaw_path_mode = quadrotor_msgs::LocalGoalSet::YAW_PATH_SHORTEST);
  /**
   * SUPER waypoint window consumption feedback: advance path_inx_ and republish the
   * sliding window. Stale batch_id messages are ignored.
   */
  void waypointProgressCallback(const quadrotor_msgs::WaypointProgressConstPtr& msg);
  /**
   * Stop all robot motion by publishing a zero-velocity command.
   */
  void stopMotion();

  /**
   * Plan the next target tracking path.
   */
  void planTrack();
  /**
   * Approach the tracking target.
   */
  void approachTrack();
  /**
   * Execute yaw scan sequence to expand field of view and update the map.
   */
  void handleYawChange();
  /**
   * Start 360-degree panorama rotation during EXPLORATION/COUNTING startup.
   */
  void startPanoramaRotation();
  /**
   * Handle the panorama yaw rotation state step.
   */
  void handlePanoramaYaw();
  /**
   * Wait for a fresh map frame after occupancy reset before starting panorama.
   *
   * @return True when a fresh map is available
   */
  bool waitForFreshMapAfterReset();
  /**
   * Navigate to the target object position.
   */
  void goTargetObject();
  /**
   * Navigate to target using intermediate waypoints.
   */
  void goTargetWithWaypoint();
  /**
   * Find and set the terminate target position.
   */
  void findTerminateTarget();
  /**
   * Execute the demonstration flight (DF demo) routine.
   */
  void execDFDemo();

  /**
   * Adjust the terminate height based on target object dimensions.
   *
   * @param[in] target_obj  Target object node
   * @param[in] init_pos    Initial position [m]
   * @param[in] final_point Whether this is the final approach point
   * @return Adjusted height [m]
   */
  double adjustTerminateHeightFindingObject(ObjectNode::Ptr target_obj, Eigen::Vector3d init_pos, bool final_point=false);
  /**
   * Adjust terminate height using default logic for non-object targets.
   *
   * @param[in] next_aim_raw  Raw next aim position [m]
   * @return Adjusted height [m]
   */
  double adjustTerminateHeightNormal(const Eigen::Vector3d& next_aim_raw);

  double yawhandle_yaw_raw;
  double yawhandle_yaw_target_left ;
  double yawhandle_yaw_target_right ;
  bool   yawhandle_left_published, yawhandle_right_published, yawhandle_back_published;
  bool   yawhandle_left_ok, yawhandle_right_ok, yawhandle_back_ok;

  /**
   * Display current mission state via ROS logging.
   */
  void displayMissionState();
  /**
   * Display the planned path via ROS logging.
   */
  void displayPath();
  /**
   * Publish RViz marker for the current navigation aim point.
   */
  void displayLocalAim();
  /**
   * Publish all visualization markers (path, state, aim).
   *
   * @param[in] e  Timer event
   */
  void visualize(const ros::TimerEvent& e);

  // TOOLS
  /**
   * Convert geometry_msgs::Point to Eigen::Vector3d.
   *
   * @param[in]  p_in  Input point [m]
   * @param[out] p_out Output vector [m]
   */
  void geoPt2Vec3d(const geometry_msgs::Point &p_in, Eigen::Vector3d &p_out);
  /**
   * Convert Eigen::Vector3d to geometry_msgs::Point.
   *
   * @param[in]  p_in  Input vector [m]
   * @param[out] p_out Output point [m]
   */
  void vec3d2GeoPt(const Eigen::Vector3d &p_in, geometry_msgs::Point &p_out);
  /**
   * Convert Eigen::Vector3d to geometry_msgs::Point (return-by-value).
   *
   * @param[in] p_in  Input vector [m]
   * @return Output point [m]
   */
  geometry_msgs::Point vec3d2GeoPt(const Eigen::Vector3d &p_in);
  /**
   * Convert geometry_msgs::Point to Eigen::Vector3d (return-by-value).
   *
   * @param[in] p_in  Input point [m]
   * @return Output vector [m]
   */
  Eigen::Vector3d geoPt2Vec3d(const geometry_msgs::Point &p_in);

public:
  /**
   * Default constructor.
   */
  MissionFSM(/* args */) {
  }
  /**
   * Destructor — stops the object factory module.
   */
  ~MissionFSM() {
      scene_graph_->object_factory_->stopThisModule();
  }

  /**
   * Initialize the FSM with ROS node handle and map interface.
   *
   * @param[in] nh   ROS node handle
   * @param[in] map  Map interface pointer
   */
  void init(ros::NodeHandle& nh, const global_belief::MapInterface::Ptr& map);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace ego planner

#endif
