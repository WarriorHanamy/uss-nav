//
// Created by gwq on 11/26/24.
//

#ifndef SRC_RC_FSM_H
#define SRC_RC_FSM_H

#include "ros/ros.h"
#include "Eigen/Eigen"
#include "map_interface/map_interface.hpp"
#include "scene_graph/scene_graph.h"
#include "tf/transform_datatypes.h"
#include "mutex"

#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include "quadrotor_msgs/GoalSet.h"

#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/TakeoffLand.h"
#include "quadrotor_msgs/EgoGoalSet.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
using ego_planner::MapInterface;

struct Vector3iHash {
    std::size_t operator()(const Eigen::Vector3i& vec) const {
      std::hash<int> hasher;
      size_t seed = 0;
      seed ^= hasher(vec.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= hasher(vec.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= hasher(vec.z()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      return seed;
    }
};

enum RCCtrlFSMState {
  INIT             = 0,
  WAIT_FOR_RC_FLAG = 1,
  EXECUTE_RC_CTRL  = 2
};

/**
 * RC (radio controller) channel data container.
 *
 * Parses raw RC input channels (ch1-ch4 for roll/pitch/yaw/thrust)
 * and manages the CH10 mode switch state machine.
 */
class RC_Data_t
{
public:

    double ch_[4];          ///< RC channels 1-4 (roll/pitch/yaw/thrust), normalized [-1, 1]
    double ch10_, last_ch10_; ///< CH10 toggle channel value [--]

    mavros_msgs::RCIn msg_;
    mavros_msgs::State state_;
    bool is_armed_;          ///< Whether the drone is armed [--]
    ros::Time rcv_stamp_;    ///< Last RC message receive timestamp [s]

    bool activate_rc_ctrl_mode_;  ///< RC control mode active flag [--]
    bool need_takeoff_land_cmd_pub_; ///< Whether takeoff/land command needs publishing [--]
    bool have_effective_rc_cmd_;  ///< Whether effective RC command is received [--]
    int  ch10_trigger_count_;     ///< CH10 trigger counter [--]
    int  ch10_value_change_count_;///< CH10 value change counter [--]
    ros::Time ch10_trigger_last_time_; ///< Last CH10 trigger time [s]

    static constexpr double GEAR_SHIFT_VALUE = 0.75;        ///< Gear shift threshold [--]
    static constexpr double API_MODE_THRESHOLD_VALUE = 0.75; ///< API mode threshold [--]
    static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;    ///< Reboot threshold [--]
    static constexpr double DEAD_ZONE = 0.25;                ///< Joystick dead zone [--]

    RC_Data_t();
    void check_validity();
    bool check_centered();
    void process_ch10_trigger();
    void check_ch10_trigger_once();
    /**
     * Feed raw RC input message.
     *
     * @param[in] pMsg  Raw RC input message
     */
    void feed(mavros_msgs::RCInConstPtr pMsg);
    /**
     * Feed MAVROS state message.
     *
     * @param[in] pMsg  MAVROS state message
     */
    void feedMavrosState(mavros_msgs::StateConstPtr pMsg);
};

/**
 * Odometry data container.
 *
 * Stores the latest odometry: position, velocity, attitude, angular velocity,
 * and yaw for the RC control FSM.
 */
class Odom_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p_;    ///< Position [m]
    Eigen::Vector3d v_;    ///< Velocity [m/s]
    Eigen::Quaterniond q_; ///< Attitude quaternion [--]
    Eigen::Vector3d w_;    ///< Angular velocity [rad/s]
    double yaw_;           ///< Yaw angle [rad]

    nav_msgs::Odometry msg_;
    ros::Time rcv_stamp_;  ///< Last odometry receive timestamp [s]
    bool recv_new_msg_;    ///< Whether a new message has been received since last read [--]

    Odom_Data_t();
    /**
     * Feed odometry message.
     *
     * @param[in] pMsg  Odometry message
     */
    void feed(nav_msgs::OdometryConstPtr pMsg);
};

/**
 * RC control FSM runtime data.
 *
 * Aggregates RC data, odometry, and FSM state for the manual control
 * finite state machine. Includes expected pose/speed for position holding.
 */
struct RCCtrlFSMData{
    ros::Time now_time_, draw_safety_check_last_time_, draw_vel_arrow_last_time_, draw_text_last_time_;
    RCCtrlFSMState      fsm_state_, last_fsm_state_;
    RC_Data_t           rc_data_;
    Odom_Data_t         odom_data_;

    Eigen::Vector3d     pose_check_;       ///< Safety check position [m]
    Eigen::Vector3d     expected_pos_;     ///< Expected hold position [m]
    Eigen::Vector3d     expected_speed_;   ///< Expected velocity command [m/s]
    double              expected_yaw_rate_; ///< Expected yaw rate command [rad/s]
    double              expected_yaw_;      ///< Expected yaw [rad]
    ros::Time           last_cmd_pub_time_; ///< Last command publish timestamp [s]

    bool is_odom_ready_{false};         ///< Odometry data is available [--]
    bool is_rc_signal_ready_{false};    ///< RC signal is available [--]
    bool is_in_collision_traj_{false};  ///< Currently in collision trajectory [--]
};

/**
 * RC control FSM parameters (loaded from ROS parameter server).
 */
struct RCCtrlFSMParams{
  double max_speed_;     ///< Maximum cruise velocity [m/s]
  double max_acc_;       ///< Maximum acceleration [m/s^2]
  double max_yaw_rate_;  ///< Maximum yaw rate [rad/s]
  double rc_timeout_;    ///< RC signal timeout [s]
  double odom_timeout_;  ///< Odometry timeout [s]
  std::string odom_sub_topic_, rc_sub_topic_, px4ctrl_cmd_topic_;
  double      fsm_exec_freq_;

  bool        rc_reverse_pitch_;
  bool        rc_reverse_roll_;
  bool        rc_reverse_yaw_;
  bool        rc_reverse_thrust_;
};

/**
 * RC manual control finite state machine.
 *
 * Implements a simple FSM (INIT -> WAIT_FOR_RC_FLAG -> EXECUTE_RC_CTRL) for
 * manual RC control of the UAV. Translates RC channel inputs to velocity/yaw
 * commands, manages position hold, and interfaces with SceneGraph for
 * skeleton mount point updates.
 */
class RCCtrlFSM {
public:
  typedef std::shared_ptr<RCCtrlFSM> Ptr;
  RCCtrlFSM(ros::NodeHandle& node, MapInterface::Ptr &map_interface);
  ~RCCtrlFSM();
private:
  std::map<int, std::string>        state_name_map_;
  ros::Subscriber                   rc_sub_, odom_sub_, px4_state_sub_, goal_from_station_sub_;
  ros::Publisher                    px4ctrl_cmd_pub_, rc_ctrl_marker_pub_,
                                    px4ctrl_takeoff_land_pub_, ego_goal_set_pub_, ego_goal_yaw_preset_pub_;
  ros::Timer                        fsm_exec_timer_, fsm_vis_timer_, skeleton_mount_point_timer_;
  std::shared_ptr<RCCtrlFSMData>    data_;
  std::shared_ptr<RCCtrlFSMParams>  params_;
  MapInterface::Ptr                 map_interface_;
  SceneGraph::Ptr                   scene_graph_;
  std::mutex                        mutex_;
  std::unique_ptr<std::thread>      rc_cmd_thread_;

  void fsmExecTimerCallback(const ros::TimerEvent& event);
  void fsmVisTimerCallback(const ros::TimerEvent& event);
  void skeletonMountPointTimerCallback(const ros::TimerEvent& event);
  void goalFromStationCallback(const quadrotor_msgs::GoalSet::ConstPtr& msg);

  /**
   * Check if RC signal is within timeout window.
   *
   * @param[in] now_time  Current time [s]
   * @return True if RC signal is ready
   */
  bool isRCReady(const ros::Time& now_time);
  /**
   * Check if odometry is within timeout window.
   *
   * @param[in] now_time  Current time [s]
   * @return True if odometry is ready
   */
  bool isOdomReady(const ros::Time& now_time);
  bool checkDataRecvSafety();
  /**
   * Check if movement is safe along the expected velocity direction.
   *
   * @param[in] safe_distance  Required safety clearance ahead [m]
   * @param[in] expected_vel   Expected velocity vector [m/s]
   * @return True if movement is safe
   */
  bool checkMovementSafety(double safe_distance, const Eigen::Vector3d & expected_vel);

  void fsmProcess();
  void rcCmdProcess();
  void setHoverWithRC();
  void changeFSMState(const std::string log_info, RCCtrlFSMState new_state);
  /**
   * Hold position at a given position and yaw.
   *
   * @param[in] pos      Hold position [m]
   * @param[in] yaw      Hold yaw [rad]
   * @param[in] use_ego  Use EGO planner for position hold [--]
   */
  void holdPosition(const Eigen::Vector3d &pos, const double &yaw, bool use_ego);
  /**
   * Find an available safe position within range around the current position.
   *
   * @param[out] pos_refine  Found safe position [m]
   * @param[in]  range       Search radius [m]
   * @param[in]  step_size   Search step size [m]
   * @return True if a safe position was found
   */
  bool findAvailablePointInRange(Eigen::Vector3d &pos_refine, const double &range, const double & step_size);

  template<typename T>
  void readParam(ros::NodeHandle &node, std::string param_name, T& param_val, T default_val);
  void drawVelArrow();
  void drawSaftyCheckPoint(Eigen::Vector3d &check_start, Eigen::Vector3d &check_end);
  void drawText();
};

#endif //SRC_RC_FSM_H
