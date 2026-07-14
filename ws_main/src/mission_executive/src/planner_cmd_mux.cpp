#include <ros/ros.h>
#include <std_msgs/String.h>
#include <quadrotor_msgs/PositionCommand.h>

#include <string>

class PlannerCmdMux {
public:
  /**
   * Construct the planner command multiplexer.
   *
   * @param[in] nh  ROS node handle
   */
  PlannerCmdMux()
      : nh_("~") {
    nh_.param("default_mode", active_mode_, std::string("ego"));
    nh_.param("ego_mode", ego_mode_, std::string("ego"));
    nh_.param("elastic_mode", elastic_mode_, std::string("elastic"));
    nh_.param("input_timeout", input_timeout_, 0.5);

    cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 20);
    ego_cmd_sub_ = nh_.subscribe("ego_position_cmd", 20, &PlannerCmdMux::egoCmdCallback, this,
                                 ros::TransportHints().tcpNoDelay());
    elastic_cmd_sub_ = nh_.subscribe("elastic_position_cmd", 20, &PlannerCmdMux::elasticCmdCallback, this,
                                     ros::TransportHints().tcpNoDelay());
    mode_sub_ = nh_.subscribe("mode", 10, &PlannerCmdMux::modeCallback, this,
                              ros::TransportHints().tcpNoDelay());

    ROS_INFO_STREAM("[planner_cmd_mux] start with mode=" << active_mode_
                    << ", ego_mode=" << ego_mode_
                    << ", elastic_mode=" << elastic_mode_
                    << ", input_timeout=" << input_timeout_);
  }

private:
  /**
   * Check whether a mode string is recognized.
   *
   * @param[in] mode  Mode string
   * @return True if mode is known
   */
  bool isKnownMode(const std::string& mode) const {
    return mode == ego_mode_ || mode == elastic_mode_;
  }

  /**
   * Check whether a timestamp is within the input timeout window.
   *
   * @param[in] stamp  Message timestamp [s]
   * @return True if the message is still fresh
   */
  bool isFresh(const ros::Time& stamp) const {
    if (input_timeout_ <= 0.0) return true;
    if (stamp.isZero()) return false;
    return (ros::Time::now() - stamp).toSec() <= input_timeout_;
  }

  /**
   * Check whether the elastic tracker command is usable (fresh and from current session).
   *
   * @return True if elastic command can be forwarded
   */
  bool isElasticCmdUsable() const {
    if (!has_elastic_cmd_ || !isFresh(elastic_cmd_stamp_)) return false;
    if (!elastic_mode_start_time_.isZero() && elastic_cmd_stamp_ < elastic_mode_start_time_) return false;
    if (blocked_elastic_traj_id_ >= 0 && last_elastic_cmd_.trajectory_id == blocked_elastic_traj_id_) return false;
    return true;
  }

  /**
   * Publish a command only if the given mode matches the active mode.
   *
   * @param[in] cmd         Position command to publish
   * @param[in] source_mode Source mode identifier
   */
  void publishIfActive(const quadrotor_msgs::PositionCommand& cmd,
                       const std::string& source_mode) {
    if (active_mode_ != source_mode) return;
    cmd_pub_.publish(cmd);
  }

  /**
   * Publish the last cached command for the currently active mode.
   *
   * @param[in] source  Caller identifier for logging
   */
  void publishLastForMode(const std::string& source) {
    if (active_mode_ == ego_mode_) {
      if (has_ego_cmd_ && isFresh(ego_cmd_stamp_)) {
        cmd_pub_.publish(last_ego_cmd_);
      } else {
        ROS_WARN_STREAM_THROTTLE(1.0, "[planner_cmd_mux] no fresh ego cmd after switch from " << source);
      }
      return;
    }

    if (active_mode_ == elastic_mode_) {
      if (isElasticCmdUsable()) {
        cmd_pub_.publish(last_elastic_cmd_);
      } else {
        ROS_WARN_STREAM_THROTTLE(1.0, "[planner_cmd_mux] no fresh elastic cmd after switch from " << source);
      }
    }
  }

  /**
   * Callback for mode switch messages.
   *
   * @param[in] msg  Mode string message
   */
  void modeCallback(const std_msgs::String::ConstPtr& msg) {
    const std::string next_mode = msg->data;
    if (!isKnownMode(next_mode)) {
      ROS_WARN_STREAM("[planner_cmd_mux] ignore unknown mode: " << next_mode);
      return;
    }
    if (next_mode == active_mode_) return;

    active_mode_ = next_mode;
    if (active_mode_ == elastic_mode_) {
      elastic_mode_start_time_ = ros::Time::now();
      blocked_elastic_traj_id_ = has_elastic_cmd_ ? last_elastic_cmd_.trajectory_id : -1;
      has_elastic_cmd_ = false;
    } else {
      blocked_elastic_traj_id_ = -1;
    }
    ROS_INFO_STREAM("[planner_cmd_mux] switch mode to " << active_mode_);
    publishLastForMode("mode_callback");
  }

  /**
   * Callback for EGO planner position commands.
   *
   * @param[in] msg  Position command message
   */
  void egoCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    last_ego_cmd_ = *msg;
    ego_cmd_stamp_ = ros::Time::now();
    has_ego_cmd_ = true;
    publishIfActive(last_ego_cmd_, ego_mode_);
  }

  /**
   * Callback for elastic tracker position commands.
   *
   * @param[in] msg  Position command message
   */
  void elasticCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    if (active_mode_ == elastic_mode_ && blocked_elastic_traj_id_ >= 0) {
      if (msg->trajectory_id == blocked_elastic_traj_id_) {
        ROS_WARN_STREAM_THROTTLE(0.5, "[planner_cmd_mux] drop stale elastic trajectory_id="
                                          << msg->trajectory_id << " after mode switch");
        return;
      }
      blocked_elastic_traj_id_ = -1;
    }
    last_elastic_cmd_ = *msg;
    elastic_cmd_stamp_ = ros::Time::now();
    has_elastic_cmd_ = true;
    publishIfActive(last_elastic_cmd_, elastic_mode_);
  }

  ros::NodeHandle nh_;
  ros::Publisher cmd_pub_;
  ros::Subscriber ego_cmd_sub_;
  ros::Subscriber elastic_cmd_sub_;
  ros::Subscriber mode_sub_;

  std::string active_mode_;
  std::string ego_mode_;
  std::string elastic_mode_;
  double input_timeout_{0.5};

  bool has_ego_cmd_{false};
  bool has_elastic_cmd_{false};
  int blocked_elastic_traj_id_{-1};
  ros::Time elastic_mode_start_time_;
  ros::Time ego_cmd_stamp_;
  ros::Time elastic_cmd_stamp_;
  quadrotor_msgs::PositionCommand last_ego_cmd_;
  quadrotor_msgs::PositionCommand last_elastic_cmd_;
};

/**
 * ROS node entry point for planner command multiplexer.
 *
 * @param[in] argc  Argument count
 * @param[in] argv  Argument vector
 * @return Exit code
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "planner_cmd_mux");
  PlannerCmdMux mux;
  ros::spin();
  return 0;
}
