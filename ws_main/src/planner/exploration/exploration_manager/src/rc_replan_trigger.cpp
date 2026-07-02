#include <exploration_manager/rc_replan_trigger.h>

namespace ego_planner {

RcReplanTrigger::RcReplanTrigger(ros::NodeHandle& nh) : nh_(nh) {
  nh_.param("rc_replan_trigger/ch8_idx", ch8_idx_, 7);
  nh_.param("rc_replan_trigger/ch9_idx", ch9_idx_, 8);

  rc_sub_ = nh_.subscribe("/mavros/rc/in", 10,
      &RcReplanTrigger::rcCallback, this,
      ros::TransportHints().tcpNoDelay());
  replan_pub_ = nh_.advertise<std_msgs::Bool>("/object_id_nav_replan", 10);

  ROS_INFO("[RcReplan] Started, ch8_idx=%d ch9_idx=%d, waiting for RC trigger...",
           ch8_idx_, ch9_idx_);
}

void RcReplanTrigger::rcCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
  int sz = static_cast<int>(msg->channels.size());
  if (sz <= ch8_idx_ || sz <= ch9_idx_) {
    ROS_WARN_THROTTLE(5.0, "[RcReplan] RC channels too few (got %d, need at least %d)",
                      sz, std::max(ch8_idx_, ch9_idx_) + 1);
    return;
  }

  bool both_high = (msg->channels[ch8_idx_] > 1500) && (msg->channels[ch9_idx_] > 1500);
  bool both_low  = (msg->channels[ch8_idx_] <= 1500) && (msg->channels[ch9_idx_] <= 1500);

  if (!locked_ && both_high) {
    std_msgs::Bool trigger;
    trigger.data = true;
    replan_pub_.publish(trigger);
    locked_ = true;
    ROS_INFO("[RcReplan] Triggered! Locked until release (ch8=%d, ch9=%d).",
             msg->channels[ch8_idx_], msg->channels[ch9_idx_]);
  } else if (locked_ && both_low) {
    locked_ = false;
    ROS_INFO("[RcReplan] Released, ready for next trigger.");
  }
}

}  // namespace ego_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "rc_replan_trigger");
  ros::NodeHandle nh("~");

  ego_planner::RcReplanTrigger trigger(nh);

  ros::spin();
  return 0;
}
