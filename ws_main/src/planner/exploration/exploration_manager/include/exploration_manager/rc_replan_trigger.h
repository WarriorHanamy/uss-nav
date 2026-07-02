#ifndef _RC_REPLAN_TRIGGER_H_
#define _RC_REPLAN_TRIGGER_H_

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Bool.h>

namespace ego_planner {

class RcReplanTrigger {
public:
  RcReplanTrigger(ros::NodeHandle& nh);
  ~RcReplanTrigger() = default;

private:
  void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber rc_sub_;
  ros::Publisher  replan_pub_;

  bool locked_{false};  // 已触发等待释放, 防止重复触发
  int  ch8_idx_{7};     // ch8 在 channels 数组中的下标(0-based)
  int  ch9_idx_{8};     // ch9 在 channels 数组中的下标(0-based)
};

}  // namespace ego_planner

#endif  // _RC_REPLAN_TRIGGER_H_
