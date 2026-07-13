#ifndef _RC_REPLAN_TRIGGER_H_
#define _RC_REPLAN_TRIGGER_H_

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Bool.h>

#include <vector>

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

  std::vector<int> channel_indices_;  // 参与触发的通道下标列表(0-based), 从 launch 参数可配
  double           hold_duration_{0.0}; // 长按持续时间阈值(s), 0=即时触发

  bool      locked_{false};           // 已触发等待释放, 防止重复触发
  ros::Time hold_begin_time_;         // 长按计时起点, isZero()=未在计时
};

}  // namespace ego_planner

#endif  // _RC_REPLAN_TRIGGER_H_
