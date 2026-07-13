#include <mission_executive/rc_replan_trigger.h>

namespace ego_planner {

RcReplanTrigger::RcReplanTrigger(ros::NodeHandle& nh) : nh_(nh) {
  nh_.param("rc_replan_trigger/hold_duration", hold_duration_, 0.0);

  // 从参数加载通道列表, 为空则兜底为 {7, 8} (ch8 + ch9)
  XmlRpc::XmlRpcValue channel_param;
  if (nh_.getParam("rc_replan_trigger/channels", channel_param) &&
      channel_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < channel_param.size(); ++i) {
      channel_indices_.push_back(static_cast<int>(channel_param[i]));
    }
  }
  if (channel_indices_.empty()) {
    channel_indices_ = {7, 8};
  }

  rc_sub_ = nh_.subscribe("/mavros/rc/in", 10,
      &RcReplanTrigger::rcCallback, this,
      ros::TransportHints().tcpNoDelay());
  replan_pub_ = nh_.advertise<std_msgs::Bool>("/object_id_nav_replan", 10);

  // 打印配置
  std::stringstream ss;
  for (size_t i = 0; i < channel_indices_.size(); ++i) {
    if (i > 0) ss << "+";
    ss << "ch" << (channel_indices_[i] + 1);
  }
  ROS_INFO("[RcReplan] Started: channels=[%s], hold_duration=%.1fs, waiting for RC trigger...",
           ss.str().c_str(), hold_duration_);
}

void RcReplanTrigger::rcCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
  int sz = static_cast<int>(msg->channels.size());

  // 检查所有触发通道是否在范围内
  bool all_high = true, all_low = true;
  for (int idx : channel_indices_) {
    if (idx >= sz) {
      ROS_WARN_THROTTLE(5.0, "[RcReplan] RC data too small (got %d, need ch%d)", sz, idx + 1);
      all_high = false;
      break;
    }
    if (msg->channels[idx] <= 1500) all_high = false;
    if (msg->channels[idx] > 1500)  all_low  = false;
  }

  if (!locked_) {
    if (all_high) {
      if (hold_duration_ <= 0.0) {
        // 即时触发(当前逻辑)
        std_msgs::Bool trigger;
        trigger.data = true;
        replan_pub_.publish(trigger);
        locked_ = true;
        ROS_INFO("[RcReplan] Triggered! Locked until release.");
      } else {
        // 长按计时模式
        if (hold_begin_time_.isZero()) {
          hold_begin_time_ = ros::Time::now();
        }
        double elapsed = (ros::Time::now() - hold_begin_time_).toSec();
        if (elapsed >= hold_duration_) {
          std_msgs::Bool trigger;
          trigger.data = true;
          replan_pub_.publish(trigger);
          locked_ = true;
          hold_begin_time_ = ros::Time();  // 重置计时器
          ROS_INFO("[RcReplan] Hold-triggered (%.1fs)! Locked until release.", elapsed);
        }
      }
    } else {
      // 未全按: 取消计时
      if (!hold_begin_time_.isZero()) {
        hold_begin_time_ = ros::Time();
      }
    }
  } else {
    if (all_low) {
      locked_ = false;
      hold_begin_time_ = ros::Time();
      ROS_INFO("[RcReplan] Released, ready for next trigger.");
    }
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
