#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <quadrotor_msgs/GoalSet.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Dense>

ros::Publisher goal_pub_;
ros::Subscriber odom_sub;
ros::Subscriber move_command_sub;
nav_msgs::Odometry _odom_msg;
int drone_id_;

/**
 * Callback for odometry updates.
 *
 * @param[in] msg  Odometry message
 */
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
  _odom_msg = *msg;
}

/**
 * Publish a position command as a GoalSet message.
 *
 * @param[in] pos  Target position [m]
 * @param[in] yaw  Target yaw [rad]
 */
void pubCmd(const Eigen::Vector3d& pos, const double yaw) 
{
  quadrotor_msgs::GoalSet goal_msg;
  geometry_msgs::Point pt;
  pt.x = pos.x(); pt.y = pos.y(); pt.z = pos.z();
  goal_msg.to_drone_ids.push_back(drone_id_);
  goal_msg.goal.push_back(pt);
  goal_msg.yaw.push_back(yaw);
  goal_msg.look_forward = false;
  goal_msg.goal_to_follower = false;
  goal_pub_.publish(goal_msg);
}

/**
 * Compute a commanded position and yaw from a body-frame displacement.
 *
 * Transforms the displacement (dp, dyaw) from the body frame to the world
 * frame using the current odometry orientation.
 *
 * @param[in]  dp    Body-frame displacement [m]
 * @param[in]  dyaw  Body-frame yaw offset [rad]
 * @param[out] pos   World-frame commanded position [m]
 * @param[out] yaw   World-frame commanded yaw [rad]
 */
void getCmd(const Eigen::Vector3d& dp, const double& dyaw,
            Eigen::Vector3d& pos, double& yaw)
{
  Eigen::Quaterniond q(_odom_msg.pose.pose.orientation.w,
                        _odom_msg.pose.pose.orientation.x, 
                        _odom_msg.pose.pose.orientation.y, 
                        _odom_msg.pose.pose.orientation.z);

  pos = Eigen::Vector3d(_odom_msg.pose.pose.position.x, 
                        _odom_msg.pose.pose.position.y,
                        _odom_msg.pose.pose.position.z);

  pos = pos + q.toRotationMatrix() * dp;

  double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  double odom_yaw = std::atan2(siny_cosp, cosy_cosp);
  yaw = odom_yaw + dyaw;

  if (yaw > M_PI)
      yaw -= 2*M_PI;
  else if (yaw < -M_PI)
      yaw += 2*M_PI;
}

/**
 * Callback for joystick move commands.
 *
 * @param[in] msg  Joystick message with axes (dx, dy, dz, dyaw)
 */
void moveCommandCallback(const sensor_msgs::Joy::ConstPtr& msg) 
{
  Eigen::Vector3d d_position(msg->axes.at(0), msg->axes.at(1), msg->axes.at(2));
  double d_yaw = msg->axes.at(3);

  Eigen::Vector3d pos_cmd;
  double yaw_cmd;
  getCmd(d_position, d_yaw, pos_cmd, yaw_cmd);
  pubCmd(pos_cmd, yaw_cmd);
}

/**
 * ROS node entry point for quadrotor move interface.
 *
 * @param[in] argc  Argument count
 * @param[in] argv  Argument vector
 * @return Exit code
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "quadrotor_move_interface");

  ros::NodeHandle nh("~");
  nh.param("drone_id", drone_id_, 0);

  odom_sub = nh.subscribe("odometry", 1, odometryCallback);
  move_command_sub = nh.subscribe("/bridge/move_command", 1, moveCommandCallback);

  goal_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/bridge/goal_user2brig", 1);

  ros::spin();

  return 0;
}


