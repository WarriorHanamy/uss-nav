#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry
import tf

br = tf.TransformBroadcaster()


def normalize_frame(frame_id, fallback):
    frame_id = (frame_id or "").strip()
    if not frame_id:
        return fallback
    return frame_id.lstrip("/")


def odom_callback(odom_msg):
    position = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z)
    orientation = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
    parent_frame = normalize_frame(odom_msg.header.frame_id, "world")
    child_frame = normalize_frame(odom_msg.child_frame_id, "robot_frame")
    br.sendTransform(position, orientation, odom_msg.header.stamp or rospy.Time.now(), child_frame, parent_frame)

def main():
    rospy.init_node('follow_robot_view', anonymous=True)
    odom_topic  = rospy.get_param('~odom_topic', '/odom_topic')
    rospy.loginfo('frame_tf | tf-odom -> %s', odom_topic)
    odom_sub    = rospy.Subscriber(odom_topic, Odometry, odom_callback)
    rate        = rospy.Rate(60)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
