#!/usr/bin/env python3
"""
REC_LEARN: extended ROS-to-MQTT bridge for EGO Planner test telemetry.

Collects 5 data dimensions:
  pos_cmd     – commanded position/velocity/acceleration    (~50 Hz)
  imu         – angular velocity, linear acceleration       (100 Hz)
  body_cloud  – sensor-frame point cloud (downsampled)       (1 Hz)
  body_depth  – depth image (JPEG compressed)               (1 Hz)
  obstacles   – occupancy grid point cloud                   (~1 Hz)

Plus the original: odom, plan_result, data_disp, state_trigger, exec_finish.
"""

import argparse
import base64
import json
import signal
import sys
import threading
import time

import cv2
import numpy as np
import paho.mqtt.client as mqtt
import rospy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import EgoPlannerResult, EgoStateTrigger, PositionCommand
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import Bool
from traj_utils.msg import DataDisp
from visualization_msgs.msg import Marker

# Downsampling: keep 1 point every N
CLOUD_DECIMATE = 10
# Depth image: scale factor for JPEG compression
DEPTH_JPEG_QUALITY = 85


class EgoMqttBridge:
    def __init__(self, mqtt_host: str, mqtt_port: int, test_id: str, topic_prefix: str):
        self.test_id = test_id
        self.topic_prefix = topic_prefix
        self.running = True
        self._bridge = CvBridge()

        self.mqtt_client = mqtt.Client(client_id=f"ego-bridge-{test_id}")
        self.mqtt_client.connect_async(mqtt_host, mqtt_port, 60)
        self.mqtt_client.loop_start()

        signal.signal(signal.SIGINT, self._shutdown)
        signal.signal(signal.SIGTERM, self._shutdown)

    def _shutdown(self, *_args):
        self.running = False
        self.mqtt_client.disconnect()
        self.mqtt_client.loop_stop()

    def publish(self, subtopic: str, data: dict):
        topic = f"{self.topic_prefix}/{self.test_id}/{subtopic}"
        payload = json.dumps(data, default=str)
        self.mqtt_client.publish(topic, payload, qos=0)

    def _odom_cb(self, msg: Odometry):
        self.publish(
            "odom",
            {
                "ts": msg.header.stamp.to_sec(),
                "pos": [
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                ],
                "vel": [
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ],
                "orient": [
                    msg.pose.pose.orientation.w,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                ],
            },
        )

    def _pos_cmd_cb(self, msg: PositionCommand):
        self.publish(
            "pos_cmd",
            {
                "ts": msg.header.stamp.to_sec(),
                "pos": [msg.position.x, msg.position.y, msg.position.z],
                "vel": [msg.velocity.x, msg.velocity.y, msg.velocity.z],
                "acc": [msg.acceleration.x, msg.acceleration.y, msg.acceleration.z],
                "yaw": msg.yaw,
                "yaw_dot": msg.yaw_dot,
            },
        )

    def _imu_cb(self, msg: Imu):
        self.publish(
            "imu",
            {
                "ts": msg.header.stamp.to_sec(),
                "orient": [
                    msg.orientation.w,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                ],
                "ang_vel": [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ],
                "lin_acc": [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ],
            },
        )

    def _body_cloud_cb(self, msg: PointCloud2):
        # Decimate: only process every N-th frame
        if getattr(self, "_cloud_frame_count", 0) % 5 != 0:
            self._cloud_frame_count = getattr(self, "_cloud_frame_count", 0) + 1
            return
        self._cloud_frame_count = getattr(self, "_cloud_frame_count", 0) + 1

        # Parse PointCloud2 into flat lists
        pts = []
        width, height = msg.width, msg.height
        step = msg.point_step
        data = msg.data

        for i in range(0, len(data), step * CLOUD_DECIMATE):
            row = data[i : i + step]
            if len(row) < step:
                break
            x, y, z = (
                _read_float(row, 0),
                _read_float(row, 4),
                _read_float(row, 8),
            )
            pts.extend([x, y, z])

        self.publish(
            "body_cloud",
            {
                "ts": msg.header.stamp.to_sec(),
                "pts": pts[:3000],
            },
        )

    def _body_depth_cb(self, msg: Image):
        if getattr(self, "_depth_frame_count", 0) % 5 != 0:
            self._depth_frame_count = getattr(self, "_depth_frame_count", 0) + 1
            return
        self._depth_frame_count = getattr(self, "_depth_frame_count", 0) + 1

        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception:
            return

        # Normalize 32FC1 -> 8-bit for JPEG
        cv_norm = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        _, jpeg_buf = cv2.imencode(
            ".jpg",
            cv_norm,
            [
                cv2.IMWRITE_JPEG_QUALITY,
                DEPTH_JPEG_QUALITY,
            ],
        )
        b64 = base64.b64encode(jpeg_buf.tobytes()).decode("ascii")

        self.publish(
            "body_depth",
            {
                "ts": msg.header.stamp.to_sec(),
                "width": msg.width,
                "height": msg.height,
                "jpeg_b64": b64,
            },
        )

    def _plan_result_cb(self, msg: EgoPlannerResult):
        self.publish(
            "plan_result",
            {
                "ts": rospy.Time.now().to_sec(),
                "goal": [msg.planner_goal.x, msg.planner_goal.y, msg.planner_goal.z],
                "plan_times": msg.plan_times,
                "plan_status": msg.plan_status,
                "modify_status": msg.modify_status,
            },
        )

    def _data_disp_cb(self, msg: DataDisp):
        self.publish(
            "data_disp",
            {
                "ts": msg.header.stamp.to_sec(),
                "a": msg.a,
                "b": msg.b,
                "c": msg.c,
                "d": msg.d,
                "e": msg.e,
            },
        )

    def _state_trigger_cb(self, msg: EgoStateTrigger):
        self.publish(
            "state_trigger",
            {
                "ts": msg.header.stamp.to_sec(),
                "triggered": msg.data,
            },
        )

    def _exec_finish_cb(self, msg: Bool):
        self.publish(
            "exec_finish",
            {
                "ts": rospy.Time.now().to_sec(),
                "finished": msg.data,
            },
        )

    def run(self):
        rospy.init_node(f"ego_mqtt_bridge_{self.test_id}", anonymous=True)

        # Original topics
        rospy.Subscriber("/drone_0_visual_slam/odom", Odometry, self._odom_cb)
        rospy.Subscriber(
            "/planning/ego_plan_result", EgoPlannerResult, self._plan_result_cb
        )
        rospy.Subscriber("/planning/data_display", DataDisp, self._data_disp_cb)
        rospy.Subscriber(
            "/planning/ego_state_trigger", EgoStateTrigger, self._state_trigger_cb
        )
        rospy.Subscriber("/exec_finish_trigger", Bool, self._exec_finish_cb)

        # REC_LEARN: extended topics for post-processing
        rospy.Subscriber("/drone_0_planning/pos_cmd", PositionCommand, self._pos_cmd_cb)
        rospy.Subscriber("/drone_0_quadrotor_simulator_so3/imu", Imu, self._imu_cb)
        rospy.Subscriber(
            "/drone_0_pcl_render_node/sensor_cloud", PointCloud2, self._body_cloud_cb
        )
        rospy.Subscriber(
            "/drone_0_pcl_render_node/depth_img", Image, self._body_depth_cb
        )
        rospy.Subscriber(
            "/rec_learn_ego_planner_node/grid_map/occupancy",
            PointCloud2,
            self._obstacles_cb,
        )
        rospy.Subscriber(
            "/rec_learn_ego_planner_node/grid_map/occupancy_inflate",
            PointCloud2,
            self._inflated_cb,
        )
        rospy.Subscriber(
            "/rec_learn_ego_planner_node/optimal_list",
            Marker,
            self._plan_traj_cb,
        )

        rospy.loginfo(
            f"ego_mqtt_bridge [{self.test_id}] started → {self.topic_prefix}/{self.test_id}/*"
        )

        rate = rospy.Rate(10)
        while self.running and not rospy.is_shutdown():
            rate.sleep()

    def _obstacles_cb(self, msg: PointCloud2):
        pts = []
        step = msg.point_step
        data = msg.data
        for i in range(0, min(len(data), step * 5000), step):
            row = data[i : i + step]
            if len(row) < step:
                break
            x = _read_float(row, 0)
            y = _read_float(row, 4)
            z = _read_float(row, 8)
            pts.extend([x, y, z])

        self.publish(
            "obstacles",
            {
                "ts": msg.header.stamp.to_sec(),
                "pts": pts,
            },
        )

    def _inflated_cb(self, msg: PointCloud2):
        pts = []
        step = msg.point_step
        data = msg.data
        for i in range(0, min(len(data), step * 5000), step):
            row = data[i : i + step]
            if len(row) < step:
                break
            pts.extend(
                [
                    _read_float(row, 0),
                    _read_float(row, 4),
                    _read_float(row, 8),
                ]
            )
        self.publish("inflated", {"ts": msg.header.stamp.to_sec(), "pts": pts})

    def _plan_traj_cb(self, msg: Marker):
        pts = [[p.x, p.y, p.z] for p in msg.points]
        self.publish("plan_traj", {"ts": rospy.Time.now().to_sec(), "pts": pts})


def _read_float(data: bytes, offset: int) -> float:
    """Read a 4-byte little-endian float from bytes."""
    import struct

    return struct.unpack("<f", data[offset : offset + 4])[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mqtt-host", default="localhost")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--test-id", required=True)
    parser.add_argument("--topic-prefix", default="test")
    args = parser.parse_args()

    bridge = EgoMqttBridge(
        args.mqtt_host, args.mqtt_port, args.test_id, args.topic_prefix
    )
    try:
        bridge.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
