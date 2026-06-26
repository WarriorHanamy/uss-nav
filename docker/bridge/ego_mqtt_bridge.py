#!/usr/bin/env python3
"""
ROS-to-MQTT bridge for EGO Planner test telemetry.

Subscribes to ego-planner topics and forwards them as JSON over MQTT.
"""

import argparse
import json
import signal
import sys
import threading
import time

import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import Header, Bool
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import EgoPlannerResult, EgoStateTrigger
from traj_utils.msg import DataDisp


class EgoMqttBridge:
    def __init__(self, mqtt_host: str, mqtt_port: int, test_id: str, topic_prefix: str):
        self.test_id = test_id
        self.topic_prefix = topic_prefix
        self.running = True

        # MQTT client
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

        rospy.Subscriber("/drone_0_visual_slam/odom", Odometry, self._odom_cb)
        rospy.Subscriber(
            "/planning/ego_plan_result", EgoPlannerResult, self._plan_result_cb
        )
        rospy.Subscriber("/planning/data_display", DataDisp, self._data_disp_cb)
        rospy.Subscriber(
            "/planning/ego_state_trigger", EgoStateTrigger, self._state_trigger_cb
        )
        rospy.Subscriber("/exec_finish_trigger", Bool, self._exec_finish_cb)

        rospy.loginfo(
            f"ego_mqtt_bridge [{self.test_id}] started → {self.topic_prefix}/{self.test_id}/*"
        )

        rate = rospy.Rate(10)
        while self.running and not rospy.is_shutdown():
            rate.sleep()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mqtt-host", default="host.docker.internal")
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
