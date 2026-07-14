#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""VLA_Swarm MoGe v2 fallback 目标定位节点。"""

import collections
import os
import sys
import threading

import cv2
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import VLASearchBBox, VLASearchTarget
from scene_graph.msg import VLASearchObservation
from sensor_msgs.msg import CameraInfo


def quaternion_matrix(quaternion):
    """将 xyzw 四元数转换为旋转矩阵。"""
    x, y, z, w = quaternion
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    scale = 2.0 / norm
    return np.array(
        [
            [1.0 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
            [scale * (x * y + z * w), 1.0 - scale * (x * x + z * z), scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1.0 - scale * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def yaw_matrix(yaw_degrees):
    yaw = np.deg2rad(yaw_degrees)
    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


class MoGeTargetEstimator:
    def __init__(self):
        self.lock = threading.Lock()
        self.history_duration = float(rospy.get_param("~history_duration", 8.0))
        self.max_sync_delta = float(rospy.get_param("~max_sync_delta", 0.25))
        self.bbox_topic = rospy.get_param("~bbox_topic", "/vla_search/bbox/fallback")
        self.target_topic = rospy.get_param("~target_topic", "/vla_search/target")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )
        self.odom_topic = rospy.get_param("~odom_topic", "/odom_world")
        self.observation_topic = rospy.get_param(
            "~observation_topic", "/vla_search/observation"
        )
        self.moge_root = rospy.get_param("~moge_root", "")
        self.model_path = rospy.get_param("~model_path", "")
        self.device_name = rospy.get_param("~device", "cuda")
        self.depth_gate = float(rospy.get_param("~depth_gate", 0.15))
        self.min_depth = float(rospy.get_param("~min_depth", 0.2))
        self.max_depth = float(rospy.get_param("~max_depth", 20.0))

        self.image_history = [collections.deque() for _ in range(4)]
        self.observation_snapshots = {}
        self.odom_history = collections.deque()
        self.intrinsics = None
        self.model = None
        self.torch = None

        camera_rpy = np.deg2rad(
            np.asarray(
                rospy.get_param("~camera_in_base_rpy_deg", [0.0, 0.0, 0.0])[:3],
                dtype=np.float64,
            )
        )
        roll, pitch, yaw = camera_rpy
        rotation_x = np.array(
            [[1.0, 0.0, 0.0], [0.0, np.cos(roll), -np.sin(roll)], [0.0, np.sin(roll), np.cos(roll)]]
        )
        rotation_y = np.array(
            [[np.cos(pitch), 0.0, np.sin(pitch)], [0.0, 1.0, 0.0], [-np.sin(pitch), 0.0, np.cos(pitch)]]
        )
        self.camera_rotation = yaw_matrix(np.rad2deg(yaw)) @ rotation_y @ rotation_x
        self.camera_translation = np.asarray(
            rospy.get_param("~camera_in_base_t_xyz", [0.17, 0.10, 0.0])[:3],
            dtype=np.float64,
        )

        self.target_publisher = rospy.Publisher(
            self.target_topic, VLASearchTarget, queue_size=10
        )
        rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1
        )
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=50)
        rospy.Subscriber(
            self.observation_topic,
            VLASearchObservation,
            self.observation_callback,
            queue_size=10,
            buff_size=2**24,
        )
        rospy.Subscriber(
            self.bbox_topic, VLASearchBBox, self.bbox_callback, queue_size=10
        )

    def trim_history(self, history):
        oldest = rospy.Time.now() - rospy.Duration(self.history_duration)
        while history and history[0][0] < oldest:
            history.popleft()

    def camera_info_callback(self, message):
        with self.lock:
            self.intrinsics = (
                float(message.K[0]),
                float(message.K[4]),
                float(message.K[2]),
                float(message.K[5]),
                int(message.width),
                int(message.height),
            )

    def odom_callback(self, message):
        with self.lock:
            self.odom_history.append((message.header.stamp, message))
            self.trim_history(self.odom_history)

    def observation_callback(self, message):
        if message.observation_index >= len(self.image_history):
            return
        encoded = np.frombuffer(message.image.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            rospy.logwarn_throttle(2.0, "[VLA_SEARCH][MoGe] Failed to decode image.")
            return
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        with self.lock:
            odom = self.nearest_item(self.odom_history, message.header.stamp)
            capture_pose = (
                odom.pose.pose if odom is not None else message.odom_pose
            )
            history = self.image_history[message.observation_index]
            history.append((message.header.stamp, image))
            self.trim_history(history)
            key = (
                int(message.task_session_id),
                int(message.observation_batch_id),
                int(message.observation_index),
            )
            self.observation_snapshots[key] = (
                message.header.stamp,
                image,
                capture_pose,
            )
            if len(self.observation_snapshots) > 16:
                oldest_key = min(
                    self.observation_snapshots,
                    key=lambda item: self.observation_snapshots[item][0],
                )
                self.observation_snapshots.pop(oldest_key, None)

    def nearest_item(self, history, stamp):
        if not history:
            return None
        best = min(history, key=lambda item: abs((item[0] - stamp).to_sec()))
        return best[1] if abs((best[0] - stamp).to_sec()) <= self.max_sync_delta else None

    def ensure_model(self):
        """仅在 LiDAR fallback 实际到达时加载 MoGe，正常主通道不占用显存。"""
        if self.model is not None:
            return
        if not self.moge_root or not os.path.isdir(self.moge_root):
            raise RuntimeError("~moge_root does not point to a MoGe v2 source directory")
        if not self.model_path:
            raise RuntimeError("~model_path is empty")
        if self.moge_root not in sys.path:
            sys.path.insert(0, self.moge_root)

        import torch
        from moge.model.v2 import MoGeModel

        self.torch = torch
        device = torch.device(
            self.device_name
            if self.device_name != "cuda" or torch.cuda.is_available()
            else "cpu"
        )
        rospy.loginfo(
            "[VLA_SEARCH][MoGe] Loading model %s on %s",
            self.model_path,
            device,
        )
        self.model = MoGeModel.from_pretrained(self.model_path).to(device)
        self.device = device

    def publish_result(self, request, success, source, position=None, error=""):
        result = VLASearchTarget()
        result.header.stamp = rospy.Time.now()
        result.header.frame_id = request.header.frame_id or "world"
        result.task_session_id = request.task_session_id
        result.observation_batch_id = request.observation_batch_id
        result.request_id = request.request_id
        result.observation_index = request.observation_index
        result.success = success
        result.source = source
        result.error = error
        result.pose.orientation.w = 1.0
        if position is not None:
            result.pose.position.x = float(position[0])
            result.pose.position.y = float(position[1])
            result.pose.position.z = float(position[2])
        self.target_publisher.publish(result)

    def bbox_callback(self, request):
        if request.observation_index >= 4:
            self.publish_result(
                request,
                False,
                VLASearchTarget.SOURCE_MOGE,
                error="observation_index is outside [0,3]",
            )
            return

        capture_stamp = (
            request.header.stamp
            if request.header.stamp != rospy.Time()
            else rospy.Time.now()
        )
        with self.lock:
            observation = self.observation_snapshots.get(
                (
                    int(request.task_session_id),
                    int(request.observation_batch_id),
                    int(request.observation_index),
                )
            )
            image = observation[1] if observation is not None else self.nearest_item(
                self.image_history[request.observation_index], capture_stamp
            )
            capture_pose = observation[2] if observation is not None else None
            odom = (
                None
                if capture_pose is not None
                else self.nearest_item(self.odom_history, capture_stamp)
            )
            intrinsics = self.intrinsics
        if image is None or (capture_pose is None and odom is None):
            self.publish_result(
                request,
                False,
                VLASearchTarget.SOURCE_MOGE,
                error="historical image or odometry is unavailable",
            )
            return

        try:
            self.ensure_model()
            height, width = image.shape[:2]
            x0 = max(0, min(width - 1, min(request.bbox_xyxy[0], request.bbox_xyxy[2])))
            x1 = max(0, min(width - 1, max(request.bbox_xyxy[0], request.bbox_xyxy[2])))
            y0 = max(0, min(height - 1, min(request.bbox_xyxy[1], request.bbox_xyxy[3])))
            y1 = max(0, min(height - 1, max(request.bbox_xyxy[1], request.bbox_xyxy[3])))
            if x1 <= x0 or y1 <= y0:
                raise RuntimeError("bbox is invalid after image-bound clamping")

            tensor = self.torch.tensor(
                image / 255.0, dtype=self.torch.float32, device=self.device
            ).permute(2, 0, 1)
            output = self.model.infer(tensor)
            depth = output["depth"]
            if self.torch.is_tensor(depth):
                depth = depth.detach().cpu().numpy()
            roi_depth = depth[y0:y1, x0:x1]
            valid_depth = roi_depth[
                np.isfinite(roi_depth)
                & (roi_depth >= self.min_depth)
                & (roi_depth <= self.max_depth)
            ]
            if valid_depth.size == 0:
                raise RuntimeError("MoGe depth ROI contains no valid values")
            median_depth = float(np.median(valid_depth))
            gated_depth = valid_depth[
                np.abs(valid_depth - median_depth) <= self.depth_gate
            ]
            target_depth = float(
                np.mean(gated_depth if gated_depth.size >= 3 else valid_depth)
            )

            if intrinsics is not None:
                fx, fy, cx, cy = intrinsics[:4]
            else:
                normalized = output["intrinsics"]
                if self.torch.is_tensor(normalized):
                    normalized = normalized.detach().cpu().numpy()
                fx = float(normalized[0, 0]) * width
                fy = float(normalized[1, 1]) * height
                cx = float(normalized[0, 2]) * width
                cy = float(normalized[1, 2]) * height

            pixel_x = 0.5 * (x0 + x1)
            pixel_y = 0.5 * (y0 + y1)
            point_camera = np.array(
                [
                    target_depth,
                    -(pixel_x - cx) / fx * target_depth,
                    -(pixel_y - cy) / fy * target_depth,
                ],
                dtype=np.float64,
            )
            point_base = (
                self.camera_rotation @ point_camera
                + self.camera_translation
            )
            pose = capture_pose if capture_pose is not None else odom.pose.pose
            position = pose.position
            orientation = pose.orientation
            world_from_base = quaternion_matrix(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )
            target_world = world_from_base @ point_base + np.array(
                [position.x, position.y, position.z], dtype=np.float64
            )
            self.publish_result(
                request,
                True,
                VLASearchTarget.SOURCE_MOGE,
                position=target_world,
            )
        except Exception as error:
            rospy.logerr("[VLA_SEARCH][MoGe] Target estimation failed: %s", error)
            self.publish_result(
                request,
                False,
                VLASearchTarget.SOURCE_MOGE,
                error=str(error),
            )


def main():
    rospy.init_node("vla_search_moge_target_estimator")
    MoGeTargetEstimator()
    rospy.spin()


if __name__ == "__main__":
    main()
