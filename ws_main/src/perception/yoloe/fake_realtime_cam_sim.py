#!/usr/bin/env python3

import copy
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage

try:
    from scene_graph.msg import EncodeMask
    from scene_graph.msg import WordVector
except ImportError:
    print("EncodeMask not found, please check if you have installed ROS")
    exit()


class FakeRealtimeCamSimNode:
    """使用仿真里程计构造并限时发布伪造的语义分割结果。"""

    def __init__(self):
        # 对齐 predict_realtime_cam_sim.py 使用的仿真话题。
        self.odom_topic = rospy.get_param("~odom_topic", "/unity_odom_sync")
        self.result_output_topic = rospy.get_param(
            "~result_output_topic", "/yoloe/encodemask"
        )

        # 伪造目标和发布行为配置
        self.label = rospy.get_param("~label", "fake_object")
        self.confidence = float(rospy.get_param("~confidence", 0.9))
        self.depth_m = float(rospy.get_param("~depth_m", 2.5))
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.publish_duration = float(rospy.get_param("~publish_duration", 1.5))
        self.use_live_odom = bool(rospy.get_param("~use_live_odom", False))

        # 对齐 advanced_param_sim.xml 中 ObjectFactory 的图像尺寸。
        self.image_width = int(rospy.get_param("~image_width", 640))
        self.image_height = int(rospy.get_param("~image_height", 480))
        self.mask_x_min = int(rospy.get_param("~mask_x_min", 220))
        self.mask_y_min = int(rospy.get_param("~mask_y_min", 140))
        self.mask_x_max = int(rospy.get_param("~mask_x_max", 420))
        self.mask_y_max = int(rospy.get_param("~mask_y_max", 340))
        self.random_seed = int(rospy.get_param("~random_seed", 0))

        self._validate_params()

        self.cv_bridge = CvBridge()
        self.odom_lock = threading.Lock()
        self.first_odom = None
        self.latest_odom = None
        self.publish_started = False

        # 固定生成一组伪造观测，保证连续发布描述同一个目标。
        self.fake_depth = self._create_fake_depth()
        self.fake_rgb = self._create_fake_rgb()
        self.fake_mask = self._create_fake_mask()
        self.fake_word_vector = self._create_fake_word_vector()

        self.result_pub = rospy.Publisher(
            self.result_output_topic, EncodeMask, queue_size=2
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=100
        )

        rospy.loginfo(
            "Fake simulation YOLOE node initialized: odom=%s, output=%s, "
            "label=%s, depth=%.3fm, rate=%.2fHz, duration=%.2fs, live_odom=%s",
            self.odom_topic,
            self.result_output_topic,
            self.label,
            self.depth_m,
            self.publish_rate,
            self.publish_duration,
            self.use_live_odom,
        )
        rospy.loginfo("Waiting for the first simulation odometry message...")

    def _validate_params(self):
        """检查会影响 EncodeMask 和下游图像解码的参数。"""
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("image_width and image_height must be positive")
        if self.depth_m <= 0.0 or self.depth_m > 65.535:
            raise ValueError("depth_m must be in range (0, 65.535]")
        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        if self.publish_duration <= 0.0:
            raise ValueError("publish_duration must be positive")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in range [0, 1]")
        if not self.label:
            raise ValueError("label must not be empty")

        mask_valid = (
            0 <= self.mask_x_min < self.mask_x_max <= self.image_width
            and 0 <= self.mask_y_min < self.mask_y_max <= self.image_height
        )
        if not mask_valid:
            raise ValueError("mask bounds must be inside the configured image")

    def _create_fake_depth(self):
        """创建 ObjectFactory 仿真分支可直接解码的 16 位 PNG 深度图。"""
        depth_mm = int(round(self.depth_m * 1000.0))
        depth_image = np.full(
            (self.image_height, self.image_width), depth_mm, dtype=np.uint16
        )

        encode_success, png_data = cv2.imencode(".png", depth_image)
        if not encode_success:
            raise RuntimeError("failed to encode fake simulation depth image")

        depth_msg = CompressedImage()
        depth_msg.format = "png"

        # advanced_param_sim.xml 配置 obj/use_realsense=false，下游会直接
        # 对完整 data 执行 cv::imdecode，因此这里不能添加 Realsense 12 字节头。
        depth_msg.data = png_data.tobytes()
        return depth_msg

    def _create_fake_rgb(self):
        """生成固定随机种子的 BGR 图像并按 JPEG 格式压缩。"""
        rng = np.random.default_rng(self.random_seed)
        rgb_image = rng.integers(
            0,
            256,
            size=(self.image_height, self.image_width, 3),
            dtype=np.uint8,
        )
        rgb_msg = self.cv_bridge.cv2_to_compressed_imgmsg(
            rgb_image, dst_format="jpg"
        )
        rgb_msg.format = "jpeg"
        return rgb_msg

    def _create_fake_mask(self):
        """创建与仿真 RGB、Depth 分辨率一致的矩形二值 Mask。"""
        mask_image = np.zeros(
            (self.image_height, self.image_width), dtype=np.uint8
        )
        mask_image[
            self.mask_y_min:self.mask_y_max,
            self.mask_x_min:self.mask_x_max,
        ] = 255

        mask_msg = self.cv_bridge.cv2_to_compressed_imgmsg(
            mask_image, dst_format="png"
        )
        mask_msg.format = "png"
        return mask_msg

    def _create_fake_word_vector(self):
        """生成符合 WordVector 定义的 512 维归一化随机特征。"""
        rng = np.random.default_rng(self.random_seed + 1)
        feature = rng.standard_normal(512).astype(np.float64)
        feature /= np.linalg.norm(feature)

        word_vector_msg = WordVector()
        word_vector_msg.word_vector = feature.tolist()
        return word_vector_msg

    def odom_callback(self, odom_msg):
        """保存首帧和最新 Odom，并在首次收到 Odom 后启动一次发布任务。"""
        should_start = False
        with self.odom_lock:
            self.latest_odom = copy.deepcopy(odom_msg)
            if self.first_odom is None:
                self.first_odom = copy.deepcopy(odom_msg)
            if not self.publish_started:
                self.publish_started = True
                should_start = True

        if should_start:
            rospy.loginfo(
                "First simulation odometry received, starting fake result publishing."
            )
            publish_thread = threading.Thread(target=self.publish_for_duration)
            publish_thread.daemon = True
            publish_thread.start()

    def _get_selected_odom(self):
        """根据参数返回固定首帧 Odom 或当前最新 Odom。"""
        with self.odom_lock:
            selected_odom = self.latest_odom if self.use_live_odom else self.first_odom
            return copy.deepcopy(selected_odom)

    def _build_encode_mask(self, odom_msg):
        """组装单目标 EncodeMask，所有数组的第 0 项保持严格对应。"""
        encode_mask_msg = EncodeMask()
        encode_mask_msg.header = copy.deepcopy(odom_msg.header)
        encode_mask_msg.current_odom = odom_msg

        encode_mask_msg.current_depth = copy.deepcopy(self.fake_depth)
        encode_mask_msg.current_rgb = copy.deepcopy(self.fake_rgb)
        encode_mask_msg.current_depth.header = copy.deepcopy(odom_msg.header)
        encode_mask_msg.current_rgb.header = copy.deepcopy(odom_msg.header)

        mask_msg = copy.deepcopy(self.fake_mask)
        mask_msg.header = copy.deepcopy(odom_msg.header)

        encode_mask_msg.labels.append(self.label)
        encode_mask_msg.confs.append(self.confidence)
        encode_mask_msg.word_vectors.append(copy.deepcopy(self.fake_word_vector))
        encode_mask_msg.masks.append(mask_msg)
        return encode_mask_msg

    def publish_for_duration(self):
        """按照配置频率持续发布指定时间，结束后保留 ROS 节点。"""
        rate = rospy.Rate(self.publish_rate)
        start_time = rospy.Time.now()
        published_count = 0

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= self.publish_duration:
                break

            odom_msg = self._get_selected_odom()
            if odom_msg is not None:
                self.result_pub.publish(self._build_encode_mask(odom_msg))
                published_count += 1

            rate.sleep()

        rospy.loginfo(
            "Fake simulation result publishing finished: %d messages "
            "published in %.2f seconds.",
            published_count,
            self.publish_duration,
        )


if __name__ == "__main__":
    rospy.init_node("fake_realtime_cam_sim", anonymous=False)
    node = FakeRealtimeCamSimNode()
    rospy.spin()
