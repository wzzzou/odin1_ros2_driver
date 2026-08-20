#!/usr/bin/env python3
"""Odin NUC 端远程图像中继。

默认只处理图像：订阅设备原始 JPEG，缩放并重新压缩后发布给远程 RViz。
`/odin1/odometry` 和 `/odin1/path` 是小消息，当前远程 RViz 直接订阅原始话题。
`enable_pose` 仅保留为诊断和兼容开关，不作为默认运行链路。
"""

import sys
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import MarkerArray


class OdinRemoteTopicRelay(Node):
    def __init__(self) -> None:
        super().__init__("odin_remote_topic_relay")

        self.image_input_topic = self.declare_parameter(
            "image_input_topic", "/odin1/image/compressed"
        ).value
        self.image_output_topic = self.declare_parameter(
            "image_output_topic", "/odin1/image_remote/compressed"
        ).value
        self.odom_input_topic = self.declare_parameter(
            "odom_input_topic", "/odin1/odometry"
        ).value
        self.odom_output_topic = self.declare_parameter(
            "odom_output_topic", "/odin1/odometry_remote"
        ).value
        self.path_input_topic = self.declare_parameter(
            "path_input_topic", "/odin1/path"
        ).value
        self.path_output_topic = self.declare_parameter(
            "path_output_topic", "/odin1/path_remote"
        ).value
        self.image_max_width = int(
            self.declare_parameter("image_max_width", 800).value
        )
        self.jpeg_quality = int(self.declare_parameter("jpeg_quality", 70).value)
        self.enable_image = bool(self.declare_parameter("enable_image", False).value)
        self.enable_pose = bool(self.declare_parameter("enable_pose", False).value)

        self.jpeg_quality = max(20, min(95, self.jpeg_quality))

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.odom_pub = None
        self.path_pub = None
        if self.enable_pose:
            self.odom_pub = self.create_publisher(Odometry, self.odom_output_topic, qos)
            self.path_pub = self.create_publisher(MarkerArray, self.path_output_topic, qos)
        self.image_pub = None
        if self.enable_image:
            self.image_pub = self.create_publisher(
                CompressedImage, self.image_output_topic, image_qos
            )

        self.odom_sub = None
        self.path_sub = None
        if self.enable_pose:
            self.odom_sub = self.create_subscription(
                Odometry, self.odom_input_topic, self.on_odom, qos
            )
            self.path_sub = self.create_subscription(
                MarkerArray, self.path_input_topic, self.on_path, qos
            )
        self.image_sub = None
        if self.enable_image:
            self.image_sub = self.create_subscription(
                CompressedImage, self.image_input_topic, self.on_image, image_qos
            )

        self.odom_count = 0
        self.path_count = 0
        self.image_count = 0

        if self.enable_pose:
            self.get_logger().info(
                f"remote relay odom: {self.odom_input_topic} -> {self.odom_output_topic}"
            )
            self.get_logger().info(
                f"remote relay path: {self.path_input_topic} -> {self.path_output_topic}"
            )
        else:
            self.get_logger().info("remote relay odom/path disabled")
        if self.enable_image:
            self.get_logger().info(
                "remote relay image: "
                f"{self.image_input_topic} -> {self.image_output_topic}, "
                f"max_width={self.image_max_width}, jpeg_quality={self.jpeg_quality}"
            )
        else:
            self.get_logger().info("remote relay image disabled")

    def on_odom(self, msg: Odometry) -> None:
        if self.odom_pub is None:
            return
        self.odom_pub.publish(msg)
        self.odom_count += 1
        if self.odom_count % 30 == 1:
            self.get_logger().info(f"relayed odom #{self.odom_count}")

    def on_path(self, msg: MarkerArray) -> None:
        if self.path_pub is None:
            return
        self.path_pub.publish(msg)
        self.path_count += 1
        if self.path_count % 10 == 1:
            self.get_logger().info(
                f"relayed path #{self.path_count}, markers={len(msg.markers)}"
            )

    def on_image(self, msg: CompressedImage) -> None:
        if self.image_pub is None:
            return
        out = self.resize_compressed_image(msg)
        self.image_pub.publish(out)
        self.image_count += 1
        if self.image_count % 10 == 1:
            self.get_logger().info(
                f"relayed image #{self.image_count}, {len(msg.data)} -> {len(out.data)} bytes"
            )

    def resize_compressed_image(self, msg: CompressedImage) -> CompressedImage:
        if self.image_max_width <= 0:
            return msg

        decoded = self.decode_image(msg.data)
        if decoded is None:
            self.get_logger().warn("failed to decode compressed image; passthrough")
            return msg

        height, width = decoded.shape[:2]
        if width > self.image_max_width:
            scale = self.image_max_width / float(width)
            target_size = (self.image_max_width, max(1, int(height * scale)))
            decoded = cv2.resize(decoded, target_size, interpolation=cv2.INTER_AREA)

        ok, encoded = cv2.imencode(
            ".jpg", decoded, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            self.get_logger().warn("failed to encode resized image; passthrough")
            return msg

        out = CompressedImage()
        out.header = msg.header
        out.format = "jpeg"
        out.data = encoded.tobytes()
        return out

    @staticmethod
    def decode_image(data: bytes) -> Optional[np.ndarray]:
        encoded = np.frombuffer(data, dtype=np.uint8)
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if decoded is None or decoded.size == 0:
            return None
        return decoded


def main() -> int:
    rclpy.init()
    node = OdinRemoteTopicRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
