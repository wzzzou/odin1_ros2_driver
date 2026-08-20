#!/usr/bin/env python3
"""本机压缩图像解码中继。

订阅 Odin 驱动发布的 sensor_msgs/CompressedImage，把设备原始 JPEG 解码成
sensor_msgs/Image，供 RViz Image display 使用。该节点应在本机运行，让跨机链路
只传压缩图像。
"""

import sys

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class OdinCompressedImageRelay(Node):
    def __init__(self) -> None:
        super().__init__("odin_compressed_image_relay")

        self.input_topic = self.declare_parameter(
            "input_topic", "/odin1/image/compressed"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/odin1/image_remote"
        ).value

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(Image, self.output_topic, qos)
        self.subscription = self.create_subscription(
            CompressedImage, self.input_topic, self.on_image, qos
        )
        self.frame_count = 0
        self.get_logger().info(
            f"compressed image relay: {self.input_topic} -> {self.output_topic}"
        )

    def on_image(self, msg: CompressedImage) -> None:
        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if decoded is None:
            self.get_logger().warn("failed to decode compressed image")
            return

        out = Image()
        out.header = msg.header
        out.height = int(decoded.shape[0])
        out.width = int(decoded.shape[1])
        out.encoding = "bgr8"
        out.is_bigendian = False
        out.step = int(decoded.shape[1] * decoded.shape[2])
        out.data = decoded.tobytes()
        self.publisher.publish(out)

        self.frame_count += 1
        if self.frame_count % 60 == 1:
            self.get_logger().info(
                f"decoded image {out.width}x{out.height}, {len(msg.data)} bytes jpeg"
            )


def main() -> int:
    rclpy.init()
    node = OdinCompressedImageRelay()
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
