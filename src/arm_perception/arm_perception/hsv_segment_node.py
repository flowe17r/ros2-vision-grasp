#!/usr/bin/env python3
"""HSV segmentation of one target color, publishing mask + side-by-side debug image."""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from arm_perception.color_presets import COLOR_PRESETS


class HsvSegmentNode(Node):
    def __init__(self):
        super().__init__('hsv_segment_node')
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('target_color', 'red')
        self.declare_parameter('open_kernel', 3)
        self.declare_parameter('close_kernel', 5)

        self._bridge = CvBridge()
        self._frames = 0
        self._proc_ms_sum = 0.0

        topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self._mask_pub = self.create_publisher(Image, '/perception/mask', 10)
        self._debug_pub = self.create_publisher(Image, '/perception/segment_debug', 10)
        self.get_logger().info(f'hsv_segment_node listening on {topic}')

    def _hsv_ranges(self):
        color = self.get_parameter('target_color').value
        if color not in COLOR_PRESETS:
            self.get_logger().warn(
                f'unknown target_color {color!r}, falling back to red',
                throttle_duration_sec=5.0,
            )
            color = 'red'
        return COLOR_PRESETS[color]

    def on_image(self, msg):
        started = time.perf_counter()
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # cv_bridge raises CvBridgeError, ValueError, ...
            self.get_logger().error(
                f'cv_bridge conversion failed: {exc}', throttle_duration_sec=5.0)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for h_min, h_max, s_min, s_max, v_min, v_max in self._hsv_ranges():
            mask = cv2.bitwise_or(
                mask,
                cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max)),
            )

        open_k = max(1, int(self.get_parameter('open_kernel').value))
        close_k = max(1, int(self.get_parameter('close_kernel').value))
        if open_k >= 3:
            kernel = np.ones((open_k, open_k), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        if close_k >= 3:
            kernel = np.ones((close_k, close_k), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        mask_msg = self._bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = msg.header
        self._mask_pub.publish(mask_msg)

        debug = cv2.hconcat([frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        debug_msg = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_msg.header = msg.header
        self._debug_pub.publish(debug_msg)

        self._frames += 1
        self._proc_ms_sum += (time.perf_counter() - started) * 1000.0
        if self._frames % 30 == 0:
            avg_ms = self._proc_ms_sum / 30.0
            self.get_logger().info(f'avg {avg_ms:.2f} ms/frame (last 30 frames)')
            self._proc_ms_sum = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = HsvSegmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
