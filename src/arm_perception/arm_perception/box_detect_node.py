#!/usr/bin/env python3
"""Detect the largest blob of each configured color and publish pixel-space detections."""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from arm_interfaces.msg import Detection2D, Detection2DArray
from arm_perception.color_presets import COLOR_PRESETS


class BoxDetectNode(Node):
    def __init__(self):
        super().__init__('box_detect_node')
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('colors', ['red', 'green', 'blue'])
        self.declare_parameter('min_area_px', 300.0)
        self.declare_parameter('open_kernel', 3)
        self.declare_parameter('close_kernel', 5)

        self._bridge = CvBridge()
        self._frames = 0
        self._proc_ms_sum = 0.0

        topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self._det_pub = self.create_publisher(
            Detection2DArray, '/perception/detections', 10)
        self._debug_pub = self.create_publisher(Image, '/perception/detect_debug', 10)
        self.get_logger().info(f'box_detect_node listening on {topic}')

    def _mask(self, hsv, color_name):
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for h_min, h_max, s_min, s_max, v_min, v_max in COLOR_PRESETS.get(color_name, []):
            mask = cv2.bitwise_or(
                mask,
                cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max)),
            )
        open_k = max(1, int(self.get_parameter('open_kernel').value))
        close_k = max(1, int(self.get_parameter('close_kernel').value))
        if open_k >= 3:
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, np.ones((open_k, open_k), np.uint8))
        if close_k >= 3:
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_CLOSE, np.ones((close_k, close_k), np.uint8))
        return mask

    def on_image(self, msg):
        started = time.perf_counter()
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(
                f'cv_bridge conversion failed: {exc}', throttle_duration_sec=5.0)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        min_area = float(self.get_parameter('min_area_px').value)
        colors = self.get_parameter('colors').value
        if isinstance(colors, str):
            colors = [colors]

        debug = frame.copy()
        out = Detection2DArray()
        out.header = msg.header

        for color_name in colors:
            if color_name not in COLOR_PRESETS:
                self.get_logger().warn(
                    f'skip unknown color {color_name!r}', throttle_duration_sec=5.0)
                continue

            contours, _ = cv2.findContours(
                self._mask(hsv, color_name), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            best = None
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < min_area:
                    continue
                if best is None or area > best[0]:
                    best = (area, contour)
            if best is None:
                continue

            area, contour = best
            moments = cv2.moments(contour)
            if moments['m00'] <= 0.0:
                continue
            cx = moments['m10'] / moments['m00']
            cy = moments['m01'] / moments['m00']
            x, y, w, h = cv2.boundingRect(contour)

            det = Detection2D()
            det.header = msg.header
            det.color_name = color_name
            det.u = float(cx)
            det.v = float(cy)
            det.area_px = float(area)
            det.bbox_x = float(x)
            det.bbox_y = float(y)
            det.bbox_w = float(w)
            det.bbox_h = float(h)
            det.fill_ratio = float(min(1.0, area / float(w * h))) if w * h else 0.0
            out.detections.append(det)

            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(debug, (int(cx), int(cy)), 3, (0, 255, 255), -1)
            cv2.putText(debug, color_name, (x, max(y - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        self._det_pub.publish(out)
        debug_msg = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_msg.header = msg.header
        self._debug_pub.publish(debug_msg)

        self._frames += 1
        self._proc_ms_sum += (time.perf_counter() - started) * 1000.0
        if self._frames % 30 == 0:
            avg_ms = self._proc_ms_sum / 30.0
            self.get_logger().info(
                f'avg {avg_ms:.2f} ms/frame, last frame had {len(out.detections)} detections')
            self._proc_ms_sum = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = BoxDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
