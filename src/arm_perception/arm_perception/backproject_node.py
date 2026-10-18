#!/usr/bin/env python3
"""Back-project 2D detections through the depth image into base_link coordinates."""

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose, PoseArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from arm_interfaces.msg import Detection2DArray, Target3D, Target3DArray

# 方块中心在 base_link 下的真值（base_link 在 world 的 (-0.35, 0, 0.25)）
GROUND_TRUTH = {
    'red': (0.23, 0.10, 0.025),
    'green': (0.35, -0.10, 0.025),
    'blue': (0.47, 0.08, 0.025),
}


class BackprojectNode(Node):
    def __init__(self):
        super().__init__('backproject_node')
        self.declare_parameter('detections_topic', '/perception/detections')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('depth_topic', '/camera/depth_image')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('depth_patch', 5)
        self.declare_parameter('max_sync_gap_sec', 0.15)
        # 方块顶面在检测框内的位置：用顶面中心采样深度，避开侧面造成的质心偏移
        self.declare_parameter('sample_u_ratio', 0.5)
        self.declare_parameter('sample_v_ratio', 0.3)
        self.declare_parameter('eval_ground_truth', True)

        self._bridge = CvBridge()
        self._camera_info = None
        self._depth_msg = None
        self._depth_encoding_logged = False
        self._frames = 0
        self._err = {name: [0.0, 0.0, 0] for name in GROUND_TRUTH}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.on_depth, qos_profile_sensor_data)
        self.create_subscription(
            Detection2DArray, self.get_parameter('detections_topic').value,
            self.on_detections, 10)
        self._pub = self.create_publisher(
            PoseArray, '/perception/targets_base', 10)
        self._pub3d = self.create_publisher(
            Target3DArray, '/perception/targets_3d', 10)
        self.get_logger().info('backproject_node ready')

    def on_camera_info(self, msg):
        if self._camera_info is None:
            k = msg.k
            self.get_logger().info(
                f'camera_info: fx={k[0]:.2f} fy={k[4]:.2f} cx={k[2]:.2f} cy={k[5]:.2f}')
        self._camera_info = msg

    def on_depth(self, msg):
        if not self._depth_encoding_logged:
            self.get_logger().info(
                f'depth image: encoding={msg.encoding} {msg.width}x{msg.height}')
            self._depth_encoding_logged = True
        self._depth_msg = msg

    def _depth_in_meters(self):
        depth = self._bridge.imgmsg_to_cv2(self._depth_msg)
        if depth.dtype == np.uint16:
            return depth.astype(np.float32) / 1000.0
        depth = depth.astype(np.float32)
        finite = depth[np.isfinite(depth) & (depth > 0.0)]
        if finite.size and float(np.median(finite)) > 50.0:
            depth = depth / 1000.0
        return depth

    def _sample_depth(self, depth, u, v):
        height, width = depth.shape[:2]
        half = max(1, int(self.get_parameter('depth_patch').value)) // 2
        x0, x1 = max(0, u - half), min(width, u + half + 1)
        y0, y1 = max(0, v - half), min(height, v + half + 1)
        roi = depth[y0:y1, x0:x1]
        if roi.size == 0:
            return None
        valid = roi[np.isfinite(roi) & (roi > 0.02) & (roi < 10.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    @staticmethod
    def _transform_point(point, tf):
        translation = tf.transform.translation
        rotation = tf.transform.rotation
        x, y, z = point.point.x, point.point.y, point.point.z
        ux = 2.0 * (rotation.y * z - rotation.z * y)
        uy = 2.0 * (rotation.z * x - rotation.x * z)
        uz = 2.0 * (rotation.x * y - rotation.y * x)
        rx = x + rotation.w * ux + rotation.y * uz - rotation.z * uy
        ry = y + rotation.w * uy + rotation.z * ux - rotation.x * uz
        rz = z + rotation.w * uz + rotation.x * uy - rotation.y * ux
        return rx + translation.x, ry + translation.y, rz + translation.z

    def on_detections(self, msg):
        if self._camera_info is None or self._depth_msg is None:
            self.get_logger().warn(
                'waiting for camera_info/depth before back-projection',
                throttle_duration_sec=5.0)
            return

        gap = abs((Time.from_msg(msg.header.stamp) -
                   Time.from_msg(self._depth_msg.header.stamp)).nanoseconds) / 1e9
        if gap > float(self.get_parameter('max_sync_gap_sec').value):
            self.get_logger().warn(
                f'detection/depth stamp gap {gap:.3f}s too large, skipping',
                throttle_duration_sec=5.0)
            return

        k = self._camera_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().error('invalid camera intrinsics')
            return

        try:
            depth = self._depth_in_meters()
        except Exception as exc:
            self.get_logger().error(
                f'depth conversion failed: {exc}', throttle_duration_sec=5.0)
            return

        target_frame = self.get_parameter('target_frame').value
        camera_frame = self.get_parameter('camera_frame').value
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame, camera_frame, Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.2))
        except Exception as exc:
            self.get_logger().warn(
                f'TF {camera_frame} -> {target_frame} unavailable: {exc}',
                throttle_duration_sec=5.0)
            return

        u_ratio = float(self.get_parameter('sample_u_ratio').value)
        v_ratio = float(self.get_parameter('sample_v_ratio').value)
        targets = PoseArray()
        targets.header.stamp = msg.header.stamp
        targets.header.frame_id = target_frame
        targets3d = Target3DArray()
        targets3d.header.stamp = msg.header.stamp
        targets3d.header.frame_id = target_frame
        lines = []

        for det in msg.detections:
            u = det.bbox_x + det.bbox_w * u_ratio
            v = det.bbox_y + det.bbox_h * v_ratio
            depth_value = self._sample_depth(depth, int(round(u)), int(round(v)))
            if depth_value is None:
                self.get_logger().warn(
                    f'{det.color_name}: no valid depth at ({int(u)},{int(v)})',
                    throttle_duration_sec=5.0)
                continue

            point = PointStamped()
            point.header.stamp = msg.header.stamp
            point.header.frame_id = camera_frame
            point.point.x = (u - cx) * depth_value / fx
            point.point.y = (v - cy) * depth_value / fy
            point.point.z = depth_value
            bx, by, bz = self._transform_point(point, tf)

            pose = Pose()
            pose.position.x = bx
            pose.position.y = by
            pose.position.z = bz
            pose.orientation.w = 1.0
            targets.poses.append(pose)

            target3d = Target3D()
            target3d.color_name = det.color_name
            target3d.position.x = bx
            target3d.position.y = by
            target3d.position.z = bz
            target3d.depth = float(depth_value)
            targets3d.targets.append(target3d)

            lines.append(f'{det.color_name}=({bx:.3f},{by:.3f},{bz:.3f})')
            self._update_error(det.color_name, bx, by)

        self._pub.publish(targets)
        self._pub3d.publish(targets3d)
        self._frames += 1
        if lines:
            self.get_logger().info(
                'targets base_link: ' + ', '.join(lines),
                throttle_duration_sec=5.0)
        if self._frames % 30 == 0:
            self._log_error_summary()

    def _update_error(self, color_name, x, y):
        if not bool(self.get_parameter('eval_ground_truth').value):
            return
        truth = GROUND_TRUTH.get(color_name)
        if truth is None:
            return
        error_mm = ((x - truth[0]) ** 2 + (y - truth[1]) ** 2) ** 0.5 * 1000.0
        stats = self._err[color_name]
        stats[0] += error_mm
        stats[1] = max(stats[1], error_mm)
        stats[2] += 1

    def _log_error_summary(self):
        parts = []
        for color_name, (total, worst, count) in self._err.items():
            if count:
                parts.append(
                    f'{color_name} mean {total / count:.1f} / max {worst:.1f} mm '
                    f'(n={count})')
        if parts:
            self.get_logger().info(
                'XY error (last 30 frames) vs cube center: ' + '; '.join(parts))
        self._err = {name: [0.0, 0.0, 0] for name in GROUND_TRUTH}


def main(args=None):
    rclpy.init(args=args)
    node = BackprojectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
