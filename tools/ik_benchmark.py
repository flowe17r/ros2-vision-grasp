#!/usr/bin/env python3
"""Sample a workspace grid and report KDL IK success rate.

Usage (after the arm launch is up):
    python3 ~/ros2-vision-grasp/tools/ik_benchmark.py
"""

import itertools

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node

from arm_interfaces.srv import SolveIK

CUBES = [(0.23, 0.10), (0.35, -0.10), (0.47, 0.08)]


def topdown_pose(x, y, z):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.x = 1.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0
    return pose


def main():
    rclpy.init()
    node = Node('ik_benchmark')
    client = node.create_client(SolveIK, '/kinematics/solve_ik')
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error('IK service not available')
        rclpy.shutdown()
        return

    targets = []
    for x, y, z in itertools.product(
            (0.22, 0.35, 0.48), (-0.12, 0.0, 0.12), (0.03, 0.10, 0.17)):
        targets.append((x, y, z, 'grid'))
    for x, y in CUBES:
        targets.append((x, y, 0.032, 'cube'))

    success = 0
    failures = []
    for x, y, z, kind in targets:
        request = SolveIK.Request()
        request.target = topdown_pose(x, y, z)
        request.seed = [0.0] * 6
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        response = future.result()
        if response is None:
            failures.append((kind, x, y, z, 'timeout'))
            continue
        if response.success:
            success += 1
        else:
            failures.append(
                (kind, x, y, z,
                 f'pos {response.pos_error * 1000:.1f} mm, '
                 f'rot {response.rot_error:.2f} rad'))

    total = len(targets)
    print(f'IK success: {success}/{total} = {100.0 * success / total:.1f}%')
    for kind, x, y, z, reason in failures:
        print(f'  FAIL {kind} ({x:.2f}, {y:.2f}, {z:.2f}): {reason}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
