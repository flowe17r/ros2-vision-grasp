#!/usr/bin/env python3
"""Grasp state machine: detection -> IK -> arm trajectory -> gripper -> lift."""

import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint

from arm_interfaces.msg import Target3DArray
from arm_interfaces.srv import Grasp, SolveIK

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
GRIPPER_JOINTS = ['finger_left_joint', 'finger_right_joint']


class GraspNode(Node):
    def __init__(self):
        super().__init__('grasp_node')
        self.declare_parameter('target_topic', '/perception/targets_3d')
        self.declare_parameter('pose_topic', '/world/vision_grasp/pose/info')
        self.declare_parameter('ik_service', '/kinematics/solve_ik')
        self.declare_parameter('pregrasp_height', 0.16)
        self.declare_parameter('grasp_height', 0.032)
        self.declare_parameter('lift_height', 0.18)
        self.declare_parameter('gripper_open', 0.015)
        self.declare_parameter('gripper_close', 0.004)
        self.declare_parameter('target_timeout_sec', 3.0)

        self._targets = {}
        self._target_time = {}
        self._cube_z = {}
        self._joints = {}
        self._busy = False
        self._lock = threading.Lock()

        self._cb = ReentrantCallbackGroup()
        self.create_subscription(
            Target3DArray, self.get_parameter('target_topic').value,
            self._on_targets, 10, callback_group=self._cb)
        self.create_subscription(
            JointState, '/joint_states', self._on_joints, 10,
            callback_group=self._cb)
        self.create_subscription(
            TFMessage, self.get_parameter('pose_topic').value,
            self._on_poses, 10, callback_group=self._cb)

        self._ik = self.create_client(
            SolveIK, self.get_parameter('ik_service').value,
            callback_group=self._cb)
        self._arm = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self._cb)
        self._gripper = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            callback_group=self._cb)
        self._service = self.create_service(
            Grasp, '/grasp/execute', self._handle_grasp,
            callback_group=self._cb)
        self.get_logger().info('grasp_node ready on /grasp/execute')

    def _on_targets(self, msg):
        now = self.get_clock().now()
        for target in msg.targets:
            self._targets[target.color_name] = (
                target.position.x, target.position.y, target.position.z)
            self._target_time[target.color_name] = now

    def _on_joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            self._joints[name] = position

    def _on_poses(self, msg):
        for tf in msg.transforms:
            name = tf.child_frame_id or tf.header.frame_id
            key = name.rsplit('/', 1)[-1]
            if key.startswith('cube_'):
                self._cube_z[key[len('cube_'):]] = tf.transform.translation.z

    def _handle_grasp(self, request, response):
        with self._lock:
            if self._busy:
                response.success = False
                response.message = 'another grasp is running'
                return response
            self._busy = True
        try:
            ok, message = self._execute(request.color.strip().lower())
        except Exception as exc:  # keep the service alive on unexpected failures
            self.get_logger().error(f'grasp aborted: {exc}')
            ok, message = False, f'exception: {exc}'
        finally:
            with self._lock:
                self._busy = False
        response.success = ok
        response.message = message
        self.get_logger().info(
            f'grasp {request.color}: success={ok} ({message})')
        return response

    def _execute(self, color):
        target = self._targets.get(color)
        if target is None:
            return False, f'no 3D target for {color}'
        age = (self.get_clock().now() - self._target_time[color]).nanoseconds / 1e9
        if age > float(self.get_parameter('target_timeout_sec').value):
            return False, f'target for {color} is stale ({age:.1f}s)'

        if not self._ik.wait_for_service(timeout_sec=10.0):
            return False, 'IK service unavailable'
        if not self._arm.wait_for_server(timeout_sec=10.0):
            return False, 'arm action server unavailable'
        if not self._gripper.wait_for_server(timeout_sec=10.0):
            return False, 'gripper action server unavailable'

        x, y, _ = target
        z_before = self._cube_z.get(color)
        self.get_logger().info(
            f'{color} target ({x:.3f}, {y:.3f}), cube z before={z_before}')

        ok, message = self._move_gripper(
            float(self.get_parameter('gripper_open').value), 1.5)
        if not ok:
            return False, f'open gripper failed: {message}'

        seed = self._current_arm_seed()
        pre_height = float(self.get_parameter('pregrasp_height').value)
        grasp_height = float(self.get_parameter('grasp_height').value)
        lift_height = float(self.get_parameter('lift_height').value)

        pre_joints = None
        last_ik_error = 'no attempt'
        for yaw in (0.0, math.pi / 2.0):
            pre_joints, last_ik_error = self._solve(x, y, pre_height, yaw, seed)
            if pre_joints is not None:
                break
        if pre_joints is None:
            return False, f'pregrasp IK failed: {last_ik_error}'

        ok, message = self._move_arm(pre_joints, 3.0)
        if not ok:
            return False, f'pregrasp move failed: {message}'

        grasp_joints = None
        for height in (grasp_height, grasp_height + 0.01, grasp_height - 0.007):
            grasp_joints, last_ik_error = self._solve(x, y, height, 0.0, pre_joints)
            if grasp_joints is not None:
                break
            grasp_joints, last_ik_error = self._solve(
                x, y, height, math.pi / 2.0, pre_joints)
            if grasp_joints is not None:
                break
        if grasp_joints is None:
            return False, f'grasp IK failed: {last_ik_error}'

        ok, message = self._move_arm(grasp_joints, 2.0)
        if not ok:
            return False, f'descend failed: {message}'

        ok, message = self._move_gripper(
            float(self.get_parameter('gripper_close').value), 2.5)
        if not ok:
            return False, f'close gripper failed: {message}'
        time.sleep(0.5)

        lift_joints = None
        for height in (lift_height, lift_height - 0.02):
            lift_joints, last_ik_error = self._solve(
                x, y, height, 0.0, grasp_joints)
            if lift_joints is not None:
                break
        if lift_joints is None:
            lift_joints = pre_joints

        ok, message = self._move_arm(lift_joints, 3.0)
        if not ok:
            return False, f'lift failed: {message}'

        time.sleep(0.8)
        z_after = self._cube_z.get(color)
        self.get_logger().info(
            f'{color} cube z after={z_after} (before={z_before})')

        if z_before is None or z_after is None:
            return True, 'joint sequence done, cube lift unverified (no pose feedback)'
        lifted = z_after - z_before
        if lifted > 0.03:
            return True, f'grasp verified, cube lifted {lifted * 1000:.0f} mm'
        return False, f'gripper closed but cube not lifted (dz={lifted * 1000:.0f} mm)'

    def _current_arm_seed(self):
        return [float(self._joints.get(name, 0.0)) for name in ARM_JOINTS]

    def _solve(self, x, y, z, yaw, seed):
        request = SolveIK.Request()
        request.target = Pose()
        request.target.position.x = float(x)
        request.target.position.y = float(y)
        request.target.position.z = float(z)
        request.target.orientation.x = math.cos(yaw / 2.0)
        request.target.orientation.y = math.sin(yaw / 2.0)
        request.target.orientation.z = 0.0
        request.target.orientation.w = 0.0
        request.seed = [float(v) for v in seed]

        future = self._ik.call_async(request)
        response = self._wait(future, 5.0)
        if response is None:
            return None, 'service timeout'
        if not response.success:
            return None, response.message
        self.get_logger().info(
            f'IK ({x:.3f},{y:.3f},{z:.3f}) ok: pos err '
            f'{response.pos_error * 1000:.1f} mm, rot err '
            f'{response.rot_error:.3f} rad')
        return [float(v) for v in response.joints], 'ok'

    def _move_arm(self, joints, seconds):
        return self._run_action(self._arm, ARM_JOINTS, joints, seconds)

    def _move_gripper(self, opening, seconds):
        return self._run_action(
            self._gripper, GRIPPER_JOINTS, [opening, opening], seconds)

    def _run_action(self, client, names, positions, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.velocities = [0.0] * len(positions)
        point.time_from_start = Duration(
            sec=int(seconds), nanosec=int((seconds - int(seconds)) * 1e9))
        goal.trajectory.points.append(point)

        handle = self._wait(client.send_goal_async(goal), 5.0)
        if handle is None:
            return False, 'goal send timeout'
        if not handle.accepted:
            return False, 'goal rejected'
        wrapped = self._wait(handle.get_result_async(), seconds + 8.0)
        if wrapped is None:
            return False, 'result timeout'
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return False, f'action status {wrapped.status}'
        return True, 'ok'

    @staticmethod
    def _wait(future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            return None
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = GraspNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
