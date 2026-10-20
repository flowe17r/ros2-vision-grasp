#!/usr/bin/env python3
"""把机械臂的 ROS 2 接口包成 MCP Server，走 stdio 的 JSON-RPC 2.0。

只用标准库实现 MCP 协议本身，不依赖 pip 的 mcp SDK —— 虚拟机里装不上第三方
包时也能跑。ROS 侧用 rclpy 订阅感知结果、调用运动学与抓取服务。

用法（一般由 tools/nl_grasp.py 拉起，也可以手动接一个 MCP 客户端）：
    source ~/ros2_ws/install/setup.bash
    ros2 run arm_mcp mcp_server
"""

import json
import os
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState

from arm_interfaces.msg import Target3DArray
from arm_interfaces.srv import Grasp, SolveFK, SolveIK

SUPPORTED_PROTOCOLS = ('2025-06-18', '2025-03-26', '2024-11-05')
FALLBACK_PROTOCOL = '2024-11-05'
SERVER_INFO = {'name': 'arm-mcp', 'version': '0.1.0'}

TARGET_TOPIC = '/perception/targets_3d'
GRASP_SERVICE = '/grasp/execute'
IK_SERVICE = '/kinematics/solve_ik'
FK_SERVICE = '/kinematics/solve_fk'
GRASP_TIMEOUT = 120.0
CALL_TIMEOUT = 10.0
FIRST_MESSAGE_WAIT = 3.0
BAG_REPORT = os.path.join(
    os.path.expanduser('~'), 'ros2-vision-grasp', 'tools', 'bag_report.py')

TOOLS = [
    {
        'name': 'list_targets',
        'description': '读取视觉节点当前检测到的方块，返回每个颜色在机械臂 '
                       'base_link 坐标系下的位置（米）与深度。抓取前先用它确认目标存在。',
        'inputSchema': {
            'type': 'object', 'properties': {}, 'additionalProperties': False,
        },
    },
    {
        'name': 'get_joint_states',
        'description': '读取机械臂当前的关节角（弧度）与力矩，用于确认手臂是否静止、'
                       '是否卡在某个位置。',
        'inputSchema': {
            'type': 'object', 'properties': {}, 'additionalProperties': False,
        },
    },
    {
        'name': 'solve_fk',
        'description': '正运动学：给 6 个关节角，算出末端 TCP 在 base_link 下的位姿。',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'joints': {
                    'type': 'array', 'items': {'type': 'number'}, 'minItems': 6,
                    'maxItems': 6, 'description': 'joint1 到 joint6 的关节角，弧度',
                },
            },
            'required': ['joints'], 'additionalProperties': False,
        },
    },
    {
        'name': 'solve_ik',
        'description': '逆运动学：给 base_link 系下的目标位置，算出一组可行的关节角。'
                       '姿态固定为夹爪朝下。返回是否求解成功与位姿误差。',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'x': {'type': 'number', 'description': 'base_link 系 x，米'},
                'y': {'type': 'number', 'description': 'base_link 系 y，米'},
                'z': {'type': 'number', 'description': 'base_link 系 z，米；'
                                                       '抓取高度不得低于 0.040'},
            },
            'required': ['x', 'y', 'z'], 'additionalProperties': False,
        },
    },
    {
        'name': 'grasp',
        'description': '对指定颜色的方块执行完整抓取：开爪 → 预抓取 → 下降 → 闭合 → '
                       '抬升，并用方块真值位姿校验抬升量。耗时可达一分钟，同步返回结果。',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'color': {
                    'type': 'string', 'enum': ['red', 'green', 'blue'],
                    'description': '方块颜色。中文「红/绿/蓝」需先翻译成 red/green/blue',
                },
            },
            'required': ['color'], 'additionalProperties': False,
        },
    },
    {
        'name': 'bag_report',
        'description': '离线分析一个 rosbag2 目录，返回抓取过程报告（话题消息数、'
                       '三色目标轨迹、方块抬升量与水平漂移、末帧关节角）。'
                       '不启动仿真也能复现一次抓取。',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'path': {'type': 'string', 'description': 'rosbag2 目录的绝对路径'},
            },
            'required': ['path'], 'additionalProperties': False,
        },
    },
]


def log(text):
    print(f'[arm-mcp] {text}', file=sys.stderr, flush=True)


class ArmBridge:
    """rclpy 侧：缓存订阅数据，封装服务调用。executor 跑在独立线程。"""

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('arm_mcp_server')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._targets = {}
        self._target_stamp = 0.0
        self._joints = None
        self._lock = threading.Lock()
        # 客户端握手后往往立刻调用工具，而订阅收到第一帧要等发现完成；
        # 用 Event 让工具最多等 FIRST_MESSAGE_WAIT 秒，避免把「刚启动」误报成「没数据」
        self._targets_ready = threading.Event()
        self._joints_ready = threading.Event()
        self.node.create_subscription(
            Target3DArray, TARGET_TOPIC, self._on_targets, 10)
        self.node.create_subscription(
            JointState, '/joint_states', self._on_joints, 10)
        self._grasp = self.node.create_client(Grasp, GRASP_SERVICE)
        self._ik = self.node.create_client(SolveIK, IK_SERVICE)
        self._fk = self.node.create_client(SolveFK, FK_SERVICE)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def _on_targets(self, msg):
        now = time.time()
        with self._lock:
            for target in msg.targets:
                self._targets[target.color_name] = (target, now)
            self._target_stamp = now
        self._targets_ready.set()

    def _on_joints(self, msg):
        with self._lock:
            self._joints = (list(msg.name), list(msg.position), list(msg.effort))
        self._joints_ready.set()

    def _call(self, client, request, timeout):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'服务 {client.srv_name} 不可用，确认 launch 已启动')
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                future.cancel()
                raise RuntimeError(f'服务 {client.srv_name} 超时（{timeout:.0f} s）')
            time.sleep(0.02)
        return future.result()

    def list_targets(self):
        self._targets_ready.wait(FIRST_MESSAGE_WAIT)
        with self._lock:
            snapshot = dict(self._targets)
        if not snapshot:
            return {'target_count': 0, 'targets': [],
                    'note': '还没收到视觉目标。检查相机是否出图、检测节点是否在跑。'}
        age = time.time() - self._target_stamp
        targets = []
        for color, (target, stamp) in sorted(snapshot.items()):
            targets.append({
                'color': color,
                'x': round(target.position.x, 4),
                'y': round(target.position.y, 4),
                'z': round(target.position.z, 4),
                'depth': round(target.depth, 4),
                'age_sec': round(time.time() - stamp, 2),
            })
        return {'target_count': len(targets), 'targets': targets,
                'frame_id': 'base_link', 'last_update_age_sec': round(age, 2)}

    def get_joint_states(self):
        self._joints_ready.wait(FIRST_MESSAGE_WAIT)
        with self._lock:
            joints = self._joints
        if joints is None:
            return {'available': False,
                    'note': '还没收到 /joint_states，确认控制器已 active。'}
        names, positions, efforts = joints
        return {
            'available': True,
            'joints': [{'name': n, 'position': round(p, 4), 'effort': round(e, 3)}
                       for n, p, e in zip(names, positions, efforts)],
        }

    def solve_fk(self, joints):
        request = SolveFK.Request()
        request.joints = [float(v) for v in joints]
        response = self._call(self._fk, request, CALL_TIMEOUT)
        return {
            'success': response.success,
            'message': response.message,
            'position': {'x': round(response.pose.position.x, 4),
                         'y': round(response.pose.position.y, 4),
                         'z': round(response.pose.position.z, 4)},
            'orientation': {'x': round(response.pose.orientation.x, 4),
                            'y': round(response.pose.orientation.y, 4),
                            'z': round(response.pose.orientation.z, 4),
                            'w': round(response.pose.orientation.w, 4)},
        }

    def solve_ik(self, x, y, z):
        request = SolveIK.Request()
        request.target.position.x = float(x)
        request.target.position.y = float(y)
        request.target.position.z = float(z)
        # 夹爪沿 TCP 的 -z 抓取，TCP 的 +z 朝上，即单位姿态
        request.target.orientation.w = 1.0
        response = self._call(self._ik, request, CALL_TIMEOUT)
        return {
            'success': response.success,
            'message': response.message,
            'joints': [round(v, 4) for v in response.joints],
            'pos_error_mm': round(response.pos_error * 1000.0, 3),
            'rot_error_rad': round(response.rot_error, 5),
        }

    def grasp(self, color):
        request = Grasp.Request()
        request.color = str(color)
        response = self._call(self._grasp, request, GRASP_TIMEOUT)
        return {'success': response.success, 'message': response.message}

    def bag_report(self, path):
        # 不经 shell，参数以列表传入；再挡掉明显不是包目录的路径
        if not os.path.isabs(path) or not os.path.isdir(path):
            raise RuntimeError(f'{path} 不是一个存在的绝对目录路径')
        if not os.path.isfile(os.path.join(path, 'metadata.yaml')):
            raise RuntimeError(f'{path} 下没有 metadata.yaml，不是 rosbag2 目录')
        done = subprocess.run(
            [sys.executable, BAG_REPORT, path],
            capture_output=True, text=True, timeout=120)
        if done.returncode != 0:
            raise RuntimeError(done.stderr.strip() or 'bag_report.py 执行失败')
        return {'report': done.stdout}

    def shutdown(self):
        # 必须先停 executor 再 join spin 线程，否则线程还在访问已销毁的节点，
        # C++ 侧会抛 "terminate called without an active exception"
        self.executor.shutdown()
        self._thread.join(timeout=5.0)
        self.node.destroy_node()
        rclpy.shutdown()


class McpServer:
    def __init__(self, bridge):
        self.bridge = bridge
        self.protocol = FALLBACK_PROTOCOL

    def run(self):
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError as error:
                self._send_error(None, -32700, f'Parse error: {error}')
                continue
            try:
                self._dispatch(payload)
            except Exception as error:  # 单个请求出错不能把 server 带崩
                log(f'处理请求失败：{error}')
                self._send_error(payload.get('id'), -32603, str(error))

    def _dispatch(self, payload):
        method = payload.get('method')
        request_id = payload.get('id')
        params = payload.get('params') or {}

        if method == 'initialize':
            client_version = params.get('protocolVersion')
            if client_version in SUPPORTED_PROTOCOLS:
                self.protocol = client_version
            self._send_result(request_id, {
                'protocolVersion': self.protocol,
                'capabilities': {'tools': {'listChanged': False}},
                'serverInfo': SERVER_INFO,
            })
        elif method == 'notifications/initialized' or method is None:
            return  # 通知不需要回包
        elif method == 'ping':
            self._send_result(request_id, {})
        elif method == 'tools/list':
            self._send_result(request_id, {'tools': TOOLS})
        elif method == 'tools/call':
            self._call_tool(request_id, params)
        else:
            self._send_error(request_id, -32601, f'Method not found: {method}')

    def _call_tool(self, request_id, params):
        name = params.get('name')
        arguments = params.get('arguments') or {}
        handler = getattr(self.bridge, name, None)
        if handler is None or name not in {t['name'] for t in TOOLS}:
            self._send_error(request_id, -32602, f'Unknown tool: {name}')
            return
        log(f'tools/call {name} {arguments}')
        try:
            result = handler(**arguments)
            is_error = False
            text = json.dumps(result, ensure_ascii=False, indent=2)
        except TypeError as error:
            is_error = True
            text = f'参数不匹配：{error}'
        except Exception as error:
            is_error = True
            text = f'{type(error).__name__}: {error}'
        self._send_result(request_id, {
            'content': [{'type': 'text', 'text': text}],
            'isError': is_error,
        })

    def _send_result(self, request_id, result):
        self._write({'jsonrpc': '2.0', 'id': request_id, 'result': result})

    def _send_error(self, request_id, code, message):
        self._write({'jsonrpc': '2.0', 'id': request_id,
                     'error': {'code': code, 'message': message}})

    def _write(self, payload):
        # stdio 传输按行分帧，JSON 里不能带裸换行
        sys.stdout.write(json.dumps(payload, ensure_ascii=False) + '\n')
        sys.stdout.flush()


def main():
    bridge = ArmBridge()
    log(f'就绪，协议 {FALLBACK_PROTOCOL}，工具 {len(TOOLS)} 个')
    server = McpServer(bridge)
    try:
        server.run()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()


if __name__ == '__main__':
    main()
