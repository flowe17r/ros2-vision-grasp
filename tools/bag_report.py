#!/usr/bin/env python3
"""离线读一个 rosbag2，输出抓取过程报告，不用重开 Gazebo。

用法：
    python3 ~/ros2-vision-grasp/tools/bag_report.py ~/bags/grasp_xxx

报告内容：每话题消息数与时间跨度、三色目标的首末位置、方块真值的抬升量与
水平漂移、末帧关节角。抬升判定沿用抓取节点的 30 mm 门槛。
"""

import re
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

LIFT_THRESHOLD = 0.03
COLORS = ('red', 'green', 'blue')


def detect_storage_id(uri):
    """metadata.yaml 里记着真正的 storage id，写死 sqlite3 会在 mcap 包上报错。"""
    try:
        with open(f'{uri}/metadata.yaml', encoding='utf-8') as handle:
            match = re.search(r'storage_identifier:\s*(\S+)', handle.read())
        if match:
            return match.group(1)
    except OSError:
        pass
    return 'sqlite3'


def open_reader(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id=detect_storage_id(uri)),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'))
    types = {meta.name: meta.type
             for meta in reader.get_all_topics_and_types()}
    return reader, types


def type_cache():
    """消息类型只解析一次；包没装的话题跳过并记账，不要让整个报告崩掉。"""
    resolved = {}
    missing = set()

    def resolve(type_name):
        if type_name in resolved:
            return resolved[type_name]
        if type_name in missing:
            return None
        try:
            resolved[type_name] = get_message(type_name)
        except (ImportError, AttributeError, ValueError):
            missing.add(type_name)
            return None
        return resolved[type_name]

    return resolve, missing


def fmt_point(point):
    return f'({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})'


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    uri = sys.argv[1].rstrip('/')

    reader, types = open_reader(uri)
    resolve, missing = type_cache()
    counters = {}
    targets = {}
    cubes = {c: {'first': None, 'max_z': None, 'first_xy': None, 'last_xy': None}
             for c in COLORS}
    last_joints = None
    first_t = last_t = None

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg_type = types.get(topic)
        if msg_type is None:
            continue
        counters[topic] = counters.get(topic, 0) + 1
        if first_t is None:
            first_t = stamp
        last_t = stamp

        message_class = resolve(msg_type)
        if message_class is None:
            continue
        msg = deserialize_message(data, message_class)

        if topic.startswith('/model/cube_'):
            # /model/cube_red/pose -> cube_red -> red
            name = topic.rsplit('/', 2)[-2]
            color = name[len('cube_'):] if name.startswith('cube_') else ''
            if color in cubes:
                pos = msg.position
                entry = cubes[color]
                if entry['first'] is None:
                    entry['first'] = pos.z
                    entry['first_xy'] = (pos.x, pos.y)
                if entry['max_z'] is None or pos.z > entry['max_z']:
                    entry['max_z'] = pos.z
                entry['last_xy'] = (pos.x, pos.y)
        elif topic == '/perception/targets_3d':
            for target in msg.targets:
                slot = targets.setdefault(
                    target.color_name, {'count': 0, 'first': None, 'last': None})
                slot['count'] += 1
                point = (target.position.x, target.position.y, target.position.z)
                if slot['first'] is None:
                    slot['first'] = point
                slot['last'] = point
        elif topic == '/joint_states':
            last_joints = (msg.name, msg.position)

    duration = (last_t - first_t) / 1e9 if first_t is not None else 0.0
    print(f'# rosbag 报告：{uri}')
    print(f'时长 {duration:.1f} s，话题 {len(counters)} 个，'
          f'消息 {sum(counters.values())} 条')
    print()
    print('| 话题 | 类型 | 消息数 |')
    print('| --- | --- | --- |')
    for topic in sorted(counters):
        print(f'| {topic} | {types[topic]} | {counters[topic]} |')

    if missing:
        print()
        print(f'> 以下类型本机没有消息定义，只计数不解析：{"、".join(sorted(missing))}')

    if targets:
        print()
        print('## 视觉目标（base_link 系，m）')
        print()
        print('| 颜色 | 帧数 | 首次 | 末次 |')
        print('| --- | --- | --- | --- |')
        for color in sorted(targets):
            slot = targets[color]
            print(f'| {color} | {slot["count"]} | {fmt_point(slot["first"])} '
                  f'| {fmt_point(slot["last"])} |')

    if any(entry['first'] is not None for entry in cubes.values()):
        print()
        print('## 方块真值抬升校验（门槛 30 mm）')
        print()
        print('| 颜色 | 抬升 | 水平漂移 | 结论 |')
        print('| --- | --- | --- | --- |')
        for color in COLORS:
            entry = cubes[color]
            if entry['first'] is None:
                continue
            lift = (entry['max_z'] - entry['first']) * 1000
            drift = ((entry['last_xy'][0] - entry['first_xy'][0]) ** 2
                     + (entry['last_xy'][1] - entry['first_xy'][1]) ** 2) ** 0.5
            verdict = '抓取成功' if lift > LIFT_THRESHOLD * 1000 else '未抬起'
            print(f'| {color} | {lift:.0f} mm | {drift * 1000:.1f} mm | {verdict} |')

    if last_joints:
        names, positions = last_joints
        print()
        print('## 末帧关节角（rad）')
        print()
        print('`' + ', '.join(f'{n}={p:.4f}' for n, p in zip(names, positions)) + '`')

    return 0


if __name__ == '__main__':
    sys.exit(main())
