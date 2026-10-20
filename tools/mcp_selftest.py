#!/usr/bin/env python3
"""MCP Server 自检：不起 DeepSeek、不用 API key，验证协议握手与只读工具。

用法（虚拟机里，仿真 launch 已启动；只跑握手检查时可以不开仿真）：
    source ~/ros2_ws/install/setup.bash
    python3 ~/ros2-vision-grasp/tools/mcp_selftest.py

做四件事：initialize 握手、tools/list 应返回 6 个工具、ping、list_targets 与
get_joint_states 两个只读工具能拿到 JSON。不调用 grasp，不会动机械臂。
"""

import json
import subprocess
import sys

SERVER_CMD = ['ros2', 'run', 'arm_mcp', 'mcp_server']
EXPECT_TOOLS = {'list_targets', 'get_joint_states', 'solve_fk', 'solve_ik',
                'grasp', 'bag_report'}
TIMEOUT = 30


def rpc(process, payload):
    process.stdin.write(json.dumps(payload, ensure_ascii=False) + '\n')
    process.stdin.flush()
    if 'id' not in payload:
        return None
    while True:
        line = process.stdout.readline()
        if not line:
            raise RuntimeError('MCP Server 已退出，看上面的 stderr')
        message = json.loads(line)
        if message.get('id') == payload['id']:
            return message


def check(label, condition, detail=''):
    print(f'[{"PASS" if condition else "FAIL"}] {label}'
          + (f' —— {detail}' if detail else ''))
    return bool(condition)


def main():
    process = subprocess.Popen(
        SERVER_CMD, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        text=True, bufsize=1)
    ok = True
    try:
        init = rpc(process, {
            'jsonrpc': '2.0', 'id': 1, 'method': 'initialize',
            'params': {'protocolVersion': '2024-11-05', 'capabilities': {},
                       'clientInfo': {'name': 'mcp_selftest', 'version': '0.1.0'}},
        })
        result = init.get('result', {})
        ok &= check('initialize 握手', 'protocolVersion' in result,
                    f"protocol={result.get('protocolVersion')} "
                    f"server={result.get('serverInfo', {}).get('name')}")

        rpc(process, {'jsonrpc': '2.0', 'method': 'notifications/initialized'})

        listed = rpc(process, {'jsonrpc': '2.0', 'id': 2, 'method': 'tools/list'})
        names = {tool['name'] for tool in listed.get('result', {}).get('tools', [])}
        ok &= check('tools/list 返回 6 个工具', names == EXPECT_TOOLS,
                    f'实际 {sorted(names)}')

        ping = rpc(process, {'jsonrpc': '2.0', 'id': 3, 'method': 'ping'})
        ok &= check('ping', 'result' in ping)

        for tool in ('list_targets', 'get_joint_states'):
            response = rpc(process, {
                'jsonrpc': '2.0', 'id': 4, 'method': 'tools/call',
                'params': {'name': tool, 'arguments': {}}})
            payload = response.get('result', {})
            text = ''.join(part.get('text', '')
                           for part in payload.get('content', [])
                           if part.get('type') == 'text')
            try:
                parsed = json.loads(text)
                valid = isinstance(parsed, dict)
            except json.JSONDecodeError:
                valid = False
            ok &= check(f'只读工具 {tool}', valid and not payload.get('isError'),
                        text.replace('\n', ' ')[:160])

        unknown = rpc(process, {
            'jsonrpc': '2.0', 'id': 5, 'method': 'tools/call',
            'params': {'name': 'shutdown', 'arguments': {}}})
        ok &= check('白名单外的工具名被拒绝', 'error' in unknown,
                    unknown.get('error', {}).get('message', ''))
    except Exception as error:
        print(f'[FAIL] 自检中断：{error}')
        ok = False
    finally:
        process.stdin.close()
        try:
            process.wait(timeout=TIMEOUT)
        except subprocess.TimeoutExpired:
            process.kill()

    print('自检通过' if ok else '自检未通过')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
