#!/usr/bin/env python3
"""自然语言驱动抓取：中文指令 → LLM 工具调用 → MCP Server → ROS 2。

只用标准库，不需要 pip 装 openai / mcp。默认接 DeepSeek，也支持任何 OpenAI
兼容接口（智谱 GLM 的 Flash 系列目前免费，适合零成本演示）。

用法（虚拟机里，仿真 launch 已启动）：
    export DEEPSEEK_API_KEY=sk-xxxx          # 只从环境变量读，不写进仓库
    source ~/ros2_ws/install/setup.bash
    python3 ~/ros2-vision-grasp/tools/nl_grasp.py "把红色方块抓起来"

    换免费模型（智谱）：
    export NL_API_BASE=https://open.bigmodel.cn/api/paas/v4/chat/completions
    export NL_MODEL=glm-4.5-flash
    export NL_API_KEY=你的智谱key

    --dry-run  只打印模型想调用哪些工具，不真的动机械臂（不需要仿真在跑）
"""

import json
import os
import subprocess
import sys
import urllib.error
import urllib.request

# 默认走 DeepSeek；换免费模型时设这三个环境变量即可（接口都是 OpenAI 兼容格式）：
#   NL_API_BASE=https://open.bigmodel.cn/api/paas/v4/chat/completions
#   NL_MODEL=glm-4.5-flash   # 4.7 高峰期会 429
#   NL_API_KEY=你的智谱 key
DEFAULT_API_BASE = 'https://api.deepseek.com/chat/completions'
DEFAULT_MODEL = 'deepseek-chat'
SERVER_CMD = ['ros2', 'run', 'arm_mcp', 'mcp_server']
MAX_ROUNDS = 8

SYSTEM_PROMPT = """你在控制一台 Gazebo 仿真里的六轴机械臂，工作台上有红、绿、蓝三个方块。
规则：
1. 抓取前先用 list_targets 确认目标存在，位置单位是米，坐标系是 base_link。
2. 颜色参数只能是 red / green / blue，用户说中文「红/绿/蓝」要自己翻译。
3. grasp 会同步执行完整抓取流程并返回抬升量（毫米），耗时可能接近一分钟，不要重复调用。
4. 用户问位姿、关节角、逆解时用 solve_fk / solve_ik / get_joint_states，不要凭空编数字。
5. 工具返回 isError 时，把错误原文说清楚并给出下一步建议，不要假装成功。
6. 最后用一句中文汇报结果，带上关键数字。"""


class McpClient:
    """最小 MCP 客户端：stdio 上按行收发 JSON-RPC 2.0。"""

    def __init__(self, command):
        self.process = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            text=True, bufsize=1)
        self._next_id = 0

    def _write(self, payload):
        self.process.stdin.write(json.dumps(payload, ensure_ascii=False) + '\n')
        self.process.stdin.flush()

    def notify(self, method):
        self._write({'jsonrpc': '2.0', 'method': method})

    def request(self, method, params=None):
        self._next_id += 1
        request_id = self._next_id
        self._write({'jsonrpc': '2.0', 'id': request_id,
                     'method': method, 'params': params or {}})
        while True:
            line = self.process.stdout.readline()
            if not line:
                raise RuntimeError('MCP Server 已退出，看上面的 stderr 日志')
            payload = json.loads(line)
            if payload.get('id') != request_id:
                continue
            if 'error' in payload:
                raise RuntimeError(payload['error'].get('message', '未知错误'))
            return payload.get('result')

    def list_tools(self):
        result = self.request('initialize', {
            'protocolVersion': '2024-11-05',
            'capabilities': {},
            'clientInfo': {'name': 'nl_grasp', 'version': '0.1.0'},
        })
        print(f'已连接 MCP Server：{result.get("serverInfo", {}).get("name")} '
              f'协议 {result.get("protocolVersion")}')
        self.notify('notifications/initialized')
        return self.request('tools/list')['tools']

    def call_tool(self, name, arguments):
        result = self.request('tools/call',
                              {'name': name, 'arguments': arguments})
        text = ''.join(part.get('text', '')
                       for part in result.get('content', [])
                       if part.get('type') == 'text')
        return text, bool(result.get('isError'))

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.close()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()


def to_deepseek_tools(tools):
    return [{
        'type': 'function',
        'function': {
            'name': tool['name'],
            'description': tool.get('description', ''),
            'parameters': tool.get('inputSchema', {'type': 'object'}),
        },
    } for tool in tools]


def chat(messages, tools, api_key, api_base, model):
    body = json.dumps({
        'model': model,
        'messages': messages,
        'tools': tools,
        'temperature': 0.1,
    }, ensure_ascii=False).encode('utf-8')
    request = urllib.request.Request(api_base, data=body, method='POST', headers={
        'Content-Type': 'application/json',
        'Authorization': f'Bearer {api_key}',
    })
    try:
        with urllib.request.urlopen(request, timeout=90) as response:
            return json.loads(response.read().decode('utf-8'))
    except urllib.error.HTTPError as error:
        detail = error.read().decode('utf-8', 'replace')
        raise RuntimeError(f'{model} 返回 HTTP {error.code}：{detail}') from None


def clean_env(value):
    """从聊天窗口复制 URL / key 时常带上零宽空格或 BOM，
    塞进 urllib 会报 unknown url type，key 也会 401。"""
    return value.replace('\u200b', '').replace('\ufeff', '').strip()


DRY_RUN_TARGETS = {
    'target_count': 3,
    'frame_id': 'base_link',
    'targets': [
        {'color': 'red', 'x': 0.24, 'y': 0.10, 'z': 0.05, 'depth': 1.00},
        {'color': 'green', 'x': 0.36, 'y': -0.10, 'z': 0.05, 'depth': 0.92},
        {'color': 'blue', 'x': 0.48, 'y': 0.08, 'z': 0.05, 'depth': 0.83},
    ],
    'note': 'dry-run 预演数据，不是实时读数',
}


def dry_run_result(name, arguments):
    """预演也要返回结构完整的结果，只回一个空壳的话模型会以为没查到、
    一轮轮重试同一个工具，直到撞上轮数上限。"""
    if name == 'list_targets':
        return DRY_RUN_TARGETS
    if name == 'get_joint_states':
        return {'available': True, 'dry_run': True,
                'joints': [{'name': f'joint{i}', 'position': 0.0, 'effort': 0.0}
                           for i in range(1, 7)],
                'note': 'dry-run 预演数据'}
    if name == 'grasp':
        return {'success': True, 'dry_run': True,
                'color': arguments.get('color'),
                'message': '预演：抓取请求已记录，真实运行时才会下发轨迹'}
    if name == 'bag_report':
        return {'dry_run': True, 'report': '预演：未解析 rosbag'}
    return {'success': True, 'dry_run': True, 'message': '预演：未调用运动学服务'}


def main():
    argv = sys.argv[1:]
    dry_run = '--dry-run' in argv
    argv = [a for a in argv if a != '--dry-run']

    api_base = clean_env(os.environ.get('NL_API_BASE', DEFAULT_API_BASE))
    model = clean_env(os.environ.get('NL_MODEL', DEFAULT_MODEL))
    api_key = clean_env(os.environ.get('NL_API_KEY')
                        or os.environ.get('DEEPSEEK_API_KEY') or '')
    if not api_key:
        print('缺少 API key 环境变量，任选其一：', file=sys.stderr)
        print('  export DEEPSEEK_API_KEY=你的key', file=sys.stderr)
        print('  export NL_API_KEY=你的key   # 配 NL_API_BASE + NL_MODEL 换免费模型',
              file=sys.stderr)
        return 1

    instruction = ' '.join(argv).strip() or input('你说：').strip()
    if not instruction:
        print('指令为空', file=sys.stderr)
        return 1

    print(f'模型：{model}（{api_base}）')
    client = McpClient(SERVER_CMD)
    try:
        tools = to_deepseek_tools(client.list_tools())
        print(f'可用工具 {len(tools)} 个：'
              f'{"、".join(t["function"]["name"] for t in tools)}')
        print(f'指令：{instruction}\n')

        messages = [{'role': 'system', 'content': SYSTEM_PROMPT},
                    {'role': 'user', 'content': instruction}]
        if dry_run:
            messages[0]['content'] += (
                '\n\n当前是 dry-run 预演：工具不会真正执行，结果里带 dry_run 标记'
                '是正常的。拿到需要的数据后直接总结，不要重复调用同一个工具。')

        for round_index in range(1, MAX_ROUNDS + 1):
            reply = chat(messages, tools, api_key, api_base,
                         model)['choices'][0]['message']
            content = reply.get('content') or ''
            tool_calls = reply.get('tool_calls') or []
            if content:
                print(f'模型：{content}')
            if not tool_calls:
                return 0

            messages.append({
                'role': 'assistant',
                'content': content,
                'tool_calls': tool_calls,
            })
            for call in tool_calls:
                function = call['function']
                name = function['name']
                try:
                    arguments = json.loads(function.get('arguments') or '{}')
                except json.JSONDecodeError:
                    arguments = {}
                print(f'[第 {round_index} 轮] 调用 {name} {arguments}')
                if dry_run:
                    text = json.dumps(dry_run_result(name, arguments),
                                      ensure_ascii=False)
                else:
                    # 工具报错要回喂给模型让它自己纠正，直接抛异常会中断整轮对话
                    try:
                        text, is_error = client.call_tool(name, arguments)
                    except Exception as error:
                        text, is_error = f'{type(error).__name__}: {error}', True
                    print(f'  → {"失败" if is_error else "成功"}：{text}')
                messages.append({
                    'role': 'tool',
                    'tool_call_id': call['id'],
                    'content': text,
                })
        print(f'超过 {MAX_ROUNDS} 轮仍未收敛，已停止。', file=sys.stderr)
        return 1
    finally:
        client.close()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as error:  # 顶层兜底，避免 traceback 糊一脸
        print(f'出错：{error}', file=sys.stderr)
        sys.exit(1)
