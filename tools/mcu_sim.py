#!/usr/bin/env python3
"""MCU 模拟器：AA55/CRC16 协议对端，配合 socat 虚拟串口演示。

帧格式：AA 55 | version | cmd | seq | len | payload[0..64] | crc_lo crc_hi
CRC16-CCITT-FALSE 覆盖 version..payload，低字节在前。
"""

import argparse
import math
import os
import select
import struct
import sys
import termios
import time
import tty

SYNC0 = 0xAA
SYNC1 = 0x55
VERSION = 0x01
MAX_PAYLOAD = 64

CMD_PING = 0x01
CMD_SET_JOINTS = 0x02
CMD_GET_STATUS = 0x03
CMD_ACK = 0x10
CMD_STATUS = 0x11
CMD_ERROR = 0x7F

STATUS_FORMAT = '<IfH6fI'
STATUS_SIZE = struct.calcsize(STATUS_FORMAT)


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(cmd, seq, payload=b''):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('payload exceeds 64 bytes')
    body = bytes([VERSION, cmd & 0xFF, seq & 0xFF, len(payload)]) + payload
    crc = crc16(body)
    return bytes([SYNC0, SYNC1]) + body + bytes([crc & 0xFF, crc >> 8])


class FrameParser:
    """与 C++ FrameParser 同构的状态机，用于自检和接收桥接端指令。"""

    def __init__(self):
        self.state = 0
        self.cmd = 0
        self.seq = 0
        self.length = 0
        self.payload = bytearray()
        self.crc_lo = 0
        self.crc_errors = 0
        self.length_errors = 0
        self.version_errors = 0

    def push(self, byte):
        state = self.state
        if state == 0:
            if byte == SYNC0:
                self.state = 1
        elif state == 1:
            if byte == SYNC1:
                self.state = 2
            elif byte != SYNC0:
                self.state = 0
        elif state == 2:
            if byte != VERSION:
                self.version_errors += 1
                self.reset()
            else:
                self.state = 3
        elif state == 3:
            self.cmd = byte
            self.state = 4
        elif state == 4:
            self.seq = byte
            self.state = 5
        elif state == 5:
            if byte > MAX_PAYLOAD:
                self.length_errors += 1
                self.reset()
            else:
                self.length = byte
                self.payload = bytearray()
                self.state = 7 if byte == 0 else 6
        elif state == 6:
            self.payload.append(byte)
            if len(self.payload) == self.length:
                self.state = 7
        elif state == 7:
            self.crc_lo = byte
            self.state = 8
        elif state == 8:
            body = bytes([VERSION, self.cmd, self.seq, self.length]) + bytes(self.payload)
            frame = None
            if crc16(body) == (self.crc_lo | (byte << 8)):
                frame = (self.cmd, self.seq, bytes(self.payload))
            else:
                self.crc_errors += 1
            self.reset()
            return frame
        return None

    def reset(self):
        self.state = 0
        self.payload = bytearray()


def self_test():
    assert crc16(b'123456789') == 0x29B1, 'CRC 已知向量不符'
    assert STATUS_SIZE == 38, f'状态帧长度 {STATUS_SIZE} != 38'

    parser = FrameParser()
    frames = [frame for frame in
              (parser.push(byte) for byte in encode(CMD_STATUS, 9, bytes(38))) if frame]
    assert len(frames) == 1 and frames[0][0] == CMD_STATUS and frames[0][1] == 9

    corrupted = bytearray(encode(CMD_PING, 1))
    corrupted[-1] ^= 0xFF
    frames = [frame for frame in
              (parser.push(byte) for byte in corrupted) if frame]
    assert not frames and parser.crc_errors == 1

    sticky = encode(CMD_PING, 2) + encode(CMD_SET_JOINTS, 3, bytes(24))
    frames = [frame for frame in
              (parser.push(byte) for byte in sticky) if frame]
    assert [frame[0] for frame in frames] == [CMD_PING, CMD_SET_JOINTS]

    parser = FrameParser()
    assert parser.push(SYNC0) is None and parser.state != 0
    parser.reset()
    assert parser.state == 0
    print('self-test ok: CRC 已知向量 / 回环 / 坏帧计数 / 粘包 / reset')


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        tty.setraw(fd)
        attrs = termios.tcgetattr(fd)
        speed = termios.B115200
        attrs[4] = speed
        attrs[5] = speed
        attrs[2] |= termios.CLOCAL | termios.CREAD
        attrs[2] &= ~getattr(termios, 'CRTSCTS', 0)
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
    except Exception:
        os.close(fd)
        raise
    return fd


class McuSim:
    def __init__(self, fd, corrupt_every=0, verbose=False):
        self.fd = fd
        self.corrupt_every = corrupt_every
        self.verbose = verbose
        self.parser = FrameParser()
        self.positions = [0.0] * 6
        self.targets = [0.0] * 6
        self.seq = 0
        self.uptime_ms = 0
        self.frames_in = 0
        self.status_sent = 0

    def write(self, data):
        view = memoryview(data)
        while view:
            try:
                written = os.write(self.fd, view)
            except BlockingIOError:
                select.select([], [self.fd], [], 0.2)
                continue
            view = view[written:]

    def next_seq(self):
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFF
        return seq

    def send_ack(self, cmd, seq, result=0):
        self.write(encode(CMD_ACK, self.next_seq(), bytes([cmd, seq, result])))

    def send_error(self, seq, code):
        self.write(encode(CMD_ERROR, self.next_seq(), bytes([seq, code])))

    def status_payload(self):
        temperature = 36.5 + 0.6 * math.sin(self.uptime_ms / 10000.0)
        return struct.pack(STATUS_FORMAT, self.uptime_ms & 0xFFFFFFFF, temperature, 0,
                           *self.positions, self.parser.crc_errors)

    def send_status(self):
        frame = bytearray(encode(CMD_STATUS, self.next_seq(), self.status_payload()))
        self.status_sent += 1
        if self.corrupt_every and self.status_sent % self.corrupt_every == 0:
            frame[-1] ^= 0xFF
            if self.verbose:
                print(f'  [注入] 第 {self.status_sent} 帧状态故意损坏 CRC')
        self.write(frame)

    def step(self, dt):
        limit = 0.8 * dt
        for index, (position, target) in enumerate(zip(self.positions, self.targets)):
            delta = max(-limit, min(limit, target - position))
            self.positions[index] = round(position + delta, 6)

    def handle(self, cmd, seq, payload):
        self.frames_in += 1
        if self.verbose:
            print(f'  [收到] cmd=0x{cmd:02X} seq={seq} len={len(payload)}')
        if cmd == CMD_PING:
            self.send_ack(cmd, seq)
        elif cmd == CMD_SET_JOINTS:
            if len(payload) != 24:
                self.send_error(seq, 0x02)
                return
            self.targets = list(struct.unpack('<6f', payload))
            self.send_ack(cmd, seq)
        elif cmd == CMD_GET_STATUS:
            self.send_ack(cmd, seq)
            self.send_status()
        else:
            self.send_error(seq, 0x01)


def run(args):
    try:
        fd = open_port(args.port)
    except OSError as error:
        print(f'打不开 {args.port}：{error}', file=sys.stderr)
        print('先运行 tools/serial_demo.sh 起 socat 虚拟串口对。', file=sys.stderr)
        return 1

    sim = McuSim(fd, corrupt_every=args.corrupt_every, verbose=args.verbose)
    interval = 1.0 / args.rate
    start = time.monotonic()
    last_tick = start
    next_status = start
    print(f'MCU 模拟器已连接 {args.port}，status {args.rate} Hz'
          + (f'，每 {args.corrupt_every} 帧损坏一次 CRC' if args.corrupt_every else ''))
    try:
        while True:
            now = time.monotonic()
            timeout = max(0.0, next_status - now)
            ready, _, _ = select.select([fd], [], [], timeout)
            if ready:
                data = os.read(fd, 4096)
                if not data:
                    print('\n串口对端已关闭（socat 停了吗？）')
                    break
                for byte in data:
                    frame = sim.parser.push(byte)
                    if frame:
                        sim.handle(*frame)
            now = time.monotonic()
            if now >= next_status:
                sim.uptime_ms = int((now - start) * 1000)
                sim.step(now - last_tick)
                last_tick = now
                sim.send_status()
                next_status += interval
                if next_status < now:
                    next_status = now + interval
            if args.duration and now - start >= args.duration:
                break
    except KeyboardInterrupt:
        print('\n已停止')
    finally:
        os.close(fd)
    print(f'统计：收到 {sim.frames_in} 帧，CRC 错误 {sim.parser.crc_errors} 帧，'
          f'发送状态 {sim.status_sent} 帧')
    return 0


def main():
    parser = argparse.ArgumentParser(description='AA55/CRC16 MCU 模拟器')
    parser.add_argument('--port', default='/tmp/ttyMCU', help='socat 的 MCU 端')
    parser.add_argument('--rate', type=float, default=10.0, help='状态帧频率 Hz')
    parser.add_argument('--corrupt-every', type=int, default=0,
                        help='每 N 帧状态故意损坏 CRC，0 表示关闭')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='运行秒数，0 表示一直运行')
    parser.add_argument('--verbose', action='store_true', help='打印收到的指令')
    parser.add_argument('--self-test', action='store_true', help='只跑编解码自检')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
