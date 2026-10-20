#!/usr/bin/env bash
# 建立一对虚拟串口：/tmp/ttyMCU <-> /tmp/ttyROS（Ctrl-C 结束）
set -euo pipefail

if ! command -v socat >/dev/null 2>&1; then
  echo "socat 未安装，先执行：sudo apt install -y socat" >&2
  exit 1
fi

rm -f /tmp/ttyMCU /tmp/ttyROS
echo "虚拟串口对已建立：MCU=/tmp/ttyMCU  ROS=/tmp/ttyROS"
exec socat -d -d pty,raw,echo=0,link=/tmp/ttyMCU \
  pty,raw,echo=0,link=/tmp/ttyROS
