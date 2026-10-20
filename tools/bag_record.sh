#!/usr/bin/env bash
# 录制一次抓取过程，供 tools/bag_report.py 离线复现。
# 用法：bash tools/bag_record.sh [data|vision] [输出目录] [时长秒]
#   data   默认。只录感知输出、方块真值位姿、关节状态与 TF，一分钟约几 MB。
#   vision 额外录相机原始图像/深度，体积大（软件渲染下约 8 MB/s），自动按 200 MB 分卷。
# 时长省略时一直录到 Ctrl+C。
set -u

PROFILE="${1:-data}"
OUT="${2:-$HOME/bags/grasp_$(date +%Y%m%d_%H%M%S)}"
DURATION="${3:-}"

DATA_TOPICS=(
  /perception/detections
  /perception/targets_3d
  /model/cube_red/pose
  /model/cube_green/pose
  /model/cube_blue/pose
  /joint_states
  /tf
  /tf_static
)

VISION_TOPICS=(
  /camera/image
  /camera/camera_info
  /camera/depth_image
)

case "$PROFILE" in
  data)   TOPICS=("${DATA_TOPICS[@]}") ;;
  vision) TOPICS=("${DATA_TOPICS[@]}" "${VISION_TOPICS[@]}") ;;
  *) echo "未知 profile: $PROFILE（可选 data / vision）" >&2; exit 1 ;;
esac

CMD=(ros2 bag record -o "$OUT" --max-cache-size 67108864)
if [ "$PROFILE" = vision ]; then
  CMD+=(--max-bag-size 209715200)
fi
if [ -n "$DURATION" ]; then
  CMD+=(--duration "$DURATION")
fi
CMD+=("${TOPICS[@]}")

echo "输出：$OUT"
echo "执行：${CMD[*]}"
echo "录完用 tools/bag_report.py 生成报告，Ctrl+C 结束录制。"
exec "${CMD[@]}"
