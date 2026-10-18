#!/usr/bin/env bash
# 在虚拟机内、Gazebo 已启动时运行：bash tools/check_gz_topics.sh
set -u

echo "== gz 侧相机相关话题 =="
gz topic -l 2>/dev/null | grep -Ei 'camera|image|depth' || echo "(未找到，确认 Gazebo 已启动)"

echo
echo "== ROS 侧话题 =="
ros2 topic list 2>/dev/null | grep -E 'camera|perception' || echo "(未找到，确认已 source install/setup.bash)"

echo
echo "== /camera/image 频率（采样 3 秒，末尾数字即 Hz）=="
timeout 3 ros2 topic hz /camera/image 2>&1 | tail -n 6

echo
echo "== 检测结果（等待一帧）=="
timeout 5 ros2 topic echo /perception/detections --once 2>/dev/null || echo "(无数据)"
