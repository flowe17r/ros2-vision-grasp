# ros2-vision-grasp

基于 ROS 2 Jazzy + Gazebo Harmonic 的机械臂视觉识别抓取项目（纯仿真，无需实体硬件）。

当前进度：**Phase 2 已完成，Phase 3 进行中**。Gazebo 世界中有工作台、红/绿/蓝三个 5 cm 方块和一台俯视 RGB-D 相机；视觉链路输出检测结果与 `base_link` 下的 3D 目标（实测平均误差 7.8 mm）。机械臂已加入 Gazebo，用 `ros2_control` + 关节轨迹控制器驱动（Phase 3a）。

## 环境

| 项 | 版本 |
| --- | --- |
| OS | Ubuntu 24.04 (VMware 虚拟机) |
| ROS 2 | Jazzy Jalisco |
| 仿真 | Gazebo Harmonic（`ros-jazzy-ros-gz`） |
| 控制 | ros2_control + gz_ros2_control |
| 语言 | Python 3.12（感知节点）、C++（后续 KDL 运动学） |

## 快速开始（虚拟机内执行）

```bash
# 1. 依赖
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-cv-bridge python3-opencv ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers

# 2. 仓库放家目录，再把 4 个包拷进工作空间
cp -r <本仓库路径> ~/ros2-vision-grasp
mkdir -p ~/ros2_ws/src
cp -r ~/ros2-vision-grasp/src/* ~/ros2_ws/src/

# 3. 编译
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 4. 视觉链路（Gazebo + 桥接 + 感知 + RViz）
ros2 launch arm_bringup vision.launch.py

# 5. Phase 3a：机械臂 + ros2_control（另开终端，不要和第 4 步同时跑）
ros2 launch arm_bringup arm_gazebo.launch.py
```

### 验证（另开一个终端）

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic hz /camera/image          # 有连续数据即可；虚拟机 RTF 低时实测会低于 30 Hz
ros2 topic echo /perception/detections --once
ros2 topic echo /perception/targets_base --once   # Phase 2：base_link 下的 3D 目标
ros2 control list_controllers        # Phase 3a：三个控制器均应为 active
bash ~/ros2-vision-grasp/tools/check_gz_topics.sh   # 打印 gz 与 ROS 侧的相机话题
```

期望 `detections` 中有 3 个目标，`color_name` 分别为 red / green / blue；RViz 中添加 `Image` 显示 `/perception/detect_debug` 可看到框与质心。

## 目录结构

```
src/
├── arm_interfaces/     # Detection2D / Detection2DArray 自定义消息
├── arm_description/    # 自建 6 轴 URDF + 夹爪、world、ros2_control 配置、RViz
├── arm_perception/     # HSV 分割节点、方块检测节点、反投影节点
└── arm_bringup/        # vision.launch.py（视觉）、arm_gazebo.launch.py（机械臂控制）
docs/                   # 开发日志、面试笔记
tools/                  # 话题自检脚本
.github/workflows/      # CI：colcon build + test
```

## 话题

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/camera/image` | `sensor_msgs/Image` | Gazebo → ROS | RGB 图像 640×480 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo → ROS | 内参，Phase 2 反投影用 |
| `/camera/depth_image` | `sensor_msgs/Image` | Gazebo → ROS | 深度图（Phase 2） |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS | 仿真时间 |
| `/perception/mask` | `sensor_msgs/Image` (mono8) | 节点输出 | 当前颜色掩膜 |
| `/perception/segment_debug` | `sensor_msgs/Image` | 节点输出 | 原图 + 掩膜拼接 |
| `/perception/detections` | `arm_interfaces/Detection2DArray` | 节点输出 | 目标颜色、质心像素、面积、外接框 |
| `/perception/detect_debug` | `sensor_msgs/Image` | 节点输出 | 画框 + 质心 |
| `/perception/targets_base` | `geometry_msgs/PoseArray` | 节点输出 | 深度反投影后的 3D 目标（base_link） |

参数：`hsv_segment_node` 支持 `image_topic`、`target_color`（red/green/blue）、`open_kernel`、`close_kernel`；`box_detect_node` 支持 `image_topic`、`colors`、`min_area_px`；`backproject_node` 支持 `camera_frame`、`target_frame`、`depth_patch`、`sample_u_ratio`、`sample_v_ratio`、`max_sync_gap_sec`、`eval_ground_truth`。

## Phase 2：TF 与深度反投影

TF 链由 launch 里的两个 `static_transform_publisher` 发布：

```
base_link --(0.7, 0, 0.9, rpy 0 0.728 π)--> camera_link
camera_link --(rpy -π/2 0 -π/2)--> camera_optical_frame
```

`backproject_node` 用 `camera_info` 内参和深度图把像素反投影到光学坐标系，再变换到 `base_link`，发布 `/perception/targets_base`。

注意：深度采样点取的是检测框的**顶面中心**（`sample_u_ratio=0.5`、`sample_v_ratio=0.3`），不是轮廓质心。轮廓质心会被方块侧面拉低，直接反投影会带来厘米级偏差。实测 z 约 0.30 m（方块顶面），XY 与方块中心真值比较。

标定采样位置：

```bash
ros2 param set /backproject_node sample_v_ratio 0.25
```

试 0.25 / 0.30 / 0.35，看日志里 `XY error vs cube center` 的 mean/max，取最小的一组。

真值（base_link 与 world 原点重合）：red (-0.12, 0.10, 0.275)、green (0.00, -0.10, 0.275)、blue (0.12, 0.08, 0.275)。

实测（2026-10-18）：red 9.3 mm、green 8.1 mm、blue 6.0 mm，平均 **7.8 mm**、最大 9.3 mm。

## Phase 3a：机械臂与 ros2_control

机械臂底座放在台面上，Gazebo 中 `base_link` 位于 world `(-0.35, 0, 0.25)`。URDF 通过 `world` 固定关节防止倾倒，`gz_ros2_control` 加载以下控制器：

| 控制器 | 关节 | 类型 |
| --- | --- | --- |
| `joint_state_broadcaster` | 全部 | 状态发布 |
| `arm_controller` | joint1–joint6 | JointTrajectoryController |
| `gripper_controller` | finger_left/right_joint | JointTrajectoryController |

```bash
ros2 launch arm_bringup arm_gazebo.launch.py
ros2 control list_controllers
```

测试动作（另开终端，机械臂会抬到测试位姿）：

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.0, 0.4, -0.6, 0.0, 0.4, 0.0], time_from_start: {sec: 3}}]}}"
```

> `vision.launch.py` 里的 `base_link` 仍按 world 原点定义，暂时不要和 `arm_gazebo.launch.py` 同时运行；Phase 3b 会把两者合并并把相机 TF、真值换算到台面上的 `base_link`。

## Phase 2 完成标准（已验收 2026-10-18）

- [x] `ros2 run tf2_ros tf2_echo base_link camera_optical_frame` 能查到位姿
- [x] `/perception/targets_base` 一次输出 3 个位置，x/y 与真值同号、量级一致
- [x] `XY error vs cube center` 的 mean < 10 mm（实测 7.8 mm）
- [x] 采样位置固定为 `sample_v_ratio=0.3`，精度已达标

## 常见问题

**Gazebo 里没有图像 / 桥接无数据**
先确认 gz 侧真实话题名：

```bash
gz topic -l | grep -Ei 'camera|image|depth'
```

本 world 通过 `<topic>camera</topic>` 指定，正常情况下是 `/camera/image`、`/camera/camera_info`、`/camera/depth_image`。若名字不同，按实际名字修改 `src/arm_bringup/launch/vision.launch.py` 中 bridge 的 arguments。

**Gazebo 黑屏 / 相机画面全黑**
VMware 需开启「加速 3D 图形」并给足显存（≥ 2 GB）。仍不行时试：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
gz sim -r <world 路径>
```

**编译找不到 arm_interfaces**
`arm_interfaces` 生成消息后其他包才能编译，`colcon build` 会自动排序；若仍失败，先单独 `colcon build --packages-select arm_interfaces`。

## Phase 1 完成标准

- [ ] `colcon build` 全部通过
- [ ] `/camera/image` 有连续图像（仿真时间 30 Hz；RTF 低时实测频率会低于此值）
- [ ] `/perception/detections` 每次输出 3 个目标，颜色与方块位置对应
- [ ] debug 图能看到掩膜、外接框、质心
- [ ] 记录 `hsv_segment_node` 日志里的 `avg xx.xx ms/frame`，填入简历的空缺指标

## 路线图

| Phase | 内容 | 产出 |
| --- | --- | --- |
| 1（已完成） | 仿真世界 + HSV 检测 | 检测消息、debug 图、耗时数据（26 ms）|
| 2（已完成） | TF2 + 针孔反投影 | 基座坐标系目标位姿，实测平均误差 7.8 mm |
| 3（当前） | ros2_control + KDL 正逆解 + 抓取状态机 | 仿真中完成抓取，统计逆解成功率 |
| 4 | rosbag2 录制回放 + MCP Server + DeepSeek 工具调用 | 自然语言驱动抓取 |
| 5 | socat 虚拟串口 + AA55/CRC16 协议桥 | MCU ↔ ROS2 通信演示 |

## License

Apache-2.0
