# ros2-vision-grasp

基于 ROS 2 Jazzy + Gazebo Harmonic 的机械臂视觉识别抓取项目（纯仿真，无需实体硬件）。

当前进度：**Phase 3b 已验收（2026-10-19）**。Gazebo 世界中有工作台、红/绿/蓝三个可抓取的 5 cm 方块、一台俯视 RGB-D 相机和自建 6 轴机械臂；视觉链路输出检测结果与 `base_link` 下的 3D 目标，KDL 正逆解服务驱动抓取状态机，抬升后用 Gazebo 位姿话题校验方块是否真的被抓起。

实测：三色定位误差 **1–9 mm**（+x 为主），IK 成功率 **28/30 = 93.3%**（单位姿态、含 3 个方块目标，全部通过），三色抓取抬升 **red 130 mm / green 128 mm / blue 117 mm**，均 > 30 mm 门槛。

> VMware 下 Gazebo 必须加 `LIBGL_ALWAYS_SOFTWARE=1` 前缀启动，否则 SVGA3D 渲染退化会让相机只输出背景色、检测全空（详见「常见问题」）。

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
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-cv-bridge python3-opencv ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-kdl-parser ros-jazzy-rosbag2 libeigen3-dev

# 2. 仓库放家目录，再把 7 个包拷进工作空间
cp -r <本仓库路径> ~/ros2-vision-grasp
mkdir -p ~/ros2_ws/src
cp -r ~/ros2-vision-grasp/src/* ~/ros2_ws/src/

# 3. 编译
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 4. 全系统：机械臂 + 相机 + 感知 + KDL + 抓取 + RViz
ros2 launch arm_bringup arm_gazebo.launch.py
# 无 RViz（虚拟机推荐）：
ros2 launch arm_bringup arm_gazebo.launch.py use_rviz:=false

# 5. 只跑视觉链路、不 spawn 机械臂（Phase 1/2 独立调试用）
ros2 launch arm_bringup vision.launch.py use_rviz:=false
```

### 验证（另开一个终端）

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic hz /camera/image          # 有连续数据即可；虚拟机 RTF 低时实测会低于 30 Hz
ros2 topic echo /perception/detections --once
ros2 topic echo /perception/targets_base --once   # base_link 下的 3D 目标
ros2 topic echo /perception/targets_3d --once     # 带颜色名的 3D 目标（抓取用）
ros2 topic echo /model/cube_red/pose --once       # 方块真值位姿，抓取校验用
ros2 control list_controllers        # 三个控制器均应为 active
ros2 service list | grep kinematics  # /kinematics/solve_fk、/kinematics/solve_ik
ros2 service call /grasp/execute arm_interfaces/srv/Grasp "{color: 'red'}"
bash ~/ros2-vision-grasp/tools/check_gz_topics.sh   # 打印 gz 与 ROS 侧的相机话题
```

期望 `detections` 中有 3 个目标，`color_name` 分别为 red / green / blue；RViz 中添加 `Image` 显示 `/perception/detect_debug` 可看到框与质心。

## 目录结构

```
src/
├── arm_interfaces/     # 检测/3D 目标消息，正逆解与抓取服务
├── arm_description/    # 自建 6 轴 URDF + 夹爪、world、ros2_control 配置、RViz
├── arm_perception/     # HSV 分割节点、方块检测节点、反投影节点
├── arm_kinematics/     # KDL 正逆解服务（C++）
├── arm_grasp/          # 抓取状态机（Python）
├── arm_mcp/            # MCP Server：把感知/运动学/抓取包成 LLM 可调用的工具
└── arm_bringup/        # vision.launch.py（仅视觉）、arm_gazebo.launch.py（全系统）
docs/                   # 开发日志、面试笔记
tools/                  # 话题自检、IK 成功率采样、rosbag 录制与离线报告、自然语言抓取
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
| `/perception/targets_3d` | `arm_interfaces/Target3DArray` | 节点输出 | 带颜色名的 3D 目标，抓取节点使用 |
| `/model/cube_{red,green,blue}/pose` | `geometry_msgs/Pose` | Gazebo → ROS | 方块模型的世界位姿，用于校验是否被抬起（`pose/info` 经桥接会丢失帧名，不用它） |

服务：

| 服务 | 类型 | 说明 |
| --- | --- | --- |
| `/kinematics/solve_fk` | `arm_interfaces/SolveFK` | 关节角 → `tcp` 位姿 |
| `/kinematics/solve_ik` | `arm_interfaces/SolveIK` | `tcp` 目标位姿 + 种子 → 关节角（多种子重试、限位与误差校验） |
| `/grasp/execute` | `arm_interfaces/Grasp` | 按颜色执行一次完整抓取 |

参数：`hsv_segment_node` 支持 `image_topic`、`target_color`（red/green/blue）、`open_kernel`、`close_kernel`；`box_detect_node` 支持 `image_topic`、`colors`、`min_area_px`；`backproject_node` 支持 `camera_frame`、`target_frame`、`depth_patch`、`sample_u_ratio`、`sample_v_ratio`、`max_sync_gap_sec`、`eval_ground_truth`。

## Phase 2：TF 与深度反投影

TF 链由 launch 里的两个 `static_transform_publisher` 发布：

```
base_link --(1.05, 0, 0.65, rpy 0 0.728 π)--> camera_link
camera_link --(rpy -π/2 0 -π/2)--> camera_optical_frame
```

`vision.launch.py` 的 `base_link` 在 world 原点，对应的相机偏移是 `(0.7, 0, 0.9)`；`arm_gazebo.launch.py` 的机械臂底座在 `(-0.35, 0, 0.25)`，所以偏移换算为 `(1.05, 0, 0.65)`。

`backproject_node` 用 `camera_info` 内参和深度图把像素反投影到光学坐标系，再变换到 `base_link`，发布 `/perception/targets_base`。

注意：深度采样点取的是检测框的**顶面中心**（`sample_u_ratio=0.5`、`sample_v_ratio=0.3`），不是轮廓质心。轮廓质心会被方块侧面拉低，直接反投影会带来厘米级偏差。实测 z 约 0.30 m（方块顶面），XY 与方块中心真值比较。

标定采样位置：

```bash
ros2 param set /backproject_node sample_v_ratio 0.25
```

试 0.25 / 0.30 / 0.35，看日志里 `XY error vs cube center` 的 mean/max，取最小的一组。

真值（`arm_gazebo.launch.py`，base_link 在 world `(-0.35, 0, 0.25)`）：red (0.23, 0.10, 0.025)、green (0.35, -0.10, 0.025)、blue (0.47, 0.08, 0.025)。

Phase 2 实测（2026-10-18，`base_link` 与 world 原点重合的旧坐标系）：red 9.3 mm、green 8.1 mm、blue 6.0 mm，平均 **7.8 mm**、最大 9.3 mm。

Phase 3b 集成后复测（2026-10-18，`base_link` 移到台面、相机 TF 改为 `(1.05, 0, 0.65)`）：`/perception/targets_3d` 输出 red (0.239, 0.099, 0.050)、green (0.358, −0.099, 0.050)、blue (0.476, 0.079, 0.051)，与真值 XY 误差 **1–9 mm**（x 偏 +6…+9 mm，y ≈1 mm）；z=0.050 是方块顶面（中心 0.025 + 半高 0.025），符合预期。**两组数字坐标系不同，不可混用。**

> 已知偏差：机械臂进入相机视野后会遮挡方块，red 的 x 从 0.239 漂到 0.254（+15…24 mm）；`/model/cube_red/pose` 显示方块并未移动，偏差来自遮挡下的 HSV 分割，不是方块被推动。

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

> `arm_gazebo.launch.py` 已把相机桥接、感知、反投影、KDL 与抓取节点整合进来（`use_vision:=true` 默认），不要再和 `vision.launch.py` 同时运行。VMware 下默认只启动 Gazebo server（`use_gz_gui:=false`），加 `use_rviz:=false` 可进一步降低 3D 负载。

## Phase 3b：TCP、KDL 正逆解与抓取

- `tcp` 固定在 `gripper_base` 下方 35 mm，KDL 链为 `base_link → tcp`。
- **工具偏置**：`gripper_mount` 为 `tool0 → gripper_base` 的 `z = -0.03`，即 TCP 在 `tool0` 下方 65 mm。夹爪沿 TCP 的 −z 抓取，TCP 的 +z 必须朝上，`joint5`/`joint6` 因此要反向折约 127°，`link5` 的腕筒（r=32 mm）会被折到两指之间；不留这 60 mm 偏置，腕筒下探时会整根插进方块体积。
- `joint5` 限位扩到 ±3.1 rad，使夹爪能朝下抓取；夹爪指间隙闭合 40 mm、张开 15 mm 时 70 mm，适配 50 mm 方块。
- `joint2`/`joint3` 的 effort 由 30 提到 **100 N·m**：gz_ros2_control 的位置控制在远端目标处索取的力矩远大于重力矩（实测 joint2 79–93、joint3 38 N·m），30 会被削顶导致轨迹中止。
- 三个方块改为动态刚体（50 g，摩擦 1.0），指垫摩擦 5.0。
- `/kinematics/solve_ik` 使用 KDL LMA，先试调用方种子，再试当前关节角、零位与 8 组随机种子；结果必须满足关节限位、位置误差 < 5 mm、姿态误差 < 0.1 rad。
- 抓取状态机：开爪 → 预抓取（z=0.16）→ 下降（z=**0.050**）→ 闭合 → 抬升（z=0.18），用 `/model/cube_<color>/pose` 比较方块抬升前后的 z，只有抬升 > 30 mm 才返回成功。
  - 下降高度不能低于 0.040：腕筒最低点约在 TCP + 10 mm，方块顶面 0.050，再低就会顶住方块，JTC 以 `goal_time_tolerance` 中止。IK 失败时的回退档位因此只往上加（+0.015 / +0.03）。

```bash
# 正解自检：零位时 tcp 约在 base_link 上方 0.785 m
ros2 service call /kinematics/solve_fk arm_interfaces/srv/SolveFK "{joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 逆解单点：红色方块抓取位姿，TCP 的 +z 朝上（单位姿态）
ros2 service call /kinematics/solve_ik arm_interfaces/srv/SolveIK "{target: {position: {x: 0.23, y: 0.10, z: 0.05}, orientation: {w: 1.0}}, seed: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 抓取红色方块
ros2 service call /grasp/execute arm_interfaces/srv/Grasp "{color: 'red'}"

# IK 成功率采样（27 个网格点 + 3 个方块点）
python3 ~/ros2-vision-grasp/tools/ik_benchmark.py
```

## Phase 3b 完成标准（已验收 2026-10-19）

- [x] 六个包在虚拟机 `colcon build --symlink-install` 全部通过
- [x] `joint_state_broadcaster` / `arm_controller` / `gripper_controller` 三个控制器 active
- [x] 零位正解 TCP z = **0.785 m**（工具偏置后的新链路）
- [x] `/perception/targets_3d` 三色输出与真值 XY 误差 1–9 mm
- [x] IK 成功率 **28/30 = 93.3%**（单位姿态；3 个方块目标全部通过，2 个失败点是 `(0.22, 0.00)` 正前方近距离，肘部内折超出 joint3 ±2.5）
- [x] 三色抓取并抬升 > 30 mm：**red 130 mm / green 128 mm / blue 117 mm**（同一次干净启动内连续完成）

## Phase 4：rosbag2 离线复现与 MCP / 大模型自然语言抓取

**4a rosbag2**：录一次抓取，之后不开 Gazebo 也能复现和出验收报告。

```bash
# 录制（终端 A，仿真已在跑）。data 只录小话题，一分钟几 MB
bash ~/ros2-vision-grasp/tools/bag_record.sh data

# 需要连相机图像一起录时用 vision，自动按 200 MB 分卷；第三个参数是时长（秒）
bash ~/ros2-vision-grasp/tools/bag_record.sh vision ~/bags/vision_demo 30

# 另开终端 B 触发抓取，录完 Ctrl+C
ros2 service call /grasp/execute arm_interfaces/srv/Grasp "{color: 'red'}"

# 离线报告：话题消息数、三色目标轨迹、方块抬升量与水平漂移、末帧关节角
python3 ~/ros2-vision-grasp/tools/bag_report.py ~/bags/grasp_xxxxx

# 回放（--clock 让 use_sim_time 的节点跟上包里的时间）
ros2 bag play ~/bags/grasp_xxxxx --clock
```

`data` profile 录 `/perception/detections`、`/perception/targets_3d`、三个 `/model/cube_*/pose`、`/joint_states`、`/tf`、`/tf_static`；`vision` 再加 `/camera/image`、`/camera/camera_info`、`/camera/depth_image`。`bag_report.py` 从 `metadata.yaml` 读 `storage_identifier`，sqlite3 与 mcap 包都能读。

**4b MCP Server**：`arm_mcp` 包用标准库实现 MCP 的 stdio JSON-RPC 2.0（`initialize` / `tools/list` / `tools/call` / `ping`），不依赖 pip 的 mcp SDK；内部起一个 rclpy 节点，把 ROS 侧能力暴露成 6 个工具：

| 工具 | 对应 ROS 接口 |
| --- | --- |
| `list_targets` | 订阅 `/perception/targets_3d` 的最新快照 |
| `get_joint_states` | 订阅 `/joint_states` |
| `solve_fk` | `/kinematics/solve_fk` |
| `solve_ik` | `/kinematics/solve_ik`（姿态固定为夹爪朝下） |
| `grasp` | `/grasp/execute`，同步等完，超时 120 s |
| `bag_report` | 子进程跑 `tools/bag_report.py`，只读不写 |

```bash
source ~/ros2_ws/install/setup.bash
ros2 run arm_mcp mcp_server     # 日志走 stderr，stdout 留给 JSON-RPC

# 自检：握手、工具清单、ping、两个只读工具、白名单拦截。不需要 API key
python3 ~/ros2-vision-grasp/tools/mcp_selftest.py
```

**4c 自然语言抓取**：`tools/nl_grasp.py` 拉起 MCP Server，把 `tools/list` 转成 function-calling schema，用 `urllib` 调模型接口，执行返回的 `tool_calls` 并把结果回喂，直到给出中文结论。默认接 DeepSeek，用三个环境变量可切到任意 OpenAI 兼容接口——智谱 GLM 的 4.x Flash 系列目前免费，适合零成本演示（注意 `GLM-5.3-Flash` 已收费，换模型前先看定价页）。本次验收用的是 `glm-4.5-flash`；`glm-4.7-flash` 高峰期会返回 429/1305 限流，换个 Flash 型号或稍后重试即可。

```bash
export DEEPSEEK_API_KEY=你的key    # 只从环境变量读，仓库里不存 key
python3 ~/ros2-vision-grasp/tools/nl_grasp.py "把红色方块抓起来"

# 换免费模型（智谱开放平台 open.bigmodel.cn，注册即可拿 key）
export NL_API_BASE=https://open.bigmodel.cn/api/paas/v4/chat/completions
export NL_MODEL=glm-4.5-flash     # 4.7 高峰期会 429，4.5 更稳
export NL_API_KEY=你的智谱key

# 先看模型打算调用哪些工具，不动机械臂（仿真没开也能跑）
python3 ~/ros2-vision-grasp/tools/nl_grasp.py --dry-run "绿色的方块在哪"
```

### Phase 4 完成标准

- [x] `bag_record.sh data` 录出的包能被 `bag_report.py` 读出报告，抬升量与在线抓取一致（实测 85.3 s / 3530 条 / red 抬升 **129 mm**、水平漂移 2.4 mm，在线同一轮为 130 mm）
- [x] `ros2 bag play --clock` 回放时 `/perception/targets_3d` 能重新 echo 到数据（在 `ROS_DOMAIN_ID=42` 下验证，与仿真隔离）
- [x] `tools/mcp_selftest.py` 自检 6/6 通过：握手、6 个工具、ping、两个只读工具、白名单拦截
- [x] `nl_grasp.py --dry-run` 能拿到模型的工具调用计划（GLM-4.5-Flash 两轮：`list_targets` → `grasp` 预演 → 中文总结）
- [x] `nl_grasp.py "把绿色方块抓起来"` 真实驱动抓取并回报抬升量（`grasp verified, cube lifted **128 mm**`，两轮收敛；同一次验收中 red 为 130 mm）

## Phase 2 完成标准（已验收 2026-10-18）

- [x] `ros2 run tf2_ros tf2_echo base_link camera_optical_frame` 能查到位姿
- [x] `/perception/targets_base` 一次输出 3 个位置，x/y 与真值同号、量级一致
- [x] `XY error vs cube center` 的 mean < 10 mm（旧 `base_link` 系实测 7.8 mm；集成后见上文 1–9 mm）
- [x] 采样位置固定为 `sample_v_ratio=0.3`，精度已达标

## 常见问题

**Gazebo 里没有图像 / 桥接无数据**
先确认 gz 侧真实话题名：

```bash
gz topic -l | grep -Ei 'camera|image|depth'
```

本 world 通过 `<topic>camera</topic>` 指定，正常情况下是 `/camera/image`、`/camera/camera_info`、`/camera/depth_image`。若名字不同，按实际名字修改 `src/arm_bringup/launch/vision.launch.py` 中 bridge 的 arguments。

**相机只输出纯色 / 检测为 0 个目标**
VMware SVGA3D 渲染退化时的特征很明确，不是黑屏：`/camera/image` 是 640×480 rgb8 但 mean≈224、min 221、max 229（只有背景色），`/camera/depth_image` 32FC1 全 `inf`；相机位姿、光照、launch 日志全都正常。用软件渲染（llvmpipe，OpenGL 4.5）绕开：

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch arm_bringup arm_gazebo.launch.py use_rviz:=false
```

启动前用 `pgrep -a gz` 确认只有一个 Gazebo 实例；两个 `gz sim` 会让 spawner 以 exit code 1 退出、话题互相干扰。

**抓取在 descend 阶段返回 `action status 6`**
`/rosout` 里 JTC 的原话是关键判据：`Aborted due to goal_time_tolerance` 表示手臂停在某个位置再也下不去（有硬阻挡），`path tolerance violation` 才是跟踪跟不上。用 `ros2 topic echo /joint_states --once` 看 effort —— 顶到 URDF 上限说明该关节被削顶，六个关节都远离位置限位却速度为 0，就是被实体顶住了。定位接触体用 `gz topic -e -t /world/vision_grasp/dynamic_pose/info`（机械臂 link 是模型局部坐标，方块是 world 坐标，`base_link` 在 world `(-0.35, 0, 0.25)`，两者要换算）。

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
| 2（已完成） | TF2 + 针孔反投影 | 基座坐标系目标位姿，集成后 XY 误差 1–9 mm |
| 3a（已完成） | ros2_control + 关节轨迹控制 | 六个关节 + 夹爪可控，动作接口验证通过 |
| 3b（已完成） | KDL 正逆解 + 抓取状态机 | 三色抓取抬升 130/128/117 mm，IK 成功率 93.3% |
| 4（已完成） | rosbag2 录制回放 + MCP Server + 大模型工具调用 | 离线报告复现 red 抬升 129 mm；中文指令实抓 green 128 mm |
| 5 | socat 虚拟串口 + AA55/CRC16 协议桥 | MCU ↔ ROS2 通信演示 |

## License

Apache-2.0
