# 开发日志

## 2026-10-17 环境搭建与 Phase 1 骨架

**目标**：在 VMware 的 Ubuntu 24.04.04 上装好 ROS 2 Jazzy + Gazebo Harmonic，把视觉感知跑起来。

### 踩坑记录

1. **ROS 2 安装不完整**
   - 现象：`/opt/ros/jazzy` 存在，但只有 133 个包，`ros2` 命令不存在，`ROS_DISTRO` 为空。
   - 原因：`apt install` 中途失败留下半配置状态。
   - 处理：`sudo dpkg --configure -a` → `sudo apt --fix-broken install -y` → `sudo apt install -y ros-jazzy-desktop ros-jazzy-ros2cli`。
   - 教训：验证安装看 `ros2 pkg list | wc -l`（desktop 应上千），不要只看目录是否存在。

2. **heredoc 粘贴破坏 `.bashrc`**
   - 现象：终端启动报 `ros2：未找到命令`、`EOF：未找到命令`。
   - 原因：交互式终端里粘贴多行 heredoc，部分行被截断/提前执行。
   - 处理：`grep -v` 过滤掉残留行重建 `.bashrc`，再用单行 `printf` 追加 `source` 与 `ROS_DOMAIN_ID`。
   - 教训：在这个终端里配置一律用 `nano` 或单行命令，不用多行粘贴。

3. **`ros2 pkg create` 参数顺序**
   - 错误写法：`ros2 pkg create --build-type ament_python --dependencies rclpy smoke_test`
   - 现象：报 `the following arguments are required: package_name`。
   - 原因：`--dependencies` 是 `nargs='+'`，把后面的 `smoke_test` 当成了依赖名。
   - 正确：`ros2 pkg create smoke_test --build-type ament_python --license Apache-2.0 --dependencies rclpy`（包名紧跟子命令）。

### 进展

- 确定版本路线：Ubuntu 24.04 + Jazzy（支持到 2029-05），仿真用 Gazebo Harmonic，不用已停止支持的 Gazebo Classic。
- 自建 6 轴 URDF + 两指夹爪，不依赖厂商机械臂包。
- 生成 4 个包：`arm_interfaces`、`arm_description`、`arm_perception`、`arm_bringup`。
- world 中三个方块的坐标即 Phase 2 的 ground truth：
  red (-0.12, 0.10, 0.275)、green (0.00, -0.10, 0.275)、blue (0.12, 0.08, 0.275)。

### Phase 1 验收（2026-10-17）

- Gazebo 里相机正常出图（工作台 + 红/绿/蓝三个方块），ros_gz_bridge 将 `/camera/image`、`/camera/camera_info`、`/camera/depth_image` 导入 ROS。
- `/perception/detections` 一帧输出 3 个目标：

  | 颜色 | u (px) | v (px) | area (px²) | bbox |
  | --- | --- | --- | --- | --- |
  | red | 373.7 | 196.5 | 1034.5 | 31 × 38 |
  | green | 260.1 | 240.0 | 1234.0 | 34 × 41 |
  | blue | 372.2 | 292.5 | 1522.0 | 36 × 46 |

- 单帧耗时（终端日志实测）：
  - `hsv_segment_node`：**30.54 ms/frame**（HSV + 形态学 + 掩膜/调试图发布）
  - `box_detect_node`：**26.44 ms/frame**（三色检测 + 轮廓 + 标注图，含 3 个目标）
- 简历中的「单帧处理耗时」填 **26 ms**（对应 HSV + 轮廓检测那条链路）。
- 已知现象：
  - Gazebo GUI 的 3D 视图和 Entity Tree 空白，但传感器出图、检测都正常；暂不处理。
  - VM 里 RTF 约 28%，仿真 30 Hz 对应实时约 8–9 Hz，`ros2 topic hz` 低于 30 属正常。
  - 检测消息的 `frame_id` 是 `camera_mount/camera_link/camera`，Phase 2 需要自建 TF 帧。

### Phase 2 实现（2026-10-17）

- `Detection2D` 增加 `bbox_x/bbox_y`，反投影采样点改用检测框**顶面中心**（0.5W, 0.3H），避开轮廓质心被方块侧面拉低的问题。
- 新增 `backproject_node`：缓存 `camera_info` 与最新深度图，检查时间戳差；用针孔模型反投影到光学坐标系，再用 TF2 变换到 `base_link`，发布 `/perception/targets_base`（PoseArray）。
- launch 增加 `base_link → camera_link → camera_optical_frame` 静态 TF，位姿与 world 里的 `camera_mount` 一致。
- 精度评估：与三色方块中心真值比较 XY，累计输出 mean / max；`sample_v_ratio` 可通过 `ros2 param set` 在线调。

### Phase 2 验收（2026-10-18）

- `/perception/targets_base` 实测（base_link 原点与 world 重合时的坐标）：

  | 颜色 | x (m) | y (m) | z (m) | 与真值 XY 误差 |
  | --- | --- | --- | --- | --- |
  | red | -0.1108 | 0.0987 | 0.3001 | 9.3 mm |
  | green | 0.0080 | -0.0991 | 0.3002 | 8.1 mm |
  | blue | 0.1259 | 0.0791 | 0.3008 | 6.0 mm |

- 平均 **7.8 mm**，最大 9.3 mm，满足 <10 mm 目标；z≈0.300 对应 5 cm 方块顶面（0.25+0.05），说明深度单位与采样点正确。
- 残余误差以 +x 方向为主（6–9 mm），推测采样点略低于顶面中心，可把 `sample_v_ratio` 调到 0.25 微调；当前精度已达标，不再调。
- 简历已写「三色目标定位误差平均约 8 mm」，无头 Chrome 复验仍为**一页 A4、2 张图**。

### Phase 3a 实现（2026-10-18）

- 机械臂改为**放在台面上**：Gazebo 中 `base_link` 位于 world `(-0.35, 0, 0.25)`，避开桌体碰撞，同时不遮挡相机看方块的视线。
- 机身的蓝色涂装改为橙棕色，避免被 HSV 蓝色区间误检为 blue 方块。
- URDF 增加 `<ros2_control>`（6 个转动关节 + 2 个平移手指，position 命令、position/velocity/effort 状态）与 `gz_ros2_control` 插件。
- 新增 `arm_description/config/controllers.yaml`：`joint_state_broadcaster`、`arm_controller`（joint1–6）、`gripper_controller`（两指），均为 `JointTrajectoryController`。
- 新增 `arm_bringup/launch/arm_gazebo.launch.py`：启动 world、`/clock` 桥接、robot_state_publisher、`ros_gz_sim create` 生成机械臂、三个 controller spawner，可选 RViz 预览。
- 注意：机械臂进入仿真后 `base_link` 不再与 world 原点重合，Phase 2 的相机静态 TF 与真值坐标需要换算到新的 base_link（相机变为 `(1.05, 0, 0.65)`），集成视觉时重跑一次精度。

### Phase 3a 验收（2026-10-18）

- `arm_gazebo.launch.py` 启动成功，Gazebo 返回 `Entity creation successful`。
- `gz_ros2_control` 从 `/robot_description` 读取 URDF，8 个关节全部加载；`Fixed joint ['world_fixed'] is skipped` 属正常（固定底座不参与控制）。
- 三个控制器均 active：`joint_state_broadcaster`、`arm_controller`、`gripper_controller`。
- FollowJointTrajectory 测试 `[0, 0.4, -0.6, 0, 0.4, 0]`：`Goal accepted`，`error_code: 0`，`SUCCEEDED`，说明轨迹接口与位置控制链路正常。
- Gazebo GUI 的 3D 视图依旧空白（旧的渲染问题）；RViz 的 RobotModel 正常显示，后续用 RViz 观察机械臂。
- 注意：gz_ros2_control 日志提示 `Enforcing command limits is disabled`，URDF 关节限位不会由仿真侧检查，KDL 逆解必须自己校验限位。
- 夹爪开合动作待验证。

### 2026-10-18 VMware 3D 渲染崩溃

- 夹爪测试前 VMware 报 `ISBRendererComm: Lost connection to mksSandbox`，虚拟机进入不可恢复错误。根因是 VMware 的 3D 渲染进程崩溃，不是 ROS 节点或项目代码问题；已编译的包和源码在虚拟机磁盘中，不会因此丢失。
- 规避：`arm_gazebo.launch.py` 默认改为 `gz sim -s -r`（只跑 Gazebo server，不打开一直空白的 Gazebo GUI），RViz 仍保留；需要 GUI 时加 `use_gz_gui:=true`。
- 若仍复现，再考虑升级 VMware Workstation、关闭 3D 加速或改用软件渲染。

### 下一步

- 测 `gripper_controller` 开合；开始 Phase 3b：加 TCP 坐标系、KDL 正逆解服务（C++）与抓取状态机。



---

<!-- 后续按日期追加：做了什么 / 遇到什么错误 / 根因 / 怎么验证 -->
