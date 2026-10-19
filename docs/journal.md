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

### Phase 3b 实现（2026-10-18，待验收）

- URDF：新增 `tcp` 固定关节（`gripper_base` 下方 35 mm），KDL 链改为 `base_link → tcp`；`joint5` 限位由 ±2.0 扩到 ±3.1，否则夹爪无法朝下；夹爪指原点改为 ±26 mm（闭合间隙 40 mm，张开 15 mm 时 70 mm），可夹 50 mm 方块；指垫加 `mu=5.0`。
- world：三个方块由 static 改为动态刚体（50 g、摩擦 1.0）；新增 `PosePublisher`，把 `/world/vision_grasp/pose/info` 桥接到 `tf2_msgs/TFMessage`，抓取后用它校验方块确实被抬起。
- 接口：新增 `Target3D`/`Target3DArray` 消息与 `SolveIK`/`SolveFK`/`Grasp` 服务；反投影节点同时发布 `/perception/targets_3d`（带颜色名，抓取节点用）。
- `arm_kinematics`（C++）：`kdl_parser` 建链、`ChainIkSolverPos_LMA` 求解；IK 依次尝试请求种子、当前关节角、零位和 8 组随机种子，结果必须满足关节限位、位置误差 <5 mm、姿态误差 <0.1 rad。
- `arm_grasp`（Python）：状态机「开爪 → 预抓取 z=0.16 → 下降 z=0.032 → 闭合 → 抬升 z=0.18」，全程走 `FollowJointTrajectory`；夹爪控制器为指关节加了 20 mm goal 容差，避免夹到方块被挡后动作被判失败。
- launch：`arm_gazebo.launch.py` 整合相机桥接、`base_link` 新相机 TF `(1.05, 0, 0.65)`、感知、反投影、KDL 与抓取节点；默认 `use_gz_gui=false`、`use_rviz=true`。
- 反投影真值换算到新 `base_link`：red (0.23, 0.10)、green (0.35, -0.10)、blue (0.47, 0.08)，z 真值 0.025。
- 待验收：编译、三色定位误差复测、IK 成功率采样（`tools/ik_benchmark.py`）、夹爪开合、红/绿/蓝抓取与抬升校验。

### 下一步

- 同步到虚拟机，安装 `ros-jazzy-kdl-parser`/`ros-jazzy-orocos-kdl`/`libeigen3-dev` 后编译；按 README Phase 3b 的四条命令逐项验收，实测数据回填 README 与简历。

## 2026-10-19 Phase 3b 验收：三个连环坑与最终通过

**结果**：同一次干净启动内连续抓取，**red 130 mm / green 128 mm / blue 117 mm**，全部 > 30 mm 门槛；IK 成功率 **28/30 = 93.3%**；零位正解 TCP z = **0.785 m**。

### 改动记录（供交接）

1. **`grasp_node._solve()` 的目标姿态翻转 180°**（根因一）
   - 原写法用绕 x 轴 180° 的四元数 `(cos(yaw/2), sin(yaw/2), 0, 0)`，把夹爪倒过来、手指朝上、腕部朝下，下探时是腕部先压方块顶。
   - 改为单位姿态 + 绕 z 的 yaw：`(0, 0, sin(yaw/2), cos(yaw/2))`。夹爪沿 TCP 的 −z 抓取，TCP 的 +z 必须朝上。
2. **`gripper_mount` 偏置 `z = +0.03` → `-0.03`**（根因二，`arm_description/urdf/arm.urdf.xacro`）
   - 姿态修正后 TCP 的 +z 朝上，`joint5`/`joint6` 就得反向折约 127°，`link5` 腕筒（r=32 mm、长 50 mm）被折到两指正中间、深到与指尖同高。指间净空 70 mm 装不下腕筒 64 mm + 方块 50 mm。
   - 夹爪整体下移 60 mm，TCP 从 `tool0` 下方 5 mm 变到下方 65 mm，腕部才能留在物体上方。零位 TCP 由 0.845 变为 **0.785**，运动学链整体变长，旧的 IK 成功率与正解数字全部作废。
3. **`joint3` effort 30 → 100**（`arm.urdf.xacro`，`joint2` 先前已同样提到 100）
   - gz_ros2_control 的位置控制在远端目标处索取的力矩远大于重力矩：blue 停点实测 joint2 79–93 N·m、joint3 37.8 N·m，而按实测位姿算的重力矩只有 joint2 ≈10.7、joint3 ≈2.1 N·m。30 N·m 会被削顶。
4. **`grasp_height` 0.032 → 0.050，回退档位改为只往上加**（根因三，`arm_grasp/grasp_node.py`）
   - 腕筒最低点 ≈ TCP + 10 mm。TCP=0.032 时腕筒最低 0.042，插进 50 mm 方块顶面（0.050）8 mm，descend 卡死在最后 3°（joint2 差 0.056 rad，其余五关节精确到位），JTC 报 `Aborted due to goal_time_tolerance exceeding by 2.002 seconds`。
   - 回退序列原为 `(h, h+0.01, h−0.007)`，`h−0.007` 会退到干涉区，改为 `(h, h+0.015, h+0.03)`。
   - red/green 之所以在 0.032 也能过：它们的 IK 在 0.032 无解、自动退到了 0.042 那一档；blue 的 IK 在 0.032 有解（`pos_error=2.9e-6`），于是真的走下去撞上。
5. **`tools/ik_benchmark.py` 采样姿态与方块高度过期**
   - `topdown_pose` 用的是 `orientation.x = 1.0, w = 0.0`（绕 x 翻 180°），即根因一的错误姿态；旧的 **96.7% 是在错误姿态下测的**，与抓取节点实际用的姿态不是一回事。改为单位姿态并更名 `grasp_pose`，方块采样高度 0.032 → 0.050。
6. **方块真值改走 per-model pose 话题，取代上文 Phase 3b 实现里的 `pose/info` 方案**
   - `PosePublisher` 插件挂到三个方块 model 内部，桥接 `/model/cube_{red,green,blue}/pose` → `geometry_msgs/Pose`；`/world/vision_grasp/pose/info` 经桥接会丢失 frame 名、且混着机械臂 link，抓取节点判抬升时不好用。`arm_grasp/package.xml` 随之去掉 `tf2_msgs` 依赖。
   - `kinematics_node.cpp` 的 IK 调用改为 `ik_solver_->CartToJnt(candidate, target, q_out)`（原先误写成 `JntToCart`，参数顺序也不对）。
   - `arm_gazebo.launch.py`：三个 controller spawner 改由 `OnProcessExit(spawn_arm)` 触发，并加 `--controller-manager-timeout 120 --switch-timeout 60`。软件渲染下 `controller_manager` 起得慢，spawner 与建臂并发时会只加载到 `gripper_controller` 就退出（表现为 `ros2 control list_controllers` 只有一个 active），串行 + 放宽超时后三个都 active。

### 排查过程中被数据推翻的假设

- 「跟踪增益不够」：`ros2 param set /gz_ros_control position_proportional_gain 0.5` 返回成功但停点完全不变，参数未动态生效；且 joint4/5/6 都能精确到位，说明不是全局跟踪问题。
- 「joint2 力矩不够」：effort 30 → 100 后 red 仍以完全相同的停点失败（q2≈1.203），瓶颈不在力矩。
- 「joint3 力矩不够」：提到 100 后 blue 仍失败，但停点从 joint3=0.9756 变到 0.7887、effort 达到 37.8（> 旧上限 30），证明新限位确实生效、旧值确实被削顶，所以保留 100；它不是 blue 失败的根因。
- 「方块被推动导致偏差」：`/model/cube_red/pose` 始终为 world (-0.1205, 0.0996, 0.2752)，方块没动；机械臂进入视野后 red 的 x 从 0.239 漂到 0.254 是**遮挡下的 HSV 分割偏差**。
- 决定性 A/B：把 `cube_red` 用 `gz service -s /world/vision_grasp/set_pose` 挪到 y=0.30，再发**完全相同**的六关节目标 → `error_code: 0 / SUCCEEDED`。方块一移开就成功，确认是接触而非控制。

### 踩坑记录

1. **`ros2 action send_goal` 的 YAML 流式映射**：`{sec:6}` 会被解析成字符串 `"sec:6"`，报 `AttributeError("'Duration' object has no attribute 'sec:6'")`；必须写 `{sec: 6}`（冒号后要空格）。
2. **JTC 拒绝只列部分关节的轨迹**：发 3 个关节直接 `Goal was rejected.`，必须列全 `arm_controller` 的 6 个关节（或开 `allow_partial_joints_goal`）。
3. **`/grasp/execute` 是 service 不是 action**：`ros2 action send_goal ... arm_interfaces/action/GraspExecute` 报 `The passed action type is invalid`；正确调用是 `ros2 service call /grasp/execute arm_interfaces/srv/Grasp '{color: "red"}'`。
4. **URDF 改动不会热生效**：`robot_state_publisher`、Gazebo 物理限位、`gz_ros2_control` 都只在启动时读一次 URDF。`--symlink-install` 下 `install/share` 是指向 `src` 的符号链接，改完源码不用重编译，但**必须重启 launch**。生效判据：`ros2 param get /robot_state_publisher robot_description | grep -c 'effort="100.0"'` 应返回 2。
5. **`gz topic -e -t /world/<w>/dynamic_pose/info` 里两套坐标系混在一起**：机械臂的 link 是模型局部坐标（`base_link` 为原点），独立模型（方块）是 world 坐标；`base_link` 在 world `(-0.35, 0, 0.25)`。link 名带前缀，grep 要写 `name: ".*(link4|link5|tool0)"`，直接 `name: "(link4|..."` 匹配不到。
6. **`tf2_echo` 的 timeout 太短会误报帧不存在**：`timeout 4 ros2 run tf2_ros tf2_echo base_link tcp` 报 `Invalid frame ID "base_link" ... frame does not exist`，实际是 TF 缓冲还没填满；改用 `/kinematics/solve_fk` 更可靠。

### 下一步

- README 与简历里的数字要用本轮实测值：定位误差 1–9 mm（新 base 系）、IK 93.3%、抬升 130/128/117 mm、零位 TCP 0.785 m。**96.7% 与 7.8 mm 都不能再用**（前者姿态错误，后者是旧 base 系）。
- Phase 4：rosbag2 录制回放、MCP Server + DeepSeek 自然语言驱动抓取。
- Windows 仓库与 VM `~/ros2_ws/src` 目前一致（VM 侧用单行 sed 打的补丁），但 VM 的 `~/ros2-vision-grasp` 是旧副本，`grasp_node.py` 仍是翻转姿态那版，别再从那里往 `~/ros2_ws/src` 覆盖。



---

<!-- 后续按日期追加：做了什么 / 遇到什么错误 / 根因 / 怎么验证 -->
