import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('arm_description')
    world_path = os.path.join(description_share, 'worlds', 'vision_grasp.sdf')
    xacro_path = os.path.join(description_share, 'urdf', 'arm.urdf.xacro')
    controllers_path = os.path.join(description_share, 'config', 'controllers.yaml')
    rviz_config = os.path.join(description_share, 'rviz', 'arm.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_gz_gui = LaunchConfiguration('use_gz_gui')
    use_vision = LaunchConfiguration('use_vision')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_path,
            ' controllers_file:=', controllers_path,
        ]),
        value_type=str,
    )
    # 同一份 URDF 供 KDL 解析，单独构造避免两个节点共享同一个 substitution
    kinematics_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_path,
            ' controllers_file:=', controllers_path,
        ]),
        value_type=str,
    )

    # 虚拟机里 Gazebo GUI 的 3D 视图一直空白且加重 VMware 3D 负载，默认只跑 server
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_path],
        output='screen',
        condition=UnlessCondition(use_gz_gui),
    )

    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
        condition=IfCondition(use_gz_gui),
    )

    # 方向符 '[' 表示 GZ -> ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/cube_red/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
            '/model/cube_green/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
            '/model/cube_blue/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # 机械臂底座放在台面 (-0.35, 0, 0.25)，避开桌体碰撞且不挡相机视线
    spawn_arm = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'vg_arm',
            '-x', '-0.35', '-y', '0.0', '-z', '0.25',
        ],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
            '--switch-timeout', '60',
        ],
        output='screen',
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
            '--switch-timeout', '60',
        ],
        output='screen',
    )

    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
            '--switch-timeout', '60',
        ],
        output='screen',
    )

    # 相机在 world 里位于 (0.7, 0, 0.9)，base_link 在 (-0.35, 0, 0.25)
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_tf',
        arguments=[
            '--x', '1.05', '--y', '0.0', '--z', '0.65',
            '--roll', '0.0', '--pitch', '0.728', '--yaw', '3.1416',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
        condition=IfCondition(use_vision),
        output='screen',
    )

    # Gazebo 相机光轴是 +X；光学坐标系遵循 REP-105（z 前、x 右、y 下）
    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '-1.5708', '--pitch', '0.0', '--yaw', '-1.5708',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_optical_frame',
        ],
        condition=IfCondition(use_vision),
        output='screen',
    )

    hsv_segment = Node(
        package='arm_perception',
        executable='hsv_segment_node',
        name='hsv_segment_node',
        parameters=[{
            'image_topic': '/camera/image',
            'use_sim_time': True,
        }],
        condition=IfCondition(use_vision),
        output='screen',
    )

    box_detect = Node(
        package='arm_perception',
        executable='box_detect_node',
        name='box_detect_node',
        parameters=[{
            'image_topic': '/camera/image',
            'use_sim_time': True,
        }],
        condition=IfCondition(use_vision),
        output='screen',
    )

    backproject = Node(
        package='arm_perception',
        executable='backproject_node',
        name='backproject_node',
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'base_link',
            'camera_frame': 'camera_optical_frame',
        }],
        condition=IfCondition(use_vision),
        output='screen',
    )

    kinematics = Node(
        package='arm_kinematics',
        executable='kinematics_node',
        name='kinematics_node',
        parameters=[{
            'robot_description': kinematics_description,
            'base_frame': 'base_link',
            'tip_frame': 'tcp',
            'use_sim_time': True,
        }],
        condition=IfCondition(use_vision),
        output='screen',
    )

    grasp = Node(
        package='arm_grasp',
        executable='grasp_node',
        name='grasp_node',
        parameters=[{
            'use_sim_time': True,
        }],
        condition=IfCondition(use_vision),
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    # 软件渲染下 controller_manager 初始化慢，等机械臂生成完成后再拉起控制器
    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_arm,
            on_exit=[
                joint_state_broadcaster,
                arm_controller,
                gripper_controller,
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否启动 RViz 预览机械臂与关节状态',
        ),
        DeclareLaunchArgument(
            'use_gz_gui',
            default_value='false',
            description='是否打开 Gazebo GUI（默认关闭，降低 VMware 3D 负载）',
        ),
        DeclareLaunchArgument(
            'use_vision',
            default_value='true',
            description='是否启动相机桥接、感知、反投影、KDL 与抓取节点',
        ),
        gz_server,
        gz_gui,
        bridge,
        robot_state_publisher,
        spawn_arm,
        start_controllers,
        camera_tf,
        camera_optical_tf,
        hsv_segment,
        box_detect,
        backproject,
        kinematics,
        grasp,
        rviz,
    ])
