import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
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

    robot_description = ParameterValue(
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

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
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
        ],
        output='screen',
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
        ],
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
        gz_server,
        gz_gui,
        clock_bridge,
        robot_state_publisher,
        spawn_arm,
        joint_state_broadcaster,
        arm_controller,
        gripper_controller,
        rviz,
    ])
