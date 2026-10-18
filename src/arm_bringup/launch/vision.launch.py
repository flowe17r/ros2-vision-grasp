import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('arm_description')
    world_path = os.path.join(description_share, 'worlds', 'vision_grasp.sdf')

    use_rviz = LaunchConfiguration('use_rviz')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
    )

    # 方向符 '[' 表示 GZ -> ROS；'@' 前是 gz 话题，'@' 后是 ROS 消息类型
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        parameters=[{'use_sim_time': True}],
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
        output='screen',
    )

    # 相机位姿与 world 里 camera_mount 的 <pose> 保持一致
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_tf',
        arguments=[
            '--x', '0.7', '--y', '0.0', '--z', '0.9',
            '--roll', '0.0', '--pitch', '0.728', '--yaw', '3.1416',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
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
        output='screen',
    )

    backproject = Node(
        package='arm_perception',
        executable='backproject_node',
        name='backproject_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'display.launch.py')
        ),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='同时启动 RViz 预览 URDF 机械臂（模型尚未放入 Gazebo，Phase 3 再 spawn）',
        ),
        gz_sim,
        bridge,
        hsv_segment,
        box_detect,
        camera_tf,
        camera_optical_tf,
        backproject,
        display,
    ])
