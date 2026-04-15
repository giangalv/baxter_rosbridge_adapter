import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    baxter_description_dir = get_package_share_directory('baxter_description')
    xacro_file = os.path.join(baxter_description_dir, 'urdf', 'baxter.urdf.xacro')

    rviz_config_file = os.path.join(
        get_package_share_directory('baxter_rosbridge_adapter'),
        'rviz',
        'baxter.rviz'
    )

    baxter_host = LaunchConfiguration('baxter_host')
    baxter_port = LaunchConfiguration('baxter_port')

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        DeclareLaunchArgument('baxter_host', default_value='130.251.13.31'),
        DeclareLaunchArgument('baxter_port', default_value='9090'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base',
            output='screen',
            arguments=['0', '0', '0.95', '0', '0', '0', 'world', 'base'],
        ),

        Node(
            package='baxter_rosbridge_adapter',
            executable='joint_state_bridge',
            name='joint_state_bridge',
            output='screen',
            parameters=[
                {'baxter_host': baxter_host},
                {'baxter_port': baxter_port},
            ],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
    ])
