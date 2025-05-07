# Requirements:
#   A LightWare SF45/B
#   Install the lightwarelidar2 ROS2 package (lightwarelidar2)
# Usage:
#   $ ros2 launch libra test_lidar.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set paths to required files and dirs
    pkg_share = FindPackageShare('libra').find('libra')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'lidar.rviz')

    # Initialize launch configs
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('rviz_config_file', default_value=default_rviz_config_path),
        DeclareLaunchArgument('use_rviz', default_value='True'),

        # LightWare 2D LIDAR
        Node(
            package='lightwarelidar2',
            executable='sf45b',
            name='sf45b',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'frameId': 'lidar_link',
                'updateRate': 12,
                'lowAngleLimit': -160,
                'highAngleLimit': 160
            }]
        ),

        # RViz
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
