# Requirements:
#   An Intel RealSense D435i or D456
#   Install the realsense2_camera ROS2 package (ros-$ROS_DISTRO-realsense2-camera)
# Usage:
#   $ ros2 launch libra test_realsense.launch.py

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
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'realsense.rviz')

    # Initialize launch configs
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('rviz_config_file', default_value=default_rviz_config_path),
        DeclareLaunchArgument('use_rviz', default_value='True'),

        # RealSense camera
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            output='screen',
            parameters=[{
                'pointcloud.enable': True,
                'enable_gyro': True,
                'enable_accel': True,
                'unite_imu_method': 2,
                'depth_module.depth_profile': '848x480x60',
                'depth_module.infra_profile': '848x480x60',
                'rgb_camera.color_profile': '848x480x60',
                'align_depth.enable': True
            }]
        ),

        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False
            }],
            remappings=[('imu/data_raw', '/camera/imu')]
        ),

        # RViz GUI
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
