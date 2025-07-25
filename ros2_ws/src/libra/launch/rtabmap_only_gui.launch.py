# Brief:
#   Runs RTAB-Map GUI by itself. Used mainly to pick up where you left off after a crash.
#
# Requirements:
#   (none)
#
# Usage:
#   $ ros2 launch libra rtabmap_only_gui.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Copy the same params and remaps from the other launch files, just in case
    rtab_params=[{
        'frame_id': 'camera_link',
        'subscribe_rgb': True,
        'subscribe_depth': True,
        'subscribe_rgbd': False,  # let RTAB-Map take care of combining them
        'subscribe_odom_info': True,
        'wait_imu_to_init': True
    }]

    rtab_remaps=[
        # Inputs: realsense2_camera
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        # Inputs: imu_filter_madgwick_node
        ('imu', '/imu/data')
    ]

    return LaunchDescription([
        # RTAB-Map's custom RViz GUI
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=rtab_params,
            remappings=rtab_remaps
        ),
    ])
