# Brief:
#   Runs RTAB-Map in stereo mode with only RealSense data as input.
#   (no robot odometry; visual-inertial odometry is used instead)
# Requirements:
#   An Intel RealSense D435i or D456
#   Install the realsense2_camera ROS2 package (ros-$ROS_DISTRO-realsense2-camera)
# Usage:
#   $ ros2 launch libra rtabmap_stereo.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set paths to required files and dirs
    pkg_share = FindPackageShare('libra').find('libra')
    config_path = os.path.join(pkg_share, 'config')

    # Configure nodes and DDS communications
    rtab_params=[{
        'frame_id': 'camera_link',
        'subscribe_stereo': True,
        'subscribe_odom_info': True,
        'wait_imu_to_init': True
    }]

    rtab_remaps=[
        # Inputs: realsense2_camera
        ('left/image_rect', '/camera/infra1/image_rect_raw'),
        ('left/camera_info', '/camera/infra1/camera_info'),
        ('right/image_rect', '/camera/infra2/image_rect_raw'),
        ('right/camera_info', '/camera/infra2/camera_info'),
        # Inputs: imu_filter_madgwick_node
        ('imu', '/imu/data')
    ]

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'realsense_config',
            default_value=os.path.join(config_path, 'realsense_stereo.yaml'),
            description='Path to YAML file with RealSense camera parameters.'
        ),

        # RealSense camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
            launch_arguments={
                'camera_namespace': '',  # avoid nesting "/camera/camera"
                'config_file': LaunchConfiguration('realsense_config')
            }.items()
        ),

        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False
            }],
            remappings=[
                # Inputs: realsense2_camera
                ('imu/data_raw', '/camera/imu')
            ]
        ),

        # Compute odometry using stereo images
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=rtab_params,
            remappings=rtab_remaps
        ),

        # Primary RTAB-Map SLAM computation
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=rtab_params,
            remappings=rtab_remaps,
            arguments=[]
        ),

        # RTAB-Map's custom RViz GUI
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=rtab_params,
            remappings=rtab_remaps
        ),
    ])
