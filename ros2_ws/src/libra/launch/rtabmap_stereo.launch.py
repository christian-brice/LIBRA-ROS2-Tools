# Requirements:
#   An Intel RealSense D435i or D456
#   Install the realsense2_camera ROS2 package (ros-$ROS_DISTRO-realsense2-camera)
# Usage:
#   $ ros2 launch libra rtabmap_stereo.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    # Configure nodes and DDS communications
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_stereo':True,
          'subscribe_odom_info':True,
          'wait_imu_to_init':True}]

    remappings=[
          ('imu', '/imu/data'),
          ('left/image_rect', '/camera/infra1/image_rect_raw'),
          ('left/camera_info', '/camera/infra1/camera_info'),
          ('right/image_rect', '/camera/infra2/image_rect_raw'),
          ('right/camera_info', '/camera/infra2/camera_info')]

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.'),

        # Hack to disable IR emitter
        SetParameter(name='depth_module.emitter_enabled', value=0),

        # RealSense camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
                launch_arguments={
                    'camera_namespace': '',
                    'enable_color': 'false',
                    'enable_depth': 'false',
                    'enable_infra1': 'true',
                    'enable_infra2': 'true',
                    'enable_gyro': 'true',
                    'enable_accel': 'true',
                    'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                    'enable_sync': 'true'
                }.items()),

        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node',output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame': 'enu', 
                         'publish_tf': False}],
            remappings=[('imu/data_raw', '/camera/imu')]),

        # Compute odometry using stereo images
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        # Primary RTAB-Map SLAM computation
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        # RTAB-Map's custom RViz GUI
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),
    ])
