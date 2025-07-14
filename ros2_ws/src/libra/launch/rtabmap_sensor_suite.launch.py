# Requirements:
#   An Intel RealSense D435i or D456
#   A LightWare SF45/B
#   Install the following ROS2 packages:
#     realsense2_camera (ros-$ROS_DISTRO-realsense2-camera)
#     lightwarelidar2 (lightwarelidar2)
# Usage:
#   $ ros2 launch libra rtabmap_sensor_suite.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set paths to required files and dirs
    pkg_share = FindPackageShare(package='libra').find('libra')
    default_urdf_model_path = os.path.join(
        pkg_share, 'models/sensor_suite.urdf.xacro')

    # Configure nodes and DDS communications
    parameters=[{
          # General params
          'frame_id': 'base_link',
          # ROS params
          'subscribe_depth': False,  # defaults to True
          'subscribe_rgbd': True,
          'subscribe_scan': True,
          'subscribe_odom_info': True,
          'wait_imu_to_init': True,  # initializes odom to be aligned with gravity
          # RTAB-Map params
          'RGBD/NeighborLinkRefining': True,  # correct odom with laser scans
          'RGBD/ProximityBySpace': True,  # find local loop closures based on robot position
          'RGBD/AngularUpdate': 0.01,  # only robot movement updates the map
          'RGBD/LinearUpdate': 0.01,  # only robot movement updates the map
          'RGBD/OptimizeFromGraphEnd': False,  # correct all previous poses based on last pose
          'Grid/FromDepth': False,  # True: depth, False: laser scans
          'Reg/Force3DoF': True,  # roll, pitch, and Z must be explicitly provided
          'Reg/Strategy': 1,  # 0: visual only, 1: ICP, 2: visual->ICP
          # ICP params
          'Icp/VoxelSize': 0.05,  # filter scans to 5 cm voxel before registration
          'Icp/MaxCorrespondenceDistance': 0.1}]  # max distance between points during registration

    remappings=[
          # RealSense
          ('imu', '/imu/data'),
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
          ('left/image_rect', '/camera/infra1/image_rect_raw'),
          ('left/camera_info', '/camera/infra1/camera_info'),
          ('right/image_rect', '/camera/infra2/image_rect_raw'),
          ('right/camera_info', '/camera/infra2/camera_info'),
          # 2D LIDAR
          ('scan', '/base_scan'),
          # RTAB-Map
          ('rgbd_image', 'rgbd_image')]

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'urdf_model', default_value=default_urdf_model_path,
            description='Full path to robot URDF file.'),

        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='0: none, 1: copy, 2: linear interpolation. Use unite_imu_method:="1" if IMU topics stop being published.'),

        # Periodically publish robot frame transforms based on URDF model
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{# Note: If ANY text in the xacro output can be interpreted as yaml,
                         # the roslaunch system will try to interpret the ENTIRE text as yaml
                         # instead of passing on the string. The biggest cause of this false
                         # interpretation is commenting out xacro calls since the xacro:property
                         # or similar looks a lot like a yaml key:value pair. Comments are not
                         # removed by xacro so they are included in the output.
                         # (source: https://answers.ros.org/question/417369/caught-exception-in-launch-see-debug-for-traceback-unable-to-parse-the-value-of-parameter-robot_description-as-yaml/)
                         'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('urdf_model')]), value_type=str)}],
            arguments=[default_urdf_model_path]),

        # Hack to disable IR emitter
        SetParameter(name='depth_module.emitter_enabled', value=0),

        # RealSense camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
                launch_arguments={'camera_namespace': '',
                                  'enable_gyro': 'true',
                                  'enable_accel': 'true',
                                  'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                                  'enable_infra1': 'true',
                                  'enable_infra2': 'true',
                                  'enable_sync': 'true',
                                  'align_depth': 'true'}.items()),

        # Compute quaternion of the camera IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame': 'enu', 
                         'publish_tf': False}],
            remappings=[('imu/data_raw', '/camera/imu')]),

        # LightWare 2D LIDAR
        Node(
            package='lightwarelidar2', executable='sf45b', output='screen',
            parameters=[{'port': '/dev/lidar',
                         'frameId': 'lidar_link',
                         'updateRate': 12,
                         'lowAngleLimit': -160,
                         'highAngleLimit': 160}],
            remappings=remappings),

        # Synchronize RGB, depth, and camera_info messages
        # (useful when subscribing to multiple sensors which output at different rates)
        Node(
            package='rtabmap_sync', executable='stereo_sync', output='screen',
            parameters=[{'approx_sync': False}],  # auto-synced by RealSense
            remappings=remappings),

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
