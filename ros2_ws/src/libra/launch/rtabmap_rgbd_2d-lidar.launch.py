# Brief:
#   Runs RTAB-Map in RGB-D mode with RealSense and 2D LIDAR data as input.
#   (no robot odometry; visual-inertial odometry is used instead)
# Requirements:
#   An Intel RealSense D435i or D456
#   A LightWare SF45/B
#   Install the following ROS2 packages:
#     realsense2_camera (ros-$ROS_DISTRO-realsense2-camera)
#     lightwarelidar2 (lightwarelidar2)
# Usage:
#   $ ros2 launch libra rtabmap_rgbd_2d-lidar.launch.py
# See:
#   https://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot#Kinect_.2B-_Odometry_.2B-_2D_laser

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set paths to required files and dirs
    pkg_share = FindPackageShare(package='libra').find('libra')
    config_path = os.path.join(pkg_share, 'config')
    default_urdf_model_path = os.path.join(
        pkg_share, 'models/sensor_suite.urdf.xacro')
    
    # Configure nodes and DDS communications
    rtab_params = [{
        'frame_id': 'base_link',
        'sync_queue_size': 30,  # default: 10
        'topic_queue_size': 30,  # default: 10

        # Input topics
        'subscribe_rgb': True,
        'subscribe_depth': True,
        'subscribe_rgbd': False,  # let RTAB-Map take care of combining them
        'subscribe_scan_cloud': True,
        'subscribe_odom_info': True,
        'wait_imu_to_init': True,

        # Mapping config
        'RGBD/NeighborLinkRefining': 'true',  # correct odom with laser scans
        'RGBD/ProximityBySpace': 'true',  # find local loop closures based on robot position
        'RGBD/AngularUpdate': '0.01',  # only robot movement updates the map
        'RGBD/LinearUpdate': '0.01',  # only robot movement updates the map
        'RGBD/OptimizeFromGraphEnd': 'false',  # correct all previous poses based on last pose
        'Grid/FromDepth': 'false',  # true: depth, false: laser scans

        # Odometry and registration
        'Reg/Force3DoF': 'true',  # roll, pitch, and Z must be explicitly provided
        #'Reg/Strategy': "2",  # 0: visual only, 1: ICP, 2: visual->ICP
        'Icp/VoxelSize': '0.05',  # filter scans to 5 cm voxel before registration
        'Icp/MaxCorrespondenceDistance': '0.1'  # max distance between points during registration
    }]

    rtab_remaps=[
        # Inputs: realsense2_camera
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        # Inputs: imu_filter_madgwick_node
        ('imu', '/imu/data'),
        # Inputs: sf45b
        ('scan_cloud', '/pointcloud')
    ]

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'realsense_config',
            default_value=os.path.join(config_path, 'realsense_rgbd.yaml'),
            description='Path to YAML file with RealSense camera parameters.'
        ),

        DeclareLaunchArgument(
            'urdf_model',
            default_value=default_urdf_model_path,
            description='Path to URDF file with robot model definition.'
        ),

        # Publish robot frame transforms based on URDF model
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                # Note: If ANY text in the xacro output can be interpreted as yaml,
                # the roslaunch system will try to interpret the ENTIRE text as yaml
                # instead of passing on the string. The biggest cause of this false
                # interpretation is commenting out xacro calls since the xacro:property
                # or similar looks a lot like a yaml key:value pair. Comments are not
                # removed by xacro so they are included in the output.
                # (source: https://answers.ros.org/question/417369/caught-exception-in-launch-see-debug-for-traceback-unable-to-parse-the-value-of-parameter-robot_description-as-yaml/)
                'robot_description': ParameterValue(
                    Command(['xacro ', LaunchConfiguration('urdf_model')]),
                    value_type=str
                )
            }],
            arguments=[default_urdf_model_path]),
        
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

        # LightWare 2D LIDAR
        Node(
            package='lightwarelidar2', executable='sf45b', output='screen',
            parameters=[{
                'port': '/dev/lidar',
                'frameId': 'lidar_link',
                'updateRate': 12,
                'lowAngleLimit': -160,
                'highAngleLimit': 160
            }]
        ),

        # Compute odometry using RGB-D images
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=rtab_params + [{
                # Without robot odometry, only visual odometry is supported
                'Reg/Strategy': "0"
            }],
            remappings=rtab_remaps
        ),

        # Primary RTAB-Map SLAM computation
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=rtab_params + [{
                # Use laser scans to inform VSLAM
                'Reg/Strategy': "2"
            }],
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
