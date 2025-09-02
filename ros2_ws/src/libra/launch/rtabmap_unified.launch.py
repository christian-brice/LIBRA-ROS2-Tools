# Brief:
#   Unified RTAB-Map launch file supporting multiple modes and odometry sources:
#   - Modes: rgbd_lidar, rgbd, stereo
#   - Odometry: robot (actuator data), vio (visual-inertial data from RealSense)
#
# Requirements:
#   An Intel RealSense D435i or D456
#   A LightWare SF45/B (for rgbd_lidar mode)
#   HEBI actuator states published on /joint_states (for robot odometry)
#   Install required ROS2 packages:
#     realsense2_camera (ros-$ROS_DISTRO-realsense2-camera)
#     lightwarelidar2 (lightwarelidar2)
#
# Usage:
#   Use defaults:
#     $ ros2 launch libra rtabmap_unified.launch.py
#   RGB+D-only mode with all RealSense streams enabled and unique working directory:
#     $ ros2 launch libra rtabmap_unified.launch.py \
#           mode:=rgbd \
#           working_dir:=/home/brice/repos/LIBRA-ROS/ros2_ws/maps \
#           realsense_config:=/home/brice/repos/LIBRA-ROS/ros2_ws/install/libra/share/libra/config/realsense_all.yaml

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

# --- TABLE OF CONTENTS ---
#  !Launch Arguments
#  !Parameters and Remappings
#  !Mode-based Parameters
#  !Nodes
#

def generate_launch_description():
    # Get general paths
    default_working_dir = os.path.join(os.path.expanduser("~"), '.rtabmap')
    
    # Get package-specific paths
    pkg_share = get_package_share_directory('libra')
    config_dir = os.path.join(pkg_share, 'config')
    default_urdf_model_path = os.path.join(pkg_share, 'models', 'robots', 'libra1.urdf.xacro')

    #--------------------------------------------------------------------------
    # !Launch Arguments
    #--------------------------------------------------------------------------
    
    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='rgbd_lidar',
        choices=['rgbd_lidar', 'rgbd', 'stereo'],
        description='RTAB-Map mode: "rgbd_lidar" for LIDAR-corrected RGB+D data, "rgbd" for RGB+D data only, or "stereo" for stereoscopic IR data only.'
    )

    declare_odom_source_cmd = DeclareLaunchArgument(
        'odom_source',
        default_value='robot',
        choices=['robot', 'vio'],
        description='Odometry source: "robot" for EKF-fused actuator + IMU data, "vio" for visual-inertial odometry fusion.'
    )

    declare_working_dir_cmd = DeclareLaunchArgument(
        'working_dir',
        default_value=default_working_dir,
        description='Path to RTAB-Map working directory. If empty, defaults to ".rtabmap/" in your user home.'
    )

    declare_delete_db_cmd = DeclareLaunchArgument(
        'delete_db',
        default_value='false',
        description='Delete previous RTAB-Map database on launch.'
    )

    declare_urdf_model_cmd = DeclareLaunchArgument(
        'urdf_model',
        default_value=default_urdf_model_path,
        description='Path to URDF file with robot model definition. (rgbd_lidar only)'
    )

    declare_frame_id_cmd = DeclareLaunchArgument(
        'frame_id',
        default_value='',  # set conditionally based on mode
        description='Robot model base frame ID. If empty, value determined by mode.'
    )

    declare_realsense_config_cmd = DeclareLaunchArgument(
        'realsense_config',
        default_value='',  # set conditionally based on mode
        description='Path to YAML file with RealSense camera parameters. If empty, value determined by mode.'
    )
    
    declare_lidar_port_cmd = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/lidar',
        description='Serial port for 2D LIDAR. If empty, defaults to "/dev/lidar". (rgbd_lidar only)'
    )

    #--------------------------------------------------------------------------
    # !Parameters and Remappings
    #--------------------------------------------------------------------------

    # NOTE: for more detailed information on RTAB-Map parameters, see
    #       https://github.com/introlab/rtabmap/wiki/Change-parameters
    #         and
    #       https://wiki.ros.org/rtabmap_ros/Tutorials/Advanced%20Parameter%20Tuning
    
    # Common RTAB-Map parameters
    base_rtab_params = {
        'sync_queue_size': 30,  # default: 10
        'topic_queue_size': 30,  # default: 10
        'subscribe_odom_info': True,  # NOTE: only published by RTAB-Map odom nodes
        'wait_imu_to_init': True,
        # Odometry and registration
        'Reg/Force3DoF': 'true',  # roll, pitch, and Z must be explicitly provided (i.e., won't be estimated)
    }

    # Mode-specific RTAB-Map parameters
    stereo_params = base_rtab_params | {
        # Input topics
        'subscribe_stereo': True
    }

    rgbd_params = base_rtab_params | {
        # Input topics
        'subscribe_rgb': True,
        'subscribe_depth': True,
        'subscribe_rgbd': False  # let RTAB-Map take care of combining them
    }

    rgbd_lidar_params = rgbd_params | {
        # Input topics
        'subscribe_scan_cloud': True,
        # Mapping config
        'RGBD/NeighborLinkRefining': 'true',  # correct odom with laser scans + ICP
        'RGBD/ProximityBySpace': 'true',  # find local loop closures based on robot position using ICP
        'RGBD/AngularUpdate': '0.01',  # only robot movement updates the map
        'RGBD/LinearUpdate': '0.01',  # only robot movement updates the map
        'RGBD/OptimizeFromGraphEnd': 'false',  # correct all previous poses based on latest pose (/odom and /map will always match)
        'RGBD/OptimizeStrategy': '2',  # 1 = g2o, 2 = GTSAM
        'RGBD/OptimizeRobust': 'true',  # use Vertigo to reduce the influence of "weak" (outlier) loop closures
        'RGBD/OptimizeMaxError': '0',  # must be 0 if RGBD/OptimizeRobust is true
        'Grid/FromDepth': 'false',  # true: depth, false: laser scans
        # Odometry and registration
        'Icp/VoxelSize': '0.05',  # in m; filter scans down to 5 cm voxel before registration
        'Icp/MaxCorrespondenceDistance': '0.1'  # in m; max distance between points during registration
    }

    # Mode-specific RTAB-Map remappings
    stereo_remaps = [
        # Inputs: realsense2_camera
        ('left/image_rect', '/camera/infra1/image_rect_raw'),
        ('left/camera_info', '/camera/infra1/camera_info'),
        ('right/image_rect', '/camera/infra2/image_rect_raw'),
        ('right/camera_info', '/camera/infra2/camera_info'),
        # Inputs: imu_filter_madgwick_node
        ('imu', '/imu/data')
    ]

    rgbd_remaps = [
        # Inputs: realsense2_camera
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        # Inputs: imu_filter_madgwick_node
        ('imu', '/imu/data')
    ]

    rgbd_lidar_remaps = rgbd_remaps + [
        # Inputs: sf45b
        ('scan_cloud', '/pointcloud')
    ]

    def launch_nodes(context, *args, **kwargs):
        """Generates a list of nodes to be launched based on mode.
        Supplied to the ROS2 `LaunchDescription` via `OpaqueFunction`.

        Usage:
            OpaqueFunction(function=launch_nodes)

        Args:
            context: Context object containing the launch configurations.
            *args: Additional arguments (unused).
            **kwargs: Additional keyword arguments (unused).
        """
        
        # Get constant parameters
        mode = context.launch_configurations['mode']
        odom_source = context.launch_configurations['odom_source']
        working_dir = context.launch_configurations['working_dir']
        delete_db = context.launch_configurations['delete_db']
        urdf_model = context.launch_configurations['urdf_model']
        lidar_port = context.launch_configurations['lidar_port']

        # Get variable parameters
        frame_id_arg = context.launch_configurations['frame_id']
        realsense_config_arg = context.launch_configurations['realsense_config']

        #----------------------------------------------------------------------
        # !Mode-based Parameters
        #----------------------------------------------------------------------

        # Resolve parameters based on mode and odom_source
        if mode == 'rgbd_lidar':
            frame_id = frame_id_arg or 'base_link'
            realsense_config = realsense_config_arg or os.path.join(config_dir, 'realsense_rgbd.yaml')

            if odom_source == 'robot':
                rtab_params = rgbd_lidar_params | {
                    'subscribe_odom_info': False,  # OdomInfo is only published by RTAB-Map odom nodes
                    'Odom/Strategy': '0',  # rely on external odometry
                }
                remaps = rgbd_lidar_remaps + [('odom', '/robot_odom')]
                slam_extra = {'Reg/Strategy': '2'}  # improve VSLAM with laser scans

            else:  # vio
                rtab_params = rgbd_lidar_params
                remaps = rgbd_lidar_remaps
                odom_exec = 'rgbd_odometry'
                odom_extra = {'Reg/Strategy': '0'}  # rely on visual odometry
                slam_extra = {'Reg/Strategy': '2'}  # improve VSLAM with laser scans

        elif mode == 'rgbd':
            frame_id = frame_id_arg or 'camera_link'
            realsense_config = realsense_config_arg or os.path.join(config_dir, 'realsense_rgbd.yaml')

            rtab_params = rgbd_params
            remaps = rgbd_remaps
            odom_exec = 'rgbd_odometry'
            odom_extra = {}
            slam_extra = {}

        else:  # mode == 'stereo'
            frame_id = frame_id_arg or 'camera_link'
            realsense_config = realsense_config_arg or os.path.join(config_dir, 'realsense_stereo.yaml')

            rtab_params = stereo_params
            remaps = stereo_remaps
            odom_exec = 'stereo_odometry'
            odom_extra = {}
            slam_extra = {}

        #----------------------------------------------------------------------
        # !Nodes
        #----------------------------------------------------------------------

        nodes = []

        if mode == 'rgbd_lidar':
            # Publish robot frame transforms based on URDF model
            nodes.append(Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{
                    # NOTE: If ANY text in the xacro output can be interpreted as yaml,
                    # the roslaunch system will try to interpret the ENTIRE text as yaml
                    # instead of passing on the string. The biggest cause of this false
                    # interpretation is commenting out xacro calls since the xacro:property
                    # convention looks a lot like a yaml key:value pair. Comments are not
                    # removed by xacro so they are included in the output.
                    # (source: https://answers.ros.org/question/417369/caught-exception-in-launch-see-debug-for-traceback-unable-to-parse-the-value-of-parameter-robot_description-as-yaml/)
                    'robot_description': ParameterValue(
                        Command(['xacro ', urdf_model]), value_type=str
                    )
                }]
            ))
        
        # --- Conditional Odometry Nodes ---

        if odom_source == 'robot':
            # Publish static transform from 'odom' -> 'base_link'
            nodes.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='odom_to_base_link_publisher',
                arguments=[
                    '0', '0', '0',  # no X, Y, Z translation
                    '0', '0', '0',  # no yaw, pitch, roll rotation
                    'odom', 'base_link'  # parent and child frames
                ]
            ))

            # Publish static odometry message to RTAB-Map
            nodes.append(Node(
                package='libra',
                executable='static_odometry_publisher.py',
                output='screen',
            ))
        
        else:  # odom_source == 'vio'
            # Compute visual-inertial odometry
            nodes.append(Node(
                package='rtabmap_odom',
                executable=odom_exec,
                output='screen',
                parameters=[ rtab_params | odom_extra | {
                    'Rtabmap\WorkingDirectory': working_dir,
                    'frame_id': frame_id
                }],
                remappings=remaps
            ))

        # --- Common Nodes ---

        # RealSense camera
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                )
            ]),
            launch_arguments={
                'camera_namespace': '',  # avoid nesting "/camera/camera"
                'config_file': realsense_config
            }.items()
        ))

        # Compute quaternion of the IMU
        nodes.append(Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False
            }],
            remappings=[
                # Inputs: realsense2_camera
                ('imu/data_raw', '/camera/imu')
            ]
        ))

        # LightWare 2D LIDAR
        if mode == 'rgbd_lidar':
            nodes.append(Node(
                package='lightwarelidar2',
                executable='sf45b',
                output='screen',
                parameters=[{
                    'port': lidar_port,
                    'frameId': 'lidar_link',
                    'baudrate': 115200,
                    'updateRate': 12,  # set fastest rotation rate  
                    'lowAngleLimit': -160,  # set max scan angles
                    'highAngleLimit': 160
                }],
            ))

        # Primary RTAB-Map SLAM computation
        nodes.append(Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[ rtab_params | slam_extra | {
                'Rtabmap\WorkingDirectory': working_dir,
                'frame_id': frame_id
            }],
            remappings=remaps,
            arguments=['-d'] if delete_db == 'true' else []
        ))

        # RTAB-Map's custom RViz GUI
        nodes.append(Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[ rtab_params | {'frame_id': frame_id} ],
            remappings=remaps
        ))
        
        return nodes

    return LaunchDescription([
        # Launch arguments
        declare_mode_cmd,
        declare_odom_source_cmd,
        declare_working_dir_cmd,
        declare_delete_db_cmd,
        declare_urdf_model_cmd,
        declare_frame_id_cmd,
        declare_realsense_config_cmd,
        declare_lidar_port_cmd,
        # Nodes
        OpaqueFunction(function=launch_nodes)
    ])
