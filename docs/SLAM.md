# Simultaneous Localization and Mapping (SLAM) in ROS 2

This document explains how to use the LIBRA's sensor suite (RGB-D camera + spinning LIDAR) for SLAM using RTAB-Map.
The official RTAB-Map repo for ROS 2 can be found [here](https://github.com/introlab/rtabmap_ros/tree/ros2).

## Table of Contents

- [Before You Start](#before-you-start)
- [Usage](#usage)
    - [*Terminal 1: rtabmap\_viz*](#terminal-1-rtabmap_viz)
- [Post Processing](#post-processing)
- [Troubleshooting](#troubleshooting)
    - [*RTAB-Map (and other GUI apps) become unresponsive to mouse inputs*](#rtab-map-and-other-gui-apps-become-unresponsive-to-mouse-inputs)
- [Notes](#notes)
    - [*Tips*](#tips)
    - [*Topics used by RTAB-Map*](#topics-used-by-rtab-map)
    - [*Resources*](#resources)

## Before You Start

As stated in the `rtabmap_ros` repo, simply installing the package via APT is enough to prepare your system for SLAM via ROS 2.

```bash
sudo apt install ros-$ROS_DISTRO-rtabmap-ros
```

The communication middleware that ROS 2 uses is DDS; specifically, [Fast DDS](https://fast-dds.docs.eprosima.com/en/latest/) by default. However, [Cyclone DDS](https://cyclonedds.io/) is recommended for SLAM, particularly if RTAB-Map's GUI or topic frequency feel laggy (even if processing time looks fast enough).

1. Install Cyclone DDS for ROS 2.

    ```bash
    sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
    ```

2. Add a persistent environment variable to your `.bashrc` defining Cyclone DDS as the preferred middleware.

    ```bash
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
    source ~/.bashrc
    ```

It may also be useful to make RTAB-Map's command-line logs appear synced with ROS 2's. Add the following to your `.bashrc` to do so.

```bash
cat << 'EOF' >> ~/.bashrc
# Sync RTAB-Map and ROS 2 (RCLCPP) command-line logs
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_COLORIZED_OUTPUT=1
EOF
source ~/.bashrc
```

## Usage

### *Terminal 1: rtabmap_viz*

```bash
cd ros2_ws/
colcon build && . install/local_setup.bash
ros2 launch libra rtabmap_realsense_d456_stereo.launch.py
```

This launches the following five nodes:

- `realsense2_camera`, in the default camera namespace and with the following options.
    - Disables the built-in IR emitter (this reduces speckling).
    - Enables the `gyro`, `accel`, `infra1`, and `infra2` streams.
    - Enables `unite_imu` using linear interpolation (`=2`).
    - Syncs frames from its various cameras under the same timestamp.
- `stereo_odometry`, for visually estimating robot odometry using a stereo camera.
- `rtabmap`, the main SLAM node w/ loop closure detector.
- `rtabmap_viz`, which brings up the RTAB-Map GUI.
- `imu_filter_madgwick`, for automatically computing the quaternion of the RealSense's angular velocity and linear acceleration data (this is necessary for generating a complete ROS 2 IMU message).

Once finished, save the database by clicking "Edit" -> "Download all clouds (update cache)". Select "Global map optimized".

## Post Processing

The point cloud can be refined in post via the "Tools" -> "Post-processing..." dialog. The following settings gave me good results.

- Detect more loop closures (generally gives good results)

    | Setting | Value |
    |---|---|
    | Cluster radius | 0.30 m |
    | Cluster angle | 30 degrees |
    | Iterations | 1 |
    | Intra-session | Checked |
    | Inter-session | Checked |

- Refine links with ICP (requires laser scans)

    | Setting | Value |
    |---|---|
    | Refine neighbor links | **UNTESTED** |
    | Refine loop closure links | **UNTESTED** |

- Sparse Bundle Adjustment (SBA)

    | Setting | Value |
    |---|---|
    | Type | [g2o](https://github.com/RainerKuemmerle/g2o)<br>([Ceres](http://ceres-solver.org/) preferred, but must build from source) |
    | Iterations | 20 |

If the resulting point cloud is worse, it may be due to SBA. To revert post-processing, click "Edit" -> "Download graph only". Select "Global map optimized".

## Troubleshooting

### *RTAB-Map (and other GUI apps) become unresponsive to mouse inputs*

By default, Ubuntu and other flavors of Linux use the Wayland display server. However, it can be buggy sometimes - and some applications actually prefer Xorg - so try switching your display server.

1. Disable Wayland by uncommenting the relevant line in the display manager config.

    ```txt
    # Open the display manager config
    sudo nano /etc/gdm3/custom.conf
    ...
    # Find and uncomment the following line
    WaylandEnable=false
    ```

2. Add an environment variable to tell Qt which display toolkit to use.

    ```bash
    echo 'QT_QPA_PLATFORM=xcb' | sudo tee -a /etc/environment
    ```

3. Restart the machine for these changes to take effect.

    ```bash
    sudo reboot
    ```

4. Verify that the display manager is now Xorg (`x11`).

    ```bash
    echo $XDG_SESSION_TYPE
    ```

## Notes

### *Tips*

- Exported point clouds (`.ply`) can be viewed using [MeshLab](https://www.meshlab.net/) or [CloudCompare](https://www.danielgm.net/cc/). The latter can also be used to combine and manipulate point clouds (auto-comparison, point filtering, etc.).

### *Topics used by RTAB-Map*

#### **Subscribes to**

- `/camera/infra1/image_rect_raw`
- `/camera/infra1/camera_info`
- `/camera/infra2/image_rect_raw`
- `/camera/infra2/camera_info`
- `/imu/data`
- `/odom`
- `/odom_info`

#### **Publishes**

- ...

### *Resources*

- [RTAB-Map ROS Wiki - RGB-D Handheld Mapping](https://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping) (note: this is for ROS 1, but gives you a general idea of the necessary setup for various types of RGB-D sensors)
- [RTAB-Map Github - Lost Odometry (RED screens!)](https://github.com/introlab/rtabmap/wiki/Kinect-mapping#lostodometry)
