# Simultaneous Localization and Mapping (SLAM) in ROS 2

This document explains how to use the LIBRA's sensor suite (RGB-D camera + spinning LIDAR) for SLAM using RTAB-Map.
The official RTAB-Map repo for ROS 2 can be found [here](https://github.com/introlab/rtabmap_ros/tree/ros2).

## Table of Contents

- [Before You Start](#before-you-start)
- [Usage](#usage)
    - [*Terminal 1: rtabmap_viz*](#terminal-1-rtabmap_viz)
- [Troubleshooting](#troubleshooting)
- [Notes](#notes)
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
2. Add a persistent environment variable to your `.bashrc` defining Cyclone DDS as the preferred middleware
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

## Troubleshooting

(none)

## Notes

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
