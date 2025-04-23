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

## Usage

### *Terminal 1: rtabmap_viz*

```bash
cd ros2_ws/
colcon build && . install/local_setup.bash
ros2 launch libra rtabmap_realsense_d456_stereo.launch.py
```

## Troubleshooting

(none)

## Notes

### *Topics used by RTAB-Map*

#### **Subscribes to**

- `/imu/data`
- `/camera/infra1/image_rect_raw`
- `/camera/infra1/camera_info`
- `/camera/infra2/image_rect_raw`
- `/camera/infra2/camera_info`

#### **Publishes**

- ...

### *Resources*

- [RTAB-Map ROS Wiki - RGB-D Handheld Mapping](https://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping) (note: this is for ROS 1, but gives you a general idea of the necessary setup for various types of RGB-D sensors)
- [RTAB-Map Github - Lost Odometry (RED screens!)](https://github.com/introlab/rtabmap/wiki/Kinect-mapping#lostodometry)
