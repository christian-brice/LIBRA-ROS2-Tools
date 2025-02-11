# Simultaneous Localization and Mapping (SLAM) in ROS 2

This document ...

## Table of Contents

- [Before You Start](#before-you-start)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Notes](#notes)

## Before You Start

1. The communication middleware that ROS uses is DDS; specifically, [Fast DDS](https://fast-dds.docs.eprosima.com/en/latest/) by default. However, [Cyclone DDS](https://cyclonedds.io/) is recommended for MoveIt 2, Nav2, slam_toolbox, etc.
    1. Install Cyclone DDS for ROS 2.

        ```bash
        sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
        ```

    2. Add a persistent environment variable to your `.bashrc` defining Cyclone DDS as the preferred middleware.

        ```bash
        echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
        source ~/.bashrc
        ```

2. Install required packages.

    ```bash
    sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-slam-toolbox
    ```

    - Optionally, install Turtlebot3 packages if you want to experiment in simulated environments. Don't forget to pick a default Turtlebot model.

        ```bash
        sudo apt install ros-humble-turtlebot3*
        echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
        ```

## Usage

TODO

## Troubleshooting

(none)

## Notes

(none)
