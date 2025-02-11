# Setup

The instructions in this file assume an Ubuntu 22.04 machine, so all ROS-related commands links are for Humble Hawksbill.
Choose the version that's right for you.

| Ubuntu Distro | ROS 2 Version | Compatible with<br>RealSense&trade; SDK 2.0? |
|---|---|---|
| 22.04 "Jammy" | [Humble](https://docs.ros.org/en/humble/index.html) | yes |
| 24.04 "Noble" | [Jazzy](https://docs.ros.org/en/jazzy/index.html) | maybe? |

## Table of Contents

- [ROS 2 Installation](#ros-2-installation)
- [Git Submodules](#git-submodules)
    - [*Intel® RealSense™ ROS2 Wrapper*](#intel-realsense-ros2-wrapper)
    - [*LightWare LIDAR© ROS2 Wrapper*](#lightware-lidar-ros2-wrapper)

## ROS 2 Installation

1. Follow the official ROS 2 install guide for [Ubuntu (deb packages)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
    - If you are behind a proxy, at the "*Now add the ROS 2 GPG key*" step you will have to add the following command-line option: `-x [protocol://][user:password@]proxyhost[:port]` (e.g., `-x http://proxy.noc.titech.ac.jp:3128`).

2. Add the ROS 2 source script to the end of your `.bashrc`.

    ```bash
    # In a terminal
    nano ~/.bashrc
    # In .bashrc
    . /opt/ros/$ROS_DISTRO/setup.bash
    ```

3. Source your `.bashrc` so the setup script takes effect in your active terminal.

    ```bash
    . ~/.bashrc
    ```

4. Install other important ROS packages.

    ```bash
    sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
    sudo apt install ros-$ROS_DISTRO-xacro
    ```

## Git Submodules

If you did not pull the submodules when you cloned this project, you can do so by executing the following command.

```bash
git submodule update --init
```

### *Intel&reg; RealSense&trade; ROS2 Wrapper*

Follow the instructions in the ["Before You Start" section of REALSENSE.md](/docs/REALSENSE.md#before-you-start).

### *LightWare LIDAR&copy; ROS2 Wrapper*

Follow the instructions in the ["Before You Start" section of LIDAR.md](/docs/REALSENSE.md#before-you-start).
