# Setup

The instructions in this file assume an Ubuntu 22.04 machine, so all ROS-related commands links are for Humble Hawksbill. Choose the version that's right for you.

| Ubuntu Distro | ROS 2 Version | Compatible with<br>RealSense&trade; SDK 2.0? |
|---|---|---|
| 22.04 "Jammy" | [Humble](https://docs.ros.org/en/humble/index.html) | yes |
| 24.04 "Noble" | [Jazzy](https://docs.ros.org/en/jazzy/index.html) | maybe? |

## Table of Contents

- [Basic Packages](#basic-packages)
    - [*VS Code Extensions*](#vs-code-extensions)
- [ROS 2 Installation](#ros-2-installation)
- [RTAB-Map (SLAM) Installation](#rtab-map-slam-installation)
- [Git Submodules](#git-submodules)
    - [*Intel® RealSense™ ROS2 Wrapper*](#intel-realsense-ros2-wrapper)
    - [*LightWare LIDAR© ROS2 Wrapper*](#lightware-lidar-ros2-wrapper)
- [Setting Device Aliases](#setting-device-aliases)
    - [*Sensors*](#sensors)
    - [*Arduinos*](#arduinos)

## Basic Packages

Update the APT package lists and ensure your system is up to date.

```bash
sudo apt update && sudo apt upgrade
```

Install the following packages via the terminal.

```bash
sudo apt install -y build-essential git terminator
```

- `build-essential`: programs and libraries necessary for basic software development.
- `git`: popular open-source version control system ([link](https://git-scm.com)).
- `terminator`: useful tool for arranging terminals and saving layouts ([link](https://gnometerminator.blogspot.com/p/introduction.html)).

### *VS Code Extensions*

The following are recommendations &ndash; they aren't required to run the project.

- **Markdown Preview Enhanced** ([shd101wyy.markdown-preview-enhanced](https://marketplace.visualstudio.com/items?itemName=shd101wyy.markdown-preview-enhanced)): prettier, more robust Markdown visualizer than the one included with VS Code.
- **URDF Visualizer** ([morningfrog.urdf-visualizer](https://marketplace.visualstudio.com/items?itemName=morningfrog.urdf-visualizer)): allows you to visualize ROS2 `urdf` and `xacro` files and update them in real-time. Make sure to follow the instructions to set your `urdf-visualizer.packages` path.

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
    sudo apt install -y ros-$ROS_DISTRO-imu-tools ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-tf-transformations
    ```

## RTAB-Map (SLAM) Installation

See [docs/RTAB-MAP.md "Installing RTAB-Map"](docs/RTAB-MAP.md#installing-rtab-map) for instructions.

## Git Submodules

If you did not pull the submodules when you cloned this project, you can do so by executing the following command.

```bash
git submodule update --init --recursive
```

### *Intel&reg; RealSense&trade; ROS2 Wrapper*

Follow the instructions in the ["Before You Start" section of REALSENSE.md](/docs/REALSENSE.md#before-you-start).

### *LightWare LIDAR&copy; ROS2 Wrapper*

Follow the instructions in the ["Before You Start" section of LIDAR.md](/docs/LIDAR.md#before-you-start).

## Setting Device Aliases

For consistency across runs, it's highly recommended that you set path aliases for certain devices. For example: in [rtabmap_sensor_suite.launch.py](/ros2_ws/src/libra/launch/rtabmap_sensor_suite.launch.py), the `lightwarelidar2` node must be launched with a port parameter. Since this may change across devices and dev environments, it's set to `/dev/lidar`.

You can do this in a single command by running the provided script: `tools/alias-devices.bash`. It already has the necessary device information hard-coded, so no further input is required (generally).

```bash
./tools/alias-devices.bash
```

For reference, the installed rules (including indentifying information for each device) are provided below.

### *Sensors*

#### **RealSense**

(not required)

#### **LIDAR**

```txt
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="23a3", ATTRS{serial}=="CPBMb137X02", SYMLINK+="lidar"
```

### *Arduinos*

#### **SerialWater ("Water")**

```txt
SUBSYSTEM=="tty", KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0058", ATTRS{serial}=="D8537DBE515146544E4B2020FF0B3D4D", SYMLINK+="water"
```

#### **SerialServo ("Manip")**

```txt
SUBSYSTEM=="tty", KERNEL=="ttyACM*", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="802f", ATTRS{serial}=="055D5A3C5055344A322E3120FF082F39", SYMLINK+="manip"
```
