# Intel RealSense Camera + ROS 2

This document lists the necessary steps to set up and use the `realsense2_camera` ROS 2 node in a Linux environment.

The official troubleshooting doc can be found [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/troubleshooting.md).

## Table of Contents

- [Before You Start](#before-you-start)
    - [*Linux (non-VM)*](#linux-non-vm)
    - [*WSL2 (Windows Subsystem for Linux)*](#wsl2-windows-subsystem-for-linux)
- [Usage](#usage)
    - [*Terminal 1 (ROS node)*](#terminal-1-ros-node)
    - [*Terminal 2 (visualization)*](#terminal-2-visualization)
- [Troubleshooting](#troubleshooting)
    - [*Errors installing/removing `librealsense2-dkms` on WSL2*](#errors-installingremoving-librealsense2-dkms-on-wsl2)
    - [*`rosdep` hangs/does nothing*](#rosdep-hangsdoes-nothing)
    - [*RViz complains that the global frame doesn't exist*](#rviz-complains-that-the-global-frame-doesnt-exist)
- [Notes](#notes)

## Before You Start

The instructions for a WSL2 (Windows Subsystem for Linux) instance are **vastly different** from those for a standalone/dual-boot Linux system.
Make sure you follow the correct section below!

### *Linux (non-VM)*

Follow the instructions in the [realsense-ros](https://github.com/IntelRealSense/realsense-ros) git repo to download and build the project.

The ROS 2 wrapper is included in this project as a git submodule (located at `ros2_ws/src/realsense-ros`), though you are free to install via debian packages.

### *WSL2 (Windows Subsystem for Linux)*

Except for "Step 1: Install the ROS2 distribution", every step of the SDK and ROS wrapper installation **is different for WSL2**.
In a nutshell: everything must be built from source.

> ***NOTE:*** (as of 2025/2/5) Do *NOT* attempt to install `librealsense2-dkms` in WSL2; this will leave the package in a broken, unmodifiable state (see [Troubleshooting](#troubleshooting)).

#### **Step 2: Install latest Intel&reg; RealSense&trade; SDK 2.0**

Rather than following the Linux Ubuntu Installation, you will have to use the [LibUVC-backend installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/libuvc_installation.md) script.
Once it's finished, you can check whether or not the installation was successful by attaching your RealSense camera's connection to WSL2 and executing `rs-enumerate-devices` in a terminal.

> ***NOTE:*** If you're not sure how to "attach" a USB connection to WSL2, check out the [WSL USB Manager project on github](https://github.com/nickbeth/wsl-usb-manager).

#### **Step 3: Install Intel&reg; RealSense&trade; ROS2 wrapper**

Do *NOT* attempt to install the ROS 2 wrapper via debian packages (i.e., `sudo apt install`) as this will override the LibUVC-backend installation and cause RealSense cameras to no longer be detected.

Instead, compile it from source.
Once the submodules have been checked out, the [realsense-ros](https://github.com/IntelRealSense/realsense-ros) source will be located at `ros2_ws/src/realsense-ros`.

```bash
# Navigate to our ROS workspace
cd ros2_ws/
# Install dependencies
sudo apt-get install python3-rosdep -y
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
# Build
colcon build --packages-up-to realsense2_camera
```

## Usage

You will need two terminals: one to run the `realsense2_camera` node and another to initialize RViz.
Remember to source your local ROS environment!

### *Terminal 1 (ROS node)*

```bash
# Launch the camera node (Python)
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true enable_gyro:=true enable_accel:=true depth_module.depth_profile:=848x480x60 depth_module.infra_profile:=848x480x60 rgb_camera.color_profile:=848x480x60

# -or-

# Start the camera node (ROS2 run)
ros2 run realsense2_camera realsense2_camera_node --ros-args -p pointcloud.enable:=true -p enable_gyro:=true -p enable_accel:=true -p depth_module.depth_profile:=848x480x60 -p depth_module.infra_profile:=848x480x60 -p rgb_camera.color_profile:=848x480x60
```

The parameters have the following effects:

- Enable "pointcloud" post-processing filter.
- Enable "accel" and "gyro" publishing (note: these can also be combined into an "imu" msg; see the official documentation).
- Sets RGB and stereo cameras to the same resolution and framerate, so that the depth point cloud can be colored using the RGB stream. The options in the command above prioritize 60 FPS.
    - Supported options (formatted as `<width>x<height>x<fps>`):

        ```txt
        1280x720x30
        848x480x60 (our default)
        640x480x60
        640x360x90
        480x270x90
        424x240x90
        ```

For a full list of parameters, see the ["Parameters" section of the realsense-ros README](/ros2_ws/src/realsense-ros/README.md#parameters).

### *Terminal 2 (visualization)*

```bash
# Run RViz with the Image, DepthCloud, and PointCloud2 displays.
rviz2 -d src/libra/rviz/realsense.rviz
```

- `Image` should be subscribed to the topic: `/camera/camera/color/image_raw`.
- `DepthCloud` shouldn't need to be configured.
- `PointCloud2` should be subscribed to the topic: `/camera/camera/depth/color/points`.

## Troubleshooting

### *Errors installing/removing `librealsense2-dkms` on WSL2*

See [https://stackoverflow.com/questions/48431372/removing-broken-packages-in-ubuntu](https://stackoverflow.com/questions/48431372/removing-broken-packages-in-ubuntu).

### *`rosdep` hangs/does nothing*

Make sure your proxy is configured system-wide by adding it to the end of your `/etc/environment`.

```txt
http_proxy=http://proxy_server:port/
https_proxy=https://proxy_server:port/
```

See [https://robotics.stackexchange.com/questions/47926/rosdep-initialization-error](https://robotics.stackexchange.com/questions/47926/rosdep-initialization-error).

### *RViz complains that the global frame doesn't exist*

I don't know why this happens, but "poking" the ROS 2 domain with a command seems to fix it.
In a terminal, try executing the following.

```bash
ros2 topic list
```

## Notes

- The **coordinate systems** differ between the ROS 2 (robot) and optical (camera) models. Some [tf](http://wiki.ros.org/tf) topics are automatically published which allow you to transform between them. For more information, see the [explanation in the realsense-ros README](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#ros2robot-vs-opticalcamera-coordination-systems).

- It's possible to **enable high dynamic range (HDR) on the depth module**, although at the time of writing (2025/2/5) this disables auto-exposure, requiring you to manually find appropriate exposure and gain values for your environment. Note that using HDR with quick movements may, conversely, introduce artifacts. For instructions on how to enable and adjust HDR, see the bullet points for the `hdr_merge` and `depth_module.hdr_enabled` parameters in the ["Post-Processing Filters" section of the realsense-ros README](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#post-processing-filters). There is also a [whitepaper](https://dev.intelrealsense.com/docs/high-dynamic-range-with-stereoscopic-depth-cameras) on the matter.
