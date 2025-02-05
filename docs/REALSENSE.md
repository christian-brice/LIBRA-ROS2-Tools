# Intel RealSense Camera + ROS2

This document lists the necessary steps to set up and use the `realsense2_camera` ROS2 nodes in a Linux environment.

The official troubleshooting doc can be found [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/troubleshooting.md).

- [Before You Start](#before-you-start)
- [Usage](#usage)
    - [*Terminal 1*](#terminal-1)
    - [*Terminal 2*](#terminal-2)
- [Troubleshooting](#troubleshooting)
    - [*Errors installing/removing `librealsense2-dkms` on WSL2*](#errors-installingremoving-librealsense2-dkms-on-wsl2)
- [Notes](#notes)


## Before You Start

> **_NOTE:_** (as of 2025/2/5) Do NOT attempt to install `librealsense2-dkms` in WSL2; this will leave the package in a broken, unmodifiable state (see [Troubleshooting](#troubleshooting)).

Follow the instructions in the [realsense-ros](https://github.com/IntelRealSense/realsense-ros) git repo to download and build the project.

**If you are using WSL2 (specifically an Ubuntu 22.04 target), you must build the RealSense SDK from source.**
Furthermore, once you get to "Step 2: Install latest Intel&reg; RealSense&trade; SDK 2.0", rather than following the Linux Ubuntu Installation, you will have to use the [LibUVC-backend installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/libuvc_installation.md) script.

After you've verified that the SDK installed successfully, you may proceed as normal (i.e., installing the ROS2 wrapper via debian packages).

## Usage

You will need two terminals: one to run the `realsense2_camera` node and another to initialize RViz.

### *Terminal 1*

```bash
# Launch the camera node
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true enable_gyro:=true enable_accel:=true
```

The three parameters have the following effects:
- Enable "pointcloud" post-processing filter.
- Enable "accel" and "gyro" publishing (note: these can also be combined into an "imu" msg; see the official documentation).

For a full list of parameters, see the ["Parameters" section of the realsense-ros README](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#parameters).

### *Terminal 2*

```bash
# Run RViz and configure to show Image, DepthCloud, PointCloud2 visualizations.
rviz2
```

## Troubleshooting

### *Errors installing/removing `librealsense2-dkms` on WSL2*

See [https://stackoverflow.com/questions/48431372/removing-broken-packages-in-ubuntu](https://stackoverflow.com/questions/48431372/removing-broken-packages-in-ubuntu).

## Notes

- The **coordinate systems** differ between the ROS 2 (robot) and optical (camera) models. Some [tf](http://wiki.ros.org/tf) topics are automatically published which allow you to transform between them. For more information, see the [explanation in the realsense-ros README](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#ros2robot-vs-opticalcamera-coordination-systems).

- It's possible to **enable high dynamic range (HDR) on the depth module**, although at the time of writing (2025/2/5) this disables auto-exposure, requiring you to manually find appropriate exposure and gain values for your environment. Note that using HDR with quick movements may, conversely, introduce artifacts. For instructions on how to enable and adjust HDR, see the bullet points for the `hdr_merge` and `depth_module.hdr_enabled` parameters in the ["Post-Processing Filters" section of the realsense-ros README](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#post-processing-filters). There is also a [whitepaper](https://dev.intelrealsense.com/docs/high-dynamic-range-with-stereoscopic-depth-cameras) on the matter.
