# Intel RealSense Camera + ROS 2

This document lists the necessary steps to set up and use the `realsense2_camera` ROS 2 node in a Linux environment.

The official troubleshooting doc can be found [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/troubleshooting.md).

## Table of Contents

- [Before You Start](#before-you-start)
    - [*Installation*](#installation)
    - [*Calibration*](#calibration)
- [Usage](#usage)
    - [*Terminal 1 (ROS node)*](#terminal-1-ros-node)
    - [*Terminal 2 (visualization)*](#terminal-2-visualization)
- [Troubleshooting](#troubleshooting)
    - [*Errors installing/removing `librealsense2-dkms` on WSL2*](#errors-installingremoving-librealsense2-dkms-on-wsl2)
    - [*`rosdep` hangs/does nothing*](#rosdep-hangsdoes-nothing)
    - [*RViz complains that the global frame doesn't exist*](#rviz-complains-that-the-global-frame-doesnt-exist)
- [Notes](#notes)
    - [*Understanding Depth Quality*](#understanding-depth-quality)
    - [*Other*](#other)

## Before You Start

### *Installation*

The instructions for a WSL2 (Windows Subsystem for Linux) instance are **vastly different** from those for a standalone/dual-boot Linux system. Make sure you follow the correct section below!

#### **Linux (non-VM)**

Follow the instructions in the [realsense-ros](https://github.com/IntelRealSense/realsense-ros) git repo to download and build the project.

The ROS 2 wrapper is included in this project as a git submodule (located at `ros2_ws/src/realsense-ros`), though you are free to install via debian packages.

#### **WSL2 (Windows Subsystem for Linux)**

Except for "Step 1: Install the ROS2 distribution", every step of the SDK and ROS wrapper installation **is different for WSL2**. In a nutshell: everything must be built from source.

> ***NOTE:*** (as of 2025/2/5) Do *NOT* attempt to install `librealsense2-dkms` in WSL2; this will leave the package in a broken, unmodifiable state (see [Troubleshooting](#troubleshooting)).

Steps that differ from the official documentation are listed below, with corrections for WSL2.

- **Step 2: Install latest Intel&reg; RealSense&trade; SDK 2.0**<br>Rather than following the Linux Ubuntu Installation, you will have to use the [LibUVC-backend installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/libuvc_installation.md) script. Once it's finished, you can check whether or not the installation was successful by attaching your RealSense camera's connection to WSL2 and executing `rs-enumerate-devices` in a terminal.

    > ***NOTE:*** If you're not sure how to "attach" a USB connection to WSL2, check out the [WSL USB Manager project on github](https://github.com/nickbeth/wsl-usb-manager).

- **Step 3: Install Intel&reg; RealSense&trade; ROS2 wrapper**<br>Do *NOT* attempt to install the ROS 2 wrapper via debian packages (i.e., `sudo apt install`) as this will override the LibUVC-backend installation and cause RealSense cameras to no longer be detected. Instead, compile it from source. Once the submodules have been checked out, the [realsense-ros](https://github.com/IntelRealSense/realsense-ros) source will be located at `ros2_ws/src/realsense-ros`.

    ```bash
    # Navigate to our ROS workspace
    cd ros2_ws/
    # Install dependencies
    sudo apt-get install python3-rosdep -y
    sudo rosdep init
    rosdep update
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
    # Build
    . /opt/ros/humble/setup.bash
    colcon build --packages-up-to realsense2_camera
    ```

### *Calibration*

#### **Camera**

To measure the effectiveness of calibration, collect depth metrics using the Depth Quality Tool (DQT) *before* and *after* applying calibration intrinsics.

1. Point the camera at a flat surface within its z-accuracy specification (i.e., if the datasheet says "2% accuracy at 2 meters", place it at 2 meters).
    > ***NOTE:*** To capture Z Accuracy metrics, you *must* specify the exact distance of the surface as ground truth in the "Configuration" tab on the left. It is recommended to use a laser range finder.

2. Open a terminal and launch the DQT.

    ```bash
    rs-depth-quality
    ```

3. Adjust the orientation of the camera according to the on-screen prompts, if any. Then, under the "Metrics" tab on the left, click "Start_record". Save at least 10 seconds of data.

4. After clicking "Stop_record", open `<filename>_depth_metrics.csv`. The columns of interest are D-H; average these to get your numbers.
    - Average: `=AVERAGE(<range>)`
    - Standard Deviation: `=STDEVP(<range>)`
    - Temporal Noise: `=(<stdev> / <average>) * 100`

5. According to the D400 Series Datasheet "Table 4-14. Depth Quality Specification", a **well-calibrated D456 camera** has the following properties. For a summary on how these metrics are calculated, see ["Understanding Depth Quality"](#understanding-depth-quality).
    - Z Accuracy (%) &plusmn;2% at 4 meters
    - Fill Rate (%) >= 99%
    - RMS Error (%) <= 2%
    - Temporal Noise (%) <= 1%

    > ***NOTE:*** If you apply the offline calibration below and the accuracy still does not meet the above specifications, follow the instructions for Dynamic Calibration in [the same article](https://realsenseai.com/news-insights/news/best-known-methods-for-optimal-camera-performance-over-lifetime/#3.5).

Calibration can be done via RealSense Viewer.

1. Prepare a textured scene. Ideally, this is the pattern provided in Appendix A of the [Self-Calibration documentation](https://dev.realsenseai.com/docs/self-calibration-for-depth-cameras), but can be anything from a textured carpet to a cluttered desk.

2. In a terminal, run the RealSense Viewer application.

    ```bash
    realsense-viewer
    ```

3. Turn on the depth stream (i.e., "Stereo Module") and point the camera at your scene. Run the self-calibration tool by clicking on "More" -> "On-Chip Calibration". **This improves the camera's precision, or relative error.**
    - If the Health Check Value is 0.25 or less and the FL Health Check Value is 0.15 or less, you're good to go!

4. Next, prepare for "Tare" calibration by doing the following:
    1. Change the status of the IR emitter depending on your scene.
        - Flat & Textured: set "Emitter Enabled" to "Off"
        - Varying Depth & Objects: set "Emitter Enabled" to "Laser Auto"
    2. Set the camera preset to "High Accuracy"
    3. Measure the ground truth distance by placing a laser range finder directly on the front glass of the camera.

5. Open the Tare calibration window via "More" -> "Tare Calibration". Input the ground truth distance in "Ground Truth (mm)", then click "Calibrate". **This improves the camera's accuracy, or absolute error.**

6. Save the calibration parameters via "More" -> "Calibration Data" -> "Save As..." at the top.

For more information, see the official documentation: [Intel&reg; RealSense&trade; Self-Calibration for D400 Series Depth Cameras](https://dev.realsenseai.com/docs/self-calibration-for-depth-cameras).

#### **IMU**

1. First, make sure you've installed the Python wrapper for Intel Realsense SDK 2.0.

    ```bash
    sudo pip install pyrealsense2 
    ```

2. Next, clone the repo and navigate to the IMU calibration tool directory.

    ```bash
    cd ~/Downloads
    git clone --depth 1 https://github.com/IntelRealSense/librealsense.git && cd librealsense/tools/rs-imu-calibration
    ```

3. Run the IMU calibration tool with sudo (the script needs admin privileges to write the resulting calibrations). Note that there are six data collection steps, each lasting approx. 2 minutes, so find a video to watch in the meantime! You *CANNOT MOVE* while the tool is collecting data, or else it will restart the current step!!

    ```bash
    sudo python3 rs-imu-calibration.py
    ```

    - For picture guidance on how to orient the camera, see the the [official documentation](https://dev.realsenseai.com/docs/imu-calibration-tool-for-intel-realsense-depth-camera): Section 4.3.2 "Capturing IMU data from 6 positions".

    > ***NOTE:*** once data collection is finished, **you will be prompted to save calibration files** for the accelerometer and gyroscope. **It is highly recommended you do so!** (e.g., with the footer "calibration")

4. Once the results have been written to the camera, you can delete the repo. If you saved calibration files, remember to move those somewhere safe first.

    ```bash
    # If you saved calibration files
    mv accel_calibration.txt ~/
    mv gyro_calibration.txt ~/
    # Return to base directory and clean up
    cd ~/Downloads && rm -rf librealsense
    ```

For more details, see the official documentation: [IMU Calibration Tool for Intel&reg; RealSense&trade; Depth Camera](https://dev.realsenseai.com/docs/imu-calibration-tool-for-intel-realsense-depth-camera).

## Usage

You will need two terminals: one to run the `realsense2_camera` node and another to initialize RViz. Remember to source your local ROS environment!

### *Terminal 1 (ROS node)*

```bash
# Launch the camera node w/ defaults (Python)
ros2 launch realsense2_camera rs_launch.py \
pointcloud.enable:=true \
enable_gyro:=true enable_accel:=true unite_imu_method:=2 \
depth_module.depth_profile:=848x480x60 depth_module.infra_profile:=848x480x60 rgb_camera.color_profile:=848x480x60 \
align_depth.enable:=true

# -or-

# Start the camera node (ROS2 run)
ros2 run realsense2_camera realsense2_camera_node --ros-args \
-p pointcloud.enable:=true \
-p enable_gyro:=true -p enable_accel:=true -p unite_imu_method:=2 \
-p depth_module.depth_profile:=848x480x60 -p depth_module.infra_profile:=848x480x60 -p rgb_camera.color_profile:=848x480x60 \
-p align_depth.enable:=true
```

The parameters have the following effects:

- Enable "pointcloud" post-processing filter.
- Enable "accel" and "gyro" publishing, and unite them into a single "imu" message (published at the rate of the gyro, i.e., 200 Hz for the D456).
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

- Aligns the depth image to the color image, publishing it to `/camera/camera/aligned_depth_to_color/image_raw`. Additionally, the pointcloud is now based on the aligned depth image.

Additionally, **diagnostics** can be enabled by adding the parameter `diagnostics_period:=<double>` (where `double` is, e.g., `10.0`).

For a full list of parameters, see the ["Parameters" section of the realsense-ros README](/ros2_ws/src/realsense-ros/README.md#parameters).

### *Terminal 2 (visualization)*

```bash
# Run RViz with the Image, DepthCloud, and PointCloud2 displays
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

I don't know why this happens, but "poking" the ROS 2 domain with a command seems to fix it. In a terminal, try executing the following.

```bash
ros2 topic list
```

## Notes

### *Understanding Depth Quality*

#### **Plane Fit RMS Error**

The RMS of the Z-Error (spatial noise).

| Term | Description | Units |
|---|---|---|
| $Z_{i}$ | Depth range of $i$-th pixel | mm |
| $Z_{pi}$ | Depth of $Z_{i}$ projection onto plane fit | mm |

Calculated as follows:

$$
\text{RMS} = \sqrt{ \frac{ \sum_{i=1}^{n} (Z_{i} - Z_{pi})^2 }{n} }
$$

#### **Subpixel RMS Error**

The subpixel accuracy.

| Term | Description | Units |
|---|---|---|
| $Z_{i}$ | Depth range of $i$-th pixel | mm |
| $Z_{pi}$ | Depth of $Z_{i}$ projection onto plane fit | mm |
| $BL$ | Optical baseline | mm |
| $FL$ | Focal length, as a multiple of pixel width | -- |

Calculated as follows:

$$
\begin{aligned}
(1)\quad & D_{i} = \frac{BL \cdot FL}{Z_{i}} \\
(2)\quad & D_{pi} = \frac{BL \cdot FL}{Z_{pi}} \\
(3)\quad & \text{RMS} = \sqrt{ \frac{ \sum_{i=1}^{n} (D_{i} - D_{pi})^2 }{n} }
\end{aligned}
$$

### *Other*

- A **latency testing tool** is provided in the `realsense-ros` repo, although it must be explicitly built.

    ```bash
    cd ros2_ws/
    colcon build --cmake-args '-DBUILD_TOOLS=ON'
    # Standard RealSense node
    ros2 launch realsense2_camera rs_intra_process_demo_launch.py intra_process_comms:=false
    # RealSense node using zero-copy communication (see "efficient intra-process communication" note below)
    ros2 launch realsense2_camera rs_intra_process_demo_launch.py intra_process_comms:=true
    ```

- The **coordinate systems** differ between the ROS 2 (robot) and optical (camera) models. Some [tf](http://wiki.ros.org/tf) topics are automatically published which allow you to transform between them. For more information, see the [explanation in the realsense-ros README](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#ros2robot-vs-opticalcamera-coordination-systems).

- It's possible to **enable high dynamic range (HDR) on the depth module**, although at the time of writing (2025/2/5) this disables auto-exposure, requiring you to manually find appropriate exposure and gain values for your environment. Note that using HDR with quick movements may, conversely, introduce artifacts. For instructions on how to enable and adjust HDR, see the bullet points for the `hdr_merge` and `depth_module.hdr_enabled` parameters in the ["Post-Processing Filters" section of the realsense-ros README](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#post-processing-filters). There is also a [whitepaper](https://dev.intelrealsense.com/docs/high-dynamic-range-with-stereoscopic-depth-cameras) on the matter.

- The Realsense node is capable of outputting a *LOT* of information, putting a strain on your computer's resources. To minimize CPU load, check out the instructions for setting up **efficient intra-process communication** in the [corresponding section of the realsense-ros README](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#efficient-intra-process-communication).
    - Note: compressed images are not supported.
