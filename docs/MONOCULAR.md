# Monocular Cameras

Although monocular cameras are no longer used in LIBRA prototypes, this file is kept in case anyone needs it.

## Table of Contents

- [Before you Start](#before-you-start)
    - [*Offline Calibration via ROS2*](#offline-calibration-via-ros2)

## Before you Start

### *Offline Calibration via ROS2*

The following is a summary of the instructions in [this blog post by Tamas Foldi](https://medium.com/starschema-blog/offline-camera-calibration-in-ros-2-45e81df12555).

Ensure the ROS2 `camera-calibration` package is installed.

```bash
sudo apt install ros-$ROS_DISTRO-camera-calibration
```

Generate a calibration pattern using [calib.io's Pattern Generator](https://calib.io/pages/camera-calibration-pattern-generator) with the following settings. **Make sure your print is NOT scaled!**

| Field | Value |
| --- | --- |
| Target Type | Checkerboard |
| Board Width x Height | 297 x 210 mm (A4 size) |
| Rows x Columns | 8 x 10 |
| Checker Width | 15 mm |

Then, run your camera's ROS2 node in one terminal and the `camera_calibration` node in another.

```bash
# Camera driver (e.g., OpenCV Cam)
ros2 run opencv_cam opencv_cam_main
# Calibration tool
ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.015 \
  --ros-args -r image:=/image_raw
```
