# LIBRA ROS Tools

ROS2 workspace for the LIBRA project. Kept separate from the main [LIBRA-App](https://github.com/christian-brice/LIBRA-App) repo so as to reduce clutter.

Project link: <https://github.com/christian-brice/LIBRA-ROS2-Tools>

## Table of Contents

- [Requirements](#requirements)
- [Usage](#usage)
    - [*Other Terminals*](#other-terminals)
- [Troubleshooting](#troubleshooting)
    - [*ROS 2 commands/nodes are having issues communicating*](#ros-2-commandsnodes-are-having-issues-communicating)
    - [*RViz: "Wrong permissions on runtime directory"*](#rviz-wrong-permissions-on-runtime-directory)
    - [*RViz (or other Qt-based app) crashes on startup*](#rviz-or-other-qt-based-app-crashes-on-startup)
    - [*RViz isn't loading the robot model*](#rviz-isnt-loading-the-robot-model)
- [About Us](#about-us)
- [Contributing](#contributing)

## Requirements

| | Minimum | Recommended |
|---|---|---|
| **Operating System** | Ubuntu 22.04.4<br>(Jammy) | Ubuntu 22.04.4<br>(Jammy) |
| **ROS 2** | Humble Hawksbill<br>(EOL: 2027/5) | Humble Hawksbill<br>(EOL: 2027/5) |
| **Intel&reg; RealSense&trade;<br>Depth Camera** | Any D400-series | D456 |
| **LightWare LIDAR&copy;<br>Device** | SF40-C | SF45-B |

## Usage

For ROS 2 command tips, see [docs/ROS_Commands.md](/docs/ROS_COMMANDS.md).

1. Open a terminal in the `ros2_ws` directory and build the project.

    ```bash
    cd ros2_ws/
    colcon build --packages-up-to libra
    ```

2. Source the workspace for the `libra` package.

    ```bash
    . install/local_setup.bash
    ```

3. Run one of the provided launch files.

    ```bash
    # Sensor Suite (RGB-D camera + LIDAR)
    ros2 launch libra sensor_suite.launch.py
    ```

### *Other Terminals*

Since I'm just testing, I still haven't combined everything into a single launch file. In the meantime, for each point below, source the workspace and run the provided command.

- RealSense node:

    ```bash
    ros2 run realsense2_camera realsense2_camera_node --ros-args \
    -p pointcloud.enable:=true \
    -p enable_gyro:=true -p enable_accel:=true -p unite_imu_method:=2 \
    -p depth_module.depth_profile:=848x480x60 -p depth_module.infra_profile:=848x480x60 -p rgb_camera.color_profile:=848x480x60 \
    -p align_depth.enable:=true
    ```

- LIDAR node:

    ```bash
    ros2 run lightwarelidar2 sf45b --ros-args -p port:=/dev/ttyACM0 -p frameId:=lidar_link -p updateRate:=12 -p lowAngleLimit:=-160 -p highAngleLimit:=160
    ```

- Recording a session:

    ```bash
    cd ros2_ws
    mkdir bag_files && cd bag_files/
    ros2 bag record -a
    # To stop recording, press Ctrl+C
    ```

## Troubleshooting

### *ROS 2 commands/nodes are having issues communicating*

If you followed `SETUP.md` and set your ROS 2 middleware to CycloneDDS, this may mean that your network environment is not playing nice with CycloneDDS. In that case, you can temporarily revert to using FastDDS with the following command (note: Nav2- and slam_toolbox-based actions require CycloneDDS to work).

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### *RViz: "Wrong permissions on runtime directory"*

`QStandardPaths: wrong permissions on runtime directory /run/user/1000/ 0755 instead of 0700`.

If you get the above message whenever you open RViz in WSL2, just execute the following in a terminal.

```bash
chmod 0700 /run/user/1000/
```

### *RViz (or other Qt-based app) crashes on startup*

Your graphics drivers might be outdated or incompatible with the versions of the software we're using. Particularly, **Qt-based GUI applications will suffer from frustrating crashes if your graphics drivers aren't up to date**. Follow the instructions below to add the right [Mesa](https://www.mesa3d.org/) package repository (PPA) and upgrade your drivers.

1. Check your current Mesa/OpenGL version using `glxinfo`. Save the output in case you have to revert the update later.

    ```bash
    sudo apt install mesa-utils
    glxinfo | grep "OpenGL version"
    ```

2. Add the appropriate PPA for updating your Mesa/OpenGL drivers.
    > ***NOTE:*** If you're on Ubuntu 24.04 "Noble" you can pick either PPA (although the second one, `oibaf/graphics-drivers`, is recommended).
    - For Ubuntu 18.04 "Bionic" to 24.04 "Noble":

        ```bash
        sudo add-apt-repository ppa:kisak/kisak-mesa -y
        ```

    - For Ubuntu 24.04 "Noble" to 25.04 "Plucky":

        ```bash
        sudo add-apt-repository ppa:oibaf/graphics-drivers -y
        ```

3. Update your graphics libraries (note: this may also update other graphics-adjacent packages, such as OpenCV).

    ```bash
    sudo apt update && sudo apt upgrade
    ```

4. Check your updated Mesa/OpenGl version once again. If it shows a newer version, try running `rviz2` by itself now. On the other hand, if you are suddenly experiencing other/new graphical issues, revert to the previous version.

    ```bash
    glxinfo | grep "OpenGL version"
    ```

For reference, see [this guide](https://linuxcapable.com/how-to-upgrade-mesa-drivers-on-ubuntu-linux/) on LinuxCapable.com and [this discussion](https://www.reddit.com/r/ROS/comments/11wh0m3/wsl2_does_not_display_gazebo_and_rviz/) on the r/ROS subreddit.

### *RViz isn't loading the robot model*

If you see the following in RViz...

```bash
"No tf data. Actual error: Frame [base_link] does not exist"
```

... this may be a timing issue with RViz opening before `robot_state_publisher` finishes parsing the URDF (since it uses Xacro). Provided there are no errors in your terminal, just wait for a while and the model will eventually load.

## About Us

**Christian Brice** ([email](mailto:brice.c.67b9@m.isct.ac.jp)) is a doctoral student in mechanical engineering at the Institute of Science Tokyo (formerly: Tokyo Institute of Technology). The *LIBRA* project is the focus of his doctoral studies.

The **[Gen Endo Laboratory](www.robotics.mech.e.titech.ac.jp/gendo/en/)** is affiliated with the Department of Mechanical Engineering at the Institute of Science Tokyo.

## Contributing

1. Ensure Git is installed.

    ```bash
    sudo apt update
    sudo apt install git
    ```

    - If you're behind a proxy, you must also add your proxy settings to Git. In a terminal, input the following command (note: the `address` should be entered **with** the `http://` prefix).

        ```bash
        # (e.g., http://proxy.noc.titech.ac.jp:3128 for Gen Endo Lab)
        git config --global http.proxy <address>:<port>
        ```

2. Check out this project and all its submodules.

    ```bash
    git clone --recurse-submodules https://github.com/christian-brice/LIBRA-ROS.git
    ```
