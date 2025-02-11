# LIBRA-ROS

ROS2 workspace for the LIBRA project. Kept separate from the main LIBRA-App repo so as to reduce clutter.

## Table of Contents

- [Requirements](#requirements)
- [Usage](#usage)
    - [*Other Terminals*](#other-terminals)
- [Troubleshooting](#troubleshooting)
    - [*RViz isn't loading the model*](#rviz-isnt-loading-the-model)
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
    colcon build --packages-select libra
    ```

2. Source the workspace for the `libra` package.

    ```bash
    . install/local_setup.bash
    ```

3. Run one of the provided launch files.

    ```bash
    # Sensor Suite (RGB-D camera + LIDAR)
    ros2 launch libra sensor_suite.launch.py gui:=false
    ```

### *Other Terminals*

- Recording a session:

    ```bash
    cd ros2_ws/bag_files/
    ros2 bag record -a
    # To stop recording, press Ctrl+C
    ```

## Troubleshooting

### *RViz isn't loading the model*

If you see the following in RViz...

```bash
"No tf data. Actual error: Frame [base_link] does not exist"
```

... this may be a timing issue with RViz opening before `robot_state_publisher` finishes parsing the URDF (since it uses Xacro).
Provided there are no errors in your terminal, just wait for a while and the model will eventually load.

## About Us

**Christian Brice** ([email](mailto:brice.c.aa@m.titech.ac.jp)) is a doctoral student in mechanical engineering at the Institute of Science Tokyo (formerly: Tokyo Institute of Technology).
The *LIBRA* project is the focus of his doctoral studies.

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
