# LightWare SF45/B LIDAR + ROS 2

This document lists the necessary steps to set up and use the `sf45b` ROS 2 node in a Linux environment.

## Table of Contents

- [Before You Start](#before-you-start)
- [Usage](#usage)
    - [*Terminal 1 (ROS node)*](#terminal-1-ros-node)
    - [*Terminal 2 (visualization)*](#terminal-2-visualization)
- [Troubleshooting](#troubleshooting)
    - [*`sf45b` node errors with: "Could not establish serial connection"*](#sf45b-node-errors-with-could-not-establish-serial-connection)
- [Notes](#notes)

## Before You Start

Once the submodules have been checked out, the [lightwarelidar2](https://github.com/LightWare-Optoelectronics/lightwarelidar2) source will be located at `ros2_ws/src/lightwarelidar2`.

Then, simply build it as you would any other ROS 2 package.

```bash
cd ros2_ws/
. /opt/ros/humble/setup.bash
colcon build --packages-select lightwarelidar2
```

## Usage

You will need two terminals: one to run the `sf45b` node and another to initialize RViz.
Remember to source your local ROS environment!

### *Terminal 1 (ROS node)*

```bash
# Start the LIDAR node (ROS2 run)
ros2 run lightwarelidar2 sf45b --ros-args -p port:=/dev/ttyACM0 -p frameId:=lidar_link -p updateRate:=12 -p lowAngleLimit:=-160 -p highAngleLimit:=160
```

The parameters have the following effects:

- Specify the communications port used to interface with the SF45/B.
- Gives the node's `tf` the same name as the LIDAR link in the URDF.
- Sets the update rate to 5000 readings per second (fastest).
- Sets the scan angle to 320 degrees (maximum).

For a full list of parameters, see the ["Parameters" section under "sf45b Node" of the lightwarelidar2 README](/ros2_ws/src/lightwarelidar2/README.md#parameters-1).

> ***NOTE:*** The `frameId` parameter is incorrectly referenced in the official documentation as `frame_id`.

### *Terminal 2 (visualization)*

```bash
# Run RViz with the PointCloud2 display.
rviz2 -d src/libra/rviz/lidar.rviz
```

- `PointCloud2` should be subscribed to the topic: `/pointcloud`.

## Troubleshooting

### *`sf45b` node errors with: "Could not establish serial connection"*

1. Check whether or not the device has been properly attached to WSL2. Look for: "Microchip Technology, Inc. lwnx device".

    ```bash
    lsusb
    ```

2. Now, find the port name (this will give us the full address in the format: `/dev/<port_name>`).
    1. Disconnect the LIDAR.
    2. Reconnect it, and shortly afterwards execute the following to get a history of USB-related kernel actions.

        ```bash
        dmesg | grep -i usb
        ```

    3. Look for something similar to the following lines. In this example, the final line gives us the port name: `ttyACM0`.

        ```txt
        [10929.475561] usb 1-1: New USB device found ...
        ...
        [10929.475564] usb 1-1: Product: lwnx device
        [10929.475565] usb 1-1: Manufacturer: LightWare Optoelectronics
        ...
        [10929.477367] cdc_acm 1-1:1.0: ttyACM0: USB ACM device
        ```

3. [Run the `sf45b` node](#terminal-1-ros-node), inputting your port name as a parameter: `port:=/dev/<port_name>`.

## Notes

(none)
