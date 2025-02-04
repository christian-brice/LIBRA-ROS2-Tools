# LIBRA-ROS

ROS2 workspace for the LIBRA project. Kept separate from the main LIBRA-App repo so as to reduce clutter.

## Usage

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
    ros2 launch libra sensor_suite_test.launch.py gui:=false
    ```

## Tips

- Get the argument list for a launch file.
    ```bash
    ros2 launch -s libra [filename].launch.py
    ```

## Troubleshooting 

### *RViz isn't loading the model*

If you see the following in RViz...

```bash
"No tf data. Actual error: Frame [base_link] does not exist"
```

... this may be a timing issue with RViz opening before `robot_state_publisher` finishes parsing the URDF (since it uses Xacro).
Provided there are no errors in your terminal, just wait for a while and the model will eventually load.
