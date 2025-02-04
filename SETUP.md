# Setup

The instructions in this file assume an Ubuntu 22.04 machine, so all ROS-related commands links are for Humble Hawksbill.
Choose the version that's right for you.

| Ubuntu Distro | ROS 2 Version |
|---|---|
| 22.04 "Jammy" | [Humble](https://docs.ros.org/en/humble/index.html) |
| 24.04 "Noble" | [Jazzy](https://docs.ros.org/en/jazzy/index.html) |

---

1. Follow the official ROS 2 install guide for [Ubuntu (deb packages)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
    - If you are behind a proxy, at the "_Now add the ROS 2 GPG key_" step you will have to add the following command-line option: `-x [protocol://][user:password@]proxyhost[:port]` (e.g., `-x http://proxy.noc.titech.ac.jp:3128`).
2. Install other important ROS packages.
    ```bash
    sudo apt install ros-humble-joint-state-publisher-gui
    sudo apt install ros-humble-xacro
    ```
3. Add the ROS 2 source script to the end of your `.bashrc`.
    ```bash
    # In a terminal
    nano ~/.bashrc
    # In .bashrc
    . /opt/ros/humble/setup.bash
    ```
4. Source your `.bashrc` so the setup script takes effect in your active terminal.
    ```bash
    . ~/.bashrc
    ```
