# Good Information for Papers and Presentations

This is a collection of useful phrases, data, etc. (with their sources) that I've found might come in handy when writing a LIBRA paper or presentation.

## Table of Contents

- [Sensors](#sensors)
    - [*Calibration*](#calibration)
    - [*Positioning*](#positioning)
- [Mapping](#mapping)
    - [*RTAB-Map*](#rtab-map)

## Sensors

### *Calibration*

- Official RealSense guidance:
    - [Best Known Methods for Optimal Camera Performance over Lifetime](https://realsenseai.com/news-insights/news/best-known-methods-for-optimal-camera-performance-over-lifetime/)
        - Covers relative, absolute ("tare"), and dynamic calibrations
    - [IMU Calibration Tool for Intel&reg; RealSense&trade; Depth Camera](https://dev.realsenseai.com/docs/imu-calibration-tool-for-intel-realsense-depth-camera)
        - `rs-imu-calibration.py` can be found at [https://github.com/IntelRealSense/librealsense](https://github.com/IntelRealSense/librealsense) under `tools/rs-imu-calibration`
        - Use of a 3-axis level is recommended, but not necessary

- ["Visual-Inertial Sensor Calibration -- A Complete Tutorial and Discussion" on YouTube](https://www.youtube.com/watch?v=BtzmsuJemgI)

### *Positioning*

- It is not a good idea to scan the same area with both a LIDAR and depth (stereo) camera. Lasers can cause "speckle" in stereo imaging, leading to temporal instability in the data. The laser tracks can show up as dots in the RGB-D data (even when using IR light), and since stereo cameras are intentionally set at different angles, the dots will appear at different locations relative to each camera. Therefore, an IR pattern projector is used to improve the reliability of the camera's depth data.
    - An Intel team has investigated various methods to improve depth data accuracy in their cameras in a whitepaper published to their developer’s website ([https://dev.intelrealsense.com/docs/projectors](https://dev.intelrealsense.com/docs/projectors)). Although the RealSense cameras have pattern projectors, the compact size of the camera limits the impact of the projector; by using a more powerful third-party projector, results can be vastly improved. In the Intel whitepaper, experiments show that a projector pattern with around 3 times higher density texture (than the RealSense camera’s integrated projector) improves the RMS error of the depth data by a factor of 3.

## Mapping

### *RTAB-Map*

#### **RGB-D and LIDAR**

- ["Can I use 2D lidar with stereo camera to create a 3D map?" on Robotics StackExchange](https://answers.ros.org/question/319611/)
    - "By default, [RTAB-Map] will use [laser scans] for the occupancy grid generation. To use them to refine odometry and loop closures, see description of RTAB-Map's parameters in the [tutorial](http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot#Kinect_.2B-_Odometry_.2B-_2D_laser). For more info, see this [paper](https://doi.org/10.1002/rob.21831)."

- [RTAB-Map as an open-source lidar and visual simultaneous localization and mapping library for large-scale and long-term online operation](https://doi.org/10.1002/rob.21831)

#### **Robust Graph Optimization**

- "[Loop] closure detection... is not perfect! It may [happen] that some [very similar (even identical)] objects... in different locations could cause wrong loop closure detections. Optimizing the graph with these wrong loop closures will add significant errors to the generated map. Hopefully, some approaches [1, 2] can deal with a lot of wrong loop closures. Vertigo has been integrated in RTAB-Map to increase robustness to graph optimization."
    - ["RTAB-Map: Robust Graph Optimization with GTSAM and Vertigo" on YouTube](https://www.youtube.com/watch?v=A8v70DZxLF8)
    - [RTAB-Map Wiki](https://github.com/introlab/rtabmap/wiki/Robust-Graph-Optimization#introduction)
    - [Latif Y, Cadena C and Neira J (2013) Robust loop closing over time for pose graph slam. Int. J. of Robotics Research 32(14): 1611—1626](https://doi.org/10.1177/0278364913498910)
    - [Sunderhauf N and Protzel P (2012) Towards a robust back-end for pose graph SLAM. In: Proc. IEEE Int. Conf. on Robotics and Automation. pp. 1254–1261](http://dx.doi.org/10.1109/ICRA.2012.6224709)

#### **ICP (Iterative Closest Point)**

- "When a loop closure happens between two images, we compute the visual transformation using the corresponding visual words. When ICP is enabled, this transformation is used as a guess for the ICP algorithm."
    - [RTAB-Map Wiki](https://github.com/introlab/rtabmap/wiki/ICP#a-good-case-for-icp)

- "ICP tries to match as many corresponding points as it can."
    - [RTAB-Map Wiki](https://github.com/introlab/rtabmap/wiki/ICP#a-bad-case-for-icp)

- "Generally, a [Euclidean] fitness under 0.1 gives good results. However, in the [misaligned] second case, the fitness is even lower than in the first case, which makes this criterion not robust to detect such wrong cases."

    - ```bash
      pcl_viewer old.pcd guess.pcd  # see transform using visual transformation guess only
      pcl_icp old.pcd guess.pcd
      pcl_viewer old.pcd guess.pcd  # see transform with ICP
      ```

    - [RTAB-Map Wiki](https://github.com/introlab/rtabmap/wiki/ICP#discussion)
