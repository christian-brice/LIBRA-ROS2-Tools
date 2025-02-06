# Useful ROS 2 Commands

## Table of Contents

- [Launch Files and Interfaces](#launch-files-and-interfaces)
    - [*`ros2 launch -s <package_name> <filename>.launch.py`*](#ros2-launch--s-package_name-filenamelaunchpy)
- [Topic Metadata](#topic-metadata)
    - [*`ros2 interface show <topic_type>`*](#ros2-interface-show-topic_type)
    - [*`ros2 topic hz <topic_name>`*](#ros2-topic-hz-topic_name)
    - [*`ros2 topic info <topic_name>`*](#ros2-topic-info-topic_name)
    - [*`ros2 topic list`*](#ros2-topic-list)
- [Visualizations](#visualizations)
    - [*`rqt_graph`*](#rqt_graph)

## Launch Files and Interfaces

### *`ros2 launch -s <package_name> <filename>.launch.py`*

Returns the argument list for a launch file, including text descriptions and default values.

## Topic Metadata

### *`ros2 interface show <topic_type>`*

Returns the data structure expected by the given topic type.

### *`ros2 topic hz <topic_name>`*

Returns statistics regarding the rate at which data is published to the given topic.

### *`ros2 topic info <topic_name>`*

Returns the topic type for the given topic, as well as the number of active publishers and subscribers.

### *`ros2 topic list`*

Returns a list of all active topics.

- Adding the `-t` flag appends the topic type, in brackets.

## Visualizations

### *`rqt_graph`*

Generates node graphs for the current ROS 2 system.
For example, you can see how `tf` (transform frame) is propagated along all links in a robot.
