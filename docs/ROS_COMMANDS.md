# Useful ROS 2 Commands

## Table of Contents

- [Launch Files and Interfaces](#launch-files-and-interfaces)
    - [*`ros2 launch -s <package_name> <filename>.launch.py`*](#ros2-launch--s-package_name-filenamelaunchpy)
- [Rosbags / Recording Data](#rosbags--recording-data)
    - [*`record`*](#record)
    - [*`info <filename>`*](#info-filename)
    - [*`play <filename>`*](#play-filename)
- [Topic Metadata](#topic-metadata)
    - [*`ros2 topic list`*](#ros2-topic-list)
    - [*`ros2 topic info <topic_name>`*](#ros2-topic-info-topic_name)
    - [*`ros2 topic hz <topic_name>`*](#ros2-topic-hz-topic_name)
    - [*`ros2 topic echo <topic_name>`*](#ros2-topic-echo-topic_name)
    - [*`ros2 interface show <topic_type>`*](#ros2-interface-show-topic_type)
- [Visualizations](#visualizations)
    - [*`rqt_graph`*](#rqt_graph)

## Launch Files and Interfaces

### *`ros2 launch -s <package_name> <filename>.launch.py`*

Returns the argument list for a launch file, including text descriptions and default values.

## Rosbags / Recording Data

The following sub-subsection headers correspond to subcommands of `ros2 bag`.

### *`record`*

Records active topics to a bag file until `Ctrl+C` is pressed.

- `<topic1_name> [topic2_name ...]`: record only the provided topic(s).
- `-a`: record ALL topics.
- `-e <regex>`: exclude topics matching the provided regex.
- `-d <max_duration>`: specify a maximum duration (in seconds) before the bag file is split.
- `-o <filename>`: specify an output destination.

### *`info <filename>`*

Simply prints metadata about a bag file to the terminal.

### *`play <filename>`*

Replays ROS2 data from a bag file.

#### **Player Settings**

- `-l`: loop playback.
- `-p`: start player in a paused state.
- `--start-offset <timestamp>`: start player a certain amount of time (in seconds) into the bag file.

#### **Manipulation**

- `--clock <freq>`: specify frequency of ROS Time Source (in Hz), published to `/clock`.
- `--topics <topic1> [topic2 ...]`: specify which topics to replay (by default, all are replayed).
- `--remap <old_topic1:=new_topic1> [old_topic2:=new_topic2 ...]`: rename certain topics in the replay.

## Topic Metadata

### *`ros2 topic list`*

Returns a list of all active topics.

- Adding the `-t` flag appends the topic type, in brackets.

### *`ros2 topic info <topic_name>`*

Returns the topic type for the given topic, as well as the number of active publishers and subscribers.

### *`ros2 topic hz <topic_name>`*

Returns statistics on the rate at which data is published to the given topic.

### *`ros2 topic echo <topic_name>`*

Continuously prints a published topic's contents to the terminal.

### *`ros2 interface show <topic_type>`*

Returns the data structure expected by the given topic type.

## Visualizations

### *`rqt_graph`*

Generates node graphs for the current ROS 2 system. For example, you can see how `tf` (transform frame) is propagated along all links in a robot.
