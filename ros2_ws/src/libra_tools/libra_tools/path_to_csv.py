# Brief:
#   A simple ROS2 node that subscribes to a nav_msgs/Path topic and saves the
#   full 3D trajectory to a CSV file. Intended for persistent Path messages.
#
# Usage:
#   ros2 run libra_tools path_to_csv

import csv
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path


# Defaults
PATH_TOPIC = '/tip_path'
OUT_FILENAME = 'tip_path'


class PathSaver(Node):
    """
    Subscribes to a persistent nav_msgs/Path and writes the poses to CSV.
    """

    def __init__(self):
        super().__init__('tip_path_saver')

        # Parameters
        self.declare_parameter('path_topic', PATH_TOPIC)
        self.declare_parameter('out_filename', OUT_FILENAME)

        self._path_topic = self.get_parameter('path_topic').value
        self._out_filename = self.get_parameter('out_filename').value

        self.get_logger().info(
            f"Saving Path data from \"{self._path_topic}\" to {self._out_filename}.csv")
        
        # Internal properties
        self._received = False

        # Initialize Path subscriber
        self._sub = self.create_subscription(
            Path,
            self._path_topic,
            self._path_callback,
            10  # QoS depth
        )


    def _path_callback(self, msg: Path):
        """
        Called once the persistent Path is received.
        """
        if self._received:
            return  # just in case - only save once

        self._received = True
        self.get_logger().info(f"Received path with {len(msg.poses)} poses")

        with open(self._out_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'stamp_sec'])

            for pose in msg.poses:
                p = pose.pose.position
                q = pose.pose.orientation
                t = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9

                writer.writerow([
                    p.x, p.y, p.z,
                    q.x, q.y, q.z, q.w,
                    t
                ])

        self.get_logger().info("File saved successfully, exiting")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PathSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
