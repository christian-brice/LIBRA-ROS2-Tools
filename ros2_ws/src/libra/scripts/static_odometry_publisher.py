#!/usr/bin/env python3

# Brief:
#   Publishes a static odometry message with zero velocity, since LIBRA robots
#   are stationary. This provides a stable odometry source for SLAM systems
#   like RTAB-Map when a robot's base does not move around in the world frame.
#
# Usage:
#   $ ros2 run libra static_odometry_publisher

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class StaticOdometryPublisher(Node):
    """
    Continuously publishes a static Odometry message with zero velocity.
    """
    def __init__(self):
        super().__init__('static_odometry_publisher_node')

        # --- Parameters ---

        # ROS2 parameters with defaults
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_frequency', 20.0)

        # Internal Python members
        self.odom_frame_ = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame_ = self.get_parameter('base_frame').get_parameter_value().string_value
        self.publish_frequency_ = self.get_parameter('publish_frequency').get_parameter_value().double_value

        # --- ROS2 Interfaces ---

        # Set up message
        self.init_static_msg()

        # Initialize and spin off publisher
        self.odom_pub_ = self.create_publisher(Odometry, '/robot_odom', 10)
        timer_period = 1.0 / self.publish_frequency_
        self.timer = self.create_timer(timer_period, self.publish_odometry)

        # --- End of Initialization ---
        
        self.get_logger().info(
            f"Static Odometry Publisher started.\n  Publishing on '/robot_odom' "
            f"with frame_id='{self.odom_frame_}' and child_frame_id='{self.base_frame_}' "
            f"at {self.publish_frequency_} Hz."
        )

    def init_static_msg(self):
        """
        Builds a static nav_msgs/msg/Odometry message with all values set to zero.
        """
        self.odom_msg_ = Odometry()

        # Set header
        self.odom_msg_.header.frame_id = self.odom_frame_
        self.odom_msg_.child_frame_id = self.base_frame_

        # Set pose (position and orientation)
        self.odom_msg_.pose.pose.position.x = 0.0
        self.odom_msg_.pose.pose.position.y = 0.0
        self.odom_msg_.pose.pose.position.z = 0.0
        self.odom_msg_.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0  # identity (no rotation)
        )

        # NOTE: Covariance is a 6x6 matrix stored as a 36-element array.
        #       The diagonal indices indicate position and orientation elements.
        #       We set a very small covariance to indicate high confidence in
        #       the static pose.
        self.odom_msg_.pose.covariance[0] = 1e-9   # x
        self.odom_msg_.pose.covariance[7] = 1e-9   # y
        self.odom_msg_.pose.covariance[14] = 1e-9  # z
        self.odom_msg_.pose.covariance[21] = 1e-9  # roll
        self.odom_msg_.pose.covariance[28] = 1e-9  # pitch
        self.odom_msg_.pose.covariance[35] = 1e-9  # yaw

        # Set twist (linear and angular velocities)
        self.odom_msg_.twist.twist.linear.x = 0.0
        self.odom_msg_.twist.twist.linear.y = 0.0
        self.odom_msg_.twist.twist.linear.z = 0.0
        self.odom_msg_.twist.twist.angular.x = 0.0
        self.odom_msg_.twist.twist.angular.y = 0.0
        self.odom_msg_.twist.twist.angular.z = 0.0

        self.odom_msg_.twist.covariance[0] = 1e-9   # vx
        self.odom_msg_.twist.covariance[7] = 1e-9   # vy
        self.odom_msg_.twist.covariance[14] = 1e-9  # vz
        self.odom_msg_.twist.covariance[21] = 1e-9  # vroll
        self.odom_msg_.twist.covariance[28] = 1e-9  # vpitch
        self.odom_msg_.twist.covariance[35] = 1e-9  # vyaw

    def publish_odometry(self):
        """
        Updates and publishes an Odometry message.
        """
        # Update header
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()

        # Publish message
        self.odom_pub_.publish(self.odom_msg_)

def main(args=None):
    """
    Standard ROS2 entry point.
    """
    # Initialize node
    rclpy.init(args=args)
    node = StaticOdometryPublisher()

    # Run until stopped by user
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
