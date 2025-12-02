import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration

import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# --- CONFIG ---

# The fixed frame to measure from (usually 'map' or 'odom')
BASE_FRAME_ID = 'map'
# The frame to track (i.e., the tip)
TIP_FRAME_ID = 'manip_out_link'
# Published Path topic (to be consumed by RViz)
PATH_TOPIC = '/tip_path'
# Frequency (in seconds) for checking the transform/publishing the path
PUB_FREQ = 0.1  # 10 Hz

# ---------------------

class PathPublisher(Node):
    """
    Simple ROS 2 node that publishes a persistent nav_msgs/Path message for the
    given frame (default: tip) for visualization in RViz.
    """

    def __init__(self):
        super().__init__('tip_path_publisher')
        self.get_logger().info(f"Starting Tip Path Publisher for frame '{TIP_FRAME_ID}'")

        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer(Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize Path publisher and message
        self.path_pub = self.create_publisher(Path, PATH_TOPIC, 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = BASE_FRAME_ID
        
        # Create timer for update loop
        self.timer = self.create_timer(PUB_FREQ, self.update_path)


    def update_path(self):
        """
        Main loop that looks up the transform, updates the path history, and
        publishes the result.
        """
        # Update timestamp
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        
        try:
            # Look up TF (Fixed Frame -> Child Frame)
            transform = self.tf_buffer.lookup_transform(
                BASE_FRAME_ID,
                TIP_FRAME_ID,
                Time(), # use latest
                Duration(seconds=1.0) # timeout
            )

            # Convert to PoseStamped
            current_pose = PoseStamped()
            current_pose.header = transform.header
            current_pose.pose.position.x = transform.transform.translation.x
            current_pose.pose.position.y = transform.transform.translation.y
            current_pose.pose.position.z = transform.transform.translation.z
            current_pose.pose.orientation = transform.transform.rotation

            # Append current pose to Path and publish
            self.path_msg.poses.append(current_pose)
            self.path_pub.publish(self.path_msg)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # Handle errors without failing
            self.get_logger().warn(f"Error looking up {BASE_FRAME_ID} to {TIP_FRAME_ID}: {e}", throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    tip_path_publisher = PathPublisher()
    try:
        rclpy.spin(tip_path_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        tip_path_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
