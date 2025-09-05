#!/usr/bin/env python3

# Brief:
#   Publishes odometry based on forward kinematics of LIBRA robot arm.
#   Computes the pose of the sensor suite in world coordinates using joint states.
#
# Usage:
#   $ ros2 run libra kinematic_odometry_publisher
#
# Note:
#   This is a simplified implementation written via GenAI (Claude), so there may be errors or inaccuracies. For better accuracy, consider using the Jacobian method for velocity computation.
#

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Twist, Point
import numpy as np
import tf_transformations

import tf2_ros
from geometry_msgs.msg import TransformStamped

class KinematicOdometryPublisher(Node):
    """
    Publishes odometry messages based on forward kinematics computation
    from joint states to sensor suite pose.
    """
    
    def __init__(self):
        super().__init__('kinematic_odometry_publisher_node')

        # --- Parameters ---
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('publish_frequency', 50.0)

        self.odom_frame_ = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.child_frame_ = self.get_parameter('child_frame').get_parameter_value().string_value
        self.publish_frequency_ = self.get_parameter('publish_frequency').get_parameter_value().double_value

        # --- Robot Parameters ---
        # Arm link length (from URDFs)
        self.link_L = 0.900
        
        # Joint offsets and corrections (from URDFs)
        self.conn_outer_L = 0.076  # every arm joint
        self.conn_inner_L = 0.056  # every arm joint
        self.adapter_L = 0.017  # only J3
        
        # Expected joint order (from /joint_states topic)
        self.expected_joints = ['Roll', 'Pitch', 'J1', 'J2', 'J3', 'Static-Manip']
        
        # State tracking
        self.last_joint_positions = None
        self.last_joint_velocities = None
        self.last_time = None
        self.current_pose = None
        self.current_twist = None

        # --- ROS2 Interfaces ---
        self.joint_sub_ = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        
        self.odom_pub_ = self.create_publisher(Odometry, '/robot_odom', 10)
        self.tf_broadcaster_ = tf2_ros.TransformBroadcaster(self)
        
        # Timer for publishing odometry at fixed rate
        timer_period = 1.0 / self.publish_frequency_
        self.timer = self.create_timer(timer_period, self.publish_odometry)

        self.get_logger().info(
            f"Kinematic Odometry Publisher started.\n"
            f"  Publishing on '/robot_odom' with frame_id='{self.odom_frame_}' "
            f"and child_frame_id='{self.child_frame_}' at {self.publish_frequency_} Hz."
        )

    def joint_states_callback(self, msg):
        """
        Process joint state updates and compute forward kinematics.
        """
        current_time = self.get_clock().now()
        
        # Validate joint order
        if msg.name != self.expected_joints:
            self.get_logger().warn(f"Unexpected joint order: {msg.name}")
            return
            
        if len(msg.position) < 6 or len(msg.velocity) < 6:
            self.get_logger().warn("Insufficient joint data")
            return

        # Extract joint values
        roll = msg.position[0]
        pitch = msg.position[1] 
        j1 = msg.position[2]
        j2 = msg.position[3]
        j3 = msg.position[4]
        # NOTE: Static-Manip (index 5) is always 0.0
        
        roll_vel = msg.velocity[0]
        pitch_vel = msg.velocity[1]
        j1_vel = msg.velocity[2]
        j2_vel = msg.velocity[3]
        j3_vel = msg.velocity[4]

        # Compute forward kinematics
        pose, twist = self.forward_kinematics(
            roll, pitch, j1, j2, j3,
            roll_vel, pitch_vel, j1_vel, j2_vel, j3_vel,
            current_time
        )
        
        self.current_pose = pose
        self.current_twist = twist
        self.last_time = current_time

    def forward_kinematics(self, roll, pitch, j1, j2, j3, 
                          roll_vel, pitch_vel, j1_vel, j2_vel, j3_vel, 
                          current_time):
        """
        Compute forward kinematics from base_link to sensor_suite_base_link.
        
        Returns:
            pose: [x, y, z, qx, qy, qz, qw] 
            twist: [vx, vy, vz, wx, wy, wz]
        """
        
        # Build transformation matrices for each joint
        # Starting from base_link, following the kinematic chain
        
        # 1. Roll joint (around X-axis) - from 2djnt assembly
        T_roll = self.tf_matrix([0, 0, 0], [roll, 0, 0])
        
        # 2. Pitch joint (around Y-axis) - from 2djnt assembly  
        T_pitch = self.tf_matrix([0, 0, 0], [0, pitch, 0])
        
        # 3. Move to start of arm (L1 segment)
        T_to_L1 = self.tf_matrix([self.link_L/2, 0, 0], [0, 0, 0])
        
        # 4. J1 joint (yaw around Z-axis)
        T_J1_offset = self.tf_matrix([self.link_L/2 + self.conn_outer_L/2, 0, 0], [0, 0, 0])
        T_J1 = self.tf_matrix([0, 0, 0], [0, 0, j1])
        
        # 5. Move to L2 segment  
        T_to_L2 = self.tf_matrix([self.conn_inner_L/2 + self.link_L/2, 0, 0], [0, 0, 0])
        
        # 6. J2 joint (yaw around Z-axis, but flipped due to origin_rpy="PI 0 0")
        T_J2_offset = self.tf_matrix([self.link_L/2 + self.conn_outer_L/2, 0, 0], [np.pi, 0, 0])
        T_J2 = self.tf_matrix([0, 0, 0], [0, 0, j2])
        
        # 7. Move to L3 segment
        T_to_L3 = self.tf_matrix([self.conn_inner_L/2 + self.link_L/2, 0, 0], [0, 0, 0])
        
        # 8. J3 joint (pitch around Y-axis after rotation, due to origin_rpy="PI/2 0 0")
        T_J3_adapter = self.tf_matrix([self.link_L/2 + self.adapter_L/2, 0, 0], [0, 0, 0])
        T_J3_offset = self.tf_matrix([self.adapter_L/2 + self.conn_outer_L/2, 0, 0], [np.pi/2, 0, 0])
        T_J3 = self.tf_matrix([0, 0, 0], [0, 0, j3])  # This becomes pitch due to the PI/2 rotation
        
        # 9. Move to L4 segment and sensor suite
        T_to_L4 = self.tf_matrix([self.conn_inner_L/2 + self.link_L/2, 0, 0], [np.pi/2, 0, 0])
        T_to_sensor_suite = self.tf_matrix([self.link_L/2, 0, 0], [0, 0, 0])
        
        # Chain all transformations
        T_total = (T_roll @ T_pitch @ T_to_L1 @ T_J1_offset @ T_J1 @ 
                  T_to_L2 @ T_J2_offset @ T_J2 @ T_to_L3 @ T_J3_adapter @ 
                  T_J3_offset @ T_J3 @ T_to_L4 @ T_to_sensor_suite)
        
        # Extract position
        position = T_total[:3, 3]
        
        # Extract rotation as quaternion
        rotation_matrix = T_total[:3, :3]
        quaternion = tf_transformations.quaternion_from_matrix(T_total)
        
        pose = [position[0], position[1], position[2], 
               quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
        
        # Compute twist (velocities)
        twist = self.compute_twist(
            roll, pitch, j1, j2, j3,
            roll_vel, pitch_vel, j1_vel, j2_vel, j3_vel
        )
        
        return pose, twist

    def tf_matrix(self, translation, rotation_rpy):
        """
        Create 4x4 transformation matrix from translation and RPY rotation.
        
        Args:
            translation: [x, y, z]
            rotation_rpy: [roll, pitch, yaw] in radians
            
        Returns:
            4x4 numpy transformation matrix
        """
        # Create rotation matrix from RPY
        R = tf_transformations.euler_matrix(
            rotation_rpy[0], rotation_rpy[1], rotation_rpy[2], 'rxyz')
        
        # Set translation
        R[:3, 3] = translation
        
        return R

    def compute_twist(self, roll, pitch, j1, j2, j3,
                     roll_vel, pitch_vel, j1_vel, j2_vel, j3_vel):
        """
        Compute linear and angular velocities using numerical differentiation.
        TODO: This is a simplified approach written via GenAI - for better accuracy, use Jacobian method.
        """
        if self.last_joint_positions is None:
            # First iteration - no velocity data
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Return joint velocities as a rough approximation
        # TODO: compute Jacobian
        linear_vel = [0.0, 0.0, 0.0]
        angular_vel = [roll_vel, pitch_vel, j1_vel + j2_vel + j3_vel]
        
        return linear_vel + angular_vel

    def publish_odometry(self):
        """
        Publish the current odometry message.
        """
        if self.current_pose is None:
            return  # no data yet
            
        odom_msg = Odometry()
        
        # Set header
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_
        odom_msg.child_frame_id = self.child_frame_
        
        # Set pose
        odom_msg.pose.pose.position.x = self.current_pose[0]
        odom_msg.pose.pose.position.y = self.current_pose[1] 
        odom_msg.pose.pose.position.z = self.current_pose[2]
        odom_msg.pose.pose.orientation.x = self.current_pose[3]
        odom_msg.pose.pose.orientation.y = self.current_pose[4]
        odom_msg.pose.pose.orientation.z = self.current_pose[5]
        odom_msg.pose.pose.orientation.w = self.current_pose[6]
        
        # Set pose covariance (low value == high confidence)
        pose_cov = [0.001] * 36  # initialize all to low covariance
        odom_msg.pose.covariance = pose_cov
        
        # Set twist
        if self.current_twist is not None:
            odom_msg.twist.twist.linear.x = self.current_twist[0]
            odom_msg.twist.twist.linear.y = self.current_twist[1]
            odom_msg.twist.twist.linear.z = self.current_twist[2] 
            odom_msg.twist.twist.angular.x = self.current_twist[3]
            odom_msg.twist.twist.angular.y = self.current_twist[4]
            odom_msg.twist.twist.angular.z = self.current_twist[5]
        
        # Set twist covariance
        twist_cov = [0.001] * 36  # initialize all to low covariance
        odom_msg.twist.covariance = twist_cov
        
        # Publish
        self.odom_pub_.publish(odom_msg)

        # TODO: this is just copy/pasted, but it should be better integrated
        #       (e.g., change function name, comments, etc.)
        # Publish the TF transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame_  # This is 'odom'
        t.child_frame_id = self.child_frame_  # This is now 'base_link'

        t.transform.translation.x = self.current_pose[0]
        t.transform.translation.y = self.current_pose[1]
        t.transform.translation.z = self.current_pose[2]

        t.transform.rotation.x = self.current_pose[3]
        t.transform.rotation.y = self.current_pose[4]
        t.transform.rotation.z = self.current_pose[5]
        t.transform.rotation.w = self.current_pose[6]

        self.tf_broadcaster_.sendTransform(t)


def main(args=None):
    """
    Standard ROS2 entry point.
    """
    rclpy.init(args=args)
    node = KinematicOdometryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
