#!/usr/bin/python3
"""
Wheel Odometry Node for Differential Drive Robot

This ROS2 node computes odometry from wheel encoder data using two methods:
1. Position-based odometry: Integrates wheel position changes
2. Velocity-based odometry: Integrates wheel velocities over time

The node also republishes LiDAR scans for visualization.
"""

# ============================================================================
# IMPORTS
# ============================================================================
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Message types
from sensor_msgs.msg import JointState, LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped, PoseStamped

# TF2 for broadcasting transforms
import tf2_ros

# NumPy for mathematical operations
import numpy as np

# Utility functions
from fra532_lab1_package.utils import get_quat


# ============================================================================
# WHEEL ODOMETRY NODE CLASS
# ============================================================================
class Wheel_odom_node(Node):
    """
    ROS2 Node for computing wheel odometry from differential drive robot.
    
    Supports three modes:
    - 'position': Odometry from wheel position integration
    - 'velocity': Odometry from wheel velocity integration
    - 'all': Both position and velocity odometry simultaneously
    """
    
    def __init__(self):
        """Initialize the Wheel Odometry Node."""
        super().__init__('Wheel_odom_node')

        # ====================================================================
        # PARAMETERS
        # ====================================================================
        self.declare_parameter('mode', 'all') 
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        # Robot physical parameters
        self.R = 0.033  # Wheel radius in meters
        self.L = 0.16   # Wheelbase (distance between wheels) in meters
        
        # Position-based odometry state (x, y, theta)
        self.x_p = 0.0   # X position from position-based odometry (m)
        self.y_p = 0.0   # Y position from position-based odometry (m)
        self.th_p = 0.0  # Orientation from position-based odometry (rad)
        
        # Velocity-based odometry state (x, y, theta)
        self.x_v = 0.0   # X position from velocity-based odometry (m)
        self.y_v = 0.0   # Y position from velocity-based odometry (m)
        self.th_v = 0.0  # Orientation from velocity-based odometry (rad)
        
        # Previous state for computing deltas
        self.last_left_pos = None   # Previous left wheel position (rad)
        self.last_right_pos = None  # Previous right wheel position (rad)
        self.last_time = None       # Previous timestamp (seconds)

        # ====================================================================
        # PUBLISHERS
        # ====================================================================
        # TF broadcaster for odom -> base_link transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Publisher for visualizing LiDAR scans
        self.scan_pub = self.create_publisher(LaserScan, '/visualize_wheel_odom_scan', 10)

        # Position-based odometry path publisher
        if self.mode in ['position', 'all']:
            self.path_p_pub = self.create_publisher(Path, '/wheel_position_path', 10)
            self.path_p_msg = Path()
            self.path_p_msg.header.frame_id = 'odom'
            
        # Velocity-based odometry path publisher
        if self.mode in ['velocity', 'all']:
            self.path_v_pub = self.create_publisher(Path, '/wheel_velocity_path', 10)
            self.path_v_msg = Path()
            self.path_v_msg.header.frame_id = 'odom'

        # ====================================================================
        # SUBSCRIPTIONS
        # ====================================================================
        # Subscribe to joint states (wheel encoder data)
        self.create_subscription(
            JointState, 
            'joint_states', 
            self.joint_states_callback, 
            10
        )

        # Subscribe to LiDAR scan for visualization
        # Use BEST_EFFORT QoS to match typical LiDAR publishers
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(
            LaserScan, 
            'scan', 
            self.scan_callback, 
            qos
        )

        self.get_logger().info(f'--- Wheel Odom Node started in mode: {self.mode} ---')

    # ========================================================================
    # CALLBACK METHODS
    # ========================================================================
    
    def scan_callback(self, msg: LaserScan):
        """
        Republish LiDAR scan for visualization.
        
        Args:
            msg (LaserScan): Incoming laser scan message
        """
        scan_to_viz = msg
        scan_to_viz.header.frame_id = 'base_link'
        # self.scan_pub.publish(scan_to_viz)

    def joint_states_callback(self, msg: JointState):
        """
        Process wheel encoder data and update odometry.
        
        This callback computes odometry using two methods:
        1. Position-based: Uses change in wheel positions
        2. Velocity-based: Uses wheel velocities and time integration
        
        Args:
            msg (JointState): Joint state message containing wheel positions and velocities
        """
        # Validate message has sufficient data
        if len(msg.position) < 2 or len(msg.velocity) < 2:
            return

        # Get current timestamp
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # t_now = self.get_clock().now().seconds_nanoseconds()[0] + \
        #               self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        
        # Initialize on first callback
        if self.last_time is None:
            self.last_time = t_now
            self.last_left_pos = msg.position[0]
            self.last_right_pos = msg.position[1]
            return

        # Compute time difference
        dt = t_now - self.last_time
        self.last_time = t_now 

        # ====================================================================
        # POSITION-BASED ODOMETRY
        # ====================================================================
        if self.mode in ['position', 'all']:
            # Compute linear distance traveled by each wheel
            d_l = (msg.position[0] - self.last_left_pos) * self.R  # Left wheel distance (m)
            d_r = (msg.position[1] - self.last_right_pos) * self.R # Right wheel distance (m)
            
            # Compute center distance and change in orientation
            d_c = (d_l + d_r) / 2.0      # Distance traveled by robot center (m)
            d_th = (d_r - d_l) / self.L  # Change in orientation (rad)
            
            # Update pose using differential drive kinematics
            # Use midpoint approximation for orientation during motion
            self.x_p += d_c * np.cos(self.th_p + d_th / 2.0)
            self.y_p += d_c * np.sin(self.th_p + d_th / 2.0)
            self.th_p += d_th

        # ====================================================================
        # VELOCITY-BASED ODOMETRY
        # ====================================================================
        if self.mode in ['velocity', 'all']:
            # Compute linear and angular velocities from wheel velocities
            v = (msg.velocity[0] + msg.velocity[1]) / 2.0  # Linear velocity (m/s)
            w = (msg.velocity[1] - msg.velocity[0]) / self.L  # Angular velocity (rad/s)
            
            # Integrate velocities to update pose
            self.x_v += v * np.cos(self.th_v) * dt
            self.y_v += v * np.sin(self.th_v) * dt
            self.th_v += w * dt

        # Update previous wheel positions for next iteration
        self.last_left_pos = msg.position[0]
        self.last_right_pos = msg.position[1]
        
        # Publish odometry data
        self.publish_data(msg.header.stamp)

    # ========================================================================
    # PUBLISHING METHODS
    # ========================================================================
    
    def publish_data(self, stamp):
        """
        Publish odometry data (path and TF) based on current mode.
        
        Args:
            stamp: ROS timestamp for the published data
        """
        # Publish position-based odometry
        if self.mode in ['position', 'all']:
            q = get_quat(self.th_p)
            self.send_path(self.path_p_pub, self.path_p_msg, self.x_p, self.y_p, q, stamp)
            self.send_tf(self.x_p, self.y_p, q, stamp)

        # Publish velocity-based odometry
        if self.mode in ['velocity', 'all']:
            q = get_quat(self.th_v)
            self.send_path(self.path_v_pub, self.path_v_msg, self.x_v, self.y_v, q, stamp)
            # Only send TF if in velocity-only mode (avoid duplicate TF in 'all' mode)
            if self.mode == 'velocity':
                self.send_tf(self.x_v, self.y_v, q, stamp)

    def send_path(self, pub, msg, x, y, q, stamp):
        """
        Append current pose to path and publish.
        
        Args:
            pub: Publisher for the path
            msg (Path): Path message to append to
            x (float): X position
            y (float): Y position
            q (Quaternion): Orientation as quaternion
            stamp: ROS timestamp
        """
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'odom'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = q
        
        msg.poses.append(pose)
        msg.header.stamp = stamp
        pub.publish(msg)

    def send_tf(self, x, y, q, stamp):
        """
        Broadcast TF transform from odom to base_link.
        
        Args:
            x (float): X position
            y (float): Y position
            q (Quaternion): Orientation as quaternion
            stamp: ROS timestamp
        """
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        
        # self.tf_broadcaster.sendTransform(t)


# ============================================================================
# MAIN FUNCTION
# ============================================================================
def main(args=None):
    """
    Main entry point for the wheel odometry node.
    
    Args:
        args: Command-line arguments (optional)
    """
    rclpy.init(args=args)
    node = Wheel_odom_node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()