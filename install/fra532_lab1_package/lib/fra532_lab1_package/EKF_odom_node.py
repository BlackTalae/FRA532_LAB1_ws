#!/usr/bin/python3
"""
EKF Odometry Node

This ROS2 node estimates robot pose using an Extended Kalman Filter (EKF).
It fuses wheel odometry (control input) with IMU measurements (yaw angle).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
import tf2_ros
import numpy as np

# Import the EKF module
from fra532_lab1_package.EKF import ExtendedKalmanFilter


class EKF_Odom_Node(Node):
    """ROS2 Node for EKF-based odometry estimation."""
    
    def __init__(self):
        super().__init__('ekf_odom_node')

        # ====================================================================
        # EKF INITIALIZATION - 4 INSTANCES WITH DIFFERENT PROCESS NOISE
        # ====================================================================
        # Define 4 different process noise configurations
        # Format: [x_std, y_std, yaw_std]
        # self.ekf_configs = [
        #     {
        #         'name': 'EKF_Low_Noise',
        #         'process_noise': [0.5, 0.5, np.deg2rad(0.000001)],
        #         'topic': '/ekf_path_low_noise'
        #     },
        #     {
        #         'name': 'EKF_Medium_Low_Noise',
        #         'process_noise': [0.5, 0.5, np.deg2rad(0.001)],
        #         'topic': '/ekf_path_medium_low_noise'
        #     },
        #     {
        #         'name': 'EKF_Medium_High_Noise',
        #         'process_noise': [0.5, 0.5, np.deg2rad(1.0)],
        #         'topic': '/ekf_path_medium_high_noise'
        #     },
        #     {
        #         'name': 'EKF_High_Noise',
        #         'process_noise': [0.5, 0.5, np.deg2rad(1000.0)],
        #         'topic': '/ekf_path_high_noise'
        #     }
        # ]

        # self.ekf_configs = [
        #     {
        #         'name': 'EKF_Low_Noise',
        #         'measurement_noise': [np.deg2rad(0.0001)],
        #         'topic': '/ekf_path_low_noise'
        #     },
        #     {
        #         'name': 'EKF_Medium_Low_Noise',
        #         'measurement_noise': [np.deg2rad(0.1)],
        #         'topic': '/ekf_path_medium_low_noise'
        #     },
        #     {
        #         'name': 'EKF_Medium_High_Noise',
        #         'measurement_noise': [np.deg2rad(1000.0)],
        #         'topic': '/ekf_path_medium_high_noise'
        #     },
        #     {
        #         'name': 'EKF_High_Noise',
        #         'measurement_noise': [np.deg2rad(100000.0)],
        #         'topic': '/ekf_path_high_noise'
        #     }
        # ]


        # # Process noise (same for all EKF instances)
        # process_noise = [0.5, 0.5, np.deg2rad(0.1)]

        # Measurement noise (same for all EKF instances)
        # measurement_noise = [np.deg2rad(0.1)]

        # Best noise parameters
        self.ekf_configs = [
            {
                'name': 'EKF_Low_Noise',
                'process_noise': [0.5, 0.5, np.deg2rad(0.001)],
                'measurement_noise': [np.deg2rad(0.01)],
                'topic': '/ekf_path'
            },
        ]
        
        # Initialize 4 EKF instances
        self.ekf_filters = []
        for config in self.ekf_configs:
            # ekf = ExtendedKalmanFilter(config['process_noise'], measurement_noise)
            # ekf = ExtendedKalmanFilter(process_noise, config['measurement_noise'])
            ekf = ExtendedKalmanFilter(config['process_noise'], config['measurement_noise'])
            self.ekf_filters.append(ekf)
            self.get_logger().info(f"Initialized {config['name']} with process noise: {config['process_noise']}")

        # IMU offset for relative yaw measurement
        self.imu_offset = None

        # ====================================================================
        # SENSOR DATA BUFFERS
        # ====================================================================
        # Latest IMU measurements
        self.latest_imu_yaw = None
        self.latest_imu_gyro_z = 0.0

        # ====================================================================
        # ROBOT PARAMETERS
        # ====================================================================
        self.R = 0.033  # Wheel radius (m)
        self.L = 0.16   # Wheelbase (m)
        
        # Previous wheel positions and timestamp
        self.last_left_pos = None
        self.last_right_pos = None
        self.last_time = None
        
        # ====================================================================
        # ROS2 PUBLISHERS
        # ====================================================================
        # TF broadcaster for odom -> base_link transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Path publishers for each EKF trajectory
        self.path_publishers = []
        self.path_messages = []
        
        for config in self.ekf_configs:
            # Create publisher for this EKF instance
            pub = self.create_publisher(Path, config['topic'], 10)
            self.path_publishers.append(pub)
            
            # Initialize path message
            path_msg = Path()
            path_msg.header.frame_id = 'odom'
            self.path_messages.append(path_msg)

        # ====================================================================
        # ROS2 SUBSCRIPTIONS
        # ====================================================================
        # Subscribe to IMU for yaw measurements
        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        
        # Subscribe to joint states for wheel odometry
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)

        # Subscribe to LiDAR for visualization
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
        self.scan_pub = self.create_publisher(LaserScan, '/visualize_EKF_scan', 10)

        self.get_logger().info('EKF Odometry Node started')

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
        self.scan_pub.publish(scan_to_viz)

    def imu_callback(self, msg: Imu):
        """
        Process IMU data to extract yaw angle and angular velocity.
        
        Args:
            msg (Imu): IMU message containing orientation and angular velocity
        """
        # Extract angular velocity (omega_z)
        self.latest_imu_gyro_z = msg.angular_velocity.z

        # Extract yaw from quaternion
        q = msg.orientation
        raw_yaw = np.arctan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        # Initialize offset on first IMU reading
        if self.imu_offset is None:
            self.imu_offset = raw_yaw

        # Compute relative yaw (starting from 0)
        relative_yaw = raw_yaw - self.imu_offset
        self.latest_imu_yaw = np.array([[relative_yaw]])

    def joint_states_callback(self, msg: JointState):
        """
        Process wheel encoder data and run EKF estimation.
        
        This callback:
        1. Computes control input (v, omega) from wheel positions
        2. Runs EKF prediction and update
        3. Publishes estimated pose
        
        Args:
            msg (JointState): Joint state message with wheel positions
        """
        # Validate message
        if len(msg.position) < 2:
            return
        
        # Get current timestamp
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Initialize on first callback
        if self.last_time is None:
            self.last_time = t_now
            self.last_left_pos = msg.position[0]
            self.last_right_pos = msg.position[1]
            return

        # Compute time difference
        dt = t_now - self.last_time
        if dt <= 0:
            return

        # ====================================================================
        # 1. COMPUTE CONTROL INPUT (u)
        # ====================================================================
        # Compute distance traveled by each wheel
        d_l = (msg.position[0] - self.last_left_pos) * self.R
        d_r = (msg.position[1] - self.last_right_pos) * self.R
        
        # Compute linear velocity from wheels
        v = ((d_l + d_r) / 2.0) / dt  # Linear velocity (m/s)
        
        # Use angular velocity from IMU instead of wheel odometry
        omega = self.latest_imu_gyro_z  # Angular velocity from IMU (rad/s)
        
        # Control input vector
        u = np.array([[v], [omega]])

        # ====================================================================
        # 2. RUN EKF ESTIMATION FOR ALL 4 FILTERS
        # ====================================================================
        # Run all EKF instances with the same control input and IMU measurement
        for ekf in self.ekf_filters:
            ekf.estimate(u, self.latest_imu_yaw, dt)
        
        # Clear IMU buffer to prevent reusing the same measurement
        self.latest_imu_yaw = None

        # ====================================================================
        # 3. PUBLISH RESULTS FOR ALL FILTERS
        # ====================================================================
        self.publish_all_ekf_data(msg.header.stamp)
        
        # Update previous state
        self.last_left_pos = msg.position[0]
        self.last_right_pos = msg.position[1]
        self.last_time = t_now

    # ========================================================================
    # PUBLISHING METHODS
    # ========================================================================

    def publish_all_ekf_data(self, stamp):
        """
        Publish all 4 EKF estimated poses as paths and TF transform.
        
        Args:
            stamp: ROS timestamp for the published data
        """
        # Publish path for each EKF instance
        for i, ekf in enumerate(self.ekf_filters):
            # Get current state estimate from this EKF
            x, y, th = ekf.get_state()
            
            # Create pose message
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = 'odom'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = np.sin(th / 2)
            pose.pose.orientation.w = np.cos(th / 2)
            
            # Append to path and publish
            self.path_messages[i].poses.append(pose)
            self.path_messages[i].header.stamp = stamp
            self.path_publishers[i].publish(self.path_messages[i])
        
        # ====================================================================
        # BROADCAST TF (odom -> base_link) - Use first EKF filter
        # ====================================================================
        x, y, th = self.ekf_filters[0].get_state()
        
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = np.sin(th / 2)
        t.transform.rotation.w = np.cos(th / 2)
        self.tf_broadcaster.sendTransform(t)


# ============================================================================
# MAIN FUNCTION
# ============================================================================
def main():
    """Main entry point for the EKF odometry node."""
    rclpy.init()
    node = EKF_Odom_Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()