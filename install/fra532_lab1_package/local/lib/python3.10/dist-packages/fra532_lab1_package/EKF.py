#!/usr/bin/python3
"""
Extended Kalman Filter (EKF) Module

This module implements the Extended Kalman Filter for robot state estimation.
It fuses wheel odometry (control input) with IMU measurements (yaw angle).

State Vector: [x, y, yaw]^T
Control Input: [v, omega]^T (linear velocity, angular velocity)
Measurement: [yaw]^T (from IMU)
"""

import numpy as np


class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for 2D robot pose estimation.
    
    The EKF estimates the robot's pose (x, y, yaw) by:
    1. Predicting the next state using wheel odometry (control input)
    2. Correcting the prediction using IMU yaw measurements
    """
    
    def __init__(self, process_noise_std=None, measurement_noise_std=None):
        """
        Initialize the Extended Kalman Filter.
        
        Args:
            process_noise_std (array-like, optional): Standard deviations for process noise [x, y, yaw].
                Defaults to [0.5, 0.5, 0.001°]
            measurement_noise_std (array-like, optional): Standard deviation for measurement noise [yaw].
                Defaults to [0.1°]
        """
        # State vector: [x, y, yaw]^T
        self.state = np.zeros((3, 1))
        
        # Covariance matrix
        self.P = np.eye(3)
        
        # Process noise covariance matrix Q
        if process_noise_std is None:
            process_noise_std = [0.5, 0.5, np.deg2rad(0.001)]
        self.Q = np.diag(process_noise_std) ** 2
        
        # Measurement noise covariance matrix R
        if measurement_noise_std is None:
            measurement_noise_std = [np.deg2rad(0.1)]
        self.R = np.diag(measurement_noise_std) ** 2
    
    def predict(self, u, dt):
        """
        Prediction step of the EKF.
        
        Predicts the next state using the motion model:
        x_{k+1} = x_k + v * cos(yaw) * dt
        y_{k+1} = y_k + v * sin(yaw) * dt
        yaw_{k+1} = yaw_k + omega * dt
        
        Args:
            u (np.ndarray): Control input [v, omega]^T where:
                - v: linear velocity (m/s)
                - omega: angular velocity (rad/s)
            dt (float): Time step (seconds)
        """
        yaw = self.state[2, 0]
        v = u[0, 0]
        omega = u[1, 0]
        
        # State prediction: f(x, u)
        self.state[0, 0] += v * np.cos(yaw) * dt
        self.state[1, 0] += v * np.sin(yaw) * dt
        self.state[2, 0] += omega * dt
        
        # Jacobian of the motion model
        G = np.array([
            [1.0, 0.0, -dt * v * np.sin(yaw)],
            [0.0, 1.0,  dt * v * np.cos(yaw)],
            [0.0, 0.0, 1.0]
        ])
        
        # Covariance prediction: P = G * P * G^T + Q
        self.P = G @ self.P @ G.T + self.Q
    
    def update(self, z):
        """
        Update (correction) step of the EKF.
        
        Corrects the predicted state using IMU yaw measurement.
        
        Args:
            z (np.ndarray): Measurement vector [yaw]^T (radians)
                If None, skip the update step (prediction only)
        """
        if z is None:
            return
        
        # Measurement model: we measure only yaw
        H = np.array([[0, 0, 1]])
        
        # Innovation (measurement residual)
        y = z - (H @ self.state)
        
        # Normalize angle to [-π, π]
        y[0, 0] = (y[0, 0] + np.pi) % (2 * np.pi) - np.pi
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.state = self.state + K @ y
        
        # Covariance update
        self.P = (np.eye(3) - K @ H) @ self.P
    
    def estimate(self, u, z, dt):
        """
        Complete EKF estimation cycle (predict + update).
        
        This is the main entry point for running the EKF.
        Combines prediction and update steps following the Bayes filter.
        
        Args:
            u (np.ndarray): Control input [v, omega]^T
            z (np.ndarray): Measurement [yaw]^T (or None to skip update)
            dt (float): Time step (seconds)
        """
        self.predict(u, dt)
        self.update(z)
    
    def get_state(self):
        """
        Get the current state estimate.
        
        Returns:
            tuple: (x, y, yaw) where:
                - x: X position (m)
                - y: Y position (m)
                - yaw: Orientation (rad)
        """
        return self.state[0, 0], self.state[1, 0], self.state[2, 0]
    
    def get_covariance(self):
        """
        Get the current covariance matrix.
        
        Returns:
            np.ndarray: 3x3 covariance matrix
        """
        return self.P.copy()
    
    def reset(self):
        """Reset the filter to initial state."""
        self.state = np.zeros((3, 1))
        self.P = np.eye(3)
