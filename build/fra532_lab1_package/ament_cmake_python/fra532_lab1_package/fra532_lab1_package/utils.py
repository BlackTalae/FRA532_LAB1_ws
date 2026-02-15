"""
Utility functions for FRA532 Lab1 Package

This module contains common utility functions used across different nodes.
"""

import numpy as np
from geometry_msgs.msg import Quaternion


def get_quat(yaw):
    """
    Convert yaw angle to quaternion representation.
    
    Args:
        yaw (float): Yaw angle in radians
        
    Returns:
        Quaternion: Quaternion representation of the yaw rotation
    """
    q = Quaternion()
    q.z = np.sin(yaw / 2.0)
    q.w = np.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q
