"""
DH Parameters implementation for 6-DOF manipulator.
Based on the paper "Inverse kinematics solution and control method of 6-degree-of-freedom manipulator based on deep reinforcement learning"
"""

import numpy as np

class DHParameters:
    """
    Class to define and use the DH parameters for a 6-DOF manipulator.
    """
    def __init__(self):
        """
        Initialize the DH parameters as specified in Table 1 of the paper.
        """
        # DH parameters [alpha, a, d, theta]
        # Note: theta is variable and will be provided during calculations
        self.dh_params = [
            [0, 0, 0, 0],  # Joint 1: alpha=0, a=0, d=0, theta=variable
            [-np.pi/2, 0, 0, 0],  # Joint 2: alpha=-90°, a=0, d=0, theta=variable
            [0, 'a2', 'd3', 0],  # Joint 3: alpha=0, a=a2, d=d3, theta=variable
            [-np.pi/2, 'a3', 'd4', 0],  # Joint 4: alpha=-90°, a=a3, d=d4, theta=variable
            [np.pi/2, 0, 0, 0],  # Joint 5: alpha=90°, a=0, d=0, theta=variable
            [-np.pi/2, 0, 0, 0],  # Joint 6: alpha=-90°, a=0, d=0, theta=variable
        ]
        
        # Default values for a2, a3, d3, d4 (these would be set based on the specific robot)
        self.a2 = 0.3  # Example value in meters
        self.a3 = 0.2  # Example value in meters
        self.d3 = 0.1  # Example value in meters
        self.d4 = 0.1  # Example value in meters
    
    def set_link_parameters(self, a2, a3, d3, d4):
        """
        Set the specific link parameters for the manipulator.
        
        Args:
            a2 (float): Link length parameter a2 in meters
            a3 (float): Link length parameter a3 in meters
            d3 (float): Link offset parameter d3 in meters
            d4 (float): Link offset parameter d4 in meters
        """
        self.a2 = a2
        self.a3 = a3
        self.d3 = d3
        self.d4 = d4
    
    def get_transformation_matrix(self, joint_idx, theta):
        """
        Calculate the transformation matrix for a specific joint.
        
        Args:
            joint_idx (int): Index of the joint (0-5)
            theta (float): Joint angle in radians
            
        Returns:
            numpy.ndarray: 4x4 transformation matrix
        """
        alpha, a, d, _ = self.dh_params[joint_idx]
        
        # Replace symbolic parameters with actual values
        if a == 'a2':
            a = self.a2
        elif a == 'a3':
            a = self.a3
            
        if d == 'd3':
            d = self.d3
        elif d == 'd4':
            d = self.d4
        
        # Calculate transformation matrix as per equation (1) in the paper
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)
        
        # Create the transformation matrix
        T = np.array([
            [cos_theta, -sin_theta, 0, a],
            [sin_theta * cos_alpha, cos_theta * cos_alpha, -sin_alpha, -sin_alpha * d],
            [sin_theta * sin_alpha, cos_theta * sin_alpha, cos_alpha, cos_alpha * d],
            [0, 0, 0, 1]
        ])
        
        return T
    
    def forward_kinematics(self, joint_angles):
        """
        Calculate the forward kinematics for the manipulator.
        
        Args:
            joint_angles (list): List of 6 joint angles in radians
            
        Returns:
            numpy.ndarray: 4x4 transformation matrix representing end-effector position and orientation
        """
        if len(joint_angles) != 6:
            raise ValueError("Expected 6 joint angles for a 6-DOF manipulator")
        
        # Initialize transformation matrix as identity
        T = np.eye(4)
        
        # Multiply transformation matrices for each joint
        for i in range(6):
            T_i = self.get_transformation_matrix(i, joint_angles[i])
            T = np.matmul(T, T_i)
        
        return T
    
    def extract_position_orientation(self, T):
        """
        Extract position and orientation from transformation matrix.
        
        Args:
            T (numpy.ndarray): 4x4 transformation matrix
            
        Returns:
            tuple: (position, rotation_matrix) where position is a 3D vector and rotation_matrix is a 3x3 matrix
        """
        position = T[:3, 3]
        rotation_matrix = T[:3, :3]
        
        return position, rotation_matrix
