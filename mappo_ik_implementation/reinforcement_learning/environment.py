"""
Environment simulation for the MAPPO-IK algorithm.
This provides a simplified simulation environment that can run without Unity.
Based on the paper "Inverse kinematics solution and control method of 6-degree-of-freedom manipulator based on deep reinforcement learning"
"""

import numpy as np
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kinematic_model.dh_parameters import DHParameters
from utils.calculations import calculate_euclidean_distance, calculate_cosine_distance, calculate_combined_distance, calculate_reward

class ManipulatorEnv:
    """
    Simplified simulation environment for a 6-DOF manipulator.
    """
    def __init__(self, dh_params=None):
        """
        Initialize the environment.
        
        Args:
            dh_params (DHParameters, optional): DH parameters for the manipulator
        """
        # Initialize DH parameters
        self.dh_params = dh_params if dh_params is not None else DHParameters()
        
        # Joint angle limits (in radians)
        self.joint_limits = [
            (-np.pi, np.pi),      # Joint 1: -180° to 180°
            (-np.pi/2, np.pi/2),  # Joint 2: -90° to 90°
            (-np.pi/2, np.pi/2),  # Joint 3: -90° to 90°
            (-np.pi, np.pi),      # Joint 4: -180° to 180°
            (-np.pi/2, np.pi/2),  # Joint 5: -90° to 90°
            (-np.pi, np.pi)       # Joint 6: -180° to 180°
        ]
        
        # Current joint angles
        self.joint_angles = np.zeros(6)
        
        # Target position and orientation
        self.target_position = None
        self.target_orientation = None
        
        # Step counter
        self.step_count = 0
        
        # Maximum steps per episode
        self.max_steps = 100
        
        # Reset the environment
        self.reset()
    
    def reset(self):
        """
        Reset the environment to a new random state.
        
        Returns:
            numpy.ndarray: Initial state observation
        """
        # Reset step counter
        self.step_count = 0
        
        # Randomly initialize joint angles within limits
        self.joint_angles = np.array([
            np.random.uniform(low, high) for low, high in self.joint_limits
        ])
        
        # Generate a new target using forward kinematics with random joint angles
        target_angles = np.array([
            np.random.uniform(low, high) for low, high in self.joint_limits
        ])
        
        # Calculate target position and orientation
        T_target = self.dh_params.forward_kinematics(target_angles)
        self.target_position, self.target_orientation = self.dh_params.extract_position_orientation(T_target)
        
        # Return initial observation
        return self._get_observation()
    
    def step(self, action):
        """
        Take a step in the environment.
        
        Args:
            action (numpy.ndarray): Action vector (changes to joint angles)
            
        Returns:
            tuple: (observation, reward, done, info)
        """
        # Increment step counter
        self.step_count += 1
        
        # Update joint angles based on action
        new_joint_angles = self.joint_angles + action
        
        # Check joint limits
        exceeded_limits = False
        for i, (angle, (low, high)) in enumerate(zip(new_joint_angles, self.joint_limits)):
            if angle < low or angle > high:
                exceeded_limits = True
                new_joint_angles[i] = np.clip(angle, low, high)
        
        # Update joint angles
        self.joint_angles = new_joint_angles
        
        # Calculate current end effector position and orientation
        T_current = self.dh_params.forward_kinematics(self.joint_angles)
        current_position, current_orientation = self.dh_params.extract_position_orientation(T_current)
        
        # Calculate distance between current and target
        euclidean_distance = calculate_euclidean_distance(current_position, self.target_position)
        cosine_distance = calculate_cosine_distance(current_orientation, self.target_orientation)
        combined_distance = calculate_combined_distance(current_position, self.target_position, 
                                                       current_orientation, self.target_orientation)
        
        # Calculate reward
        reward = calculate_reward(combined_distance)
        
        # Apply punishment for exceeding joint limits
        if exceeded_limits:
            reward -= 1.0
        
        # Apply small punishment for each step to encourage faster solutions
        reward -= 0.0005
        
        # Check if done
        done = False
        
        # Done if target reached (distance below threshold)
        if euclidean_distance < 0.005 and cosine_distance < 1.1:  # 5mm position and small orientation error
            done = True
            reward += 10.0  # Bonus reward for reaching target
        
        # Done if maximum steps reached
        if self.step_count >= self.max_steps:
            done = True
        
        # Get observation
        observation = self._get_observation()
        
        # Additional info
        info = {
            'euclidean_distance': euclidean_distance,
            'cosine_distance': cosine_distance,
            'combined_distance': combined_distance,
            'exceeded_limits': exceeded_limits,
            'step_count': self.step_count
        }
        
        return observation, reward, done, info
    
    def _get_observation(self):
        """
        Get the current observation.
        
        Returns:
            numpy.ndarray: Observation vector
        """
        # Calculate current end effector position and orientation
        T_current = self.dh_params.forward_kinematics(self.joint_angles)
        current_position, current_orientation = self.dh_params.extract_position_orientation(T_current)
        
        # Calculate relative position (normalized)
        relative_position = self.target_position - current_position
        relative_position_norm = np.linalg.norm(relative_position)
        if relative_position_norm > 0:
            relative_position = relative_position / relative_position_norm
        
        # Calculate relative orientation (simplified)
        # In a real implementation, this would involve quaternion differences or other orientation metrics
        relative_orientation = np.zeros(3)  # Simplified for this implementation
        
        # Calculate distances
        euclidean_distance = calculate_euclidean_distance(current_position, self.target_position)
        cosine_distance = calculate_cosine_distance(current_orientation, self.target_orientation)
        combined_distance = calculate_combined_distance(current_position, self.target_position, 
                                                      current_orientation, self.target_orientation)
        
        # Create observation vector as described in Table 2 of the paper
        observation = np.concatenate([
            self.joint_angles,                # Joint angles (6)
            relative_position,                # Relative position (3)
            relative_orientation,             # Relative orientation (3)
            [euclidean_distance],             # Euclidean distance (1)
            [cosine_distance],                # Cosine distance (1)
            [combined_distance],              # Final distance (1)
            [self.step_count / self.max_steps]  # Step count (normalized) (1)
        ])
        
        return observation
    
    def render(self):
        """
        Render the environment (simplified version).
        In a full implementation, this would use Unity or another visualization tool.
        """
        # Calculate current end effector position
        T_current = self.dh_params.forward_kinematics(self.joint_angles)
        current_position, current_orientation = self.dh_params.extract_position_orientation(T_current)
        
        print(f"Step: {self.step_count}")
        print(f"Joint angles: {np.degrees(self.joint_angles)}")
        print(f"Current position: {current_position}")
        print(f"Target position: {self.target_position}")
        print(f"Position error: {np.linalg.norm(current_position - self.target_position) * 1000:.2f} mm")
        print(f"Orientation error: {calculate_cosine_distance(current_orientation, self.target_orientation):.2f}")
        print("-" * 50)
