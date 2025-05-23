"""
Utility functions for calculations and visualizations in the MAPPO-IK implementation.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def calculate_euclidean_distance(pos1, pos2):
    """
    Calculate the Euclidean distance between two positions.
    
    Args:
        pos1 (numpy.ndarray): First position vector (3D)
        pos2 (numpy.ndarray): Second position vector (3D)
        
    Returns:
        float: Euclidean distance
    """
    return np.linalg.norm(pos1 - pos2)

def calculate_cosine_distance(rot1, rot2):
    """
    Calculate the cosine distance between two rotation matrices.
    As per the paper, this is scaled to range [1,3].
    
    Args:
        rot1 (numpy.ndarray): First rotation matrix (3x3)
        rot2 (numpy.ndarray): Second rotation matrix (3x3)
        
    Returns:
        float: Cosine distance in range [1,3]
    """
    # Flatten matrices and calculate cosine similarity
    v1 = rot1.flatten()
    v2 = rot2.flatten()
    
    cosine_similarity = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    
    # Convert to distance and scale to [1,3] as mentioned in the paper
    # 1 means identical orientation, 3 means completely opposite
    cosine_distance = 1 + (1 - cosine_similarity)
    
    return cosine_distance

def calculate_combined_distance(pos1, pos2, rot1, rot2):
    """
    Calculate the combined distance using both Euclidean and cosine distances
    as defined in equations (5) and (6) in the paper.
    
    Args:
        pos1 (numpy.ndarray): First position vector (3D)
        pos2 (numpy.ndarray): Second position vector (3D)
        rot1 (numpy.ndarray): First rotation matrix (3x3)
        rot2 (numpy.ndarray): Second rotation matrix (3x3)
        
    Returns:
        float: Combined distance
    """
    euclidean_distance = calculate_euclidean_distance(pos1, pos2)
    cosine_distance = calculate_cosine_distance(rot1, rot2)
    
    # As per equation (5) and (6) in the paper
    b = cosine_distance - 1
    distance = euclidean_distance * cosine_distance + b
    
    return distance

def calculate_reward(distance):
    """
    Calculate the reward based on the distance as defined in equation (4) in the paper.
    
    Args:
        distance (float): Combined distance between end effector and target
        
    Returns:
        float: Reward value
    """
    return 2 + 0.006 - distance * 1000

def visualize_manipulator(joint_positions, target_position=None):
    """
    Visualize the manipulator in 3D space.
    
    Args:
        joint_positions (list): List of 3D positions for each joint
        target_position (numpy.ndarray, optional): Target position for the end effector
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the manipulator links
    joint_positions = np.array(joint_positions)
    ax.plot(joint_positions[:, 0], joint_positions[:, 1], joint_positions[:, 2], 'bo-', linewidth=2, markersize=8)
    
    # Plot the target if provided
    if target_position is not None:
        ax.plot([target_position[0]], [target_position[1]], [target_position[2]], 'ro', markersize=10)
    
    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('6-DOF Manipulator Visualization')
    
    # Set equal aspect ratio
    ax.set_box_aspect([1, 1, 1])
    
    plt.tight_layout()
    plt.show()

def plot_training_results(rewards, value_losses, policy_losses):
    """
    Plot training results similar to Figure 4 in the paper.
    
    Args:
        rewards (list): List of accumulated rewards
        value_losses (list): List of value losses
        policy_losses (list): List of policy losses
    """
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    
    # Plot accumulated rewards
    axes[0].plot(rewards)
    axes[0].set_title('Accumulated Rewards')
    axes[0].set_xlabel('Iterations')
    axes[0].set_ylabel('Reward')
    axes[0].grid(True)
    
    # Plot value loss
    axes[1].plot(value_losses)
    axes[1].set_title('Value Loss')
    axes[1].set_xlabel('Iterations')
    axes[1].set_ylabel('Loss')
    axes[1].grid(True)
    
    # Plot policy loss
    axes[2].plot(policy_losses)
    axes[2].set_title('Policy Loss')
    axes[2].set_xlabel('Iterations')
    axes[2].set_ylabel('Loss')
    axes[2].grid(True)
    
    plt.tight_layout()
    plt.show()

def plot_error_comparison(mappo_errors, mappo_ik_errors):
    """
    Plot error comparison similar to Figure 5 in the paper.
    
    Args:
        mappo_errors (tuple): (distance_errors, angle_errors) for MAPPO
        mappo_ik_errors (tuple): (distance_errors, angle_errors) for MAPPO-IK
    """
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    
    # Plot MAPPO errors
    axes[0].scatter(mappo_errors[0], mappo_errors[1])
    axes[0].set_title('MAPPO')
    axes[0].set_xlabel('Distance Error (mm)')
    axes[0].set_ylabel('Angle Error (degrees)')
    axes[0].grid(True)
    
    # Plot MAPPO-IK errors
    axes[1].scatter(mappo_ik_errors[0], mappo_ik_errors[1])
    axes[1].set_title('MAPPO-IK')
    axes[1].set_xlabel('Distance Error (mm)')
    axes[1].set_ylabel('Angle Error (degrees)')
    axes[1].grid(True)
    
    plt.tight_layout()
    plt.show()
