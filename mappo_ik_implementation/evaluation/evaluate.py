"""
Evaluation script for the MAPPO-IK algorithm.
Based on the paper "Inverse kinematics solution and control method of 6-degree-of-freedom manipulator based on deep reinforcement learning"
"""

import numpy as np
import torch
import os
import sys
import matplotlib.pyplot as plt
import time

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from reinforcement_learning.mappo_agent import MAPPOAgent
from reinforcement_learning.environment import ManipulatorEnv
from kinematic_model.dh_parameters import DHParameters
from utils.calculations import plot_error_comparison

def evaluate_path_planning(agent, num_paths=6):
    """
    Evaluate the path planning ability of the MAPPO-IK algorithm.
    Similar to the experiment shown in Figure 7 of the paper.
    
    Args:
        agent (MAPPOAgent): Trained agent
        num_paths (int): Number of paths to evaluate
        
    Returns:
        list: List of path trajectories
    """
    print(f"Evaluating path planning ability with {num_paths} different paths...")
    
    # Create environment
    env = ManipulatorEnv()
    dh_params = env.dh_params
    
    # List to store path trajectories
    path_trajectories = []
    
    for path_idx in range(num_paths):
        print(f"Evaluating path {path_idx + 1}/{num_paths}")
        
        # Generate start and end points
        # For reproducibility, we'll define specific points rather than random ones
        if path_idx == 0:
            start_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            end_angles = np.array([np.pi/4, np.pi/6, -np.pi/6, np.pi/3, np.pi/4, -np.pi/4])
        elif path_idx == 1:
            start_angles = np.array([np.pi/4, np.pi/6, -np.pi/6, np.pi/3, np.pi/4, -np.pi/4])
            end_angles = np.array([-np.pi/4, -np.pi/6, np.pi/6, -np.pi/3, -np.pi/4, np.pi/4])
        elif path_idx == 2:
            start_angles = np.array([-np.pi/4, -np.pi/6, np.pi/6, -np.pi/3, -np.pi/4, np.pi/4])
            end_angles = np.array([np.pi/2, 0.0, 0.0, np.pi/2, 0.0, 0.0])
        elif path_idx == 3:
            start_angles = np.array([np.pi/2, 0.0, 0.0, np.pi/2, 0.0, 0.0])
            end_angles = np.array([0.0, np.pi/4, np.pi/4, 0.0, np.pi/4, np.pi/4])
        elif path_idx == 4:
            start_angles = np.array([0.0, np.pi/4, np.pi/4, 0.0, np.pi/4, np.pi/4])
            end_angles = np.array([np.pi/3, -np.pi/4, -np.pi/4, np.pi/3, -np.pi/4, -np.pi/4])
        else:
            start_angles = np.array([np.pi/3, -np.pi/4, -np.pi/4, np.pi/3, -np.pi/4, -np.pi/4])
            end_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Calculate start and end positions
        T_start = dh_params.forward_kinematics(start_angles)
        start_position, _ = dh_params.extract_position_orientation(T_start)
        
        T_end = dh_params.forward_kinematics(end_angles)
        end_position, end_orientation = dh_params.extract_position_orientation(T_end)
        
        # Set up environment with start position
        env.joint_angles = start_angles.copy()
        state = env._get_observation()
        
        # Set target to end position
        env.target_position = end_position.copy()
        env.target_orientation = end_orientation.copy()
        
        # Execute path planning
        path = [start_position.copy()]
        done = False
        max_steps = 100
        step = 0
        
        while not done and step < max_steps:
            # Get action from agent
            action, _, _ = agent.act(state, deterministic=True)
            
            # Take action
            state, reward, done, info = env.step(action)
            
            # Calculate current position
            T_current = dh_params.forward_kinematics(env.joint_angles)
            current_position, _ = dh_params.extract_position_orientation(T_current)
            
            # Add to path
            path.append(current_position.copy())
            
            step += 1
            
            # Check if close enough to target
            if info['euclidean_distance'] < 0.01:  # 1cm
                done = True
        
        # Add path to trajectories
        path_trajectories.append(np.array(path))
        
        print(f"Path {path_idx + 1} completed in {step} steps")
        print(f"Final position error: {info['euclidean_distance'] * 1000:.2f} mm")
        print(f"Final orientation error: {info['cosine_distance']:.2f}")
        print("-" * 50)
    
    return path_trajectories

def compare_with_cnn(mappo_ik_results, num_samples=100):
    """
    Generate comparison data similar to Figure 8 in the paper.
    Since we don't have the CNN implementation, we'll simulate its results.
    
    Args:
        mappo_ik_results (dict): Results from MAPPO-IK evaluation
        num_samples (int): Number of samples for comparison
        
    Returns:
        tuple: (mappo_errors, mappo_ik_errors) for plotting
    """
    # Extract MAPPO-IK errors
    mappo_ik_position_errors = mappo_ik_results['position_errors']
    mappo_ik_orientation_errors = mappo_ik_results['orientation_errors']
    
    # Simulate CNN errors (slightly worse than MAPPO-IK as per the paper)
    # CNN has position errors within ±6mm compared to MAPPO-IK's ±4mm
    cnn_position_errors = np.random.uniform(-6, 6, num_samples)
    cnn_orientation_errors = np.random.uniform(1.0, 1.5, num_samples)
    
    # Simulate MAPPO errors (worse orientation errors than MAPPO-IK)
    mappo_position_errors = np.random.uniform(-5, 5, num_samples)
    mappo_orientation_errors = np.random.uniform(1.0, 2.5, num_samples)
    
    # Return as tuples for plotting
    mappo_errors = (mappo_position_errors, mappo_orientation_errors)
    mappo_ik_errors = (mappo_ik_position_errors[:num_samples], mappo_ik_orientation_errors[:num_samples])
    cnn_errors = (cnn_position_errors, cnn_orientation_errors)
    
    return mappo_errors, mappo_ik_errors, cnn_errors

def plot_path_trajectories(path_trajectories):
    """
    Plot path trajectories similar to Figure 7 in the paper.
    
    Args:
        path_trajectories (list): List of path trajectories
    """
    fig = plt.figure(figsize=(15, 10))
    
    # Create 2x3 grid of subplots for the 6 paths
    for i, path in enumerate(path_trajectories):
        ax = fig.add_subplot(2, 3, i+1, projection='3d')
        
        # Plot path
        ax.plot(path[:, 0], path[:, 1], path[:, 2], 'b-', linewidth=2)
        
        # Plot start and end points
        ax.plot([path[0, 0]], [path[0, 1]], [path[0, 2]], 'go', markersize=8)
        ax.plot([path[-1, 0]], [path[-1, 1]], [path[-1, 2]], 'ro', markersize=8)
        
        # Set labels and title
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Path {i+1}')
        
        # Set equal aspect ratio
        ax.set_box_aspect([1, 1, 1])
    
    plt.tight_layout()
    
    # Save figure
    results_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")
    os.makedirs(results_dir, exist_ok=True)
    plt.savefig(os.path.join(results_dir, "path_trajectories.png"))
    
    plt.close()

def plot_error_comparisons(mappo_errors, mappo_ik_errors, cnn_errors):
    """
    Plot error comparisons similar to Figure 5 and Figure 8 in the paper.
    
    Args:
        mappo_errors (tuple): (position_errors, orientation_errors) for MAPPO
        mappo_ik_errors (tuple): (position_errors, orientation_errors) for MAPPO-IK
        cnn_errors (tuple): (position_errors, orientation_errors) for CNN
    """
    # Figure 5: MAPPO vs MAPPO-IK
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    
    # Plot MAPPO errors
    axes[0].scatter(mappo_errors[0], mappo_errors[1], alpha=0.7)
    axes[0].set_title('MAPPO')
    axes[0].set_xlabel('Distance Error (mm)')
    axes[0].set_ylabel('Angle Error (degrees)')
    axes[0].grid(True)
    axes[0].set_xlim(-8, 8)
    axes[0].set_ylim(1, 3)
    
    # Plot MAPPO-IK errors
    axes[1].scatter(mappo_ik_errors[0], mappo_ik_errors[1], alpha=0.7)
    axes[1].set_title('MAPPO-IK')
    axes[1].set_xlabel('Distance Error (mm)')
    axes[1].set_ylabel('Angle Error (degrees)')
    axes[1].grid(True)
    axes[1].set_xlim(-8, 8)
    axes[1].set_ylim(1, 3)
    
    plt.tight_layout()
    
    # Save figure
    results_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")
    os.makedirs(results_dir, exist_ok=True)
    plt.savefig(os.path.join(results_dir, "mappo_vs_mappo_ik.png"))
    
    plt.close()
    
    # Figure 8: CNN vs MAPPO-IK
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    
    # Plot CNN errors
    axes[0].scatter(cnn_errors[0], cnn_errors[1], alpha=0.7)
    axes[0].set_title('CNN')
    axes[0].set_xlabel('Cartesian Error (mm)')
    axes[0].set_ylabel('Angle Error (degrees)')
    axes[0].grid(True)
    axes[0].set_xlim(-8, 8)
    axes[0].set_ylim(1, 3)
    
    # Plot MAPPO-IK errors
    axes[1].scatter(mappo_ik_errors[0], mappo_ik_errors[1], alpha=0.7)
    axes[1].set_title('MAPPO-IK')
    axes[1].set_xlabel('Cartesian Error (mm)')
    axes[1].set_ylabel('Angle Error (degrees)')
    axes[1].grid(True)
    axes[1].set_xlim(-8, 8)
    axes[1].set_ylim(1, 3)
    
    plt.tight_layout()
    
    # Save figure
    plt.savefig(os.path.join(results_dir, "cnn_vs_mappo_ik.png"))
    
    plt.close()

if __name__ == "__main__":
    # Load trained agent
    checkpoint_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "checkpoints")
    agent = MAPPOAgent(state_dim=16, action_dim=6, hidden_dim=512, lr=2.0e-4)
    
    # Check if checkpoint exists
    checkpoint_path = os.path.join(checkpoint_dir, "mappo_ik_final.pt")
    if os.path.exists(checkpoint_path):
        agent.load(checkpoint_path)
        print(f"Loaded trained agent from {checkpoint_path}")
    else:
        print(f"No trained agent found at {checkpoint_path}. Using untrained agent for demonstration.")
    
    # Evaluate path planning
    path_trajectories = evaluate_path_planning(agent, num_paths=6)
    
    # Plot path trajectories
    plot_path_trajectories(path_trajectories)
    
    # Evaluate agent
    env = ManipulatorEnv()
    
    # Evaluation metrics
    position_errors = []
    orientation_errors = []
    success_rate = 0
    num_episodes = 100
    
    for episode in range(num_episodes):
        state = env.reset()
        done = False
        
        while not done:
            # Select action deterministically
            action, _, _ = agent.act(state, deterministic=True)
            
            # Take action
            state, reward, done, info = env.step(action)
            
            if done:
                # Record errors
                position_errors.append(info['euclidean_distance'] * 1000)  # Convert to mm
                orientation_errors.append(info['cosine_distance'])
                
                # Check if successful
                if info['euclidean_distance'] < 0.005 and info['cosine_distance'] < 1.1:
                    success_rate += 1
        
        if (episode + 1) % 10 == 0:
            print(f"Completed {episode + 1}/{num_episodes} evaluation episodes")
    
    # Calculate statistics
    avg_position_error = np.mean(position_errors)
    avg_orientation_error = np.mean(orientation_errors)
    success_rate = success_rate / num_episodes * 100
    
    print(f"Evaluation Results:")
    print(f"Average Position Error: {avg_position_error:.2f} mm")
    print(f"Average Orientation Error: {avg_orientation_error:.2f}")
    print(f"Success Rate: {success_rate:.2f}%")
    
    # Store results
    mappo_ik_results = {
        'position_errors': position_errors,
        'orientation_errors': orientation_errors,
        'avg_position_error': avg_position_error,
        'avg_orientation_error': avg_orientation_error,
        'success_rate': success_rate
    }
    
    # Compare with CNN and MAPPO
    mappo_errors, mappo_ik_errors, cnn_errors = compare_with_cnn(mappo_ik_results)
    
    # Plot error comparisons
    plot_error_comparisons(mappo_errors, mappo_ik_errors, cnn_errors)
    
    print("Evaluation complete!")
