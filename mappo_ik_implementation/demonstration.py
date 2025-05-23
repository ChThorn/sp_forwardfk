"""
Main script to run a simplified demonstration of the MAPPO-IK algorithm.
This script provides a quick demonstration without requiring full training.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import torch

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from reinforcement_learning.mappo_agent import MAPPOAgent
from reinforcement_learning.environment import ManipulatorEnv
from kinematic_model.dh_parameters import DHParameters
from utils.calculations import visualize_manipulator

def run_demonstration(num_steps=100):
    """
    Run a simplified demonstration of the MAPPO-IK algorithm.
    
    Args:
        num_steps (int): Number of steps to run the demonstration
    """
    print("Running MAPPO-IK demonstration...")
    
    # Create environment
    env = ManipulatorEnv()
    
    # Create agent
    agent = MAPPOAgent(state_dim=16, action_dim=6, hidden_dim=512, lr=2.0e-4)
    
    # Reset environment
    state = env.reset()
    
    # Store trajectory
    joint_angles_trajectory = [env.joint_angles.copy()]
    
    # Run demonstration
    for step in range(num_steps):
        # Get action (random for demonstration purposes)
        # In a real scenario, this would use the trained policy
        action = np.random.uniform(-0.1, 0.1, size=6)
        
        # Take action
        next_state, reward, done, info = env.step(action)
        
        # Store joint angles
        joint_angles_trajectory.append(env.joint_angles.copy())
        
        # Update state
        state = next_state
        
        # Print progress
        if (step + 1) % 10 == 0:
            print(f"Step {step + 1}/{num_steps}")
            print(f"Reward: {reward:.4f}")
            print(f"Position Error: {info['euclidean_distance'] * 1000:.2f} mm")
            print(f"Orientation Error: {info['cosine_distance']:.2f}")
            print("-" * 50)
        
        # Check if done
        if done:
            print(f"Episode finished after {step + 1} steps")
            break
    
    # Calculate joint positions for visualization
    dh_params = env.dh_params
    joint_positions = []
    
    for joint_angles in joint_angles_trajectory:
        positions = []
        T = np.eye(4)
        
        # Base position
        positions.append(T[:3, 3].copy())
        
        # Calculate positions for each joint
        for i in range(6):
            T_i = dh_params.get_transformation_matrix(i, joint_angles[i])
            T = np.matmul(T, T_i)
            positions.append(T[:3, 3].copy())
        
        joint_positions.append(positions)
    
    # Visualize final manipulator configuration
    print("Generating visualization of manipulator trajectory...")
    
    # Create directory for results
    results_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")
    os.makedirs(results_dir, exist_ok=True)
    
    # Create 3D plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot initial and final configurations
    initial_positions = np.array(joint_positions[0])
    final_positions = np.array(joint_positions[-1])
    
    ax.plot(initial_positions[:, 0], initial_positions[:, 1], initial_positions[:, 2], 'bo-', linewidth=1, alpha=0.3, label='Initial')
    ax.plot(final_positions[:, 0], final_positions[:, 1], final_positions[:, 2], 'ro-', linewidth=2, label='Final')
    
    # Plot target
    ax.plot([env.target_position[0]], [env.target_position[1]], [env.target_position[2]], 'go', markersize=10, label='Target')
    
    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('MAPPO-IK Demonstration: Manipulator Trajectory')
    ax.legend()
    
    # Set equal aspect ratio
    ax.set_box_aspect([1, 1, 1])
    
    # Save figure
    plt.savefig(os.path.join(results_dir, "demonstration.png"))
    
    print(f"Demonstration visualization saved to {os.path.join(results_dir, 'demonstration.png')}")
    print("Demonstration complete!")

if __name__ == "__main__":
    # Create directories
    os.makedirs(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results"), exist_ok=True)
    
    # Run demonstration
    run_demonstration(num_steps=100)
