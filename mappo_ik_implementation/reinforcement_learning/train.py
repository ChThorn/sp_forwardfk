"""
Training script for the MAPPO-IK algorithm.
Based on the paper "Inverse kinematics solution and control method of 6-degree-of-freedom manipulator based on deep reinforcement learning"
"""

import numpy as np
import torch
import os
import sys
import time
import matplotlib.pyplot as plt
from collections import deque

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from reinforcement_learning.mappo_agent import MAPPOAgent
from reinforcement_learning.environment import ManipulatorEnv
from kinematic_model.dh_parameters import DHParameters
from utils.calculations import plot_training_results

def train_mappo_ik(num_agents=8, total_timesteps=30000000, save_interval=100000):
    """
    Train the MAPPO-IK algorithm with multiple agents in parallel.
    
    Args:
        num_agents (int): Number of agents to train in parallel
        total_timesteps (int): Total number of timesteps to train for
        save_interval (int): Interval for saving model checkpoints
        
    Returns:
        tuple: (trained_agent, training_stats)
    """
    print(f"Starting MAPPO-IK training with {num_agents} agents...")
    
    # Create DH parameters
    dh_params = DHParameters()
    
    # Create environments for each agent
    envs = [ManipulatorEnv(dh_params) for _ in range(num_agents)]
    
    # Create MAPPO agent
    agent = MAPPOAgent(state_dim=16, action_dim=6, hidden_dim=512, lr=2.0e-4)
    
    # Training parameters (as specified in Table 3 of the paper)
    batch_size = 1024
    buffer_size = 20480
    time_horizon = 512
    
    # Initialize buffers
    states_buffer = []
    actions_buffer = []
    log_probs_buffer = []
    rewards_buffer = []
    dones_buffer = []
    values_buffer = []
    
    # Initialize training statistics
    accumulated_rewards = []
    value_losses = []
    policy_losses = []
    
    # Reset environments
    states = [env.reset() for env in envs]
    
    # Training loop
    timestep = 0
    episode_rewards = [0] * num_agents
    
    while timestep < total_timesteps:
        # Clear buffers if they exceed buffer size
        if len(states_buffer) >= buffer_size:
            states_buffer = []
            actions_buffer = []
            log_probs_buffer = []
            rewards_buffer = []
            dones_buffer = []
            values_buffer = []
        
        # Collect experience for time_horizon steps
        for _ in range(time_horizon):
            timestep += num_agents
            
            # Get actions and values for all agents
            actions = []
            log_probs = []
            values = []
            
            # for i, state in enumerate(states):
            #     action, log_prob, _ = agent.act(state)
            #     value = agent.evaluate(state)
                
            #     actions.append(action)
            #     log_probs.append(log_prob)
            #     values.append(value)

            for i, state in enumerate(states):
                action, log_prob, _ = agent.act(state)
                value = agent.evaluate(state)
                
                actions.append(action)
                log_probs.append(log_prob.item())  # Convert tensor to scalar
                values.append(value)
            
            # Take actions in environments
            next_states = []
            rewards = []
            dones = []
            
            for i, (env, action) in enumerate(zip(envs, actions)):
                next_state, reward, done, _ = env.step(action)
                
                next_states.append(next_state)
                rewards.append(reward)
                dones.append(done)
                
                episode_rewards[i] += reward
                
                # Reset environment if done
                if done:
                    accumulated_rewards.append(episode_rewards[i])
                    episode_rewards[i] = 0
                    next_states[i] = env.reset()
            
            # Store experience in buffers
            states_buffer.extend(states)
            actions_buffer.extend(actions)
            log_probs_buffer.extend(log_probs)
            rewards_buffer.extend(rewards)
            dones_buffer.extend(dones)
            values_buffer.extend(values)
            
            # Update states
            states = next_states
        
        # Compute returns and advantages
        returns = []
        advantages = []
        
        # Convert to numpy arrays for easier processing
        states_np = np.array(states_buffer)
        actions_np = np.array(actions_buffer)
        log_probs_np = np.array(log_probs_buffer)
        rewards_np = np.array(rewards_buffer)
        dones_np = np.array(dones_buffer)
        values_np = np.array(values_buffer)
        
        # Compute returns using Generalized Advantage Estimation (GAE)
        gamma = 0.99
        lam = 0.95
        
        # Initialize gae
        gae = 0
        
        for t in reversed(range(len(rewards_np))):
            if t == len(rewards_np) - 1:
                next_value = agent.evaluate(states[t % num_agents])
            else:
                next_value = values_np[t + 1]
            
            delta = rewards_np[t] + gamma * next_value * (1 - dones_np[t]) - values_np[t]
            gae = delta + gamma * lam * (1 - dones_np[t]) * gae
            
            returns.insert(0, gae + values_np[t])
            advantages.insert(0, gae)
        
        # Convert to numpy arrays
        returns_np = np.array(returns)
        advantages_np = np.array(advantages)
        
        # Normalize advantages
        advantages_np = (advantages_np - advantages_np.mean()) / (advantages_np.std() + 1e-8)
        
        # Update policy using PPO
        # Split data into batches
        indices = np.arange(len(states_np))
        np.random.shuffle(indices)
        
        for start_idx in range(0, len(states_np), batch_size):
            end_idx = start_idx + batch_size
            batch_indices = indices[start_idx:end_idx]
            
            # Convert to PyTorch tensors
            states_batch = torch.FloatTensor(states_np[batch_indices])
            actions_batch = torch.FloatTensor(actions_np[batch_indices])
            old_log_probs_batch = torch.FloatTensor(log_probs_np[batch_indices])
            returns_batch = torch.FloatTensor(returns_np[batch_indices])
            advantages_batch = torch.FloatTensor(advantages_np[batch_indices])
            
            # Update policy and value networks
            policy_loss, value_loss, _ = agent.update(
                states_batch, actions_batch, old_log_probs_batch, returns_batch, advantages_batch
            )
            
            policy_losses.append(policy_loss)
            value_losses.append(value_loss)
        
        # Print progress
        if timestep % 10000 == 0:
            avg_reward = np.mean(accumulated_rewards[-100:]) if accumulated_rewards else 0
            print(f"Timestep: {timestep}/{total_timesteps}, Average Reward: {avg_reward:.2f}")
            print(f"Policy Loss: {policy_loss:.4f}, Value Loss: {value_loss:.4f}")
        
        # Save model checkpoint
        if timestep % save_interval == 0:
            checkpoint_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "checkpoints")
            os.makedirs(checkpoint_dir, exist_ok=True)
            agent.save(os.path.join(checkpoint_dir, f"mappo_ik_checkpoint_{timestep}.pt"))
    
    # Save final model
    checkpoint_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "checkpoints")
    os.makedirs(checkpoint_dir, exist_ok=True)
    agent.save(os.path.join(checkpoint_dir, "mappo_ik_final.pt"))
    
    # Return trained agent and training statistics
    training_stats = {
        'accumulated_rewards': accumulated_rewards,
        'value_losses': value_losses,
        'policy_losses': policy_losses
    }
    
    return agent, training_stats

def evaluate_mappo_ik(agent, num_episodes=100):
    """
    Evaluate the trained MAPPO-IK agent.
    
    Args:
        agent (MAPPOAgent): Trained agent
        num_episodes (int): Number of episodes to evaluate
        
    Returns:
        dict: Evaluation results
    """
    print(f"Evaluating MAPPO-IK agent over {num_episodes} episodes...")
    
    # Create environment
    env = ManipulatorEnv()
    
    # Evaluation metrics
    position_errors = []
    orientation_errors = []
    success_rate = 0
    
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
    
    # Return results
    results = {
        'position_errors': position_errors,
        'orientation_errors': orientation_errors,
        'avg_position_error': avg_position_error,
        'avg_orientation_error': avg_orientation_error,
        'success_rate': success_rate
    }
    
    return results

if __name__ == "__main__":
    # Create directories
    os.makedirs(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "checkpoints"), exist_ok=True)
    os.makedirs(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results"), exist_ok=True)
    
    # Train MAPPO-IK agent (reduced timesteps for demonstration)
    agent, training_stats = train_mappo_ik(num_agents=8, total_timesteps=1000000, save_interval=100000)
    
    # Plot training results
    plot_training_results(
        training_stats['accumulated_rewards'],
        training_stats['value_losses'],
        training_stats['policy_losses']
    )
    
    # Evaluate trained agent
    evaluation_results = evaluate_mappo_ik(agent, num_episodes=100)
    
    # Save results
    results_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")
    
    # Plot position and orientation errors
    plt.figure(figsize=(10, 6))
    plt.scatter(evaluation_results['position_errors'], evaluation_results['orientation_errors'])
    plt.xlabel('Position Error (mm)')
    plt.ylabel('Orientation Error')
    plt.title('MAPPO-IK Evaluation Results')
    plt.grid(True)
    plt.savefig(os.path.join(results_dir, "mappo_ik_errors.png"))
    
    print("Training and evaluation complete!")
