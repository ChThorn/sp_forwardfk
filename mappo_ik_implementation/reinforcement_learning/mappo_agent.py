"""
Environment and agent implementation for the MAPPO-IK algorithm.
Based on the paper "Inverse kinematics solution and control method of 6-degree-of-freedom manipulator based on deep reinforcement learning"
"""

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal

class PolicyNetwork(nn.Module):
    """
    Policy network for the MAPPO-IK algorithm.
    Takes environmental perception parameters as input and outputs joint angle decisions.
    """
    def __init__(self, input_dim=16, hidden_dim=512, output_dim=6):
        """
        Initialize the policy network.
        
        Args:
            input_dim (int): Dimension of input features (16 as per the paper)
            hidden_dim (int): Dimension of hidden layers (512 as per the paper)
            output_dim (int): Dimension of output (6 for 6-DOF manipulator)
        """
        super(PolicyNetwork, self).__init__()
        
        # Three hidden layers with 512 units each as specified in the paper
        self.fc1 = nn.Linear(input_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, hidden_dim)
        
        # Output layer for mean of action distribution
        self.mean_layer = nn.Linear(hidden_dim, output_dim)
        
        # Output layer for log standard deviation of action distribution
        self.log_std_layer = nn.Linear(hidden_dim, output_dim)
        
    def forward(self, x):
        """
        Forward pass through the network.
        
        Args:
            x (torch.Tensor): Input tensor of environmental perception parameters
            
        Returns:
            tuple: (mean, log_std) for action distribution
        """
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        
        mean = self.mean_layer(x)
        log_std = self.log_std_layer(x)
        
        # Clamp log_std for numerical stability
        log_std = torch.clamp(log_std, -20, 2)
        
        return mean, log_std
    
    def get_action(self, state, deterministic=False):
        """
        Sample an action from the policy.
        
        Args:
            state (torch.Tensor): Current state
            deterministic (bool): If True, return the mean action
            
        Returns:
            tuple: (action, log_prob, entropy)
        """
        mean, log_std = self.forward(state)
        std = log_std.exp()
        
        if deterministic:
            action = mean
            log_prob = None
            entropy = None
        else:
            normal = Normal(mean, std)
            action = normal.sample()
            log_prob = normal.log_prob(action).sum(dim=-1)
            entropy = normal.entropy().sum(dim=-1)
            
        return action, log_prob, entropy

class ValueNetwork(nn.Module):
    """
    Value network for the MAPPO-IK algorithm.
    Estimates the value function for a given state.
    """
    def __init__(self, input_dim=16, hidden_dim=512):
        """
        Initialize the value network.
        
        Args:
            input_dim (int): Dimension of input features (16 as per the paper)
            hidden_dim (int): Dimension of hidden layers (512 as per the paper)
        """
        super(ValueNetwork, self).__init__()
        
        # Three hidden layers with 512 units each as specified in the paper
        self.fc1 = nn.Linear(input_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, hidden_dim)
        
        # Output layer for value
        self.value_layer = nn.Linear(hidden_dim, 1)
        
    def forward(self, x):
        """
        Forward pass through the network.
        
        Args:
            x (torch.Tensor): Input tensor of environmental perception parameters
            
        Returns:
            torch.Tensor: Estimated value
        """
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        
        value = self.value_layer(x)
        
        return value

class MAPPOAgent:
    """
    Implementation of the MAPPO agent for the MAPPO-IK algorithm.
    """
    def __init__(self, state_dim=16, action_dim=6, hidden_dim=512, lr=2.0e-4):
        """
        Initialize the MAPPO agent.
        
        Args:
            state_dim (int): Dimension of state space
            action_dim (int): Dimension of action space
            hidden_dim (int): Dimension of hidden layers
            lr (float): Learning rate
        """
        self.policy = PolicyNetwork(state_dim, hidden_dim, action_dim)
        self.value = ValueNetwork(state_dim, hidden_dim)
        
        self.policy_optimizer = torch.optim.Adam(self.policy.parameters(), lr=lr)
        self.value_optimizer = torch.optim.Adam(self.value.parameters(), lr=lr)
        
        # PPO clip parameter
        self.clip_param = 0.2
        
        # Value function coefficient
        self.vf_coef = 0.5
        
        # Entropy coefficient
        self.entropy_coef = 0.01
        
    def update(self, states, actions, old_log_probs, returns, advantages):
        """
        Update the policy and value networks using PPO.
        
        Args:
            states (torch.Tensor): Batch of states
            actions (torch.Tensor): Batch of actions
            old_log_probs (torch.Tensor): Batch of log probabilities from old policy
            returns (torch.Tensor): Batch of returns
            advantages (torch.Tensor): Batch of advantages
            
        Returns:
            tuple: (policy_loss, value_loss, entropy)
        """
        # Get current policy distribution
        mean, log_std = self.policy(states)
        std = log_std.exp()
        normal = Normal(mean, std)
        
        # Calculate log probabilities and entropy
        new_log_probs = normal.log_prob(actions).sum(dim=-1)
        entropy = normal.entropy().mean()
        
        # Calculate ratio for PPO
        ratio = torch.exp(new_log_probs - old_log_probs)
        
        # Calculate surrogate losses
        surr1 = ratio * advantages
        surr2 = torch.clamp(ratio, 1.0 - self.clip_param, 1.0 + self.clip_param) * advantages
        
        # Calculate policy loss
        policy_loss = -torch.min(surr1, surr2).mean()
        
        # Calculate value loss
        value_pred = self.value(states).squeeze()
        value_loss = F.mse_loss(value_pred, returns)
        
        # Calculate total loss
        loss = policy_loss + self.vf_coef * value_loss - self.entropy_coef * entropy
        
        # Update policy
        self.policy_optimizer.zero_grad()
        self.value_optimizer.zero_grad()
        loss.backward()
        self.policy_optimizer.step()
        self.value_optimizer.step()
        
        return policy_loss.item(), value_loss.item(), entropy.item()
    
    def act(self, state, deterministic=False):
        """
        Select an action based on the current state.
        
        Args:
            state (numpy.ndarray): Current state
            deterministic (bool): If True, select the mean action
            
        Returns:
            tuple: (action, log_prob, entropy)
        """
        state = torch.FloatTensor(state).unsqueeze(0)
        with torch.no_grad():
            action, log_prob, entropy = self.policy.get_action(state, deterministic)
        
        return action.squeeze().numpy(), log_prob, entropy
    
    def evaluate(self, state):
        """
        Evaluate the value of a state.
        
        Args:
            state (numpy.ndarray): Current state
            
        Returns:
            float: Estimated value
        """
        state = torch.FloatTensor(state).unsqueeze(0)
        with torch.no_grad():
            value = self.value(state)
        
        return value.item()
    
    def save(self, path):
        """
        Save the model parameters.
        
        Args:
            path (str): Path to save the model
        """
        torch.save({
            'policy_state_dict': self.policy.state_dict(),
            'value_state_dict': self.value.state_dict(),
            'policy_optimizer_state_dict': self.policy_optimizer.state_dict(),
            'value_optimizer_state_dict': self.value_optimizer.state_dict(),
        }, path)
    
    def load(self, path):
        """
        Load the model parameters.
        
        Args:
            path (str): Path to load the model from
        """
        checkpoint = torch.load(path)
        self.policy.load_state_dict(checkpoint['policy_state_dict'])
        self.value.load_state_dict(checkpoint['value_state_dict'])
        self.policy_optimizer.load_state_dict(checkpoint['policy_optimizer_state_dict'])
        self.value_optimizer.load_state_dict(checkpoint['value_optimizer_state_dict'])
