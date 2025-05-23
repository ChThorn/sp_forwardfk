# MAPPO-IK: Inverse Kinematics Solution for 6-DOF Manipulator

This repository contains the implementation of the MAPPO-IK algorithm described in the paper "Inverse kinematics solution and control method of 6-degree-of-freedom manipulator based on deep reinforcement learning".

## Project Structure

- `kinematic_model/`: Implementation of the DH parameter-based kinematic model
- `reinforcement_learning/`: MAPPO algorithm implementation
- `simulation/`: Unity simulation environment setup (instructions only)
- `evaluation/`: Tools for evaluating the trained model
- `utils/`: Utility functions for calculations and visualizations

## Requirements

- Python 3.8+
- PyTorch 1.8+
- NumPy
- Matplotlib
- Unity 2020.3+ (for simulation environment)
- Unity ML-Agents 2.0+

## Implementation Notes

This implementation focuses on reproducing the core MAPPO-IK algorithm as described in the paper. The full reproduction would require Unity for the simulation environment, but we provide a simplified simulation that can run without Unity for testing purposes.

For complete reproduction including the digital twin system, additional hardware and software would be required as specified in the paper.
