#!/usr/bin/env python3
"""
Reset Franka robot to initial joint positions using Polymetis directly.

Usage:
    python reset_franka_direct.py
"""

import torch
from polymetis import RobotInterface

#Initial joint positions from FrankaInterface

initial_joint_positions = torch.Tensor([
    0.2193, 
    -0.4610, 
    -0.0269, 
    -2.8138,  
    0.1747,  
    2.7694,  
    0.4440,
])


# initial_joint_positions = torch.Tensor([
#     -1.3130, 
#     -1.0359,  
#     1.6974, 
#     -2.5287,  
#     2.1806,  
#     2.4662, 
#     -0.5177
# ])

print("Resetting Franka robot to initial joint positions...")
print(f"Target joints: {initial_joint_positions}")

# Connect to robot
robot = RobotInterface(ip_address="localhost")

# Get current joint positions
current_joints = robot.get_joint_positions()
print(f"Current joints: {current_joints}")

# Move to initial positions
robot.move_to_joint_positions(
    positions=initial_joint_positions,
    time_to_go=3.0
)

print("Reset complete!")
