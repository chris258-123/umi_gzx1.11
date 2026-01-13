#!/usr/bin/env python3
"""
Reset Franka robot to initial joint positions using Polymetis directly.

Usage:
    python reset_franka_direct.py
"""

import torch
from polymetis import RobotInterface

# Initial joint positions from FrankaInterface

# Connect to robot
robot = RobotInterface(ip_address="localhost")

# Get current joint positions
current_joints = robot.get_joint_positions()
print(f"Current joints: {current_joints}")
