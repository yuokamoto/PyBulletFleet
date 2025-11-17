#!/usr/bin/env python3
"""
robot_urdf_demo.py
Demo showing Agent class with both Mesh and URDF support.
"""
import os
import sys
import pybullet as p
import time

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.robot import Agent, Pose, AgentSpawnParams

# Initialize simulation
params = SimulationParams(
    gui=True,
    timestep=0.01
)
sim_core = MultiRobotSimulationCore(params)

# Create mesh robot
print("\n=== Creating Mesh Agent ===")
mesh_robot = Agent.from_mesh(
    mesh_path=os.path.join(os.path.dirname(__file__), '../robots/mobile_robot.urdf'),  # Using URDF path as fallback
    position=[0, 0, 0.5],
    orientation=[0, 0, 0, 1],
    max_vel=1.0
)
print(f"Mesh Agent: {mesh_robot}")

# Create URDF robot (arm)
print("\n=== Creating URDF Agent ===")
urdf_robot = Agent.from_urdf(
    urdf_path=os.path.join(os.path.dirname(__file__), '../robots/arm_robot.urdf'),
    position=[2, 0, 0],
    orientation=[0, 0, 0, 1],
    use_fixed_base=True
)
print(f"URDF Agent: {urdf_robot}")
print(f"Number of joints: {urdf_robot.get_num_joints()}")

# Test attach/detach (inherited from SimObject)
print("\n=== Testing SimObject features (attach/detach) ===")
print(f"Mesh agent has attached_objects: {hasattr(mesh_robot, 'attached_objects')}")
print(f"URDF agent has attached_objects: {hasattr(urdf_robot, 'attached_objects')}")

# Set goal for mesh robot
print("\n=== Setting goal for mesh agent ===")
goal = Pose.from_xyz(3, 3, 0.5)
mesh_robot.set_goal_pose(goal)

# Control URDF robot joints
print("\n=== Controlling URDF agent joints ===")
import numpy as np
joint_targets = [np.sin(time.time()) for _ in range(urdf_robot.get_num_joints())]
urdf_robot.set_all_joint_targets(joint_targets)

# Run simulation
print("\n=== Running simulation ===")
for i in range(500):
    # Update mesh robot (goal-based navigation)
    mesh_robot.update(sim_core.params.timestep)
    
    # Update URDF joint targets (oscillate)
    if i % 10 == 0:
        t = i * sim_core.params.timestep
        joint_targets = [0.5 * np.sin(t + j * 0.5) for j in range(urdf_robot.get_num_joints())]
        urdf_robot.set_all_joint_targets(joint_targets)
    
    sim_core.step_once()
    
    if i % 50 == 0:
        print(f"Step {i}: Mesh agent pos={mesh_robot.get_pose().position[:2]}, "
              f"URDF joint[0]={urdf_robot.get_joint_state(0)[0]:.2f}")

# Get final states before disconnecting
final_mesh_pos = mesh_robot.get_pose().position[:2]
final_urdf_joints = [urdf_robot.get_joint_state(i)[0] for i in range(urdf_robot.get_num_joints())]

print("\n=== Demo complete ===")
print(f"Final mesh agent pose: {final_mesh_pos}")
print(f"Final URDF agent joints: {final_urdf_joints}")
