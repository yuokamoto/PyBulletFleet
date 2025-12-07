#!/usr/bin/env python3
"""
Simple test to verify robot spawning and movement
"""

import time

import pybullet as p
import pybullet_data

from pybullet_fleet.agent import Agent
from pybullet_fleet.sim_object import Pose

# Connect to PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# Load ground plane
planeId = p.loadURDF("plane.urdf")

# Test 1: Create a simple cube robot
print("Creating robot...")
robot = Agent.from_mesh(
    mesh_path="mesh/cube.obj",
    pose=Pose.from_xyz(0.0, 0.0, 0.5),
    mesh_scale=[0.3, 0.3, 0.3],
    collision_half_extents=[0.15, 0.15, 0.15],
    rgba_color=[1.0, 0.0, 0.0, 1.0],  # Red
    base_mass=1.0,
    max_linear_vel=1.0,
    max_linear_accel=2.0,
    motion_mode="omnidirectional",
)

print(f"Robot created with body_id: {robot.body_id}")

# Check robot position
pos, orn = p.getBasePositionAndOrientation(robot.body_id)
print(f"Initial position: {pos}")
print(f"Initial orientation: {orn}")

# Set camera to look at robot
p.resetDebugVisualizerCamera(cameraDistance=3.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

# Test 2: Set a goal and move
print("\nSetting goal to (2, 0, 0.5)...")
robot.set_goal_pose(Pose.from_xyz(2.0, 0.0, 0.5))

print(f"Goal set. is_moving: {robot.is_moving}")
print(f"Goal pose: {robot.goal_pose.position if robot.goal_pose else None}")

# Simulation loop
dt = 1.0 / 240.0
for i in range(2000):  # Run for ~8 seconds
    # Update robot
    robot.update(dt)

    # Step simulation
    p.stepSimulation()

    # Print status every second
    if i % 240 == 0:
        pos, _ = p.getBasePositionAndOrientation(robot.body_id)
        print(f"[{i*dt:.1f}s] Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), is_moving: {robot.is_moving}")

    time.sleep(dt)

print("\nTest complete!")
p.disconnect()
