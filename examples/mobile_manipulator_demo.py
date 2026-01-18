#!/usr/bin/env python3
"""
mobile_manipulator_demo.py
Demo: Single mobile manipulator robot (mobile base + arm)

Demonstrates:
- Mobile manipulator URDF (combined mobile base + arm)
- Hybrid motion: mobile base movement + arm manipulation
- Pick and drop using both mobility and manipulation
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import logging
import pybullet as p
import numpy as np

from pybullet_fleet.agent import Agent, AgentSpawnParams, MotionMode
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams, SimObjectSpawnParams
from pybullet_fleet.action import MoveAction, PickAction, DropAction, WaitAction, JointAction
from pybullet_fleet.geometry import Path

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(levelname)s:%(name)s:%(message)s",
)

print("\n" + "=" * 80)
print("MOBILE MANIPULATOR DEMO - Single Robot")
print("=" * 80)
print("\nDemonstrates:")
print("- Mobile manipulator (mobile base + 4-DOF arm)")
print("- Navigation to target position")
print("- Arm manipulation for pick and drop")
print("- Combined mobility and manipulation\n")

# Simulation setup
params = SimulationParams(gui=True, timestep=0.1, ignore_static_collision=True)
sim_core = MultiRobotSimulationCore(params)

# Spawn mobile manipulator
print("=== Spawning Mobile Manipulator ===")
mobile_manipulator_urdf = os.path.join(os.path.dirname(__file__), "../robots/mobile_manipulator.urdf")

spawn_params = AgentSpawnParams(
    urdf_path=mobile_manipulator_urdf,
    initial_pose=Pose.from_euler(0, 0, 0.3, yaw=0),
    motion_mode=MotionMode.DIFFERENTIAL,
    max_linear_vel=2.0,
    max_linear_accel=3.0,
    max_angular_vel=1.5,
    max_angular_accel=3.0,
    mass=1.0,  # Use URDF mass values for physics simulation (required for arm joint control)
    use_fixed_base=False,
)

robot = Agent.from_params(spawn_params, sim_core=sim_core)
print(f"✓ Spawned mobile manipulator at {robot.get_pose().position}")

# Print joint information
num_joints = p.getNumJoints(robot.body_id)
print("\n=== Robot Joint Information ===")
print(f"Total joints: {num_joints}")
for i in range(num_joints):
    joint_info = p.getJointInfo(robot.body_id, i)
    joint_name = joint_info[1].decode("utf-8")
    joint_type = joint_info[2]  # 0=REVOLUTE, 1=PRISMATIC, 4=FIXED
    type_str = {0: "REVOLUTE", 1: "PRISMATIC", 4: "FIXED"}.get(joint_type, "OTHER")
    print(f"  Joint {i}: {joint_name} ({type_str})")

# Identify arm joints (excluding wheels and fixed joints)
arm_joint_names = ["mount_to_shoulder", "shoulder_to_elbow", "elbow_to_wrist", "wrist_to_end"]
arm_joint_indices = []
for name in arm_joint_names:
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot.body_id, i)
        if joint_info[1].decode("utf-8") == name:
            arm_joint_indices.append(i)
            break

print(f"\nArm joint indices: {arm_joint_indices}")

# Find end effector link index
end_effector_link_index = -1
for i in range(num_joints):
    joint_info = p.getJointInfo(robot.body_id, i)
    link_name = joint_info[12].decode("utf-8")  # Child link name
    if link_name == "end_effector":
        end_effector_link_index = i
        break

print(f"End effector link index: {end_effector_link_index}")

# Store in robot user_data
robot.user_data["arm_joint_indices"] = arm_joint_indices
robot.user_data["end_effector_link_index"] = end_effector_link_index

# Spawn pallet
print("\n=== Spawning Pallet ===")
mesh_dir = os.path.join(os.path.dirname(__file__), "../mesh")
pallet_mesh_path = os.path.join(mesh_dir, "11pallet.obj")
pallet_orientation_quat = p.getQuaternionFromEuler([np.pi / 2, 0, 0])

pallet_position = [2.0, 0.0, 0.1]  # Pallet at (2, 0, 0.1)

pallet_params = SimObjectSpawnParams(
    visual_shape=ShapeParams(
        shape_type="mesh", mesh_path=pallet_mesh_path, mesh_scale=[0.5, 0.5, 0.5], rgba_color=[0.8, 0.6, 0.4, 1.0]
    ),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.4, 0.1]),
    initial_pose=Pose.from_pybullet(pallet_position, pallet_orientation_quat),
    mass=0.0,
    pickable=True,
)

pallet = SimObject.from_params(pallet_params, sim_core=sim_core)
print(f"✓ Spawned pallet at {pallet_position}")

# Define arm poses using joint name dictionary (to avoid specifying all 7 joints)
# Home pose (arm retracted)
arm_home_pose = {
    "mount_to_shoulder": np.pi / 2,
    "shoulder_to_elbow": np.pi / 2,
    "elbow_to_wrist": np.pi / 2,
    "wrist_to_end": 0.0,
}

# Pick pose (arm extended forward)
arm_pick_pose = {
    "mount_to_shoulder": np.pi / 2,
    "shoulder_to_elbow": np.pi / 4,
    "elbow_to_wrist": np.pi / 4,
    "wrist_to_end": 0.0,
}

# Carry pose (arm lifted)
arm_carry_pose = {
    "mount_to_shoulder": 0.0,
    "shoulder_to_elbow": -0.8,
    "elbow_to_wrist": 1.2,
    "wrist_to_end": 0.0,
}

print("\n=== Creating Action Sequence ===")
print("1. Move arm to home position")
print("2. Navigate to pallet position")
print("3. Pick pallet (arm extends automatically)")
print("4. Retract arm to carry pose")
print("5. Navigate to drop position")
print("6. Drop pallet (arm extends automatically)")
print("7. Retract arm to home pose")
print("8. Return to start position\n")

# Action sequence
actions = [
    # 1. Move arm to home position
    JointAction(
        target_joint_positions=arm_home_pose,
        max_force=100.0,
        tolerance=0.05,
    ),
    WaitAction(duration=0.5, action_type="arm_home"),
    # 2. Navigate to pallet position (1.5m in front of pallet)
    MoveAction(
        path=Path.from_positions([[1.3, 0.0, 0.3]]),
        final_orientation_align=False,
    ),
    WaitAction(duration=0.5, action_type="prepare_pick"),
    # 3. Pick pallet (with arm extension integrated)
    PickAction(
        target_object_id=pallet.body_id,
        use_approach=False,
        pick_offset=0.7,  # Pick at end effector
        attach_link=end_effector_link_index,
        attach_relative_pose=Pose.from_euler(0.0, 0.0, 0.0, roll=np.pi / 2, pitch=0, yaw=0),
        joint_targets=arm_pick_pose,  # Extend arm to pick pose
        joint_tolerance=0.05,
        joint_max_force=100.0,
    ),
    # 4. Retract arm to carry pose
    JointAction(
        target_joint_positions=arm_carry_pose,
        max_force=100.0,
        tolerance=0.05,
    ),
    WaitAction(duration=0.5, action_type="arm_carry"),
    # 5. Navigate to drop position
    MoveAction(
        path=Path.from_positions([[0.0, 2.0, 0.3]]),
        final_orientation_align=False,
    ),
    WaitAction(duration=0.5, action_type="prepare_drop"),
    # 6. Drop pallet (with arm extension integrated)
    DropAction(
        drop_position=[1.5, 2.0, 0.1],
        drop_orientation=list(pallet_orientation_quat),
        place_gently=True,
        use_approach=False,
        drop_offset=0.0,
        joint_targets=arm_pick_pose,  # Extend arm to drop pose
        joint_tolerance=0.05,
        joint_max_force=100.0,
    ),
    # 7. Retract arm to home pose
    JointAction(
        target_joint_positions=arm_home_pose,
        max_force=100.0,
        tolerance=0.05,
    ),
    WaitAction(duration=0.5, action_type="arm_retract"),
    # 8. Return to start position
    MoveAction(
        path=Path.from_positions([[0.0, 0.0, 0.3]]),
        final_orientation_align=True,
    ),
    WaitAction(duration=1.0, action_type="complete"),
]

# Add actions to robot
for action in actions:
    robot.add_action(action)

print(f"✓ Added {len(actions)} actions to robot")

# Run simulation
print("\n=== Starting Simulation ===")
print("Simulation running... Close GUI window to exit\n")

try:
    sim_core.run_simulation()
except KeyboardInterrupt:
    print("\nSimulation interrupted by user")

print("\n=== Simulation Complete ===\n")
