#!/usr/bin/env python3
"""
mobile_manipulator_demo.py
Demo: Single mobile manipulator robot (mobile base + arm) in kinematic mode.

Demonstrates:
- Mobile manipulator URDF (combined mobile base + 4-DOF arm)
- Kinematic mode (mass=0) — base teleportation + joint interpolation
- Sequential action chain: navigate → pick → carry → navigate → drop
- Attached object following EE link during arm motion and base movement
- IK-based pick/drop using ee_target_position (Part 2)
- Explicit ik_joint_names to restrict IK to arm joints only
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import logging
import pybullet as p
import math

from pybullet_fleet.agent import Agent, AgentSpawnParams, IKParams, MotionMode
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams
from pybullet_fleet.action import (
    MoveAction,
    PickAction,
    DropAction,
    WaitAction,
    JointAction,
)
from pybullet_fleet.geometry import Path

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(levelname)s:%(name)s:%(message)s",
)

print("\n" + "=" * 80)
print("MOBILE MANIPULATOR DEMO — Kinematic Mode")
print("=" * 80)
print("\nPart 1 — Joint-target pick/drop:")
print("  Arm poses defined as joint angle dicts")
print("Part 2 — IK pick/drop:")
print("  ee_target_position solved by inverse kinematics\n")

# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------
params = SimulationParams(gui=True, timestep=0.1, physics=False, target_rtf=3)
sim_core = MultiRobotSimulationCore(params)

# ---------------------------------------------------------------------------
# Robot  (ik_joint_names tells IK to only solve arm joints, not wheels)
# ---------------------------------------------------------------------------
mobile_manipulator_urdf = os.path.join(os.path.dirname(__file__), "../robots/mobile_manipulator.urdf")

ARM_JOINT_NAMES = (
    "mount_to_shoulder",
    "shoulder_to_elbow",
    "elbow_to_wrist",
    "wrist_to_end",
)

spawn_params = AgentSpawnParams(
    urdf_path=mobile_manipulator_urdf,
    initial_pose=Pose.from_euler(0, 0, 0.3, yaw=0),
    motion_mode=MotionMode.DIFFERENTIAL,
    max_linear_vel=2.0,
    max_linear_accel=3.0,
    max_angular_vel=1.5,
    max_angular_accel=3.0,
    mass=0.0,  # kinematic mode
    use_fixed_base=False,
    ik_params=IKParams(ik_joint_names=ARM_JOINT_NAMES),
)

robot = Agent.from_params(spawn_params, sim_core=sim_core)
print(f"Spawned mobile manipulator at {robot.get_pose().position}")

# Print joint layout
num_joints = p.getNumJoints(robot.body_id)
print(f"\nJoints ({num_joints} total):")
for i in range(num_joints):
    info = p.getJointInfo(robot.body_id, i)
    name = info[1].decode("utf-8")
    jtype = {0: "REV", 1: "PRIS", 4: "FIXED"}.get(info[2], "?")
    print(f"  {i}: {name} ({jtype}) -> {info[12].decode('utf-8')}")

# End-effector link index
end_effector_link_index = -1
for i in range(num_joints):
    if p.getJointInfo(robot.body_id, i)[12].decode("utf-8") == "end_effector":
        end_effector_link_index = i
        break
print(f"End-effector link index: {end_effector_link_index}")

# ---------------------------------------------------------------------------
# Target box (small, graspable by EE)
# ---------------------------------------------------------------------------
BOX_START = [1.5, 0.0, 0.8]

box = SimObject.from_mesh(
    visual_shape=ShapeParams(
        shape_type="box",
        half_extents=[0.05, 0.05, 0.05],
        rgba_color=[0.9, 0.2, 0.2, 1.0],
    ),
    collision_shape=ShapeParams(
        shape_type="box",
        half_extents=[0.05, 0.05, 0.05],
    ),
    pose=Pose.from_xyz(*BOX_START),
    mass=0.0,
    sim_core=sim_core,
    pickable=True,
)
print(f"Spawned box 1 (red) at {BOX_START}")

# Box 2 for the IK section
BOX2_START = [0.5, -1.5, 0.6]

box2 = SimObject.from_mesh(
    visual_shape=ShapeParams(
        shape_type="box",
        half_extents=[0.05, 0.05, 0.05],
        rgba_color=[0.2, 0.5, 0.9, 1.0],
    ),
    collision_shape=ShapeParams(
        shape_type="box",
        half_extents=[0.05, 0.05, 0.05],
    ),
    pose=Pose.from_xyz(*BOX2_START),
    mass=0.0,
    sim_core=sim_core,
    pickable=True,
)
print(f"Spawned box 2 (blue) at {BOX2_START}")

# ---------------------------------------------------------------------------
# Arm poses (dict targets — only arm joints, wheels untouched)
#
# Joint axes:
#   mount_to_shoulder  — Z (yaw)       pi/2 = face +X (forward)
#   shoulder_to_elbow  — X (pitch)     positive = tilt forward
#   elbow_to_wrist     — X (pitch)     positive = tilt forward
#   wrist_to_end       — Z (roll)
#
# With robot at (0.9, 0, 0.3) and box at (1.5, 0, 0.8):
#   pick  se=1.2 ew=1.0  ->  EE ~ (1.50, 0, 0.80)
# ---------------------------------------------------------------------------
ARM_HOME = {
    "mount_to_shoulder": math.pi / 2,  # face forward
    "shoulder_to_elbow": math.pi / 4,
    "elbow_to_wrist": math.pi / 4,
    "wrist_to_end": 0.0,
}

ARM_PICK = {
    "mount_to_shoulder": math.pi / 2,
    "shoulder_to_elbow": 1.2,
    "elbow_to_wrist": 1.0,
    "wrist_to_end": 0.0,
}

ARM_CARRY = {
    "mount_to_shoulder": math.pi / 2,
    "shoulder_to_elbow": -0.3,
    "elbow_to_wrist": 0.8,
    "wrist_to_end": 0.0,
}

# ARM_DROP: shoulder=0 extends arm along body forward direction.
# At MOVE_NEAR_DROP the body faces ~135° (toward drop area), so
# shoulder=0 reaches forward from the base toward the drop point.
ARM_DROP = {
    "mount_to_shoulder": 0.0,
    "shoulder_to_elbow": 1.2,
    "elbow_to_wrist": 1.0,
    "wrist_to_end": 0.0,
}

# ---------------------------------------------------------------------------
# Action sequence
# ---------------------------------------------------------------------------
MOVE_NEAR_BOX = [0.9, 0.0, 0.3]  # base position near box
MOVE_NEAR_DROP = [0.0, 0.9, 0.3]  # base position near drop point

print("\n--- Part 1: Joint-target pick/drop ---")
print("  1. Arm -> home (face forward, straight up)")
print("  2. Navigate near box")
print("  3. Pick box (arm extends to pick pose)")
print("  4. Arm -> carry (retract with box)")
print("  5. Navigate near drop position")
print("  6. Drop box (arm extends)")
print("  7. Arm -> home")
print("  8. Return to start\n")

BOX_OFFSET = Pose.from_xyz(0, 0, 0.07)  # small offset above EE center
BOX_DROP = Pose.from_xyz(0.07, 1.33, 0.8)  # fallback (unused with drop_relative_pose)

actions = [
    # 1. Arm to home
    JointAction(target_joint_positions=ARM_HOME, tolerance=0.05),
    WaitAction(duration=0.3),
    # 2. Navigate near box
    MoveAction(
        path=Path.from_positions([MOVE_NEAR_BOX]),
        final_orientation_align=False,
    ),
    WaitAction(duration=0.3),
    # 3. Pick box
    PickAction(
        target_object_id=box.body_id,
        use_approach=False,
        pick_offset=0.5,
        attach_link=end_effector_link_index,
        attach_relative_pose=BOX_OFFSET,
        joint_targets=ARM_PICK,
        joint_tolerance=0.05,
        joint_max_force=100.0,
    ),
    WaitAction(duration=0.3),
    # 4. Arm to carry (box follows EE)
    JointAction(target_joint_positions=ARM_CARRY, tolerance=0.05),
    WaitAction(duration=0.3),
    # 5. Navigate to drop area (box follows base)
    MoveAction(
        path=Path.from_positions([MOVE_NEAR_DROP]),
        final_orientation_align=False,
    ),
    WaitAction(duration=0.3),
    # 6. Drop box (arm extends forward, box placed at EE position)
    DropAction(
        drop_pose=BOX_DROP,  # base navigation target (also used if drop_relative_pose is None)
        place_gently=True,
        use_approach=False,
        drop_offset=0.0,
        joint_targets=ARM_DROP,
        joint_tolerance=0.05,
        joint_max_force=100.0,
        drop_relative_pose=Pose.from_xyz(0, 0, 0),  # drop at current EE position
    ),
    WaitAction(duration=0.3),
    # 7. Arm to home
    JointAction(target_joint_positions=ARM_HOME, tolerance=0.05),
    WaitAction(duration=0.3),
    # 8. Return to start
    MoveAction(
        path=Path.from_positions([[0.0, 0.0, 0.3]]),
        final_orientation_align=True,
    ),
    WaitAction(duration=1.0),
]

for action in actions:
    robot.add_action(action)
print(f"Part 1: Added {len(actions)} actions")

# ---------------------------------------------------------------------------
# Part 2: IK-based pick/drop (ee_target_position)
# ---------------------------------------------------------------------------
print("\n--- Part 2: IK pick/drop ---")
print("  9.  Navigate near box 2")
print("  10. Pick box 2 (IK solves arm joints)")
print("  11. Arm -> carry")
print("  12. Navigate to drop area")
print("  13. Drop box 2 (IK solves arm joints)")
print("  14. Arm -> home")
print("  15. Return to start\n")

MOVE_NEAR_BOX2 = [0.0, -1.0, 0.3]  # base position near box 2
MOVE_IK_DROP = [-1.0, 0.0, 0.3]  # base position for IK drop

# IK targets — world-frame position where EE should reach
# box2 is at (0.5, -1.5, 0.6), robot will be at (0.0, -1.0, 0.3)
# → EE needs to reach (0.5, -1.5, 0.6) in world frame
IK_PICK_TARGET = BOX2_START  # [0.5, -1.5, 0.6]
# For drop: robot at (-1.0, 0, 0.3), arm forward → EE ~(-1.0+0.65, 0, 0.6)
IK_DROP_TARGET = [-0.35, 0.0, 0.6]

ik_actions = [
    # 9. Navigate near box 2
    MoveAction(
        path=Path.from_positions([MOVE_NEAR_BOX2]),
        final_orientation_align=False,
    ),
    WaitAction(duration=0.3),
    # 10. Pick box 2 via IK
    PickAction(
        target_object_id=box2.body_id,
        use_approach=False,
        pick_offset=0.5,
        attach_link=end_effector_link_index,
        attach_relative_pose=BOX_OFFSET,
        ee_target_position=IK_PICK_TARGET,
        ee_end_effector_link=end_effector_link_index,
        ee_tolerance=0.05,
    ),
    WaitAction(duration=0.3),
    # 11. Arm to carry (box follows EE)
    JointAction(target_joint_positions=ARM_CARRY, tolerance=0.05),
    WaitAction(duration=0.3),
    # 12. Navigate to IK drop area
    MoveAction(
        path=Path.from_positions([MOVE_IK_DROP]),
        final_orientation_align=False,
    ),
    WaitAction(duration=0.3),
    # 13. Drop box 2 via IK
    DropAction(
        drop_pose=Pose.from_xyz(*IK_DROP_TARGET),  # base navigation target near drop area
        place_gently=True,
        use_approach=False,
        drop_offset=0.0,
        ee_target_position=IK_DROP_TARGET,
        ee_end_effector_link=end_effector_link_index,
        ee_tolerance=0.05,
        drop_relative_pose=Pose.from_xyz(0, 0, 0),  # drop at current EE position
    ),
    WaitAction(duration=0.3),
    # 14. Arm to home
    JointAction(target_joint_positions=ARM_HOME, tolerance=0.05),
    WaitAction(duration=0.3),
    # 15. Return to start
    MoveAction(
        path=Path.from_positions([[0.0, 0.0, 0.3]]),
        final_orientation_align=True,
    ),
    WaitAction(duration=1.0),
]

for action in ik_actions:
    robot.add_action(action)
print(f"Part 2: Added {len(ik_actions)} actions")

# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
print("\nSimulation running ... close GUI window to exit.\n")

try:
    sim_core.run_simulation()
except KeyboardInterrupt:
    print("\nInterrupted")

print("\nDone.\n")
