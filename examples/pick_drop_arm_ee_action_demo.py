#!/usr/bin/env python3
"""
pick_drop_arm_ee_action_demo.py
Demo: Robot arm picks and drops a box using the Action system (EE position control).

Uses PoseAction (IK) / PickAction / DropAction with ee_target_position to
declaratively define the pick-drop sequence. The Action system handles execution.

Low-level version (without Actions): pick_drop_arm_ee_demo.py
Joint-target variant (Action): pick_drop_arm_action_demo.py
Joint-target variant (low-level): pick_drop_arm_demo.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pybullet as p
from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams
from pybullet_fleet.action import PoseAction, PickAction, DropAction, WaitAction

# Simulation setup (kinematic mode, fast execution)
params = SimulationParams(gui=True, timestep=0.1, physics=False, target_rtf=10, log_level="info")
sim_core = MultiRobotSimulationCore(params)

# Spawn robot arm (fixed base)
arm_urdf = os.path.join(os.path.dirname(__file__), "../robots/arm_robot.urdf")
arm_agent = Agent.from_urdf(
    urdf_path=arm_urdf,
    pose=Pose.from_xyz(0, 0, 0),
    use_fixed_base=True,
    sim_core=sim_core,
)

# Spawn box to pick/drop
box = SimObject.from_mesh(
    visual_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05], rgba_color=[1, 0, 0, 1]),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
    pose=Pose.from_xyz(0.0, -0.3, 0.1),
    mass=0.0,
    sim_core=sim_core,
)

# End-effector link = last link
EE_LINK = p.getNumJoints(arm_agent.body_id) - 1

# EE target positions (world frame)
HOME_POS = [0.0, 0.0, 0.75]  # Arm fully upright
PICK_POS = [0.0, -0.3, 0.3]  # Reach toward box (Y-)
DROP_POS = [0.0, 0.3, 0.3]  # Opposite side (Y+)

# Box positions (for DropAction placement)
BOX_PICK_POSE = Pose.from_xyz(0.0, -0.3, 0.1)
BOX_DROP_POSE = Pose.from_xyz(0.0, 0.3, 0.1)

# Attachment offset (box above EE)
OFFSET_POSE = Pose.from_xyz(0, 0, 0.14)


def create_action_sequence():
    """Create one complete pick-drop cycle (forward + reverse)."""

    # Cycle 1: Pick from Y-, Drop to Y+
    cycle1 = [
        # 1. Pick box (move EE to pick position via IK, then attach)
        PickAction(
            target_object_id=box.body_id,
            use_approach=False,
            ee_target_position=PICK_POS,
            attach_link=EE_LINK,
            attach_relative_pose=OFFSET_POSE,
        ),
        # 2. Drop box (move EE to drop position via IK, then detach)
        DropAction(
            drop_pose=BOX_DROP_POSE,
            use_approach=False,
            ee_target_position=DROP_POS,
        ),
        # 3. Return to home
        PoseAction(target_position=HOME_POS, tolerance=0.02),
        WaitAction(duration=0.5, action_type="idle"),
    ]

    # Cycle 2: Pick from Y+, Drop to Y- (reverse)
    cycle2 = [
        # 1. Pick box from Y+ (move EE via IK, then attach)
        PickAction(
            target_object_id=box.body_id,
            use_approach=False,
            ee_target_position=DROP_POS,
            attach_link=EE_LINK,
            attach_relative_pose=OFFSET_POSE,
        ),
        # 2. Drop box at Y- (move EE via IK, then detach)
        DropAction(
            drop_pose=BOX_PICK_POSE,
            use_approach=False,
            ee_target_position=PICK_POS,
        ),
        # 3. Return to home
        PoseAction(target_position=HOME_POS, tolerance=0.02),
        WaitAction(duration=0.5, action_type="idle"),
    ]

    return cycle1 + cycle2


def repeat_callback(sim_core, dt):
    """Repeat action sequence when queue is empty."""
    if arm_agent.is_action_queue_empty():
        arm_agent.add_action_sequence(create_action_sequence())


# Setup
print("\n=== Pick/Drop Arm EE Action Demo ===")
print("Uses PoseAction (IK) + PickAction/DropAction (ee_target_position)")
print(f"Robot arm: {arm_agent.get_num_joints()} joints, EE link: {EE_LINK}")
print(f"Cycle 1: Pick from {PICK_POS} -> Drop at {DROP_POS}")
print(f"Cycle 2: Pick from {DROP_POS} -> Drop at {PICK_POS}")
print("=" * 50 + "\n")

arm_agent.add_action_sequence(create_action_sequence())
sim_core.register_callback(repeat_callback, frequency=10)

# Camera setup
sim_core.setup_camera(
    camera_config={
        "camera_mode": "manual",
        "camera_distance": 2.0,
        "camera_yaw": 45,
        "camera_pitch": -30,
        "camera_target": [0, 0, 0.2],
    }
)

print("Starting simulation... Press Ctrl+C to stop.\n")
sim_core.run_simulation()
