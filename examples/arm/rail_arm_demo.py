#!/usr/bin/env python3
"""
rail_arm_demo.py
Demo: Rail arm (prismatic + revolute) picks a box from a high shelf and drops
it at a low position, then reverses — showcasing how the prismatic rail
extends the arm's vertical workspace.

Uses EE position control (PoseAction / PickAction / DropAction with
ee_target_position) so that the IK solver decides the optimal rail height
and arm joint configuration automatically.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
import argparse
import pybullet as p
from pybullet_fleet.action import JointAction, PoseAction, PickAction, DropAction, WaitAction
from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams

_parser = argparse.ArgumentParser(description="Rail arm pick/drop demo")
_parser.add_argument("--duration", type=float, default=None, help="Simulation duration in seconds (default: run forever)")
_parser.add_argument("--rtf", type=float, default=None, help="Target real-time factor override")
_args = _parser.parse_args()

# ---------------------------------------------------------------------------
# Simulation setup
# ---------------------------------------------------------------------------

params = SimulationParams(
    gui=True, timestep=0.1, physics=False, target_rtf=_args.rtf if _args.rtf is not None else 1, log_level="info"
)
sim_core = MultiRobotSimulationCore(params)

# Spawn rail arm (fixed base)
rail_arm_urdf = os.path.join(os.path.dirname(__file__), "../../robots/rail_arm_robot.urdf")
agent = Agent.from_urdf(
    urdf_path=rail_arm_urdf,
    pose=Pose.from_xyz(0, 0, 0),
    use_fixed_base=True,
    sim_core=sim_core,
)

# Spawn box on the "high shelf"
HIGH_POS = [0.1, -0.3, 1.0]  # High pick position (rail must rise)
LOW_POS = [0.1, 0.3, 0.3]  # Low drop position (rail stays low)

box = SimObject.from_mesh(
    visual_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05], rgba_color=[1, 0, 0, 1]),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
    pose=Pose.from_xyz(*HIGH_POS),
    mass=0.0,
    sim_core=sim_core,
)

# End-effector link = last joint
EE_LINK = p.getNumJoints(agent.body_id) - 1

# Attachment offset (box above EE)
OFFSET_POSE = Pose.from_xyz(0, 0, 0.14)

# Home: arm extended on +X side (away from rail column at X=0)
HOME_POS = [0.3, 0.0, 0.75]

# Drop/pick poses for DropAction placement
BOX_HIGH_POSE = Pose.from_xyz(*HIGH_POS)
BOX_LOW_POSE = Pose.from_xyz(*LOW_POS)


# ---------------------------------------------------------------------------
# Action sequence
# ---------------------------------------------------------------------------


def create_action_sequence():
    """One complete cycle: high→low, then low→high."""

    # Cycle 1: Pick from high shelf, drop at low position
    cycle1 = [
        PickAction(
            target_object_id=box.body_id,
            use_approach=False,
            ee_target_position=HIGH_POS,
            attach_link=EE_LINK,
            attach_relative_pose=OFFSET_POSE,
        ),
        DropAction(
            drop_pose=BOX_LOW_POSE,
            use_approach=False,
            ee_target_position=LOW_POS,
        ),
        # Return home via explicit joint targets with per-joint tolerance.
        # Prismatic rail (metres) gets a tighter tolerance than revolute
        # joints (radians) since 0.05 rad ≈ 3° is fine, but 0.05 m = 5 cm
        # would be too coarse for precise rail positioning.
        JointAction(
            target_joint_positions={
                "rail_joint": 0.0,  # rail back to bottom (metres)
                "base_to_shoulder": 0.0,  # revolute home (radians)
                "shoulder_to_elbow": 0.0,
                "elbow_to_wrist": 0.0,
                "wrist_to_end": 0.0,
            },
            tolerance={
                "rail_joint": 0.005,  # ±5 mm for prismatic
                "base_to_shoulder": 0.05,  # ±0.05 rad for revolute
                "shoulder_to_elbow": 0.05,
                "elbow_to_wrist": 0.05,
                "wrist_to_end": 0.05,
            },
        ),
        WaitAction(duration=0.5, action_type="idle"),
    ]

    # Cycle 2: Pick from low position, return to high shelf
    cycle2 = [
        PickAction(
            target_object_id=box.body_id,
            use_approach=False,
            ee_target_position=LOW_POS,
            attach_link=EE_LINK,
            attach_relative_pose=OFFSET_POSE,
        ),
        DropAction(
            drop_pose=BOX_HIGH_POSE,
            use_approach=False,
            ee_target_position=HIGH_POS,
        ),
        # Return home via EE position (IK decides joint values)
        PoseAction(target_position=HOME_POS, tolerance=0.03),
        WaitAction(duration=0.5, action_type="idle"),
    ]

    return cycle1 + cycle2


def repeat_callback(sim_core, dt):
    """Repeat action sequence when queue is empty."""
    if agent.is_action_queue_empty():
        agent.add_action_sequence(create_action_sequence())


# ---------------------------------------------------------------------------
# Setup & Run
# ---------------------------------------------------------------------------

print("\n=== Rail Arm Pick/Drop Demo ===")
print("Prismatic rail + 4-DOF arm (5 joints total)")
print(f"Cycle 1: Pick from high {HIGH_POS} -> Drop at low {LOW_POS}")
print(f"Cycle 2: Pick from low {LOW_POS} -> Drop at high {HIGH_POS}")
print("=" * 50 + "\n")

agent.add_action_sequence(create_action_sequence())
sim_core.register_callback(repeat_callback, frequency=10)

sim_core.setup_camera(
    camera_config={
        "camera_mode": "manual",
        "camera_distance": 2.5,
        "camera_yaw": 45,
        "camera_pitch": -30,
        "camera_target": [0, 0, 0.5],
    }
)

print("Starting simulation... Press Ctrl+C to stop.\n")
sim_core.run_simulation(duration=_args.duration)
