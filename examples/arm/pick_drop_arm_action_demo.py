#!/usr/bin/env python3
"""
pick_drop_arm_action_demo.py
Demo: Robot arm picks and drops a box using the Action system (joint control).

Uses JointAction / PickAction / DropAction to declaratively define
the pick-drop sequence. The Action system handles execution and transitions.

Low-level version (without Actions): pick_drop_arm_demo.py
EE position variant (Action): pick_drop_arm_ee_action_demo.py
EE position variant (low-level): pick_drop_arm_ee_demo.py
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams
from pybullet_fleet.action import JointAction, PickAction, DropAction, WaitAction
from pybullet_fleet.robot_models import resolve_urdf, auto_detect_profile

parser = argparse.ArgumentParser(description="Robot arm pick & drop demo (Action system, joint control)")
parser.add_argument("--robot", default="panda", help="Robot name (e.g. panda, kuka_iiwa, arm_robot) or URDF path")
args = parser.parse_args()

# ── Per-robot joint presets (pick / place / init targets + box positions) ──
JOINT_PRESETS = {
    "arm_robot": {
        "pick": [1.5, 1.5, 1.5, 0.0],
        "place": [-1.5, 1.5, 1.5, 0.0],
        "init": [0.0, 0.0, 0.0, 0.0],
        "box_pick": [0.3, 0.0, 0.1],
        "box_place": [-0.3, 0.0, 0.1],
    },
    "panda": {
        "pick": [0.0, 0.4, 0.0, -1.5, 0.0, 1.9, 0.8, 0.0, 0.0, 0.04, 0.04, 0.0],
        "place": [3.14, 0.4, 0.0, -1.5, 0.0, 1.9, 0.8, 0.0, 0.0, 0.04, 0.04, 0.0],
        "init": [0.0, -0.8, 0.0, -2.3, 0.0, 1.6, 0.8, 0.0, 0.0, 0.04, 0.04, 0.0],
        "box_pick": [0.65, 0.0, 0.25],
        "box_place": [-0.65, 0.0, 0.25],
    },
    "kuka_iiwa": {
        "pick": [0.0, 0.5, 0.0, -1.4, 0.0, 1.2, 0.0],
        "place": [3.14, 0.5, 0.0, -1.4, 0.0, 1.2, 0.0],
        "init": [0.0, -0.5, 0.0, -1.5, 0.0, 0.7, 0.0],
        "box_pick": [0.65, 0.0, 0.25],
        "box_place": [-0.65, 0.0, 0.25],
    },
}
if args.robot not in JOINT_PRESETS:
    raise SystemExit(f"No preset for '{args.robot}'. Available: {', '.join(JOINT_PRESETS)}")
_P = JOINT_PRESETS[args.robot]
PICK_JOINTS = _P["pick"]
PLACE_JOINTS = _P["place"]
JOINT_INIT = _P["init"]
BOX_PICK_POSE = Pose.from_xyz(*_P["box_pick"])
BOX_PLACE_POSE = Pose.from_xyz(*_P["box_place"])

# Simulation setup
params = SimulationParams(
    gui=True, timestep=0.1, physics=False, target_rtf=1, log_level="info"
)  # for kinematics demo with faster execution (no physics means we can run faster than real-time)
# params = SimulationParams(gui=True, timestep=0.01, physics=True) # for physics-based pick/drop with gravity and dynamics
sim_core = MultiRobotSimulationCore(params)

# Spawn robot arm (fixed base)
arm_urdf = resolve_urdf(args.robot)
arm_agent = Agent.from_urdf(
    urdf_path=arm_urdf,
    pose=Pose.from_xyz(0, 0, 0),
    use_fixed_base=True,
    sim_core=sim_core,
)
profile = auto_detect_profile(arm_urdf, sim_core.client)
print(f"Using {args.robot}: EE={profile.ee_link_name}, joints={len(profile.movable_joint_names)}")
print(f"  PICK={[round(x,2) for x in PICK_JOINTS]}  PLACE={[round(x,2) for x in PLACE_JOINTS]}")

# Spawn box to pick/drop
box_sim = SimObject.from_mesh(
    visual_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05], rgba_color=[1, 0, 0, 1]),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
    pose=BOX_PICK_POSE,
    mass=0.0,
    sim_core=sim_core,
)

# End-effector link index
PICK_LINK_INDEX = profile.ee_link_index if profile.ee_link_index >= 0 else arm_agent.get_num_joints() - 1
print(f"  EE link index: {PICK_LINK_INDEX} ({profile.ee_link_name})")

# Offset for attachment (box positioned above end-effector)
BOX_OFFSET = 0.14
OFFSET_POSE = Pose.from_xyz(0, 0, BOX_OFFSET)

# Create action sequence
# Cycle 1: Pick from right, drop on left
# Cycle 2: Pick from left, drop on right
# Then repeat


def create_action_sequence():
    """Create one complete pick-drop cycle."""

    # Cycle 1: Pick from right, drop on left
    actions_cycle1 = [
        # 1. Pick box from right side (move joints to pick position, then attach)
        PickAction(
            target_object_id=box_sim.body_id,
            use_approach=False,
            joint_targets=PICK_JOINTS,
            joint_tolerance=0.05,
            attach_link=PICK_LINK_INDEX,
            attach_relative_pose=OFFSET_POSE,
        ),
        # 2. Drop box on left side (move joints to place position, then detach)
        DropAction(
            drop_pose=BOX_PLACE_POSE,
            use_approach=False,
            joint_targets=PLACE_JOINTS,
            joint_tolerance=0.05,
        ),
        # 3. Return to initial position
        JointAction(target_joint_positions=JOINT_INIT, tolerance=0.05),
        WaitAction(duration=0.5, action_type="idle"),
    ]

    # Cycle 2: Pick from left, drop on right
    actions_cycle2 = [
        # 1. Pick box from left side (move joints to place position, then attach)
        PickAction(
            target_object_id=box_sim.body_id,
            use_approach=False,
            joint_targets=PLACE_JOINTS,
            joint_tolerance=0.05,
            attach_link=PICK_LINK_INDEX,
            attach_relative_pose=OFFSET_POSE,
        ),
        # 2. Drop box on right side (move joints to pick position, then detach)
        DropAction(
            drop_pose=BOX_PICK_POSE,
            use_approach=False,
            joint_targets=PICK_JOINTS,
            joint_tolerance=0.05,
        ),
        # 3. Return to initial position
        JointAction(target_joint_positions=JOINT_INIT, tolerance=0.05),
        WaitAction(duration=0.5, action_type="idle"),
    ]

    # Combine both cycles into one sequence
    return actions_cycle1 + actions_cycle2


def repeat_callback(sim_core, dt):
    """Callback to repeat action sequence when queue is empty."""
    # Check if action queue is empty
    if arm_agent.is_action_queue_empty():
        # Add new sequence
        arm_agent.add_action_sequence(create_action_sequence())


# Setup initial sequence
print("\n=== Setting up Action Sequence ===")
print("Cycle 1: Pick from right (0.3, 0, 0.1) -> Drop on left (-0.3, 0, 0.1)")
print("Cycle 2: Pick from left (-0.3, 0, 0.1) -> Drop on right (0.3, 0, 0.1)")
print("=" * 50 + "\n")

arm_agent.add_action_sequence(create_action_sequence())

# Register callback to repeat sequence
sim_core.register_callback(repeat_callback, frequency=10)

print("Action sequence configured and started!")

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

# Run simulation
print("\nStarting simulation...")
print("Watch the arm pick and drop the box in a continuous cycle.")
print("Press Ctrl+C to stop.\n")

sim_core.run_simulation()
