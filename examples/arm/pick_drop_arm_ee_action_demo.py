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
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams
from pybullet_fleet.action import PoseAction, PickAction, DropAction, WaitAction
from pybullet_fleet.robot_models import resolve_urdf, auto_detect_profile

parser = argparse.ArgumentParser(description="Robot arm pick & drop demo (Action system, EE position control)")
parser.add_argument("--robot", default="panda", help="Robot name (e.g. panda, kuka_iiwa, arm_robot) or URDF path")
args = parser.parse_args()

# ── Per-robot EE position presets ──
# "orn" = EE orientation quaternion [qx, qy, qz, qw].  None = position-only IK.
_DOWN = (1.0, 0.0, 0.0, 0.0)  # gripper pointing straight down (180° around X)
EE_PRESETS = {
    "arm_robot": {
        "pick": [0.3, 0.0, 0.3],
        "drop": [-0.3, 0.0, 0.3],
        "home": [0.0, 0.0, 0.75],
        "box_pick": [0.3, 0.0, 0.1],
        "box_drop": [-0.3, 0.0, 0.1],
        "orn": None,  # 4-DOF: position-only IK is more stable
    },
    "panda": {
        "pick": [0.5, 0.0, 0.4],
        "drop": [-0.5, 0.0, 0.4],
        "home": [0.0, 0.0, 1.5],
        "box_pick": [0.5, 0.0, 0.1],
        "box_drop": [-0.5, 0.0, 0.1],
        "orn": _DOWN,  # 7-DOF: constrain orientation for natural pose
    },
    "kuka_iiwa": {
        "pick": [0.5, 0.0, 0.4],
        "drop": [-0.5, 0.0, 0.4],
        "home": [0.0, 0.0, 1.2],
        "box_pick": [0.5, 0.0, 0.1],
        "box_drop": [-0.5, 0.0, 0.1],
        "orn": _DOWN,
    },
}
if args.robot not in EE_PRESETS:
    raise SystemExit(f"No preset for '{args.robot}'. Available: {', '.join(EE_PRESETS)}")
_P = EE_PRESETS[args.robot]
PICK_POS = list(_P["pick"])
DROP_POS = list(_P["drop"])
HOME_POS = list(_P["home"])
BOX_PICK_XYZ = _P["box_pick"]
BOX_DROP_XYZ = _P["box_drop"]
EE_ORN = _P["orn"]  # None or quaternion tuple

# Simulation setup (kinematic mode, fast execution)
params = SimulationParams(gui=True, timestep=0.1, physics=False, target_rtf=3, log_level="info")
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
print(f"  PICK={PICK_POS} DROP={DROP_POS} HOME={HOME_POS}")

# Spawn box to pick/drop
box = SimObject.from_mesh(
    visual_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05], rgba_color=[1, 0, 0, 1]),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
    pose=Pose.from_xyz(*BOX_PICK_XYZ),
    mass=0.0,
    sim_core=sim_core,
)

# End-effector link
EE_LINK = profile.ee_link_index if profile.ee_link_index >= 0 else arm_agent.get_num_joints() - 1
print(f"  EE link index: {EE_LINK} ({profile.ee_link_name})")

# Box positions (for DropAction placement — use EE height so box doesn't snap to ground)
BOX_PICK_POSE = Pose.from_xyz(*PICK_POS)
BOX_DROP_POSE = Pose.from_xyz(*DROP_POS)

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
            ee_target_orientation=EE_ORN,
            attach_link=EE_LINK,
            attach_relative_pose=OFFSET_POSE,
        ),
        # 2. Drop box (move EE to drop position via IK, then detach)
        DropAction(
            drop_pose=BOX_DROP_POSE,
            use_approach=False,
            ee_target_position=DROP_POS,
            ee_target_orientation=EE_ORN,
        ),
        # 3. Return to home
        PoseAction(target_position=HOME_POS, target_orientation=EE_ORN, tolerance=0.02),
        WaitAction(duration=0.5, action_type="idle"),
    ]

    # Cycle 2: Pick from Y+, Drop to Y- (reverse)
    cycle2 = [
        # 1. Pick box from Y+ (move EE via IK, then attach)
        PickAction(
            target_object_id=box.body_id,
            use_approach=False,
            ee_target_position=DROP_POS,
            ee_target_orientation=EE_ORN,
            attach_link=EE_LINK,
            attach_relative_pose=OFFSET_POSE,
        ),
        # 2. Drop box at Y- (move EE via IK, then detach)
        DropAction(
            drop_pose=BOX_PICK_POSE,
            use_approach=False,
            ee_target_position=PICK_POS,
            ee_target_orientation=EE_ORN,
        ),
        # 3. Return to home
        PoseAction(target_position=HOME_POS, target_orientation=EE_ORN, tolerance=0.02),
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
