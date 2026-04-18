#!/usr/bin/env python3
"""
pick_drop_arm_ee_demo.py
Demo: Robot arm picks and drops a box using EE position control (low-level, without Actions).

Drives the end-effector directly via move_end_effector() (IK) in a callback state machine.

Action version: pick_drop_arm_ee_action_demo.py
Joint-target variant (low-level): pick_drop_arm_demo.py
Joint-target variant (Action): pick_drop_arm_action_demo.py
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams
from pybullet_fleet.robot_models import resolve_model, auto_detect_profile

parser = argparse.ArgumentParser(description="Robot arm pick & drop demo (EE position control)")
parser.add_argument("--robot", default="panda", help="Robot name (e.g. panda, kuka_iiwa, arm_robot) or URDF path")
args = parser.parse_args()

# ── Per-robot EE position presets ──
# "orn" = EE orientation quaternion [qx, qy, qz, qw].  None = position-only IK.
# Robots with >= 6 DOF benefit from orientation constraints (more natural poses).
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
EE_ORN = _P["orn"]  # None or quaternion tuple

# Initialize simulation from YAML config
_CONFIG = os.path.join(os.path.dirname(__file__), "..", "..", "config", "config.yaml")
sim_core = MultiRobotSimulationCore.from_yaml(_CONFIG)

# Spawn robot arm (fixed base)
arm_urdf = resolve_model(args.robot)
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

# Attachment offset (box above EE)
OFFSET_POSE = Pose.from_xyz(0, 0, 0.14)
JOINT_TOL = 0.05  # tolerance for joint convergence check

# State machine
# Cycle 1: pick(Y-) -> drop(Y+) -> home
# Cycle 2: pick(Y+) -> drop(Y-) -> home -> loop
step_state = 0
_ee_set = False


def pick_drop_ee_callback(sim_core, dt):
    global step_state, _ee_set
    step = sim_core.step_count

    def _move_ee_once(pos):
        """Call move_end_effector() only once per state."""
        global _ee_set
        if not _ee_set:
            reachable = arm_agent.move_end_effector(pos, target_orientation=EE_ORN)
            _ee_set = True
            status = "reachable" if reachable else "best-effort"
            print(f"[STEP {step}] move_end_effector({pos}) -> {status}")

    def _at_target():
        """Check if joints have converged to last commanded targets."""
        return arm_agent.are_all_joints_at_targets(tolerance=JOINT_TOL)

    def _advance(next_state, msg):
        global step_state, _ee_set
        step_state = next_state
        _ee_set = False
        print(f"[STEP {step}] {msg}")

    # --- Cycle 1: Pick from Y-, Drop to Y+ ---
    if step_state == 0:
        _move_ee_once(PICK_POS)
        if _at_target():
            _advance(1, "At pick position -> picking")

    elif step_state == 1:
        arm_agent.attach_object(box, parent_link_index=EE_LINK, relative_pose=OFFSET_POSE)
        _advance(2, "Picked box -> moving to drop")

    elif step_state == 2:
        _move_ee_once(DROP_POS)
        if _at_target():
            _advance(3, "At drop position -> dropping")

    elif step_state == 3:
        arm_agent.detach_object(box)
        _advance(4, "Dropped box -> returning home")

    elif step_state == 4:
        _move_ee_once(HOME_POS)
        if _at_target():
            _advance(5, "At home -> starting reverse cycle")

    # --- Cycle 2: Pick from Y+, Drop to Y- ---
    elif step_state == 5:
        _move_ee_once(DROP_POS)
        if _at_target():
            _advance(6, "At Y+ position (reverse) -> picking")

    elif step_state == 6:
        arm_agent.attach_object(box, parent_link_index=EE_LINK, relative_pose=OFFSET_POSE)
        _advance(7, "Picked box (reverse) -> moving to Y-")

    elif step_state == 7:
        _move_ee_once(PICK_POS)
        if _at_target():
            _advance(8, "At Y- position (reverse) -> dropping")

    elif step_state == 8:
        arm_agent.detach_object(box)
        _advance(9, "Dropped box (reverse) -> returning home")

    elif step_state == 9:
        _move_ee_once(HOME_POS)
        if _at_target():
            _advance(0, "At home -> loop")


# Register callback
sim_core.register_callback(pick_drop_ee_callback, frequency=10)

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

print("Pick/Drop Arm EE Demo — move_end_effector() callback")
print(f"Robot arm: {arm_agent.get_num_joints()} joints, EE link: {EE_LINK}")
print(f"PICK: {PICK_POS}  DROP: {DROP_POS}  HOME: {HOME_POS}")
print()

sim_core.run_simulation()
