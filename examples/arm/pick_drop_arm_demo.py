#!/usr/bin/env python3
"""
pick_drop_arm_demo.py
Demo: Robot arm picks and drops a box using joint target control (low-level, without Actions).

Drives joints directly via set_all_joints_targets() in a callback state machine.

Action version: pick_drop_arm_action_demo.py
EE position variant (low-level): pick_drop_arm_ee_demo.py
EE position variant (Action): pick_drop_arm_ee_action_demo.py
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams
from pybullet_fleet.robot_models import resolve_model, auto_detect_profile

parser = argparse.ArgumentParser(description="Robot arm pick & drop demo (joint control)")
parser.add_argument("--robot", default="panda", help="Robot name (e.g. panda, kuka_iiwa, arm_robot) or URDF path")
parser.add_argument("--rtf", type=float, default=None, help="Target real-time factor override")
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
    gui=True, timestep=0.1, physics=False, target_rtf=args.rtf if args.rtf is not None else 3
)  # for kinematics demo with faster execution (no physics means we can run faster than real-time)
# params = SimulationParams(gui=True, timestep=0.01, physics=True) # for physics-based pick/drop with gravity and dynamics
sim_core = MultiRobotSimulationCore(params)

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
print(f"  PICK={[round(x,2) for x in PICK_JOINTS]}  PLACE={[round(x,2) for x in PLACE_JOINTS]}")

# Spawn box to pick/drop
box_sim = SimObject.from_mesh(
    visual_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05], rgba_color=[1, 0, 0, 1]),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
    pose=BOX_PICK_POSE,
    mass=0.0,
    sim_core=sim_core,
)

# Resolve EE link index
PICK_LINK_INDEX = profile.ee_link_index if profile.ee_link_index >= 0 else arm_agent.get_num_joints() - 1
print(f"  EE link index: {PICK_LINK_INDEX} ({profile.ee_link_name})")


picked = False
# States: move_to_pick -> pick -> move_to_place -> drop -> move_to_init
#       -> move_to_place2 -> pick2 -> move_to_pick2 -> drop2 -> move_to_init2 (loop)
step_state = 0
JOINT_TOL = 0.05  # tolerance for joint target check (radians)
BOX_OFFSET = 0.14
OFFSET_POSE = Pose.from_xyz(0, 0, BOX_OFFSET)
_targets_set = False  # Track whether targets were set for current state


def pick_drop_callback(sim_core, dt):
    global picked, step_state, _targets_set
    step = sim_core.step_count

    def _set_targets_once(targets):
        """Set joint targets only once per state transition (avoids redundant calls)."""
        global _targets_set
        if not _targets_set:
            arm_agent.set_all_joints_targets(targets)
            _targets_set = True

    def _advance(next_state, msg):
        """Advance to next state and reset target flag."""
        global step_state, _targets_set
        step_state = next_state
        _targets_set = False
        print(f"[STEP {step}] {msg}")

    if step_state == 0:
        # Move to initial pose, place box at pick position
        _set_targets_once(JOINT_INIT)
        box_sim.set_pose(BOX_PICK_POSE)
        if arm_agent.are_all_joints_at_targets(JOINT_INIT, tolerance=JOINT_TOL):
            _advance(1, "At init -> moving to pick")

    elif step_state == 1:
        # Move arm to pick position
        _set_targets_once(PICK_JOINTS)
        if arm_agent.are_all_joints_at_targets(PICK_JOINTS, tolerance=JOINT_TOL):
            arm_agent.attach_object(box_sim, parent_link_index=PICK_LINK_INDEX, relative_pose=OFFSET_POSE)
            picked = True
            _advance(2, "Picked box (attached)")

    elif step_state == 2:
        # Move arm to place position (box follows via attach)
        _set_targets_once(PLACE_JOINTS)
        if arm_agent.are_all_joints_at_targets(PLACE_JOINTS, tolerance=JOINT_TOL):
            arm_agent.detach_object(box_sim)
            picked = False
            _advance(3, "Dropped box (detached)")

    elif step_state == 3:
        # Return to initial pose
        _set_targets_once(JOINT_INIT)
        if arm_agent.are_all_joints_at_targets(JOINT_INIT, tolerance=JOINT_TOL):
            _advance(4, "At init -> starting reverse")

    elif step_state == 4:
        # Place box at place position, move to initial
        _set_targets_once(JOINT_INIT)
        box_sim.set_pose(BOX_PLACE_POSE)
        if arm_agent.are_all_joints_at_targets(JOINT_INIT, tolerance=JOINT_TOL):
            _advance(5, "Moving to place (reverse)")

    elif step_state == 5:
        # Move arm to place position to pick box
        _set_targets_once(PLACE_JOINTS)
        if arm_agent.are_all_joints_at_targets(PLACE_JOINTS, tolerance=JOINT_TOL):
            arm_agent.attach_object(box_sim, parent_link_index=PICK_LINK_INDEX, relative_pose=OFFSET_POSE)
            picked = True
            _advance(6, "Picked box (attached, reverse)")

    elif step_state == 6:
        # Move arm to pick position (carrying box back)
        _set_targets_once(PICK_JOINTS)
        if arm_agent.are_all_joints_at_targets(PICK_JOINTS, tolerance=JOINT_TOL):
            arm_agent.detach_object(box_sim)
            picked = False
            _advance(7, "Dropped box (detached, reverse)")

    elif step_state == 7:
        # Return to initial pose, then loop
        _set_targets_once(JOINT_INIT)
        if arm_agent.are_all_joints_at_targets(JOINT_INIT, tolerance=JOINT_TOL):
            _advance(0, "At init (loop)")


# Register callback
sim_core.register_callback(pick_drop_callback, frequency=10)

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

sim_core.run_simulation()
