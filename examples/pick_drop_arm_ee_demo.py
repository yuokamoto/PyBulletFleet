#!/usr/bin/env python3
"""
pick_drop_arm_ee_demo.py
Demo: Robot arm picks and drops a box using EE position control (low-level, without Actions).

Drives the end-effector directly via move_end_effector() (IK) in a callback state machine.

Action version: pick_drop_arm_ee_action_demo.py
Joint-target variant (low-level): pick_drop_arm_demo.py
Joint-target variant (Action): pick_drop_arm_action_demo.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pybullet as p
from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams

# Simulation setup (kinematic mode, fast execution)
params = SimulationParams(gui=True, timestep=0.1, physics=False, target_rtf=10)
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
            reachable = arm_agent.move_end_effector(pos)
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
