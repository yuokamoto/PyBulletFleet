#!/usr/bin/env python3
"""
pick_drop_arm_demo.py
Demo: Robot arm picks and drops a box at the end-effector (link).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pybullet as p
from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams

# Simulation setup
params = SimulationParams(
    gui=True, timestep=0.1, physics=False, target_rtf=10
)  # for kinematics demo with faster execution (no physics means we can run faster than real-time)
# params = SimulationParams(gui=True, timestep=0.01, physics=True) # for physics-based pick/drop with gravity and dynamics
sim_core = MultiRobotSimulationCore(params)

# Spawn robot arm (fixed base)
arm_urdf = os.path.join(os.path.dirname(__file__), "../robots/arm_robot.urdf")
arm_agent = Agent.from_urdf(
    urdf_path=arm_urdf,
    pose=Pose.from_xyz(0, 0, 0),
    use_fixed_base=True,
    sim_core=sim_core,
)


# Target joint angles
pick_joints = [1.5, 1.5, 1.5, 0.0]  # Extend toward box
place_joints = [-1.5, 1.5, 1.5, 0.0]  # Extend to opposite side

# Spawn box to pick/drop
box_sim = SimObject.from_mesh(
    visual_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05], rgba_color=[1, 0, 0, 1]),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
    pose=Pose.from_xyz(0.3, 0, 0.1),
    mass=0.0,
    sim_core=sim_core,
)

# Pick/drop logic
PICK_LINK_INDEX = p.getNumJoints(arm_agent.body_id) - 1  # End-effector link


picked = False
# States: move_to_pick -> pick -> move_to_place -> drop -> move_to_init
#       -> move_to_place2 -> pick2 -> move_to_pick2 -> drop2 -> move_to_init2 (loop)
step_state = 0
JOINT_TOL = 0.05  # tolerance for joint target check (radians)
box_pick_pose = Pose.from_xyz(0.3, 0, 0.1)
box_place_pose = Pose.from_xyz(-0.3, 0, 0.1)
joint_init = [0.0, 0.0, 0.0, 0.0]
box_offset = 0.14
offset_pose = Pose.from_xyz(0, 0, box_offset)
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
        _set_targets_once(joint_init)
        box_sim.set_pose(box_pick_pose)
        if arm_agent.are_all_joints_at_targets(joint_init, tolerance=JOINT_TOL):
            _advance(1, "At init -> moving to pick")

    elif step_state == 1:
        # Move arm to pick position
        _set_targets_once(pick_joints)
        if arm_agent.are_all_joints_at_targets(pick_joints, tolerance=JOINT_TOL):
            arm_agent.attach_object(box_sim, parent_link_index=PICK_LINK_INDEX, relative_pose=offset_pose)
            picked = True
            _advance(2, "Picked box (attached)")

    elif step_state == 2:
        # Move arm to place position (box follows via attach)
        _set_targets_once(place_joints)
        if arm_agent.are_all_joints_at_targets(place_joints, tolerance=JOINT_TOL):
            arm_agent.detach_object(box_sim)
            picked = False
            _advance(3, "Dropped box (detached)")

    elif step_state == 3:
        # Return to initial pose
        _set_targets_once(joint_init)
        if arm_agent.are_all_joints_at_targets(joint_init, tolerance=JOINT_TOL):
            _advance(4, "At init -> starting reverse")

    elif step_state == 4:
        # Place box at place position, move to initial
        _set_targets_once(joint_init)
        box_sim.set_pose(box_place_pose)
        if arm_agent.are_all_joints_at_targets(joint_init, tolerance=JOINT_TOL):
            _advance(5, "Moving to place (reverse)")

    elif step_state == 5:
        # Move arm to place position to pick box
        _set_targets_once(place_joints)
        if arm_agent.are_all_joints_at_targets(place_joints, tolerance=JOINT_TOL):
            arm_agent.attach_object(box_sim, parent_link_index=PICK_LINK_INDEX, relative_pose=offset_pose)
            picked = True
            _advance(6, "Picked box (attached, reverse)")

    elif step_state == 6:
        # Move arm to pick position (carrying box back)
        _set_targets_once(pick_joints)
        if arm_agent.are_all_joints_at_targets(pick_joints, tolerance=JOINT_TOL):
            arm_agent.detach_object(box_sim)
            picked = False
            _advance(7, "Dropped box (detached, reverse)")

    elif step_state == 7:
        # Return to initial pose, then loop
        _set_targets_once(joint_init)
        if arm_agent.are_all_joints_at_targets(joint_init, tolerance=JOINT_TOL):
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
