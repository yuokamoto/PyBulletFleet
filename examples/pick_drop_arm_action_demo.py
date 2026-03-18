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
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pybullet as p
from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams
from pybullet_fleet.action import JointAction, PickAction, DropAction, WaitAction

# Simulation setup
params = SimulationParams(
    gui=True, timestep=0.1, physics=False, target_rtf=10, log_level="info"
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

# Spawn box to pick/drop
box_sim = SimObject.from_mesh(
    visual_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05], rgba_color=[1, 0, 0, 1]),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
    pose=Pose.from_xyz(0.3, 0, 0.1),
    mass=0.0,
    sim_core=sim_core,
)

# Joint configurations
JOINT_INIT = [0.0, 0.0, 0.0, 0.0]  # Initial/neutral position
PICK_JOINTS = [1.5, 1.5, 1.5, 0.0]  # Extend toward box (right side)
PLACE_JOINTS = [-1.5, 1.5, 1.5, 0.0]  # Extend to opposite side (left)

# Pick/drop positions
BOX_PICK_POSE = Pose.from_xyz(0.3, 0, 0.1)
BOX_PLACE_POSE = Pose.from_xyz(-0.3, 0, 0.1)

# End-effector link index
PICK_LINK_INDEX = p.getNumJoints(arm_agent.body_id) - 1

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
