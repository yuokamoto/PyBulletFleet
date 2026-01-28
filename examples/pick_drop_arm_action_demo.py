#!/usr/bin/env python3
"""
pick_drop_arm_action_demo.py
Demo: Robot arm picks and drops a box using Action-based interface.

Demonstrates:
- JointAction: Move arm joints to target positions
- PickAction: Pick object and attach to end-effector link
- DropAction: Drop object at target position
- Action sequence management
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
params = SimulationParams(gui=True, timestep=0.01, physics=True)
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
joint_init = [0.0, 0.0, 0.0, 0.0]  # Initial/neutral position
pick_joints = [1.5, 1.5, 1.5, 0.0]  # Extend toward box (right side)
place_joints = [-1.5, 1.5, 1.5, 0.0]  # Extend to opposite side (left)

# Pick/drop positions
box_pick_pose = Pose.from_xyz(0.3, 0, 0.1)
box_place_pose = Pose.from_xyz(-0.3, 0, 0.1)

# End-effector link index
PICK_LINK_INDEX = p.getNumJoints(arm_agent.body_id) - 1

# Offset for attachment (box positioned above end-effector)
box_offset = 0.14
offset_pose = Pose.from_xyz(0, 0, box_offset)

# Create action sequence
# Cycle 1: Pick from right, drop on left
# Cycle 2: Pick from left, drop on right
# Then repeat


def create_action_sequence():
    """Create one complete pick-drop cycle."""

    # Cycle 1: Pick from right, drop on left
    actions_cycle1 = [
        # 1. Move to initial position and place box at pick location
        JointAction(target_joint_positions=joint_init, tolerance=0.05),
        # 2. Move arm to pick position
        JointAction(target_joint_positions=pick_joints, tolerance=0.05),
        # 3. Pick box from right side
        PickAction(
            target_object_id=box_sim.body_id,
            use_approach=False,  # No approach needed for arm (already positioned)
            attach_link=PICK_LINK_INDEX,
            attach_relative_pose=offset_pose,
        ),
        # 4. Move arm to place position (with box attached)
        JointAction(target_joint_positions=place_joints, tolerance=0.05),
        # 5. Drop box on left side
        DropAction(
            drop_position=box_place_pose.position,
            use_approach=False,  # No approach needed for arm
        ),
        # 6. Return to initial position
        JointAction(target_joint_positions=joint_init, tolerance=0.05),
        # 7. Small wait between cycles
        WaitAction(duration=0.5, action_type="idle"),
    ]

    # Cycle 2: Pick from left, drop on right
    actions_cycle2 = [
        # 1. Move arm to place position (where box was dropped)
        JointAction(target_joint_positions=place_joints, tolerance=0.05),
        # 2. Pick box from left side
        PickAction(
            target_object_id=box_sim.body_id,
            use_approach=False,
            attach_link=PICK_LINK_INDEX,
            attach_relative_pose=offset_pose,
        ),
        # 3. Move arm to pick position (with box attached)
        JointAction(target_joint_positions=pick_joints, tolerance=0.05),
        # 4. Drop box on right side
        DropAction(
            drop_position=box_pick_pose.position,
            use_approach=False,
        ),
        # 5. Return to initial position
        JointAction(target_joint_positions=joint_init, tolerance=0.05),
        # 6. Small wait before repeating
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
p.resetDebugVisualizerCamera(2.0, 45, -30, [0, 0, 0.2])

# Run simulation
print("\nStarting simulation...")
print("Watch the arm pick and drop the box in a continuous cycle.")
print("Press Ctrl+C to stop.\n")

sim_core.run_simulation()
