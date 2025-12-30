#!/usr/bin/env python3
"""
pick_drop_mobile_100robots_demo.py
Demo: 100 mobile robots picking and dropping pallets using Action-based interface.

Demonstrates:
- AgentManager for grid-based agent spawning
- Multi-agent simulation with 100 mobile robots
- Bulk action management via AgentManager callback
- MoveAction, PickAction, DropAction coordination
- Two-area shuttle system: All robots move from Area A to Area B, then back
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pybullet as p
import numpy as np

from pybullet_fleet.agent import AgentSpawnParams, MotionMode
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams, SimObjectManager
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams, SimObjectSpawnParams
from pybullet_fleet.action import MoveAction, PickAction, DropAction, WaitAction
from pybullet_fleet.geometry import Path

# Simulation setup
params = SimulationParams(gui=True, timestep=0.1, ignore_structure_collision=True)
sim_core = MultiRobotSimulationCore(params)

# Create AgentManager
agent_manager = AgentManager(sim_core=sim_core, update_frequency=10.0)

# Create SimObjectManager for pallets
pallet_manager = SimObjectManager(sim_core=sim_core)

# Configuration
NUM_ROBOTS = 100  # Number of robots to spawn
GRID_SIZE = 10  # 10x10 = 100 robots
SPACING = 1.5  # Distance between robots (meters)

# Define two areas
# Area A: Left side (pickup area)
AREA_A_CENTER = [0, 0, 0]
AREA_A_GRID_OFFSET = [-5, -5, 0.3]  # Grid starts at (-5, -5)

# Area B: Right side (dropoff area)
AREA_B_CENTER = [30, 0, 0]
AREA_B_GRID_OFFSET = [25, -5, 0.3]  # Grid starts at (25, -5)

# Pallet spawn offset relative to robot
PALLET_OFFSET = [0.6, 0, 0.1]  # In front of robot


# Global state tracking
class SimulationState:
    direction = "A_to_B"  # "A_to_B" or "B_to_A"
    completed_robots = 0
    cycle_count = 0


state = SimulationState()

print("\n=== Spawning 100 Mobile Robots using AgentManager ===")

# Setup grid spawn parameters for Area A
grid_params_A = GridSpawnParams(
    x_min=0,
    x_max=GRID_SIZE - 1,
    y_min=0,
    y_max=GRID_SIZE - 1,
    z_min=0,
    z_max=0,  # 2D grid
    spacing=[SPACING, SPACING, 0.0],
    offset=AREA_A_GRID_OFFSET,
)

# Setup agent spawn parameters
mobile_urdf = os.path.join(os.path.dirname(__file__), "../robots/mobile_robot.urdf")
agent_spawn_params = AgentSpawnParams(
    urdf_path=mobile_urdf,
    motion_mode=MotionMode.DIFFERENTIAL,
    max_linear_vel=3.0,
    max_linear_accel=5.0,
    max_angular_vel=2.0,
    max_angular_accel=5.0,
    mass=0.0,  # Kinematic control
    use_fixed_base=False,
)

# Spawn all agents in grid using AgentManager
mobile_agents = agent_manager.spawn_agents_grid(
    num_agents=NUM_ROBOTS, grid_params=grid_params_A, spawn_params=agent_spawn_params
)

print(f"✓ Successfully spawned {len(mobile_agents)} robots in Area A")
print(f"  Area A center: {AREA_A_CENTER}")
print(f"  Area B center: {AREA_B_CENTER}\n")

# Spawn pallets for each robot using SimObjectManager
print("=== Spawning Pallets for Each Robot ===")
mesh_dir = os.path.join(os.path.dirname(__file__), "../mesh")
pallet_mesh_path = os.path.join(mesh_dir, "11pallet.obj")

pallet_orientation_quat = p.getQuaternionFromEuler([np.pi / 2, 0, 0])  # Horizontal orientation

# Create pallet spawn parameters list for batch spawning
pallet_params_list = []
for idx, robot in enumerate(mobile_agents):
    robot_pose = robot.get_pose()
    robot_pos = robot_pose.position

    # Pallet starts in front of robot
    pallet_position = [robot_pos[0] + PALLET_OFFSET[0], robot_pos[1] + PALLET_OFFSET[1], PALLET_OFFSET[2]]

    pallet_params = SimObjectSpawnParams(
        visual_shape=ShapeParams(
            shape_type="mesh", mesh_path=pallet_mesh_path, mesh_scale=[0.5, 0.5, 0.5], rgba_color=[0.8, 0.6, 0.4, 1.0]
        ),
        collision_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.4, 0.1]),
        initial_pose=Pose.from_pybullet(pallet_position, pallet_orientation_quat),
        mass=0.0,
        pickable=True,
    )
    pallet_params_list.append(pallet_params)

    # Store robot metadata (will link to pallet after batch spawn)
    robot.user_data["robot_index"] = idx
    robot.user_data["area_a_position"] = [robot_pos[0], robot_pos[1], robot_pos[2]]
    robot.user_data["area_b_position"] = [
        robot_pos[0] + (AREA_B_CENTER[0] - AREA_A_CENTER[0]),
        robot_pos[1] + (AREA_B_CENTER[1] - AREA_A_CENTER[1]),
        robot_pos[2],
    ]

# Batch spawn all pallets using SimObjectManager
pallet_objects = pallet_manager.spawn_objects_batch(pallet_params_list)

# Link each robot to its corresponding pallet
for idx, (robot, pallet) in enumerate(zip(mobile_agents, pallet_objects)):
    robot.user_data["pallet"] = pallet

print(f"✓ Successfully spawned {len(pallet_objects)} pallets using SimObjectManager")
print(f"  Total managed objects: Robots={agent_manager.get_object_count()}, " f"Pallets={pallet_manager.get_object_count()}\n")


def create_action_sequence_A_to_B(robot, pallet):
    """
    Create action sequence: Area A -> Area B

    Steps:
    1. Pick pallet from current position (Area A)
    2. Move to Area B
    3. Drop pallet at Area B position
    4. Rotate to face East (90 degrees) to indicate completion
    """
    area_b_pos = robot.user_data["area_b_position"]

    # Calculate drop position (in front of robot at Area B)
    drop_position = [area_b_pos[0] + PALLET_OFFSET[0], area_b_pos[1] + PALLET_OFFSET[1], PALLET_OFFSET[2]]

    # Get current robot pose for debugging
    current_pose = robot.get_pose()
    _, _, current_yaw = p.getEulerFromQuaternion(current_pose.orientation)
    print(f"[Robot {robot.user_data['robot_index']}] Creating A->B sequence, current yaw: {np.degrees(current_yaw):.1f}°")

    actions = [
        # 1. Pick pallet
        PickAction(
            target_object_id=pallet.body_id,
            use_approach=False,  # Already at position
            pick_offset=0.6,
            attach_relative_pose=Pose.from_euler(0.6, 0, -0.2, roll=np.pi / 2, pitch=0, yaw=0),
        ),
        # 2. Move to Area B
        MoveAction(
            path=Path.from_positions([area_b_pos]),
            final_orientation_align=False,
        ),
        # 3. Drop pallet at Area B
        DropAction(
            drop_position=drop_position,
            drop_orientation=list(pallet_orientation_quat),
            place_gently=True,
            use_approach=False,
            drop_offset=0.6,
        ),
        # 4. Small wait before rotation
        WaitAction(duration=0.5, action_type="prepare_rotate"),
        # 5. Rotate 180 degrees (turn around) to indicate drop completion
        # After moving to Area B (East direction), rotate to face West (π = 180 degrees)
        MoveAction(
            path=Path([Pose.from_yaw(area_b_pos[0], area_b_pos[1], area_b_pos[2], np.pi / 2)]),
            final_orientation_align=True,
        ),
        # 6. Final wait
        WaitAction(duration=0.5, action_type="idle"),
    ]

    print(f"[Robot {robot.user_data['robot_index']}] A->B sequence: {len(actions)} actions, rotation target: 180°")

    return actions


def create_action_sequence_B_to_A(robot, pallet):
    """
    Create action sequence: Area B -> Area A

    Steps:
    1. Pick pallet from current position (Area B)
    2. Move to Area A
    3. Drop pallet at Area A position
    4. Rotate to face North (0 degrees) to indicate completion
    """
    area_a_pos = robot.user_data["area_a_position"]

    # Calculate drop position (in front of robot at Area A)
    drop_position = [area_a_pos[0] + PALLET_OFFSET[0], area_a_pos[1] + PALLET_OFFSET[1], PALLET_OFFSET[2]]

    actions = [
        # 1. Pick pallet
        PickAction(
            target_object_id=pallet.body_id,
            use_approach=False,  # Already at position
            pick_offset=0.6,
            attach_relative_pose=Pose.from_euler(0.6, 0, -0.2, roll=np.pi / 2, pitch=0, yaw=0),
        ),
        # 2. Move to Area A
        MoveAction(
            path=Path.from_positions([area_a_pos]),
            final_orientation_align=False,
        ),
        # 3. Drop pallet at Area A
        DropAction(
            drop_position=drop_position,
            drop_orientation=list(pallet_orientation_quat),
            place_gently=True,
            use_approach=False,
            drop_offset=0.6,
        ),
        # 4. Small wait before rotation
        WaitAction(duration=0.5, action_type="prepare_rotate"),
        # 5. Rotate 180 degrees (turn around) to indicate drop completion
        # After moving to Area A (West direction), rotate to face East (0 degrees)
        MoveAction(
            path=Path([Pose.from_yaw(area_a_pos[0], area_a_pos[1], area_a_pos[2], np.pi / 2)]),
            final_orientation_align=True,
        ),
        # 6. Final wait
        WaitAction(duration=0.5, action_type="idle"),
    ]

    return actions


def action_coordination_callback(manager: AgentManager, dt: float):
    """
    AgentManager callback to coordinate all robots.

    When all robots complete their current task, switch direction and assign new tasks.
    """
    # Debug: Print current action status for first robot with detailed info
    if len(manager.objects) > 0:
        robot = manager.objects[0]
        action_str = "None"
        action_status_str = "N/A"
        action_phase_str = ""

        if robot._current_action:
            action = robot._current_action
            action_type = type(action).__name__
            action_status = action.status.name
            action_str = action_type
            action_status_str = action_status

            # Get phase info if available
            if hasattr(action, "phase") and hasattr(action.phase, "name"):  # type: ignore
                action_phase_str = f", Phase={action.phase.name}"  # type: ignore

            # Special logging for MoveAction
            if action_type == "MoveAction":
                is_moving = robot.is_moving
                print(f"[MoveAction] Status={action_status}, is_moving={is_moving}{action_phase_str}")

        queue_size = robot.get_action_queue_size()
        pose = robot.get_pose()
        _, _, yaw = p.getEulerFromQuaternion(pose.orientation)

        # Print queue contents for debugging
        queue_actions = []
        for a in robot._action_queue:
            act_name = type(a).__name__
            if hasattr(a, "path") and act_name == "MoveAction":
                path = getattr(a, "path", None)  # type: ignore
                num_waypoints = len(path.waypoints) if path else 0
                queue_actions.append(f"{act_name}({num_waypoints}wp)")
            else:
                queue_actions.append(act_name)

        status_msg = (
            f"[STATUS] Current={action_str}/{action_status_str}{action_phase_str}, "
            f"Queue={queue_size} {queue_actions}, Yaw={np.degrees(yaw):.1f}°"
        )
        print(status_msg)

    # Count how many robots have empty queues
    empty_count = sum(1 for robot in manager.objects if robot.is_action_queue_empty())

    # If all robots have completed their tasks, switch direction
    if empty_count == NUM_ROBOTS:
        state.completed_robots = 0

        if state.direction == "A_to_B":
            # Switch to B -> A
            state.direction = "B_to_A"
            state.cycle_count += 1
            print(f"\n{'='*60}")
            print(f"[Cycle {state.cycle_count}] All robots completed A->B. Starting B->A...")
            print(f"{'='*60}\n")

            # Assign B->A sequence to all robots
            for robot in manager.objects:
                pallet = robot.user_data["pallet"]
                robot.add_action_sequence(create_action_sequence_B_to_A(robot, pallet))

        else:
            # Switch to A -> B
            state.direction = "A_to_B"
            print(f"\n{'='*60}")
            print(f"[Cycle {state.cycle_count}] All robots completed B->A. Starting A->B...")
            print(f"{'='*60}\n")

            # Assign A->B sequence to all robots
            for robot in manager.objects:
                pallet = robot.user_data["pallet"]
                robot.add_action_sequence(create_action_sequence_A_to_B(robot, pallet))


# Setup initial sequence for all robots (A -> B)
print("=== Setting up Initial Action Sequences (A -> B) ===")
print(f"Direction: Area A {AREA_A_CENTER} -> Area B {AREA_B_CENTER}")
print("=" * 60 + "\n")

for idx, robot in enumerate(mobile_agents):
    pallet = robot.user_data["pallet"]
    robot.add_action_sequence(create_action_sequence_A_to_B(robot, pallet))

    if (idx + 1) % 20 == 0:
        print(f"  Configured actions for {idx + 1}/{NUM_ROBOTS} robots...")

print(f"✓ All {len(mobile_agents)} robots configured!\n")

# Register callback with AgentManager
agent_manager.register_callback(action_coordination_callback, frequency=1.0)
print("✓ Registered AgentManager callback for automatic coordination\n")

# Add area markers
cube_mesh_path = os.path.join(mesh_dir, "cube.obj")

# Area A marker (green)
area_a_marker = SimObject.from_mesh(
    visual_shape=ShapeParams(
        shape_type="mesh", mesh_path=cube_mesh_path, mesh_scale=[10.0, 10.0, 0.1], rgba_color=[0.0, 1.0, 0.0, 0.3]
    ),
    collision_shape=None,
    pose=Pose.from_xyz(AREA_A_CENTER[0], AREA_A_CENTER[1], 0.05),
    mass=0.0,
    pickable=False,
    sim_core=sim_core,
)

# Area B marker (blue)
area_b_marker = SimObject.from_mesh(
    visual_shape=ShapeParams(
        shape_type="mesh", mesh_path=cube_mesh_path, mesh_scale=[10.0, 10.0, 0.1], rgba_color=[0.0, 0.0, 1.0, 0.3]
    ),
    collision_shape=None,
    pose=Pose.from_xyz(AREA_B_CENTER[0], AREA_B_CENTER[1], 0.05),
    mass=0.0,
    pickable=False,
    sim_core=sim_core,
)

# Add labels
p.addUserDebugText("AREA A", [AREA_A_CENTER[0], AREA_A_CENTER[1], 2.0], textColorRGB=[0, 1, 0], textSize=2.0, lifeTime=0)
p.addUserDebugText("AREA B", [AREA_B_CENTER[0], AREA_B_CENTER[1], 2.0], textColorRGB=[0, 0, 1], textSize=2.0, lifeTime=0)

# Camera setup - view between two areas
camera_target = [(AREA_A_CENTER[0] + AREA_B_CENTER[0]) / 2, (AREA_A_CENTER[1] + AREA_B_CENTER[1]) / 2, 0]
camera_distance = 40.0
p.resetDebugVisualizerCamera(camera_distance, 45, -30, camera_target)

# Run simulation
print("Starting simulation...")
print(f"Watch {len(mobile_agents)} mobile robots shuttle between Area A and Area B.")
print("All robots move together from one area to another.")
print("AgentManager handles bulk action coordination.")
print("Press Ctrl+C to stop.\n")

sim_core.run_simulation()
