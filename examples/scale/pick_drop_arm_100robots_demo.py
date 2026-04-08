#!/usr/bin/env python3
"""
pick_drop_arm_100robots_demo.py
Demo: 100 robot arms picking and dropping boxes using Action-based interface.

Demonstrates:
- AgentManager for grid-based agent spawning
- Multi-agent simulation with 100 robot arms
- Bulk action management via AgentManager callback
- JointAction, PickAction, DropAction coordination
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
import pybullet as p
from pybullet_fleet.agent import AgentSpawnParams
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams
from pybullet_fleet.action import JointAction, PickAction, DropAction, WaitAction
from pybullet_fleet.robot_models import resolve_urdf

parser = argparse.ArgumentParser(description="100 robot arms pick & drop demo")
parser.add_argument("--robot", default="panda", help="Robot name (e.g. panda, kuka_iiwa, arm_robot) or URDF path")
parser.add_argument("--duration", type=float, default=None, help="Simulation duration in seconds (default: run forever)")
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

# Simulation setup with memory profiling enabled
params = SimulationParams(
    gui=True,
    timestep=0.1,
    target_rtf=args.rtf if args.rtf is not None else 0,
    physics=False,
    ignore_static_collision=True,
    log_level="info",
    enable_time_profiling=False,  # Time profiling
    enable_memory_profiling=False,  # Memory profiling
    profiling_interval=500,  # Report every 500 steps
)
sim_core = MultiRobotSimulationCore(params)

# Create AgentManager
agent_manager = AgentManager(sim_core=sim_core, update_frequency=10.0)

# Grid configuration for 100 robots (10x10 grid)
NUM_ROBOTS = 100
GRID_SIZE = 10  # 10x10 = 100 robots
SPACING = 1.0  # Distance between robots (meters)

# Joint configurations from preset
joint_init = _P["init"]
pick_joints = _P["pick"]
place_joints = _P["place"]

# Offset for attachment (box positioned above end-effector)
box_offset = 0.14
offset_pose = Pose.from_xyz(0, 0, box_offset)

# Pick/drop offset relative to robot position (from preset box positions)
pick_offset = list(_P["box_pick"])
place_offset = list(_P["box_place"])

# Storage for boxes (one per robot)
box_objects = []

print("\n=== Spawning 100 Robot Arms using AgentManager ===")

# Setup grid spawn parameters
arm_urdf = resolve_urdf(args.robot)
grid_params = GridSpawnParams(
    x_min=0,
    x_max=GRID_SIZE - 1,
    y_min=0,
    y_max=GRID_SIZE - 1,
    z_min=0,
    z_max=0,  # 2D grid
    spacing=[SPACING, SPACING, 0.0],
    offset=[0.0, 0.0, 0.0],
)

# Setup agent spawn parameters
agent_spawn_params = AgentSpawnParams(
    urdf_path=arm_urdf,
    use_fixed_base=True,
    mass=None,  # None = use URDF mass values (physics mode); 0.0 = kinematic
)

# Spawn all agents in grid using AgentManager
arm_agents = agent_manager.spawn_agents_grid(num_agents=NUM_ROBOTS, grid_params=grid_params, spawn_params=agent_spawn_params)

print(f"✓ Successfully spawned {len(arm_agents)} robots using AgentManager")

# Spawn boxes for each robot
print("=== Spawning Boxes for Each Robot ===")
for idx, arm_agent in enumerate(arm_agents):
    robot_pose = arm_agent.get_pose()
    robot_pos = robot_pose.position

    # Box starts at pick position (right side of robot)
    box_position = [robot_pos[0] + pick_offset[0], robot_pos[1] + pick_offset[1], pick_offset[2]]
    box_sim = SimObject.from_mesh(
        visual_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05], rgba_color=[1, 0, 0, 1]),
        collision_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
        pose=Pose.from_xyz(*box_position),
        mass=0.0,
        sim_core=sim_core,
    )
    box_objects.append(box_sim)

    # Store box reference in agent's user_data for easy access
    arm_agent.user_data["box"] = box_sim

    if (idx + 1) % 20 == 0:
        print(f"  Spawned {idx + 1}/{NUM_ROBOTS} boxes...")

print(f"✓ Successfully spawned {len(box_objects)} boxes\n")


def create_action_sequence(arm_agent, box_sim):
    """
    Create one complete pick-drop cycle for a specific robot.

    Args:
        arm_agent: Agent instance
        box_sim: SimObject instance for the box

    Returns:
        List of actions for one complete cycle
    """
    # Get robot position
    robot_pose = arm_agent.get_pose()
    robot_pos = robot_pose.position

    # Calculate pick and place positions relative to robot
    pick_position = [robot_pos[0] + pick_offset[0], robot_pos[1] + pick_offset[1], pick_offset[2]]
    place_position = [robot_pos[0] + place_offset[0], robot_pos[1] + place_offset[1], place_offset[2]]

    # Get end-effector link index
    pick_link_index = p.getNumJoints(arm_agent.body_id) - 1

    # Cycle 1: Pick from right, drop on left
    actions_cycle1 = [
        # 1. Move to initial position
        JointAction(target_joint_positions=joint_init, tolerance=0.05),
        # 2. Move arm to pick position
        JointAction(target_joint_positions=pick_joints, tolerance=0.05),
        # 3. Pick box from right side
        PickAction(
            target_object_id=box_sim.body_id,
            use_approach=False,
            attach_link=pick_link_index,
            attach_relative_pose=offset_pose,
        ),
        # 4. Move arm to place position (with box attached)
        JointAction(target_joint_positions=place_joints, tolerance=0.05),
        # 5. Drop box on left side
        DropAction(
            drop_pose=Pose(position=place_position),
            use_approach=False,
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
            attach_link=pick_link_index,
            attach_relative_pose=offset_pose,
        ),
        # 3. Move arm to pick position (with box attached)
        JointAction(target_joint_positions=pick_joints, tolerance=0.05),
        # 4. Drop box on right side
        DropAction(
            drop_pose=Pose(position=pick_position),
            use_approach=False,
        ),
        # 5. Return to initial position
        JointAction(target_joint_positions=joint_init, tolerance=0.05),
        # 6. Small wait before repeating
        WaitAction(duration=0.5, action_type="idle"),
    ]

    # Combine both cycles into one sequence
    return actions_cycle1 + actions_cycle2


def action_repeat_callback(manager: AgentManager, dt: float):
    """
    AgentManager callback to repeat action sequence when queue is empty for each robot.

    This callback is automatically called by AgentManager at the specified frequency.

    Args:
        manager: AgentManager instance with access to all agents
        dt: Time delta since last callback
    """
    for arm_agent in manager.objects:
        # Check if this robot's action queue is empty
        if arm_agent.is_action_queue_empty():
            # Get box from agent's user_data
            box_sim = arm_agent.user_data.get("box")
            if box_sim:
                # Add new sequence for this robot
                arm_agent.add_action_sequence(create_action_sequence(arm_agent, box_sim))


# Setup initial sequence for all robots
print("=== Setting up Action Sequences for 100 Robots ===")
print("Each robot will:")
print("  Cycle 1: Pick from right (0.3, 0, 0.1) -> Drop on left (-0.3, 0, 0.1)")
print("  Cycle 2: Pick from left (-0.3, 0, 0.1) -> Drop on right (0.3, 0, 0.1)")
print("=" * 60 + "\n")

# Setup all robots using helper method
agent_manager.add_action_sequence_all(lambda robot: create_action_sequence(robot, robot.user_data["box"]))
print(f"✓ All {len(arm_agents)} robots configured!\n")

# Register callback with AgentManager to repeat sequences
agent_manager.register_callback(action_repeat_callback, frequency=10.0)
print("✓ Registered AgentManager callback for automatic action repeat\n")

# Auto camera setup - calculate from agent positions
agent_positions = [agent.get_pose().position for agent in arm_agents]
# sim_core.setup_camera(
#     camera_config={
#         "camera_mode": "auto",
#         "camera_view_type": "perspective",
#         "camera_auto_scale": 1.0,  # Zoom out to see entire 10x10 grid
#     },
#     entity_positions=agent_positions,
# )
sim_core.setup_camera(
    camera_config={
        "camera_mode": "manual",
        "camera_distance": 3.0,
        "camera_yaw": 45,
        "camera_pitch": -35,
        "camera_target": [7.5, 0.5, 0.5],
    }
)

# Run simulation
print("Starting simulation...")
print(f"Watch all {len(arm_agents)} robot arms pick and drop boxes in synchronized cycles.")
print("Camera positioned to view entire 10x10 grid.")
print("AgentManager handles bulk action coordination.")
print("Press Ctrl+C to stop.\n")

sim_core.run_simulation(duration=args.duration)
