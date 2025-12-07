#!/usr/bin/env python3
"""
100robots_grid_demo.py
Unified demo for spawning 100 robots with AgentManager.

Modes:
  --mode=mixed (default): Spawn mixed robot types (mobile 60% + arm 40%)
  --mode=single: Spawn single robot type (mobile only)

Examples:
  python 100robots_grid_demo.py              # Mixed mode (default)
  python 100robots_grid_demo.py --mode=mixed # Mixed mode (explicit)
  python 100robots_grid_demo.py --mode=single # Single type mode
"""
import argparse
import os
import sys

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pybullet as p

from pybullet_fleet.agent import Agent, AgentSpawnParams, Pose
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.config_utils import load_config
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams

# ========================================
# Parse Arguments
# ========================================

parser = argparse.ArgumentParser(description="100 Robots Grid Demo")
parser.add_argument(
    "--mode",
    type=str,
    default="mixed",
    choices=["mixed", "single"],
    help="Spawn mode: mixed (mobile+arm) or single (mobile only)",
)
args = parser.parse_args()

mode = args.mode

# ========================================
# Load Configuration
# ========================================

config_dir = os.path.join(os.path.dirname(__file__), "../config")
base_config_path = os.path.join(config_dir, "config.yaml")
robots_config_path = os.path.join(config_dir, "100robots_config.yaml")

# Load both configs (100robots_config overrides config.yaml)
config = load_config([base_config_path, robots_config_path])

# Extract parameters from config
num_robots = config.get("num_robots", 100)
grid_size = int(np.ceil(np.sqrt(num_robots)))

# Grid parameters
grid_config = config.get("grid", {})
spacing_config = config.get("spacing", [2.0, 2.0, 0.0])
offset_config = config.get("offset", [0.0, 0.0, 0.0])

# Robot parameters
mobile_robot_config = config.get("mobile_robot", {})
arm_robot_config = config.get("arm_robot", {})
mixed_mode_config = config.get("mixed_mode", {})

# Mixed mode probabilities
mobile_robot_prob = mixed_mode_config.get("mobile_robot_prob", 0.5)
arm_robot_prob = mixed_mode_config.get("arm_robot_prob", 0.5)

print("=== 100 Robots Grid Demo ===")
print(f"Mode: {mode}")
print(f"Total robots: {num_robots}")
print(f"Grid size: {grid_size}x{grid_size}")
if mode == "mixed":
    print(f"Mobile robot probability: {mobile_robot_prob*100:.0f}%")
    print(f"Arm robot probability: {arm_robot_prob*100:.0f}%")

# ========================================
# Initialize Simulation
# ========================================

params = SimulationParams.from_dict(config)
sim_core = MultiRobotSimulationCore(params)

# ========================================
# Setup Spawn Parameters
# ========================================

agent_manager = AgentManager(sim_core=sim_core)

# Create grid parameters from config
grid_params = GridSpawnParams(
    x_min=grid_config.get("x_min", 0),
    x_max=grid_config.get("x_max", grid_size - 1),
    y_min=grid_config.get("y_min", 0),
    y_max=grid_config.get("y_max", grid_size - 1),
    z_min=grid_config.get("z_min", 0),
    z_max=grid_config.get("z_max", 0),
    spacing=spacing_config,
    offset=offset_config,
)

# Mobile robot spawn params (shared by both modes) - from config
mobile_urdf_path = mobile_robot_config.get("urdf_path", "robots/mobile_robot.urdf")
mobile_urdf = os.path.join(os.path.dirname(__file__), "..", mobile_urdf_path)
if not os.path.exists(mobile_urdf):
    raise FileNotFoundError(f"Mobile robot URDF not found: {mobile_urdf}")

mobile_params = AgentSpawnParams(
    urdf_path=mobile_urdf,
    initial_pose=Pose.from_xyz(0, 0, mobile_robot_config.get("initial_z", 0.3)),
    use_fixed_base=mobile_robot_config.get("use_fixed_base", False),
    max_linear_vel=mobile_robot_config.get("max_linear_vel", 2.0),
    max_linear_accel=mobile_robot_config.get("max_linear_accel", 5.0),
    user_data={"robot_type": "mobile_robot"},
)

if mode == "mixed":
    # ========================================
    # Mixed Mode: Spawn mixed robot types
    # ========================================

    # Arm robot spawn params - from config
    arm_urdf_path = arm_robot_config.get("urdf_path", "robots/arm_robot.urdf")
    arm_urdf = os.path.join(os.path.dirname(__file__), "..", arm_urdf_path)
    if not os.path.exists(arm_urdf):
        raise FileNotFoundError(f"Arm robot URDF not found: {arm_urdf}")

    arm_params = AgentSpawnParams(
        urdf_path=arm_urdf,
        initial_pose=Pose.from_xyz(0, 0, arm_robot_config.get("initial_z", 0.0)),
        use_fixed_base=arm_robot_config.get("use_fixed_base", True),
        max_linear_vel=arm_robot_config.get("max_linear_vel", 0.0),
        max_linear_accel=arm_robot_config.get("max_linear_accel", 0.0),
        user_data={"robot_type": "arm_robot"},
    )

    print("\n✓ Using URDFs:")
    print(f"  - Mobile: {mobile_urdf}")
    print(f"  - Arm: {arm_urdf}")

    # Spawn mixed agents
    print(f"\nSpawning {num_robots} agents with mixed types...")
    spawn_params_list = [(mobile_params, mobile_robot_prob), (arm_params, arm_robot_prob)]

    spawned_agents = agent_manager.spawn_agents_grid_mixed(
        num_agents=num_robots, grid_params=grid_params, spawn_params_list=spawn_params_list
    )

    print(f"✓ Spawned {len(spawned_agents)} agents using spawn_agents_grid_mixed()")

else:  # mode == 'single'
    # ========================================
    # Single Mode: Spawn single robot type
    # ========================================

    print(f"\n✓ Using URDF: {mobile_urdf}")

    # Spawn agents in grid
    print(f"\nSpawning {num_robots} agents...")
    spawned_agents = agent_manager.spawn_agents_grid(
        num_agents=num_robots, grid_params=grid_params, spawn_params=mobile_params
    )

    print(f"✓ Spawned {len(spawned_agents)} agents using spawn_agents_grid()")

print("\n=== AgentManager Status ===")
print(agent_manager)

# ========================================
# Setup Camera
# ========================================

# Setup camera view using agent_manager (automatically extracts all object positions)
camera_config = config.get("camera", {})
agent_manager.setup_camera(camera_config)

# ========================================
# Movement Callback
# ========================================


def batch_agent_movement_callback(sim_objects, sim_core, dt):
    """
    Callback for mixed robot types.
    - Mobile robots: random forward/backward movement with rotation
    - Arm robots: random joint movements
    """
    for obj in sim_objects:
        if not isinstance(obj, Agent):
            continue

        agent = obj
        robot_type = agent.user_data.get("robot_type", "mobile_robot")

        if robot_type == "mobile_robot":
            # Mobile robot movement
            pose = agent.get_pose()
            pos, orn = pose.as_tuple()
            euler = p.getEulerFromQuaternion(orn)
            yaw = euler[2]

            max_linear_speed = 2.0
            max_angular_speed = 1.5
            forward_vel = np.random.uniform(-max_linear_speed, max_linear_speed)
            yaw_vel = np.random.uniform(-max_angular_speed, max_angular_speed)

            forward_x = forward_vel * np.cos(yaw)
            forward_y = forward_vel * np.sin(yaw)
            linear_vel = [forward_x, forward_y, 0]
            angular_vel = [0, 0, yaw_vel]

            corrected_orn = p.getQuaternionFromEuler([0, 0, yaw])
            new_pose = Pose.from_pybullet([pos[0], pos[1], 0.3], corrected_orn)
            agent.set_pose(new_pose)
            p.resetBaseVelocity(agent.body_id, linear_vel, angular_vel)

        elif robot_type == "arm_robot":
            # Arm robot joint control
            num_joints = p.getNumJoints(agent.body_id)
            for j in range(num_joints):
                info = p.getJointInfo(agent.body_id, j)
                joint_type = info[2]
                if joint_type == p.JOINT_REVOLUTE:
                    lower, upper = info[8:10]
                    if lower < upper:
                        target_pos = np.random.uniform(lower, upper)
                    else:
                        target_pos = np.random.uniform(-1.0, 1.0)
                    p.setJointMotorControl2(agent.body_id, j, p.POSITION_CONTROL, targetPosition=target_pos, force=500)


# ========================================
# Register Callbacks and Run Simulation
# ========================================

sim_core.register_callback(batch_agent_movement_callback, frequency=1)
sim_core.run_simulation()
