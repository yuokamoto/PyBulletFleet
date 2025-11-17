#!/usr/bin/env python3
"""
100robots_grid_demo.py
Simplified demo using AgentManager.spawn_robots_grid() for 100 robots.

This demonstrates the cleanest way to spawn many robots using AgentManager.
Configuration is loaded from config.yaml and 100robots_config.yaml.
"""
import sys
import os
import pybullet as p
import numpy as np
import random

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.robot import Agent, Pose, AgentSpawnParams
from pybullet_fleet.robot_manager import AgentManager, GridSpawnParams
from pybullet_fleet.config_utils import load_config

# ========================================
# Load Configuration
# ========================================

config_dir = os.path.join(os.path.dirname(__file__), '../config')
base_config_path = os.path.join(config_dir, 'config.yaml')
robots_config_path = os.path.join(config_dir, '100robots_config.yaml')

# Load and merge configurations
config = load_config([base_config_path, robots_config_path])

# Extract parameters
NUM_ROBOTS = config.get('num_robots', 100)
GRID_SIZE = config.get('grid_size', int(np.ceil(np.sqrt(NUM_ROBOTS))))

print(f"=== 100 Robots Grid Demo ===")
print(f"Loaded configs:")
print(f"  - Base: {base_config_path}")
print(f"  - Robots: {robots_config_path}")
print(f"Total robots: {NUM_ROBOTS}")
print(f"Grid size: {GRID_SIZE}x{GRID_SIZE}")

# ========================================
# Initialize Simulation
# ========================================

# Use merged config for simulation parameters
params = SimulationParams.from_dict(config)
sim_core = MultiRobotSimulationCore(params)

# ========================================
# Spawn Agents using AgentManager.spawn_robots_grid()
# ========================================

robot_manager = AgentManager(sim_core=sim_core)

# Get robot configuration
robot_config = config.get('robot', {})

# Resolve URDF path
robot_urdf_path = robot_config.get('urdf_path', 'robots/mobile_robot.urdf')
robot_urdf = os.path.join(os.path.dirname(__file__), '..', robot_urdf_path)

# Check if URDF exists
if not os.path.exists(robot_urdf):
    raise FileNotFoundError(f"Robot URDF not found: {robot_urdf}")

print(f"\n✓ Using URDF: {robot_urdf}")

# Update config with resolved absolute path
robot_config['urdf_path'] = robot_urdf

# Create spawn parameters from config (now includes urdf_path)
spawn_params = AgentSpawnParams.from_dict(robot_config)

# Create grid parameters
grid_params = GridSpawnParams(
    x_min=config.get('x_min', 0),
    x_max=config.get('x_max', 9),
    y_min=config.get('y_min', 0),
    y_max=config.get('y_max', 9),
    spacing=config.get('spacing', [2.0, 2.0, 0.0]),
    offset=config.get('offset', [0.0, 0.0, 0.5])
)

# Spawn robots in grid using AgentManager
print(f"\nSpawning {NUM_ROBOTS} robots using AgentManager.spawn_robots_grid()...")
spawned_robots = robot_manager.spawn_robots_grid(
    num_robots=NUM_ROBOTS,
    grid_params=grid_params,
    spawn_params=spawn_params
)

print(f"✓ Spawned {len(spawned_robots)} robots using spawn_robots_grid()")

print(f"\n=== AgentManager Status ===")
print(robot_manager)

# ========================================
# Enable Rendering
# ========================================

# Enable rendering after all robots are spawned
sim_core.enable_rendering()
print("\n✓ Rendering enabled")

# ========================================
# Setup Camera
# ========================================

# Get all robot positions for auto camera
robot_positions = [robot.get_pose().position for robot in spawned_robots]

# Setup camera view
camera_config = config.get('camera', {})
sim_core.setup_camera(camera_config=camera_config, entity_positions=robot_positions)

# ========================================
# Random Goal Assignment
# ========================================

# Get goal parameters from config
goal_mode = config.get('goal_mode', 'random')
goal_bounds = config.get('goal_bounds', {
    'x_min': 0, 'x_max': 20,
    'y_min': 0, 'y_max': 20,
    'z_min': 0.3, 'z_max': 0.3
})

def goal_update_callback(robots, manager, dt):
    """Assign random goals to robots that have reached their destination."""
    for robot in robots:
        if not robot.is_moving and not robot.use_fixed_base:
            if goal_mode == 'random':
                # Assign random goal within bounds
                goal_x = random.uniform(goal_bounds['x_min'], goal_bounds['x_max'])
                goal_y = random.uniform(goal_bounds['y_min'], goal_bounds['y_max'])
                goal_z = random.uniform(goal_bounds['z_min'], goal_bounds['z_max'])
                goal = Pose.from_xyz(goal_x, goal_y, goal_z)
                robot.set_goal_pose(goal)

robot_manager.register_goal_update_callback(goal_update_callback)

# ========================================
# Control Callback
# ========================================

def control_callback(robots, sim_core, dt):
    """Update all robots."""
    robot_manager.update_all_with_goals(dt)

sim_core.register_callback(control_callback, frequency=30)

# ========================================
# Run Simulation
# ========================================

print(f"\n=== Running simulation ===")
print(f"All robots will move to random goals")

# Run for limited steps (for testing)
MAX_STEPS = 500
print(f"Running {MAX_STEPS} simulation steps...")

for step in range(MAX_STEPS):
    # Update robots
    robot_manager.update_all_with_goals(sim_core.params.timestep)
    
    # Step simulation
    sim_core.step_once()
    
    # Print progress
    if step % 100 == 0:
        moving = robot_manager.get_moving_count()
        print(f"  Step {step}: {moving}/{NUM_ROBOTS} robots moving")

print(f"\nFinal statistics:")
print(f"  Total robots: {robot_manager.get_robot_count()}")
print(f"  Moving robots: {robot_manager.get_moving_count()}")
print(f"  Simulation time: {sim_core.sim_time:.1f}s")
print(f"\n✓ Demo complete!")
