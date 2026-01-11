#!/usr/bin/env python3
"""
performance_demo.py
Demonstrates performance optimization settings for PyBullet Fleet.

This example shows how to configure the simulation for maximum performance
using collision_check_2d and other optimization parameters.

Key Performance Settings:
  - collision_check_2d=True: Use 2D collision detection (9 neighbors vs 27)
  - speed=0: Maximum speed mode (no sleep, no real-time sync)
  - enable_monitor_gui=False: Headless data collection
  - enable_profiling=True: Enable performance logging

Performance Results (with optimizations):
  - 100 agents:  RTF 6.6x  (1.5ms per step)
  - 250 agents:  RTF 2.3x  (4.3ms per step)
  - 500 agents:  RTF 0.9x  (10.8ms per step)
  - 1000 agents: RTF 0.4x  (22.3ms per step)

See benchmark/README.md for detailed performance analysis.
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
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams

# ========================================
# Parse Arguments
# ========================================

parser = argparse.ArgumentParser(description="Performance Optimization Demo")
parser.add_argument(
    "--agents",
    type=int,
    default=100,
    help="Number of agents to spawn (default: 100)",
)
parser.add_argument(
    "--duration",
    type=float,
    default=10.0,
    help="Simulation duration in seconds (default: 10.0)",
)
parser.add_argument(
    "--gui",
    action="store_true",
    help="Enable GUI (default: headless for performance)",
)
parser.add_argument(
    "--2d",
    action="store_true",
    dest="collision_2d",
    help="Enable 2D collision optimization (9 neighbors instead of 27)",
)
parser.add_argument(
    "--speed",
    type=float,
    default=0.0,
    help="Simulation speed: 0=max speed, 1.0=real-time (default: 0)",
)
parser.add_argument(
    "--profile",
    action="store_true",
    help="Enable profiling (logs performance metrics)",
)

args = parser.parse_args()

print("=" * 70)
print("PyBullet Fleet - Performance Optimization Demo")
print("=" * 70)
print(f"Configuration:")
print(f"  Number of agents: {args.agents}")
print(f"  Simulation duration: {args.duration}s")
print(f"  GUI enabled: {args.gui}")
print(f"  2D collision optimization: {args.collision_2d}")
print(f"  Speed mode: {args.speed} (0=max speed, 1.0=real-time)")
print(f"  Profiling enabled: {args.profile}")
print("=" * 70)

# ========================================
# Performance-Optimized Simulation Parameters
# ========================================

params = SimulationParams(
    timestep=0.01,
    duration=args.duration,
    gui=args.gui,
    monitor=True,  # Always collect data
    enable_monitor_gui=args.gui,  # Only show GUI if requested
    speed=args.speed,  # 0 = maximum speed (no sleep)
    collision_check_2d=args.collision_2d,  # 2D optimization (9 neighbors)
    enable_profiling=args.profile,  # Enable performance logging
    log_level="INFO",
)

# ========================================
# Create Simulation Core
# ========================================

sim_core = MultiRobotSimulationCore(params=params)
agent_manager = AgentManager(sim_core)

# ========================================
# Spawn Robots in Grid
# ========================================

num_robots = args.agents
grid_size = int(np.ceil(np.sqrt(num_robots)))

grid_params = GridSpawnParams(
    x_min=0,
    x_max=grid_size - 1,
    y_min=0,
    y_max=grid_size - 1,
    z_min=0,
    z_max=0,
    spacing=[2.0, 2.0, 0.0],  # 2m spacing between robots
    offset=[0.0, 0.0, 0.0],
)

# Mobile robot spawn params
mobile_urdf = os.path.join(os.path.dirname(__file__), "..", "robots/mobile_robot.urdf")
if not os.path.exists(mobile_urdf):
    raise FileNotFoundError(f"Mobile robot URDF not found: {mobile_urdf}")

mobile_params = AgentSpawnParams(
    urdf_path=mobile_urdf,
    initial_pose=Pose.from_xyz(0, 0, 0.3),
    use_fixed_base=False,
    max_linear_vel=2.0,
    max_linear_accel=5.0,
    user_data={"robot_type": "mobile_robot"},
)

print(f"\nSpawning {num_robots} robots in {grid_size}x{grid_size} grid...")
spawned_agents = agent_manager.spawn_agents_grid(
    num_agents=num_robots, grid_params=grid_params, spawn_params=mobile_params
)

print(f"✓ Spawned {len(spawned_agents)} agents")

# ========================================
# Setup Camera
# ========================================

if args.gui:
    agent_manager.setup_camera(
        {
            "distance": grid_size * 2.0,
            "yaw": 45,
            "pitch": -30,
            "target": [grid_size, grid_size, 0],
        }
    )

# ========================================
# Movement Callback
# ========================================


def random_movement_callback(sim_core, dt):
    """Random movement for performance testing - simple goal-based navigation."""
    import random
    
    # Randomly select a few agents to update
    if len(sim_core.sim_objects) == 0:
        return
        
    # Update 5% of agents per step
    num_to_update = max(1, int(len(sim_core.sim_objects) * 0.05))
    agents_to_update = random.sample(
        [obj for obj in sim_core.sim_objects if isinstance(obj, Agent)],
        min(num_to_update, sum(1 for obj in sim_core.sim_objects if isinstance(obj, Agent)))
    )
    
    for agent in agents_to_update:
        # Random goal position within grid
        grid_size = int(np.ceil(np.sqrt(args.agents)))
        new_goal = Pose.from_xyz(
            np.random.uniform(0, grid_size * 2.0),
            np.random.uniform(0, grid_size * 2.0),
            0.3
        )
        agent.set_goal_pose(new_goal)


sim_core.register_callback(random_movement_callback)

# ========================================
# Run Simulation
# ========================================

print(f"\nRunning simulation for {args.duration}s...")
print("Performance metrics will be displayed below.")
print("-" * 70)

if args.profile:
    print("[INFO] Profiling enabled - performance metrics will be logged")
    print("[INFO] Use benchmark/parse_profile.py to analyze logs")
    print("-" * 70)

# Run simulation
sim_core.run_simulation()

# ========================================
# Performance Summary
# ========================================

print("-" * 70)
print("Simulation complete!")
print("\nPerformance Tips:")
print("  1. Use --2d for 2D simulations (9 neighbors vs 27): ~67% faster")
print("  2. Use --speed=0 for maximum speed (no sleep)")
print("  3. Use headless mode (no --gui) for better performance")
print("  4. Use --profile to analyze bottlenecks")
print("\nFor detailed performance analysis:")
print("  python benchmark/performance_benchmark.py --agents {} --duration {}".format(
    args.agents, args.duration
))
print("\nFor comprehensive benchmarking:")
print("  python benchmark/performance_benchmark_sweep.py")
print("\nSee benchmark/README.md for more information.")
print("=" * 70)
