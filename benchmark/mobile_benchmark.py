#!/usr/bin/env python3
"""
mobile_benchmark.py
Benchmark worker for mobile agent (simple cube) simulations.

This is a clean worker process that:
- Runs a single benchmark test with N mobile agents
- Outputs results as JSON to stdout
- No self-recursion, no process management

Usage (typically called by run_benchmark.py):
    python benchmark/mobile_benchmark.py --agents 1000 --duration 10
"""
import os
import sys
import time
import json
import math
import psutil
import tracemalloc
import argparse
from typing import Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pybullet as p
from benchmark.tools import get_system_info, get_memory_info, force_cleanup, cpu_time_s, ensure_disconnected, load_config
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.agent import AgentSpawnParams, MotionMode
from pybullet_fleet.geometry import Pose


def run_benchmark(
    num_agents: int, duration: float, gui: bool = False, config_path: Optional[str] = None, scenario: Optional[str] = None
) -> dict:
    """
    Run a single benchmark test.

    Args:
        num_agents: Number of agents to spawn
        duration: Simulation duration in seconds
        gui: Whether to enable GUI
        config_path: Path to benchmark config file
        scenario: Scenario name from config file

    Returns:
        Dictionary with benchmark results
    """
    proc = psutil.Process()
    tracemalloc.start()
    force_cleanup()
    ensure_disconnected()

    # Load config
    if config_path is None:
        config_path = os.path.join(os.path.dirname(__file__), "configs", "general.yaml")

    config = load_config(config_path, scenario)
    sim_config = config.get("simulation", {})

    # Setup simulation parameters from config
    params = SimulationParams(
        gui=gui,
        timestep=sim_config.get("timestep", 0.1),
        duration=duration,
        target_rtf=sim_config.get("target_rtf", sim_config.get("speed", 0.0)),
        physics=sim_config.get("physics", False),
        monitor=sim_config.get("monitor", True),
        enable_monitor_gui=sim_config.get("enable_monitor_gui", False),
        enable_time_profiling=sim_config.get("enable_time_profiling", False),  # Default: disabled for benchmark accuracy
        log_level=sim_config.get("log_level", "WARN"),  # Quiet for benchmark
        collision_check_frequency=sim_config.get("collision_check_frequency", None),
        ignore_static_collision=sim_config.get("ignore_static_collision", sim_config.get("ignore_structure_collision", True)),
    )

    sim_core = MultiRobotSimulationCore(params)

    # Memory before spawning
    mem_before = get_memory_info()

    # Setup agent manager
    agent_manager = AgentManager(sim_core=sim_core)

    # Calculate grid size
    grid_size = int(math.ceil(math.sqrt(num_agents)))

    grid_params = GridSpawnParams(
        x_min=0,
        x_max=grid_size - 1,
        y_min=0,
        y_max=grid_size - 1,
        z_min=0,
        z_max=0,
        spacing=[1.0, 1.0, 0.0],
        offset=[0.0, 0.0, 0.1],
    )

    # Use simple cube URDF for performance testing
    robot_urdf = os.path.join(os.path.dirname(__file__), "../robots/simple_cube.urdf")
    agent_spawn_params = AgentSpawnParams(
        urdf_path=robot_urdf,
        motion_mode=MotionMode.OMNIDIRECTIONAL,
        max_linear_vel=2.0,
        max_angular_vel=3.0,
        mass=0.0,  # Kinematic control
    )

    # Spawn agents (timed)
    cpu_spawn_start = cpu_time_s(proc)
    wall_spawn_start = time.perf_counter()

    agents = agent_manager.spawn_agents_grid(
        num_agents=num_agents,
        grid_params=grid_params,
        spawn_params=agent_spawn_params,
    )

    wall_spawn_time = time.perf_counter() - wall_spawn_start
    cpu_spawn_time = cpu_time_s(proc) - cpu_spawn_start

    # Memory after spawning
    mem_after_spawn = get_memory_info()

    # Set random goals for some agents (to simulate real usage)
    import random

    num_moving_agents = max(num_agents // 2, min(num_agents, 10))  # Move max(half, 10) agents, capped to total agent count
    moving_indices = random.sample(range(len(agents)), num_moving_agents)
    for agent_idx in moving_indices:
        target_x = random.uniform(-10, grid_size + 10)
        target_y = random.uniform(-10, grid_size + 10)
        agents[agent_idx].set_goal_pose(Pose.from_xyz(target_x, target_y, 0.1))

    # Warmup steps to stabilize timing
    for _ in range(10):
        sim_core.step_once()

    # Run simulation (timed)
    cpu_sim_start = cpu_time_s(proc)
    wall_sim_start = time.perf_counter()

    sim_core.run_simulation(duration=duration)

    wall_sim_time = time.perf_counter() - wall_sim_start
    cpu_sim_time = cpu_time_s(proc) - cpu_sim_start

    # Memory after simulation
    mem_after_sim = get_memory_info()

    # Calculate metrics
    expected_steps = int(duration / params.timestep)
    actual_steps = sim_core.step_count
    real_time_factor = duration / wall_sim_time if wall_sim_time > 0 else 0.0
    avg_step_time_ms = (wall_sim_time / actual_steps * 1000.0) if actual_steps > 0 else 0.0

    # Cleanup
    if p.isConnected():
        p.resetSimulation()
        p.disconnect()
    if tracemalloc.is_tracing():
        tracemalloc.stop()
    del agents, agent_manager, sim_core
    force_cleanup()

    # Return results
    return {
        "num_agents": num_agents,
        "duration_s": duration,
        "gui": gui,
        "scenario": scenario,
        "spawn_time_s": wall_spawn_time,
        "spawn_cpu_s": cpu_spawn_time,
        "spawn_cpu_percent": (cpu_spawn_time / wall_spawn_time * 100.0) if wall_spawn_time > 0 else 0.0,
        "simulation_wall_s": wall_sim_time,
        "simulation_cpu_s": cpu_sim_time,
        "simulation_cpu_percent": (cpu_sim_time / wall_sim_time * 100.0) if wall_sim_time > 0 else 0.0,
        "expected_steps": expected_steps,
        "actual_steps": actual_steps,
        "real_time_factor": real_time_factor,
        "avg_step_time_ms": avg_step_time_ms,
        "mem_spawn_mb": {
            "rss_mb": mem_after_spawn["rss_mb"] - mem_before["rss_mb"],
            "py_traced_mb": mem_after_spawn["py_traced_mb"] - mem_before["py_traced_mb"],
        },
        "mem_total_mb": {
            "rss_mb": mem_after_sim["rss_mb"] - mem_before["rss_mb"],
            "py_traced_mb": mem_after_sim["py_traced_mb"] - mem_before["py_traced_mb"],
        },
        "system_info": get_system_info(),
    }


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="PyBullet Fleet Benchmark Worker")
    parser.add_argument("--agents", type=int, required=True, help="Number of agents to spawn")
    parser.add_argument("--duration", type=float, required=True, help="Simulation duration in seconds")
    parser.add_argument("--gui", action="store_true", help="Enable GUI (slower, for visualization)")
    parser.add_argument("--config", type=str, default=None, help="Path to benchmark config file")
    parser.add_argument("--scenario", type=str, default=None, help="Scenario name from config file")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    result = run_benchmark(
        num_agents=args.agents,
        duration=args.duration,
        gui=args.gui,
        config_path=args.config,
        scenario=args.scenario,
    )

    # Output JSON to stdout
    print(json.dumps(result))
