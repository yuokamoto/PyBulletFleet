#!/usr/bin/env python3
"""
performance_benchmark.py
Simple benchmark worker that executes one simulation and outputs JSON results.

This is a clean worker process that:
- Runs a single benchmark test
- Outputs results as JSON to stdout
- No self-recursion, no process management

Usage (typically called by run_benchmark.py):
    python benchmark/performance_benchmark_worker.py --agents 1000 --duration 10
"""
import os
import sys
import time
import gc
import json
import psutil
import tracemalloc
import argparse
import platform
import subprocess
from typing import Dict, Optional, Any
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pybullet as p
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.agent import AgentSpawnParams, MotionMode
from pybullet_fleet.geometry import Pose


def get_system_info() -> Dict[str, Any]:
    """Collect system information (CPU, memory, OS)."""
    info = {
        "platform": platform.system(),
        "platform_release": platform.release(),
        "platform_version": platform.version(),
        "architecture": platform.machine(),
        "processor": platform.processor(),
        "python_version": platform.python_version(),
    }

    # Memory info
    mem = psutil.virtual_memory()
    info["total_memory_gb"] = round(mem.total / (1024**3), 2)

    # CPU info
    info["cpu_count"] = psutil.cpu_count(logical=False)  # Physical cores
    info["cpu_count_logical"] = psutil.cpu_count(logical=True)  # Logical cores

    # Try to get more detailed CPU info on Linux
    if platform.system() == "Linux":
        try:
            # Get CPU model name
            result = subprocess.run(["lscpu"], capture_output=True, text=True, timeout=2)
            for line in result.stdout.split("\n"):
                if "Model name:" in line:
                    info["cpu_model"] = line.split(":", 1)[1].strip()
                elif "CPU max MHz:" in line:
                    try:
                        info["cpu_max_mhz"] = float(line.split(":", 1)[1].strip())
                    except (ValueError, IndexError):
                        pass
        except (FileNotFoundError, IOError):
            pass

    return info


def load_config(config_path: str, scenario: Optional[str] = None) -> Dict[str, Any]:
    """Load benchmark configuration from YAML file."""
    if not os.path.exists(config_path):
        return {}

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    # Merge scenario config if specified
    if scenario and "scenarios" in config and scenario in config["scenarios"]:
        base_config = config.copy()
        scenario_config = config["scenarios"][scenario]

        for key, value in scenario_config.items():
            if isinstance(value, dict) and key in base_config:
                base_config[key].update(value)
            else:
                base_config[key] = value

        return base_config

    return config


def get_memory_info():
    """Get memory info (RSS + tracemalloc current)."""
    process = psutil.Process()
    mem = process.memory_info()

    py_traced_mb = 0.0
    if tracemalloc.is_tracing():
        current, _peak = tracemalloc.get_traced_memory()
        py_traced_mb = current / 1024 / 1024

    rss_mb = mem.rss / 1024 / 1024
    return {
        "rss_mb": rss_mb,
        "vms_mb": mem.vms / 1024 / 1024,
        "py_traced_mb": py_traced_mb,
        "rss_minus_tracemalloc_mb": rss_mb - py_traced_mb,
    }


def force_cleanup():
    """Best-effort cleanup inside a process."""
    gc.collect()
    gc.collect()
    gc.collect()
    time.sleep(0.05)


def cpu_time_s(process: psutil.Process) -> float:
    """Return user+sys CPU time seconds."""
    t = process.cpu_times()
    return float(t.user + t.system)


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

    if p.isConnected():
        p.disconnect()

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
        enable_time_profiling=sim_config.get("enable_time_profiling", False),  # Disable for benchmark
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
    import math

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
