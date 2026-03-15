#!/usr/bin/env python3
"""
arm_benchmark.py
Benchmark worker for robot arm joint control.

Pure worker process — runs a single benchmark and outputs JSON to stdout.
Sweep, repetitions, and comparison are handled by run_benchmark.py.

Usage (typically called by run_benchmark.py --type arm):
    python benchmark/arm_benchmark.py --agents 10 --duration 5
    python benchmark/arm_benchmark.py --agents 10 --duration 5 --scenario kinematic
    python benchmark/arm_benchmark.py --agents 10 --duration 5 --config benchmark/configs/arm.yaml

Measures:
    - step_once() time with N arm robots executing JointAction sequences
    - Physics (mass=None, setJointMotorControl2) vs Kinematic (mass=0.0, resetJointState)
"""
import os
import sys
import time
import math
import json
import argparse
import tracemalloc
from typing import Dict, Any, List, Optional

import psutil

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pybullet as p
from benchmark.tools import (
    get_system_info,
    get_memory_info,
    force_cleanup,
    cpu_time_s,
    ensure_disconnected,
    warmup_steps,
    load_config,
)
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent import Agent
from pybullet_fleet.geometry import Pose
from pybullet_fleet.action import JointAction


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

JOINT_TARGETS_A = [1.0, 1.0, 1.0, 0.5]
JOINT_TARGETS_B = [-1.0, -0.5, 0.5, -0.3]
JOINT_TARGETS_HOME = [0.0, 0.0, 0.0, 0.0]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def spawn_arms(
    sim_core: MultiRobotSimulationCore,
    num_agents: int,
    mass: Optional[float],
    urdf_path: str,
    spacing: float = 1.5,
) -> List[Agent]:
    """Spawn arm robots in a grid layout."""
    agents = []
    grid_size = max(1, int(math.ceil(math.sqrt(num_agents))))

    for i in range(num_agents):
        row = i // grid_size
        col = i % grid_size
        x = col * spacing
        y = row * spacing

        agent = Agent.from_urdf(
            urdf_path=urdf_path,
            pose=Pose.from_xyz(x, y, 0.0),
            mass=mass,
            use_fixed_base=True,
            sim_core=sim_core,
        )
        agents.append(agent)

    return agents


def enqueue_joint_cycle(agents: List[Agent]):
    """Give each agent a JointAction cycle: A → B → home."""
    for agent in agents:
        agent.add_action_sequence(
            [
                JointAction(target_joint_positions=JOINT_TARGETS_A, tolerance=0.05),
                JointAction(target_joint_positions=JOINT_TARGETS_B, tolerance=0.05),
                JointAction(target_joint_positions=JOINT_TARGETS_HOME, tolerance=0.05),
            ]
        )


# ---------------------------------------------------------------------------
# Benchmark runner
# ---------------------------------------------------------------------------


def run_benchmark(
    num_agents: int,
    duration: float,
    config_path: Optional[str] = None,
    scenario: Optional[str] = None,
) -> Dict[str, Any]:
    """Run a single arm benchmark and return metrics dict.

    All parameters (physics, mass, timestep, etc.) are loaded from config/scenario.
    """
    proc = psutil.Process()
    tracemalloc.start()
    force_cleanup()
    ensure_disconnected()

    # Load config
    if config_path is None:
        config_path = os.path.join(os.path.dirname(__file__), "configs", "arm.yaml")
    config = load_config(config_path, scenario)

    sim_config = config.get("simulation", {})
    arm_config = config.get("arm", {})

    # Resolve arm parameters
    timestep = sim_config.get("timestep", 0.01)
    physics = sim_config.get("physics", True)
    mass_raw = arm_config.get("mass", None)
    mass: Optional[float] = None if mass_raw is None else float(mass_raw)
    urdf_path = arm_config.get("urdf_path", "../robots/arm_robot.urdf")
    spacing = arm_config.get("spacing", 1.5)

    # Resolve URDF path relative to benchmark/
    if not os.path.isabs(urdf_path):
        urdf_path = os.path.join(os.path.dirname(__file__), urdf_path)

    # Build SimulationParams
    params = SimulationParams(
        gui=False,
        timestep=timestep,
        duration=duration,
        target_rtf=sim_config.get("target_rtf", 0.0),
        physics=physics,
        monitor=sim_config.get("monitor", False),
        enable_monitor_gui=sim_config.get("enable_monitor_gui", False),
        enable_time_profiling=sim_config.get("enable_time_profiling", False),
        log_level=sim_config.get("log_level", "WARN"),
        collision_check_frequency=sim_config.get("collision_check_frequency", 0),
        ignore_static_collision=sim_config.get("ignore_static_collision", True),
    )

    sim_core = MultiRobotSimulationCore(params)

    # Memory before spawning
    mem_before = get_memory_info()

    # Spawn
    cpu_spawn_start = cpu_time_s(proc)
    wall_spawn_start = time.perf_counter()
    agents = spawn_arms(sim_core, num_agents, mass, urdf_path, spacing)
    wall_spawn_time = time.perf_counter() - wall_spawn_start
    cpu_spawn_time = cpu_time_s(proc) - cpu_spawn_start

    # Memory after spawning
    mem_after_spawn = get_memory_info()

    # Enqueue initial joint actions + refill callback
    enqueue_joint_cycle(agents)
    _agents = agents

    def refill_callback(sim_core_ref, dt):
        for agent in _agents:
            if agent.is_action_queue_empty():
                enqueue_joint_cycle([agent])

    sim_core.register_callback(refill_callback, frequency=2.0)

    # Warmup
    warmup_steps(sim_core, n=10)

    # Timed run
    cpu_sim_start = cpu_time_s(proc)
    wall_sim_start = time.perf_counter()
    sim_core.run_simulation(duration=duration)
    wall_sim_time = time.perf_counter() - wall_sim_start
    cpu_sim_time = cpu_time_s(proc) - cpu_sim_start

    # Memory after simulation
    mem_after_sim = get_memory_info()

    actual_steps = sim_core.step_count
    rtf = duration / wall_sim_time if wall_sim_time > 0 else 0.0
    avg_step_ms = (wall_sim_time / actual_steps * 1000.0) if actual_steps > 0 else 0.0
    total_joints = sum(a.get_num_joints() for a in agents)

    # Cleanup
    if p.isConnected():
        p.resetSimulation()
        p.disconnect()
    if tracemalloc.is_tracing():
        tracemalloc.stop()
    del agents, sim_core
    force_cleanup()

    return {
        "num_agents": num_agents,
        "total_joints": total_joints,
        "mass": mass,
        "mass_source": "explicit" if mass is not None else "urdf",
        "mode": "kinematic" if mass == 0.0 else "physics",
        "physics_engine": physics,
        "timestep": timestep,
        "duration_s": duration,
        "scenario": scenario,
        "spawn_time_s": wall_spawn_time,
        "spawn_cpu_s": cpu_spawn_time,
        "spawn_cpu_percent": (cpu_spawn_time / wall_spawn_time * 100.0) if wall_spawn_time > 0 else 0.0,
        "simulation_wall_s": wall_sim_time,
        "simulation_cpu_s": cpu_sim_time,
        "simulation_cpu_percent": (cpu_sim_time / wall_sim_time * 100.0) if wall_sim_time > 0 else 0.0,
        "actual_steps": actual_steps,
        "real_time_factor": rtf,
        "avg_step_time_ms": avg_step_ms,
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


# ---------------------------------------------------------------------------
# Main (pure worker)
# ---------------------------------------------------------------------------


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Robot Arm Benchmark Worker")
    parser.add_argument("--agents", type=int, required=True, help="Number of arm robots")
    parser.add_argument("--duration", type=float, required=True, help="Simulation duration in seconds")
    parser.add_argument("--config", type=str, default=None, help="Path to benchmark config file")
    parser.add_argument("--scenario", type=str, default=None, help="Scenario name from config (e.g., physics, kinematic)")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    result = run_benchmark(
        num_agents=args.agents,
        duration=args.duration,
        config_path=args.config,
        scenario=args.scenario,
    )

    # Output JSON to stdout
    print(json.dumps(result))
