#!/usr/bin/env python3
"""
arm_benchmark.py
Performance benchmark for robot arm joint control in physics and kinematic modes.

Measures:
- step_once() time with N arm robots executing JointAction sequences
- Physics (mass=1.0, setJointMotorControl2) vs Kinematic (mass=0.0, resetJointState)
- Scaling behaviour: 1, 10, 50, 100 arms

Usage:
    python benchmark/arm_benchmark.py
    python benchmark/arm_benchmark.py --arms 50 --duration 5
    python benchmark/arm_benchmark.py --arms 100 --mode kinematic
"""
import os
import sys
import time
import math
import json
import argparse
import platform
from typing import Dict, Any, List

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pybullet as p
import psutil

from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent import Agent
from pybullet_fleet.geometry import Pose
from pybullet_fleet.action import JointAction


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ARM_URDF = os.path.join(os.path.dirname(__file__), "../robots/arm_robot.urdf")

# Joint target sequences for cycling
JOINT_TARGETS_A = [1.0, 1.0, 1.0, 0.5]
JOINT_TARGETS_B = [-1.0, -0.5, 0.5, -0.3]
JOINT_TARGETS_HOME = [0.0, 0.0, 0.0, 0.0]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def get_system_info() -> Dict[str, Any]:
    """Collect system info for reproducibility."""
    info = {
        "platform": platform.system(),
        "processor": platform.processor(),
        "python_version": platform.python_version(),
        "cpu_count": psutil.cpu_count(logical=False),
        "cpu_count_logical": psutil.cpu_count(logical=True),
        "total_memory_gb": round(psutil.virtual_memory().total / (1024**3), 2),
    }
    if platform.system() == "Linux":
        try:
            import subprocess

            result = subprocess.run(["lscpu"], capture_output=True, text=True, timeout=2)
            for line in result.stdout.split("\n"):
                if "Model name:" in line:
                    info["cpu_model"] = line.split(":", 1)[1].strip()
        except (FileNotFoundError, IOError):
            pass
    return info


def spawn_arms(
    sim_core: MultiRobotSimulationCore,
    num_arms: int,
    mass: float,
) -> List[Agent]:
    """Spawn arm robots in a grid layout."""
    agents = []
    grid_size = max(1, int(math.ceil(math.sqrt(num_arms))))
    spacing = 1.5  # meters between arms (enough clearance)

    for i in range(num_arms):
        row = i // grid_size
        col = i % grid_size
        x = col * spacing
        y = row * spacing

        agent = Agent.from_urdf(
            urdf_path=ARM_URDF,
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


def run_single_bench(
    num_arms: int,
    mass: float,
    duration: float,
    timestep: float,
    physics: bool,
) -> Dict[str, Any]:
    """Run one benchmark configuration and return metrics."""

    if p.isConnected():
        p.disconnect()

    params = SimulationParams(
        gui=False,
        timestep=timestep,
        duration=duration,
        target_rtf=0.0,  # max speed
        physics=physics,
        monitor=False,
        enable_monitor_gui=False,
        log_level="WARN",
        collision_check_frequency=0,  # disable collision for pure joint perf
    )

    sim_core = MultiRobotSimulationCore(params)

    # Spawn
    t_spawn_start = time.perf_counter()
    agents = spawn_arms(sim_core, num_arms, mass)
    t_spawn = time.perf_counter() - t_spawn_start

    # Enqueue initial joint actions
    enqueue_joint_cycle(agents)

    # Callback to re-enqueue actions when idle
    _agents = agents  # bind for closure

    def refill_callback(sim_core_ref, dt):
        for agent in _agents:
            if agent.is_action_queue_empty():
                enqueue_joint_cycle([agent])

    sim_core.register_callback(refill_callback, frequency=2.0)

    # Warmup (10 steps)
    for _ in range(10):
        sim_core.step_once()

    # Timed run
    proc = psutil.Process()
    mem_before = proc.memory_info().rss / 1024 / 1024

    t_sim_start = time.perf_counter()
    sim_core.run_simulation(duration=duration)
    t_sim = time.perf_counter() - t_sim_start

    mem_after = proc.memory_info().rss / 1024 / 1024

    actual_steps = sim_core.step_count
    rtf = duration / t_sim if t_sim > 0 else 0.0
    avg_step_ms = (t_sim / actual_steps * 1000.0) if actual_steps > 0 else 0.0

    # Count how many joints total
    total_joints = sum(a.get_num_joints() for a in agents)

    # Cleanup
    if p.isConnected():
        p.resetSimulation()
        p.disconnect()
    del agents, sim_core

    return {
        "num_arms": num_arms,
        "total_joints": total_joints,
        "mass": mass,
        "mode": "kinematic" if mass == 0.0 else "physics",
        "physics_engine": physics,
        "timestep": timestep,
        "duration_s": duration,
        "spawn_time_s": round(t_spawn, 4),
        "sim_wall_s": round(t_sim, 4),
        "actual_steps": actual_steps,
        "real_time_factor": round(rtf, 2),
        "avg_step_ms": round(avg_step_ms, 3),
        "mem_delta_mb": round(mem_after - mem_before, 2),
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description="Robot Arm Performance Benchmark")
    parser.add_argument("--arms", type=int, default=None, help="Number of arms (single run)")
    parser.add_argument("--duration", type=float, default=5.0, help="Simulation duration (seconds)")
    parser.add_argument("--timestep", type=float, default=0.01, help="Simulation timestep (seconds)")
    parser.add_argument("--mode", choices=["physics", "kinematic", "both"], default="both", help="Joint control mode")
    parser.add_argument("--sweep", nargs="+", type=int, default=None, help="Sweep multiple arm counts")
    parser.add_argument("--json", action="store_true", help="Output raw JSON only")
    args = parser.parse_args()

    # Determine arm counts
    if args.sweep:
        arm_counts = args.sweep
    elif args.arms:
        arm_counts = [args.arms]
    else:
        arm_counts = [1, 10, 50, 100]

    # Determine modes
    if args.mode == "physics":
        modes = [("physics", 1.0, True)]
    elif args.mode == "kinematic":
        modes = [("kinematic", 0.0, False)]
    else:
        modes = [("physics", 1.0, True), ("kinematic", 0.0, False)]

    results = []

    if not args.json:
        print("=" * 80)
        print("Robot Arm Performance Benchmark")
        print("=" * 80)
        system = get_system_info()
        print(f"CPU: {system.get('cpu_model', system['processor'])}")
        print(f"Cores: {system['cpu_count']}P / {system['cpu_count_logical']}L")
        print(f"RAM: {system['total_memory_gb']} GB")
        print(f"Duration: {args.duration}s, Timestep: {args.timestep}s")
        print(f"Arm counts: {arm_counts}")
        print(f"Modes: {[m[0] for m in modes]}")
        print("-" * 80)
        print(
            f"{'Arms':>5} {'Joints':>7} {'Mode':>10} {'Steps':>7} " f"{'RTF':>7} {'ms/step':>8} {'Spawn(s)':>8} {'Sim(s)':>7}"
        )
        print("-" * 80)

    for n_arms in arm_counts:
        for mode_name, mass, physics in modes:
            result = run_single_bench(
                num_arms=n_arms,
                mass=mass,
                duration=args.duration,
                timestep=args.timestep,
                physics=physics,
            )
            results.append(result)

            if not args.json:
                print(
                    f"{result['num_arms']:5d} "
                    f"{result['total_joints']:7d} "
                    f"{result['mode']:>10} "
                    f"{result['actual_steps']:7d} "
                    f"{result['real_time_factor']:7.1f}x "
                    f"{result['avg_step_ms']:8.3f} "
                    f"{result['spawn_time_s']:8.3f} "
                    f"{result['sim_wall_s']:7.2f}"
                )

    if not args.json:
        print("=" * 80)

        # Summary comparison if both modes ran
        if len(modes) == 2 and len(arm_counts) > 0:
            print("\nPhysics vs Kinematic comparison:")
            print(f"{'Arms':>5} {'Phys ms/step':>13} {'Kine ms/step':>13} {'Ratio':>7}")
            print("-" * 42)
            for n_arms in arm_counts:
                phys = [r for r in results if r["num_arms"] == n_arms and r["mode"] == "physics"]
                kine = [r for r in results if r["num_arms"] == n_arms and r["mode"] == "kinematic"]
                if phys and kine:
                    p_ms = phys[0]["avg_step_ms"]
                    k_ms = kine[0]["avg_step_ms"]
                    ratio = p_ms / k_ms if k_ms > 0 else float("inf")
                    print(f"{n_arms:5d} {p_ms:13.3f} {k_ms:13.3f} {ratio:6.2f}x")

    # Always save JSON
    output = {
        "benchmark": "arm_performance",
        "system_info": get_system_info(),
        "config": {
            "duration": args.duration,
            "timestep": args.timestep,
            "arm_counts": arm_counts,
            "modes": [m[0] for m in modes],
        },
        "results": results,
    }

    output_dir = os.path.join(os.path.dirname(__file__), "results")
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, "arm_benchmark.json")
    with open(output_file, "w") as f:
        json.dump(output, f, indent=2)

    if args.json:
        print(json.dumps(output, indent=2))
    else:
        print(f"\nResults saved to: {output_file}")


if __name__ == "__main__":
    main()
