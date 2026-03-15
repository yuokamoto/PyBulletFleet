#!/usr/bin/env python3
"""
arm_joint_update.py
Detailed profiling of arm robot joint update performance.

Measures per-component timing when arm robots execute JointAction sequences,
comparing physics (motor control) vs kinematic (resetJointState) modes.

Analysis Methods:
    1. builtin (default) — step_once(return_profiling=True) component breakdown
    2. cprofile          — Function-level cProfile analysis
    3. scaling           — Step time vs arm count (1, 5, 10, 25, 50)

Usage:
    # Built-in profiling (recommended)
    python benchmark/profiling/arm_joint_update.py --arms 10 --steps 100

    # cProfile analysis
    python benchmark/profiling/arm_joint_update.py --arms 10 --steps 100 --test cprofile

    # Scaling analysis
    python benchmark/profiling/arm_joint_update.py --steps 50 --test scaling

    # All analyses
    python benchmark/profiling/arm_joint_update.py --arms 10 --steps 100 --test all

Related Files:
    - agent_update.py: Mobile agent update profiling
    - simulation_profiler.py: Overall step_once() component breakdown
    - arm_benchmark.py: Overall arm benchmark (RTF, memory)
"""
import os
import sys
import math
import time
import cProfile
import pstats
import io
import argparse
import statistics
from typing import List, Optional, Dict

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import pybullet as p
from benchmark.tools import suppress_stdout
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent import Agent
from pybullet_fleet.geometry import Pose
from pybullet_fleet.action import JointAction

ARM_URDF = os.path.join(os.path.dirname(__file__), "../../robots/arm_robot.urdf")

JOINT_TARGETS_A = [1.0, 1.0, 1.0, 0.5]
JOINT_TARGETS_B = [-1.0, -0.5, 0.5, -0.3]
JOINT_TARGETS_HOME = [0.0, 0.0, 0.0, 0.0]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _create_sim(physics: bool = True, timestep: float = 0.01) -> MultiRobotSimulationCore:
    """Create a headless sim for profiling."""
    if p.isConnected():
        p.disconnect()

    params = SimulationParams(
        gui=False,
        timestep=timestep,
        duration=0,
        target_rtf=0,
        physics=physics,
        monitor=False,
        enable_monitor_gui=False,
        log_level="error",
        collision_check_frequency=0,  # disable collision for pure joint perf
    )
    with suppress_stdout():
        return MultiRobotSimulationCore(params)


def _spawn_arms(
    sim_core: MultiRobotSimulationCore,
    num_arms: int,
    mass: Optional[float] = None,
) -> List[Agent]:
    """Spawn arm robots in a grid."""
    agents = []
    grid_size = max(1, int(math.ceil(math.sqrt(num_arms))))
    spacing = 1.5

    with suppress_stdout():
        for i in range(num_arms):
            x = (i % grid_size) * spacing
            y = (i // grid_size) * spacing
            agent = Agent.from_urdf(
                urdf_path=ARM_URDF,
                pose=Pose.from_xyz(x, y, 0.0),
                mass=mass,
                use_fixed_base=True,
                sim_core=sim_core,
            )
            agents.append(agent)
    return agents


def _enqueue_joint_cycle(agents: List[Agent]):
    """Queue A → B → home joint actions."""
    for agent in agents:
        agent.add_action_sequence(
            [
                JointAction(target_joint_positions=JOINT_TARGETS_A, tolerance=0.05),
                JointAction(target_joint_positions=JOINT_TARGETS_B, tolerance=0.05),
                JointAction(target_joint_positions=JOINT_TARGETS_HOME, tolerance=0.05),
            ]
        )


def _keep_busy_callback(agents: List[Agent]):
    """Return a callback that re-enqueues actions when idle."""

    def cb(sim_core_ref, dt):
        for agent in agents:
            if agent.is_action_queue_empty():
                _enqueue_joint_cycle([agent])

    return cb


# ---------------------------------------------------------------------------
# Analysis methods
# ---------------------------------------------------------------------------


def profile_builtin(
    num_arms: int,
    num_steps: int,
    physics: bool = True,
    mass: Optional[float] = None,
):
    """Built-in profiling via step_once(return_profiling=True)."""
    mode_label = "kinematic" if mass == 0.0 else "physics"
    print(f"\n{'='*70}")
    print(f"Built-in Profiling ({mode_label}): {num_arms} arms, {num_steps} steps")
    print(f"{'='*70}\n")

    sim_core = _create_sim(physics=physics)
    agents = _spawn_arms(sim_core, num_arms, mass=mass)
    _enqueue_joint_cycle(agents)
    sim_core.register_callback(_keep_busy_callback(agents), frequency=2.0)

    # Warmup
    for _ in range(10):
        sim_core.step_once()

    # Collect
    timings: Dict[str, list] = {}
    for _ in range(num_steps):
        data = sim_core.step_once(return_profiling=True)
        if data:
            for key, value in data.items():
                timings.setdefault(key, []).append(value)

    # Print
    def fmt_stats(vals):
        return {
            "mean": statistics.mean(vals),
            "median": statistics.median(vals),
            "stdev": statistics.stdev(vals) if len(vals) > 1 else 0,
            "min": min(vals),
            "max": max(vals),
        }

    total_mean = statistics.mean(timings.get("total", [1.0]))

    print(f"{'Component':<22} {'Mean (ms)':>10} {'Median':>10} {'StdDev':>10} {'Share':>8}")
    print("-" * 64)
    for component, vals in timings.items():
        s = fmt_stats(vals)
        pct = (s["mean"] / total_mean * 100) if total_mean > 0 else 0
        print(f"{component:<22} {s['mean']:10.3f} {s['median']:10.3f} {s['stdev']:10.3f} {pct:7.1f}%")

    p.disconnect()
    return timings


def profile_cprofile(
    num_arms: int,
    num_steps: int,
    physics: bool = True,
    mass: Optional[float] = None,
):
    """cProfile function-level analysis."""
    mode_label = "kinematic" if mass == 0.0 else "physics"
    print(f"\n{'='*70}")
    print(f"cProfile ({mode_label}): {num_arms} arms, {num_steps} steps")
    print(f"{'='*70}\n")

    sim_core = _create_sim(physics=physics)
    agents = _spawn_arms(sim_core, num_arms, mass=mass)
    _enqueue_joint_cycle(agents)
    sim_core.register_callback(_keep_busy_callback(agents), frequency=2.0)

    # Warmup
    for _ in range(10):
        sim_core.step_once()

    # Profile
    profiler = cProfile.Profile()
    profiler.enable()
    for _ in range(num_steps):
        sim_core.step_once()
    profiler.disable()

    s = io.StringIO()
    stats = pstats.Stats(profiler, stream=s)
    stats.sort_stats("cumulative")
    stats.print_stats(30)
    print(s.getvalue())

    p.disconnect()
    return stats


def profile_scaling(num_steps: int = 50):
    """Measure step time as arm count increases (physics vs kinematic)."""
    arm_counts = [1, 5, 10, 25, 50]

    print(f"\n{'='*70}")
    print(f"Scaling Analysis: {arm_counts} arms, {num_steps} steps each")
    print(f"{'='*70}\n")

    # physics  : mass=None, physics=True  → setJointMotorControl2 + stepSimulation()
    # kinematic: mass=0.0,  physics=False → resetJointState only (no stepSimulation)
    # This reflects realistic usage: kinematic skips physics for speed.
    configs = [
        ("physics", True, None),
        ("kinematic", False, 0.0),
    ]

    results = {}

    for mode_label, physics, mass in configs:
        results[mode_label] = []
        for n in arm_counts:
            sim_core = _create_sim(physics=physics)
            agents = _spawn_arms(sim_core, n, mass=mass)
            _enqueue_joint_cycle(agents)
            sim_core.register_callback(_keep_busy_callback(agents), frequency=2.0)

            # Warmup
            for _ in range(10):
                sim_core.step_once()

            # Timed run
            t0 = time.perf_counter()
            for _ in range(num_steps):
                sim_core.step_once()
            elapsed = time.perf_counter() - t0

            avg_ms = elapsed / num_steps * 1000.0
            total_joints = sum(a.get_num_joints() for a in agents)

            results[mode_label].append(
                {
                    "arms": n,
                    "joints": total_joints,
                    "avg_step_ms": avg_ms,
                }
            )

            p.disconnect()

    # Print comparison table
    print(f"{'Arms':>5} {'Joints':>7} {'Physics (ms)':>13} {'Kinematic (ms)':>15} {'Ratio':>7}")
    print("-" * 52)
    for i, n in enumerate(arm_counts):
        p_ms = results["physics"][i]["avg_step_ms"]
        k_ms = results["kinematic"][i]["avg_step_ms"]
        joints = results["physics"][i]["joints"]
        ratio = p_ms / k_ms if k_ms > 0 else float("inf")
        print(f"{n:5d} {joints:7d} {p_ms:13.3f} {k_ms:15.3f} {ratio:6.2f}x")

    return results


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description="Arm Joint Update Profiling")
    parser.add_argument("--arms", type=int, default=10, help="Number of arms (default: 10)")
    parser.add_argument("--steps", type=int, default=100, help="Number of steps (default: 100)")
    parser.add_argument(
        "--test",
        choices=["all", "builtin", "cprofile", "scaling"],
        default="builtin",
        help="Which analysis to run (default: builtin)",
    )
    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("Arm Joint Update Profiling")
    print("=" * 70)

    if args.test in ["all", "builtin"]:
        profile_builtin(args.arms, args.steps, physics=True, mass=None)
        profile_builtin(args.arms, args.steps, physics=False, mass=0.0)

    if args.test in ["all", "cprofile"]:
        profile_cprofile(args.arms, args.steps, physics=True, mass=None)
        profile_cprofile(args.arms, args.steps, physics=False, mass=0.0)

    if args.test in ["all", "scaling"]:
        profile_scaling(num_steps=args.steps)

    print("\n" + "=" * 70)
    print("Profiling Complete!")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    main()
