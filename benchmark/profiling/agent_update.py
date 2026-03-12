#!/usr/bin/env python3
"""
agent_update.py
Detailed performance analysis tool for Agent.update()

Overview:
    Agent.update() is the most frequently called operation, invoked every frame for all agents.
    This tool uses 5 different analysis methods to identify bottlenecks.

Analysis Methods:
    1. cProfile analysis (--test=cprofile)
       - Function-level detailed analysis
       - Automatically records call times for all functions
       - Best for identifying bottleneck candidates
       - Note: Rare segfaults may occur due to compatibility issues with PyBullet

    2. Manual timing analysis (--test=manual)
       - Measures only specific methods (minimal overhead)
       - Statistical information (mean, median, max)
       - Best for verifying optimization effects
       - Recommended: Use this when stability is a priority

    3. PyBullet API call analysis (--test=pybullet)
       - PyBullet C++ API call counts and timings
       - Measures cost at the Python/C++ boundary
       - Best for optimizing API usage patterns

    4. Stationary vs moving cost comparison (--test=stationary)
       - Compares update() cost between agents without goals (stationary) and with goals (moving)
       - Measures performance difference based on presence of movement/joint update processing
       - Useful for confirming optimization effects when many agents are stationary

    5. Motion Mode comparison (--test=motion_modes)
       - Performance comparison of DIFFERENTIAL vs OMNIDIRECTIONAL
       - Reference data for mode selection

Troubleshooting:
    - If a Segmentation Fault occurs:
      -> Use --test=manual or --test=stationary
      -> Caused by compatibility issues between cProfile and PyBullet C++ extensions

    - If "Xft: locking error too many file unlocks" appears:
      -> Safe to ignore (X11 GUI is disabled, no actual impact)
      -> Or set environment variable: export QT_X11_NO_MITSHM=1

Usage:
    # Run all analyses (includes cProfile)
    python agent_update.py --agents=1000 --updates=100

    # cProfile only (bottleneck search)
    python agent_update.py --agents=1000 --test=cprofile

    # Manual timing only (detailed measurement, recommended)
    python agent_update.py --agents=1000 --updates=100 --test=manual

    # PyBullet API analysis only
    python agent_update.py --agents=100 --updates=10 --test=pybullet

    # Stationary vs moving only
    python agent_update.py --agents=1000 --test=stationary

    # Motion Mode comparison only
    python agent_update.py --agents=1000 --test=motion_modes

Example Output:
    Manual Timing:
        Component                 Mean (us)    Median (us)  Max (us)
        ----------------------------------------------------------------------
        total                     120.50       118.30       250.10
        update_differential       80.20        78.50        180.00
        update_actions            35.10        34.00        90.00

    PyBullet API:
        Function                                  Calls      Total (ms)   Avg (us)
        ---------------------------------------------------------------------------
        resetBasePositionAndOrientation           1000       45.20        45.20
        getBasePositionAndOrientation             2000       30.50        15.25

    Stationary vs Moving:
        Overhead ratio (moving/stationary): 7.93x
        Potential savings if 50% stationary: 67.85 ms

Related Files:
    - agent_manager_set_goal.py: Profiling of set_goal_pose()
    - step_breakdown.py: Simulation-wide step breakdown
    - collision_check.py: Profiling of collision detection
"""
import os
import sys
import time
import cProfile
import pstats
import io
from collections import defaultdict

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import pybullet as p
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent import Agent, AgentSpawnParams, MotionMode
from pybullet_fleet.geometry import Pose


def create_test_agents(num_agents: int, motion_mode: MotionMode = MotionMode.DIFFERENTIAL):
    """Generate test Agents"""
    # Clean up any leftover connection before creating a fresh sim_core.
    if p.isConnected():
        p.disconnect()

    # Create sim_core - Load from profiling config
    config_path = os.path.join(os.path.dirname(__file__), "profiling_config.yaml")
    params = SimulationParams.from_config(config_path)

    sim_core = MultiRobotSimulationCore(params)

    robot_urdf = os.path.join(os.path.dirname(__file__), "../../robots/simple_cube.urdf")

    agents = []
    for i in range(num_agents):
        x = (i % 100) * 0.5
        y = (i // 100) * 0.5
        spawn_params = AgentSpawnParams(
            urdf_path=robot_urdf,
            initial_pose=Pose.from_xyz(x, y, 0.1),
            motion_mode=motion_mode,
            max_linear_vel=1.0,
            max_angular_vel=1.0,
            mass=0.0,  # Kinematic
        )
        agent = Agent.from_params(spawn_params, sim_core=sim_core)

        # Set goal to make agents "moving"
        goal_x = x + 5.0
        goal_y = y + 5.0
        agent.set_goal_pose(Pose.from_xyz(goal_x, goal_y, 0.1))

        agents.append(agent)

    return agents, sim_core


def profile_with_cprofile(num_agents: int = 1000, num_updates: int = 1, motion_mode: MotionMode = MotionMode.DIFFERENTIAL):
    """Profile using cProfile"""
    mode_name = "DIFFERENTIAL" if motion_mode == MotionMode.DIFFERENTIAL else "OMNIDIRECTIONAL"
    print(f"\n{'='*70}")
    print(f"cProfile Profiling ({mode_name}): {num_agents} agents, {num_updates} update(s)")
    print(f"{'='*70}\n")

    agents, sim_core = create_test_agents(num_agents, motion_mode)

    # Warm-up
    for agent in agents:
        agent.update(dt=0.01)

    # Profile
    profiler = cProfile.Profile()
    profiler.enable()

    for _ in range(num_updates):
        for agent in agents:
            agent.update(dt=0.01)

    profiler.disable()

    # Results
    s = io.StringIO()
    stats = pstats.Stats(profiler, stream=s)
    stats.sort_stats("cumulative")
    stats.print_stats(30)  # Top 30 functions

    print(s.getvalue())

    # Cleanup
    p.disconnect()

    return stats


def profile_manual_timing(num_agents: int = 1000, num_updates: int = 100):
    """Manually measure timings (more detailed)"""
    print(f"\n{'='*70}")
    print(f"Manual Timing: {num_agents} agents, {num_updates} update(s)")
    print(f"{'='*70}\n")

    agents, sim_core = create_test_agents(num_agents)

    # Timing buckets
    timings = defaultdict(list)

    # Only wrap leaf-level internal methods, NOT update() itself.
    # Wrapping both update() and its children (e.g. _update_differential)
    # would double-count perf_counter overhead since update() calls these
    # methods internally. "total" is computed from the sum of leaf timings.
    original_update_actions = Agent._update_actions
    original_update_differential = Agent._update_differential
    original_update_omnidirectional = Agent._update_omnidirectional

    def timed_update_actions(self, dt):
        t0 = time.perf_counter()
        original_update_actions(self, dt)
        elapsed = (time.perf_counter() - t0) * 1000.0
        timings["update_actions"].append(elapsed)

    def timed_update_differential(self, dt):
        t0 = time.perf_counter()
        original_update_differential(self, dt)
        elapsed = (time.perf_counter() - t0) * 1000.0
        timings["update_differential"].append(elapsed)

    def timed_update_omnidirectional(self, dt):
        t0 = time.perf_counter()
        original_update_omnidirectional(self, dt)
        elapsed = (time.perf_counter() - t0) * 1000.0
        timings["update_omnidirectional"].append(elapsed)

    Agent._update_actions = timed_update_actions
    Agent._update_differential = timed_update_differential
    Agent._update_omnidirectional = timed_update_omnidirectional

    # Run updates (update() is NOT wrapped; total is derived from leaf timings)
    for i in range(num_updates):
        for agent in agents:
            agent.update(dt=0.01)

    # Restore original methods
    Agent._update_actions = original_update_actions
    Agent._update_differential = original_update_differential
    Agent._update_omnidirectional = original_update_omnidirectional

    # Compute "total" from leaf timings to avoid double-counting overhead
    motion_key = "update_differential" if timings["update_differential"] else "update_omnidirectional"
    motion_vals = timings.get(motion_key, [])
    action_vals = timings.get("update_actions", [])
    if action_vals and motion_vals and len(action_vals) == len(motion_vals):
        timings["total"] = [a + m for a, m in zip(action_vals, motion_vals)]
    elif motion_vals:
        timings["total"] = list(motion_vals)
    elif action_vals:
        timings["total"] = list(action_vals)

    # Calculate statistics
    print("Timing Statistics (per update call):\n")
    print(f"{'Component':<25} {'Mean (μs)':<12} {'Median (μs)':<12} {'Max (μs)':<12} {'Calls':<10}")
    print("-" * 70)

    for key in ["total", "update_actions", "update_differential", "update_omnidirectional"]:
        if key in timings and len(timings[key]) > 0:
            values = timings[key]
            mean_us = sum(values) / len(values) * 1000.0  # μs
            median_us = sorted(values)[len(values) // 2] * 1000.0
            max_us = max(values) * 1000.0
            print(f"{key:<25} {mean_us:<12.2f} {median_us:<12.2f} {max_us:<12.2f} {len(values):<10}")

    # Summary
    total_time_ms = sum(timings["total"])
    total_calls = len(timings["total"])
    avg_time_us = (total_time_ms / total_calls) * 1000.0 if total_calls > 0 else 0

    print("\n" + "=" * 70)
    print(f"Total update time: {total_time_ms:.2f} ms")
    print(f"Total update calls: {total_calls}")
    print(f"Average time per update: {avg_time_us:.2f} μs")
    print(f"For {num_agents} agents: {avg_time_us * num_agents / 1000.0:.2f} ms per frame")
    print("=" * 70)

    # Cleanup
    p.disconnect()

    return timings


def count_pybullet_calls(num_agents: int = 100, num_updates: int = 10):
    """Count PyBullet API call frequency"""
    print(f"\n{'='*70}")
    print(f"PyBullet API Call Counting: {num_agents} agents, {num_updates} update(s)")
    print(f"{'='*70}\n")

    agents, sim_core = create_test_agents(num_agents)

    # Wrap PyBullet functions to count calls
    call_counts = defaultdict(int)
    call_times = defaultdict(list)

    original_funcs = {}
    pybullet_funcs = [
        "getBasePositionAndOrientation",
        "resetBasePositionAndOrientation",
        "getBaseVelocity",
        "resetBaseVelocity",
        "getDynamicsInfo",
        "getJointState",
        "setJointMotorControl2",
    ]

    for func_name in pybullet_funcs:
        if hasattr(p, func_name):
            original_funcs[func_name] = getattr(p, func_name)

            def make_wrapper(fname, orig_func):
                def wrapper(*args, **kwargs):
                    t0 = time.perf_counter()
                    result = orig_func(*args, **kwargs)
                    elapsed = (time.perf_counter() - t0) * 1e6  # μs
                    call_counts[fname] += 1
                    call_times[fname].append(elapsed)
                    return result

                return wrapper

            setattr(p, func_name, make_wrapper(func_name, original_funcs[func_name]))

    # Run updates
    for _ in range(num_updates):
        for agent in agents:
            agent.update(dt=0.01)

    # Restore original functions
    for func_name, orig_func in original_funcs.items():
        setattr(p, func_name, orig_func)

    # Print results
    print(f"{'Function':<40} {'Calls':<10} {'Total (ms)':<12} {'Avg (μs)':<12}")
    print("-" * 75)

    total_pybullet_time = 0.0
    for func_name in sorted(call_counts.keys(), key=lambda k: sum(call_times[k]), reverse=True):
        count = call_counts[func_name]
        times = call_times[func_name]
        total_ms = sum(times) / 1000.0
        avg_us = sum(times) / len(times) if times else 0
        total_pybullet_time += total_ms
        print(f"{func_name:<40} {count:<10} {total_ms:<12.2f} {avg_us:<12.2f}")

    print("-" * 75)
    print(f"{'TOTAL PyBullet API time:':<40} {'':<10} {total_pybullet_time:<12.2f} ms")

    total_updates = num_agents * num_updates
    print(f"\nPer update call: {total_pybullet_time / total_updates * 1000.0:.2f} μs")
    print(f"For {num_agents} agents per frame: {total_pybullet_time / num_updates:.2f} ms")

    # Cleanup
    p.disconnect()

    return call_counts, call_times


def analyze_stationary_vs_moving(num_agents: int = 1000):
    """Stationary vs moving Agent.update() cost comparison

    Compares update() processing time between agents without goals (stationary)
    and agents moving toward goals.

    Stationary: Skips movement/joint update processing (early return)
    Moving: Executes all processing including trajectory tracking, velocity calculation, pose update

    Useful for confirming optimization effects when many agents are stationary.
    """
    print(f"\n{'='*70}")
    print(f"Stationary vs Moving Analysis: {num_agents} agents")
    print(f"{'='*70}\n")

    # Use a single sim_core for both stationary and moving agents to avoid leakage
    if p.isConnected():
        p.disconnect()

    config_path = os.path.join(os.path.dirname(__file__), "profiling_config.yaml")
    params = SimulationParams.from_config(config_path)
    sim_core = MultiRobotSimulationCore(params)

    robot_urdf = os.path.join(os.path.dirname(__file__), "../../robots/simple_cube.urdf")

    try:
        # Spawn stationary agents (no goal)
        agents_stationary = []
        for i in range(num_agents):
            x = (i % 100) * 0.5
            y = (i // 100) * 0.5
            spawn_params = AgentSpawnParams(
                urdf_path=robot_urdf,
                initial_pose=Pose.from_xyz(x, y, 0.1),
                motion_mode=MotionMode.DIFFERENTIAL,
                max_linear_vel=1.0,
                max_angular_vel=1.0,
                mass=0.0,
            )
            agent = Agent.from_params(spawn_params, sim_core=sim_core)
            # No goal set - agent will be stationary
            agents_stationary.append(agent)

        # Spawn moving agents (with goal, offset to avoid overlap)
        agents_moving = []
        for i in range(num_agents):
            x = (i % 100) * 0.5 + 100.0  # offset to avoid overlap
            y = (i // 100) * 0.5
            spawn_params = AgentSpawnParams(
                urdf_path=robot_urdf,
                initial_pose=Pose.from_xyz(x, y, 0.1),
                motion_mode=MotionMode.DIFFERENTIAL,
                max_linear_vel=1.0,
                max_angular_vel=1.0,
                mass=0.0,
            )
            agent = Agent.from_params(spawn_params, sim_core=sim_core)
            goal_x = x + 5.0
            goal_y = y + 5.0
            agent.set_goal_pose(Pose.from_xyz(goal_x, goal_y, 0.1))
            agents_moving.append(agent)

        # Time stationary updates
        t0 = time.perf_counter()
        for agent in agents_stationary:
            agent.update(dt=0.01)
        stationary_time = (time.perf_counter() - t0) * 1000.0  # ms

        # Time moving updates
        t0 = time.perf_counter()
        for agent in agents_moving:
            agent.update(dt=0.01)
        moving_time = (time.perf_counter() - t0) * 1000.0  # ms

        # Results
        print(f"Stationary agents ({num_agents}):")
        print(f"  Total time: {stationary_time:.2f} ms")
        print(f"  Per agent: {stationary_time / num_agents * 1000.0:.2f} μs")
        print()
        print(f"Moving agents ({num_agents}):")
        print(f"  Total time: {moving_time:.2f} ms")
        print(f"  Per agent: {moving_time / num_agents * 1000.0:.2f} μs")
        print()
        print(f"Overhead ratio (moving/stationary): {moving_time / stationary_time:.2f}x")
        print(f"Potential savings if 50% stationary: {(moving_time + stationary_time) / 2:.2f} ms")

        return {
            "stationary_ms": stationary_time,
            "moving_ms": moving_time,
            "ratio": moving_time / stationary_time,
        }
    finally:
        if p.isConnected():
            p.disconnect()


def analyze_motion_modes(num_agents: int = 1000):
    """DIFFERENTIAL vs OMNIDIRECTIONAL mode comparison"""
    print(f"\n{'='*70}")
    print(f"Motion Mode Comparison: {num_agents} agents")
    print(f"{'='*70}\n")

    results = {}

    for mode in [MotionMode.DIFFERENTIAL, MotionMode.OMNIDIRECTIONAL]:
        agents, sim_core = create_test_agents(num_agents, mode)

        # Measure update time (3 iterations for stability)
        times = []
        for _ in range(3):
            t0 = time.perf_counter()
            for agent in agents:
                agent.update(dt=0.01)
            elapsed = (time.perf_counter() - t0) * 1000.0
            times.append(elapsed)

        avg_time = sum(times) / len(times)
        per_agent = avg_time / num_agents * 1000.0  # μs

        mode_name = "DIFFERENTIAL" if mode == MotionMode.DIFFERENTIAL else "OMNIDIRECTIONAL"
        print(f"{mode_name}:")
        print(f"  Total time: {avg_time:.2f} ms")
        print(f"  Per agent: {per_agent:.2f} μs")
        print()

        results[mode_name] = {
            "total_ms": avg_time,
            "per_agent_us": per_agent,
        }

        p.disconnect()

    diff_time = results["DIFFERENTIAL"]["total_ms"]
    omni_time = results["OMNIDIRECTIONAL"]["total_ms"]
    ratio = diff_time / omni_time

    print(f"Performance ratio (DIFFERENTIAL/OMNIDIRECTIONAL): {ratio:.2f}x")

    if ratio > 1.0:
        print(f"  → OMNIDIRECTIONAL is {ratio:.2f}x faster")
    else:
        print(f"  → DIFFERENTIAL is {1/ratio:.2f}x faster")

    return results


def main():
    """Main execution"""
    import argparse

    parser = argparse.ArgumentParser(description="Profile Agent.update() performance")
    parser.add_argument("--agents", type=int, default=1000, help="Number of agents")
    parser.add_argument("--updates", type=int, default=100, help="Number of update iterations")
    parser.add_argument(
        "--test",
        choices=["all", "cprofile", "manual", "pybullet", "stationary", "motion_modes"],
        default="all",
        help="Which test to run",
    )

    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("Agent.update() Performance Profiling")
    print("=" * 70)

    if args.test in ["all", "stationary"]:
        analyze_stationary_vs_moving(args.agents)

    if args.test in ["all", "motion_modes"]:
        analyze_motion_modes(args.agents)

    if args.test in ["all", "manual"]:
        profile_manual_timing(args.agents, args.updates)

    if args.test in ["all", "pybullet"]:
        count_pybullet_calls(min(args.agents, 100), min(args.updates, 10))

    if args.test in ["all", "cprofile"]:
        # Run cProfile for both motion modes
        print("\n" + "=" * 70)
        print("cProfile Comparison: DIFFERENTIAL vs OMNIDIRECTIONAL")
        print("=" * 70)

        stats_diff = profile_with_cprofile(args.agents, num_updates=1, motion_mode=MotionMode.DIFFERENTIAL)
        stats_omni = profile_with_cprofile(args.agents, num_updates=1, motion_mode=MotionMode.OMNIDIRECTIONAL)

        print("\n" + "=" * 70)
        print("Key Differences:")
        print("=" * 70)
        print("\nDIFFERENTIAL mode calls update_differential()")
        print("OMNIDIRECTIONAL mode calls update_omnidirectional()")
        print("\nLook for differences in:")
        print("  - _update_differential vs _update_omnidirectional")
        print("  - resetBaseVelocity call patterns")
        print("  - Computation complexity differences")

    print("\n" + "=" * 70)
    print("Profiling Complete!")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    main()
