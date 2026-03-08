#!/usr/bin/env python3
"""
simulation_profiler.py
Detailed profiling tool for simulation steps

Overview:
    Profiles each component of step_once() to
    identify which parts are slow.

    Differences from performance_benchmark.py:
      - performance_benchmark.py: Overall performance measurement (JSON output)
      - simulation_profiler.py: Detailed per-component analysis (statistical output)

Measured Components:
    1. Agent Update: State update for all agents (trajectory tracking, kinematics)
    2. Collision Check: Collision detection (spatial hashing, AABB filtering)
    3. PyBullet Step: PyBullet physics simulation step
    4. Monitor Update: Data monitor update (optional)
    5. Other: Miscellaneous overhead

Analysis Methods:
    1. Built-in Profiling: Component measurement via step_once() return_profiling (default)
    2. cProfile: Bottleneck analysis across all functions
    3. Motion Mode Comparison: DIFFERENTIAL vs OMNIDIRECTIONAL

Usage:
    # Basic usage (Built-in Profiling, 1000 agents, 100 steps)
    python simulation_profiler.py --agents=1000 --steps=100

    # cProfile analysis
    python simulation_profiler.py --agents=1000 --steps=100 --test=cprofile

    # Motion Mode comparison
    python simulation_profiler.py --agents=1000 --steps=100 --test=motion_modes

    # All analyses
    python simulation_profiler.py --agents=1000 --steps=100 --test=all

Example Output:
    Simulation Step Breakdown for 1000 Agents (100 steps)
    ======================================================================

    Agent Update:
      Mean:      12.45ms ( 35.2%)
      Median:    12.38ms
      StdDev:     0.87ms
      Range:  [ 11.23, 15.67]ms

    Collision Check:
      Mean:       5.12ms ( 14.5%)  <- detailed analysis available in collision_check.py
      Median:     5.08ms
      Range:  [  4.89,  6.23]ms

    Pybullet Step:
      Mean:      15.34ms ( 43.4%)  <- physics computation cost
      Median:    15.21ms
      Range:  [ 14.78, 17.12]ms

    Total Step Time:
      Mean:      35.37ms (100.0%)
      Real-Time Factor: 28.3x

When to Use:
    - Identifying overall simulation bottlenecks -> this tool
    - Overall performance measurement (JSON output) -> performance_benchmark.py
    - Detailed analysis of Agent.update() -> agent_update.py
    - Detailed analysis of collision detection -> collision_check.py

Related Files:
    - performance_benchmark.py: Comprehensive benchmark (memory, CPU, JSON output)
    - agent_update.py: Detailed profiling of Agent.update()
    - collision_check.py: Detailed profiling of collision detection
"""
import os
import sys
import cProfile
import pstats
import io
import math
import statistics

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import pybullet as p
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent import Agent, AgentSpawnParams, MotionMode
from pybullet_fleet.geometry import Pose


def create_test_agents(
    num_agents: int, sim_core: MultiRobotSimulationCore, motion_mode: MotionMode = MotionMode.OMNIDIRECTIONAL
):
    """Generate test Agents directly (without using AgentManager)"""
    robot_urdf = os.path.join(os.path.dirname(__file__), "../../robots/simple_cube.urdf")

    agents = []
    grid_size = int(math.ceil(math.sqrt(num_agents)))

    for i in range(num_agents):
        x = (i % grid_size) * 1.0
        y = (i // grid_size) * 1.0

        spawn_params = AgentSpawnParams(
            urdf_path=robot_urdf,
            initial_pose=Pose.from_xyz(x, y, 0.1),
            motion_mode=motion_mode,
            max_linear_vel=2.0,
            max_angular_vel=3.0,
            mass=0.0,  # Kinematic
        )
        agent = Agent.from_params(spawn_params, sim_core=sim_core)

        # Set goal to make agents "moving"
        goal_x = x + 5.0
        goal_y = y + 5.0
        agent.set_goal_pose(Pose.from_xyz(goal_x, goal_y, 0.1))

        agents.append(agent)

    return agents


def profile_builtin_profiling(num_agents: int, num_steps: int = 100, motion_mode: MotionMode = MotionMode.OMNIDIRECTIONAL):
    """Built-in Profiling: Measure each component using step_once() return_profiling"""
    mode_name = "DIFFERENTIAL" if motion_mode == MotionMode.DIFFERENTIAL else "OMNIDIRECTIONAL"

    print(f"\n{'='*70}")
    print(f"Built-in Profiling ({mode_name}): {num_agents} agents, {num_steps} steps")
    print(f"{'='*70}\n")

    # Clean up any leftover connection before creating a fresh sim_core.
    # MultiRobotSimulationCore.setup_pybullet() handles p.connect(p.DIRECT).
    if p.isConnected():
        p.disconnect()

    # Setup - Load from profiling config
    config_path = os.path.join(os.path.dirname(__file__), "profiling_config.yaml")
    params = SimulationParams.from_config(config_path)

    sim_core = MultiRobotSimulationCore(params)

    # Spawn agents directly
    print(f"Spawning {num_agents} agents...")
    agents = create_test_agents(num_agents, sim_core, motion_mode)
    print(f"Spawned {len(agents)} agents")

    # Collect timing data using step_once(return_profiling=True)
    timings = {
        "agent_update": [],
        "callbacks": [],
        "step_simulation": [],
        "collision_check": [],
        "monitor_update": [],
        "total": [],
    }

    print(f"Profiling {num_steps} steps...")

    for step in range(num_steps):
        # Get profiling data directly from step_once()
        timing_data = sim_core.step_once(return_profiling=True)

        if timing_data:
            for key, value in timing_data.items():
                if key in timings:
                    timings[key].append(value)

    # Calculate statistics
    def stats(data):
        return {
            "mean": statistics.mean(data),
            "median": statistics.median(data),
            "stdev": statistics.stdev(data) if len(data) > 1 else 0,
            "min": min(data),
            "max": max(data),
        }

    print("\n" + "=" * 70)
    print(f"Step Breakdown ({mode_name}): {num_agents} agents ({num_steps} steps)")
    print("=" * 70)

    for component, times in timings.items():
        s = stats(times)
        percentage = (s["mean"] / stats(timings["total"])["mean"]) * 100
        print(f"\n{component.replace('_', ' ').title()}:")
        print(f"  Mean:   {s['mean']:>8.3f}ms ({percentage:>5.1f}%)")
        print(f"  Median: {s['median']:>8.3f}ms")
        print(f"  StdDev: {s['stdev']:>8.3f}ms")
        print(f"  Range:  [{s['min']:>6.3f}, {s['max']:>6.3f}]ms")

    # Cleanup
    p.disconnect()

    return timings


def profile_with_cprofile(num_agents: int, num_steps: int = 100, motion_mode: MotionMode = MotionMode.OMNIDIRECTIONAL):
    """cProfile: Bottleneck analysis across all functions"""
    mode_name = "DIFFERENTIAL" if motion_mode == MotionMode.DIFFERENTIAL else "OMNIDIRECTIONAL"

    print(f"\n{'='*70}")
    print(f"cProfile ({mode_name}): {num_agents} agents, {num_steps} steps")
    print(f"{'='*70}\n")

    # Clean up any leftover connection before creating a fresh sim_core.
    if p.isConnected():
        p.disconnect()

    # Setup - Load from profiling config
    config_path = os.path.join(os.path.dirname(__file__), "profiling_config.yaml")
    params = SimulationParams.from_config(config_path)
    params.monitor = False  # Disable monitor for cleaner cProfile

    sim_core = MultiRobotSimulationCore(params)

    # Spawn agents
    print(f"Spawning {num_agents} agents...")
    agents = create_test_agents(num_agents, sim_core, motion_mode)
    print(f"Spawned {len(agents)} agents")

    # Warm-up
    print("Warm-up...")
    for _ in range(10):
        sim_core.step_once()

    # Profile - simply call step_once() directly
    print(f"Profiling {num_steps} steps with cProfile...")
    profiler = cProfile.Profile()
    profiler.enable()

    for _ in range(num_steps):
        sim_core.step_once()  # No re-implementation needed, call step_once() directly

    profiler.disable()

    # Results
    s = io.StringIO()
    stats = pstats.Stats(profiler, stream=s)
    stats.sort_stats("cumulative")
    stats.print_stats(40)  # Top 40 functions

    print(s.getvalue())

    # Cleanup
    p.disconnect()

    return stats


def analyze_motion_modes(num_agents: int, num_steps: int = 100):
    """Motion Mode comparison: DIFFERENTIAL vs OMNIDIRECTIONAL"""
    print(f"\n{'='*70}")
    print(f"Motion Mode Comparison: {num_agents} agents, {num_steps} steps")
    print(f"{'='*70}")

    results = {}

    for mode in [MotionMode.DIFFERENTIAL, MotionMode.OMNIDIRECTIONAL]:
        mode_name = "DIFFERENTIAL" if mode == MotionMode.DIFFERENTIAL else "OMNIDIRECTIONAL"

        # Run built-in profiling
        timings = profile_builtin_profiling(num_agents, num_steps, mode)

        # Extract key metrics
        total_mean = statistics.mean(timings["total"])
        agent_update_mean = statistics.mean(timings["agent_update"])
        collision_mean = statistics.mean(timings["collision_check"])

        results[mode_name] = {
            "total_ms": total_mean,
            "agent_update_ms": agent_update_mean,
            "collision_ms": collision_mean,
        }

    # Compare
    diff = results["DIFFERENTIAL"]
    omni = results["OMNIDIRECTIONAL"]

    print(f"\n{'='*70}")
    print("Performance Comparison:")
    print(f"{'='*70}")

    ratio_total = diff["total_ms"] / omni["total_ms"]
    ratio_agent = diff["agent_update_ms"] / omni["agent_update_ms"]

    print("\nTotal Step Time:")
    print(f"  DIFFERENTIAL:     {diff['total_ms']:.3f} ms")
    print(f"  OMNIDIRECTIONAL:  {omni['total_ms']:.3f} ms")
    print(f"  Ratio: {ratio_total:.2f}x")

    print("\nAgent Update Time:")
    print(f"  DIFFERENTIAL:     {diff['agent_update_ms']:.3f} ms")
    print(f"  OMNIDIRECTIONAL:  {omni['agent_update_ms']:.3f} ms")
    print(f"  Ratio: {ratio_agent:.2f}x")

    if ratio_total > 1.0:
        print(f"\n  → OMNIDIRECTIONAL is {ratio_total:.2f}x faster overall")
    else:
        print(f"\n  → DIFFERENTIAL is {1/ratio_total:.2f}x faster overall")

    return results


def main():
    """Main execution"""
    import argparse

    parser = argparse.ArgumentParser(description="Profile step_once() breakdown")
    parser.add_argument("--agents", type=int, default=1000, help="Number of agents")
    parser.add_argument("--steps", type=int, default=100, help="Number of steps to profile")
    parser.add_argument(
        "--test", choices=["all", "builtin", "cprofile", "motion_modes"], default="builtin", help="Which test to run"
    )
    parser.add_argument(
        "--mode", choices=["differential", "omnidirectional"], default="omnidirectional", help="Motion mode for single tests"
    )

    args = parser.parse_args()

    motion_mode = MotionMode.DIFFERENTIAL if args.mode == "differential" else MotionMode.OMNIDIRECTIONAL

    print("\n" + "=" * 70)
    print("Simulation Step Breakdown Profiling")
    print("=" * 70)

    if args.test in ["all", "builtin"]:
        profile_builtin_profiling(args.agents, args.steps, motion_mode)

    if args.test in ["all", "motion_modes"]:
        analyze_motion_modes(args.agents, args.steps)

    if args.test in ["all", "cprofile"]:
        # Run cProfile for both modes
        print("\n" + "=" * 70)
        print("cProfile Comparison: DIFFERENTIAL vs OMNIDIRECTIONAL")
        print("=" * 70)

        profile_with_cprofile(args.agents, args.steps, MotionMode.DIFFERENTIAL)
        profile_with_cprofile(args.agents, args.steps, MotionMode.OMNIDIRECTIONAL)

    print("\n" + "=" * 70)
    print("Profiling Complete!")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    main()
