#!/usr/bin/env python3
"""
profile_agent_update.py
Agent.update()の詳細プロファイリング

目的:
- Agent.update()の内部でどの処理が遅いか特定
- PyBullet API呼び出しの頻度と時間を測定
- 最適化ポイントを明確化
"""
import os
import sys
import time
import cProfile
import pstats
import io
from collections import defaultdict

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pybullet as p
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent import Agent, AgentSpawnParams, MotionMode
from pybullet_fleet.geometry import Pose


def create_test_agents(num_agents: int, motion_mode: MotionMode = MotionMode.DIFFERENTIAL):
    """テスト用のAgentを生成"""
    if p.isConnected():
        p.disconnect()

    params = SimulationParams(gui=False, timestep=0.01)
    sim_core = MultiRobotSimulationCore(params)

    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    robot_urdf = os.path.join(os.path.dirname(__file__), "../robots/simple_cube.urdf")

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


def profile_with_cprofile(num_agents: int = 1000, num_updates: int = 1):
    """cProfileでプロファイリング"""
    print(f"\n{'='*70}")
    print(f"cProfile Profiling: {num_agents} agents, {num_updates} update(s)")
    print(f"{'='*70}\n")

    agents, sim_core = create_test_agents(num_agents)

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
    """手動でタイミングを測定（より詳細）"""
    print(f"\n{'='*70}")
    print(f"Manual Timing: {num_agents} agents, {num_updates} update(s)")
    print(f"{'='*70}\n")

    agents, sim_core = create_test_agents(num_agents)

    # Timing buckets
    timings = defaultdict(list)

    # Instrument Agent.update() by monkey-patching
    original_update = Agent.update
    original_update_actions = Agent._update_actions
    original_update_differential = Agent._update_differential
    original_update_omnidirectional = Agent._update_omnidirectional

    def timed_update(self, dt):
        t0 = time.perf_counter()
        original_update(self, dt)
        elapsed = (time.perf_counter() - t0) * 1000.0  # ms
        timings["total"].append(elapsed)

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

    Agent.update = timed_update
    Agent._update_actions = timed_update_actions
    Agent._update_differential = timed_update_differential
    Agent._update_omnidirectional = timed_update_omnidirectional

    # Run updates
    for i in range(num_updates):
        for agent in agents:
            agent.update(dt=0.01)

    # Restore original methods
    Agent.update = original_update
    Agent._update_actions = original_update_actions
    Agent._update_differential = original_update_differential
    Agent._update_omnidirectional = original_update_omnidirectional

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
    """PyBullet API呼び出しの頻度をカウント"""
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
    """静止中 vs 移動中のAgent.update()コスト比較"""
    print(f"\n{'='*70}")
    print(f"Stationary vs Moving Analysis: {num_agents} agents")
    print(f"{'='*70}\n")

    agents_moving, sim_core1 = create_test_agents(num_agents, MotionMode.DIFFERENTIAL)

    # Create stationary agents (no goal)
    if p.isConnected():
        p.disconnect()

    params = SimulationParams(gui=False, timestep=0.01)
    sim_core2 = MultiRobotSimulationCore(params)
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    robot_urdf = os.path.join(os.path.dirname(__file__), "../robots/simple_cube.urdf")
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
        agent = Agent.from_params(spawn_params, sim_core=sim_core2)
        # No goal set - agent will be stationary
        agents_stationary.append(agent)

    # Time stationary updates
    t0 = time.perf_counter()
    for agent in agents_stationary:
        agent.update(dt=0.01)
    stationary_time = (time.perf_counter() - t0) * 1000.0  # ms

    # Switch to moving agents
    p.disconnect()
    agents_moving, sim_core1 = create_test_agents(num_agents, MotionMode.DIFFERENTIAL)

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

    # Cleanup
    p.disconnect()

    return {
        "stationary_ms": stationary_time,
        "moving_ms": moving_time,
        "ratio": moving_time / stationary_time,
    }


def main():
    """メイン実行"""
    import argparse

    parser = argparse.ArgumentParser(description="Profile Agent.update() performance")
    parser.add_argument("--agents", type=int, default=1000, help="Number of agents")
    parser.add_argument("--updates", type=int, default=100, help="Number of update iterations")
    parser.add_argument(
        "--test", choices=["all", "cprofile", "manual", "pybullet", "stationary"], default="all", help="Which test to run"
    )

    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("Agent.update() Performance Profiling")
    print("=" * 70)

    if args.test in ["all", "stationary"]:
        analyze_stationary_vs_moving(args.agents)

    if args.test in ["all", "manual"]:
        profile_manual_timing(args.agents, args.updates)

    if args.test in ["all", "pybullet"]:
        count_pybullet_calls(min(args.agents, 100), min(args.updates, 10))

    if args.test in ["all", "cprofile"]:
        profile_with_cprofile(args.agents, num_updates=1)

    print("\n" + "=" * 70)
    print("Profiling Complete!")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    main()
