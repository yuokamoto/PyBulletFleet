#!/usr/bin/env python3
"""
simple_agent_profile.py (ARCHIVED - 非推奨)
Agent 操作のシンプルなプロファイリングツール

⚠️ 非推奨: このファイルは archive に移動されました。
✅ 推奨: ../profiling/agent_update.py を使用してください。

移行理由:
    このファイルの機能は agent_update.py に統合されました。
    
    旧版の問題:
    - ❌ agent_update.py と機能が重複（Stationary vs Moving）
    - ❌ test_with_without_goal() が test_stationary_vs_moving() と重複
    - ❌ 2つのファイルでメンテナンスコストが増加
    
    新版の利点:
    - ✅ 全ての Agent 分析が1箇所に集約
    - ✅ Motion Mode 比較も追加
    - ✅ メンテナンスポイントが減少

移行ガイド:
    # 旧版（このファイル）
    python benchmark/archive/simple_agent_profile.py --agents=1000
    
    # 新版（推奨）- 全テスト実行
    python benchmark/profiling/agent_update.py --agents=1000 --test=all
    
    # Stationary vs Moving のみ
    python benchmark/profiling/agent_update.py --agents=1000 --test=stationary
    
    # Motion Mode 比較のみ
    python benchmark/profiling/agent_update.py --agents=1000 --test=motion_modes

参考用として残す理由:
    - シンプルな実装例として教育的価値がある
    - cProfile 不使用のアプローチの参考

---

概要（旧版）:
    Agent の基本操作（spawn, update, get_pose, set_pose）を簡潔に測定します。
    cProfile を使わずに perf_counter() で直接測定するため、オーバーヘッドが最小です。

測定項目:
    1. Stationary vs Moving: 静止中と移動中の Agent.update() コスト比較
    2. Spawn Performance: エージェント生成の時間
    3. Get/Set Pose: 姿勢の取得・設定の時間
    4. Basic Operations: 基本操作の時間測定

使い方:
    # 基本実行（全テスト）
    python simple_agent_profile.py
    
    # 特定のテストのみ
    python simple_agent_profile.py --test=stationary
    python simple_agent_profile.py --test=spawn
    python simple_agent_profile.py --test=pose

出力例:
    Test 1: Stationary vs Moving Agents (1000 agents)
    ======================================================================
    
    Stationary agents:
      Total time: 15.20 ms
      Per agent: 15.20 μs
    
    Moving agents:
      Total time: 120.50 ms
      Per agent: 120.50 μs
    
    Overhead ratio (moving/stationary): 7.93x
    Potential savings if 50% stationary: 67.85 ms
    
    Test 2: Spawn Performance
    ======================================================================
    
    Total spawn time: 2.345 s
    Per agent: 2.345 ms
    
    Test 3: Get/Set Pose Performance
    ======================================================================
    
    get_pose(): 12.3 μs per call
    set_pose(): 45.6 μs per call

特徴:
    - cProfile 不使用（オーバーヘッド最小）
    - セグフォルト回避版（安定動作）
    - シンプルで理解しやすい

使い分け:
    - 簡単な性能確認 → このツール
    - 詳細な関数レベル分析 → agent_update.py --test=cprofile
    - 内部メソッドの測定 → agent_update.py --test=manual

関連ファイル:
    - agent_update.py: Agent.update() の詳細プロファイリング
    - agent_manager_set_goal.py: ゴール設定のプロファイリング
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pybullet as p
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent import Agent, AgentSpawnParams, MotionMode
from pybullet_fleet.geometry import Pose


def test_stationary_vs_moving(num_agents: int = 1000):
    """静止中 vs 移動中のAgent.update()コスト比較"""
    print(f"\n{'='*70}")
    print(f"Test 1: Stationary vs Moving Agents ({num_agents} agents)")
    print(f"{'='*70}\n")

    robot_urdf = os.path.join(os.path.dirname(__file__), "../../robots/simple_cube.urdf")

    # Test 1: Stationary agents (no goal)
    if p.isConnected():
        p.disconnect()

    params = SimulationParams(gui=False, timestep=0.01)
    sim_core = MultiRobotSimulationCore(params)
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

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
        # No goal - stationary
        agents_stationary.append(agent)

    # Measure stationary update time
    times = []
    for _ in range(3):
        t0 = time.perf_counter()
        for agent in agents_stationary:
            agent.update(dt=0.01)
        elapsed = (time.perf_counter() - t0) * 1000.0
        times.append(elapsed)

    stationary_time = sum(times) / len(times)
    stationary_per_agent = stationary_time / num_agents * 1000.0  # μs

    # Test 2: Moving agents (with goals)
    for i, agent in enumerate(agents_stationary):
        x = agent.get_pose().position[0]
        y = agent.get_pose().position[1]
        goal = Pose.from_xyz(x + 5.0, y + 5.0, 0.1)
        agent.set_goal_pose(goal)

    # Measure moving update time
    times = []
    for _ in range(3):
        t0 = time.perf_counter()
        for agent in agents_stationary:
            agent.update(dt=0.01)
        elapsed = (time.perf_counter() - t0) * 1000.0
        times.append(elapsed)

    moving_time = sum(times) / len(times)
    moving_per_agent = moving_time / num_agents * 1000.0  # μs

    # Results
    print("Stationary agents:")
    print(f"  Total time: {stationary_time:.2f} ms")
    print(f"  Per agent: {stationary_per_agent:.2f} μs")
    print()
    print("Moving agents:")
    print(f"  Total time: {moving_time:.2f} ms")
    print(f"  Per agent: {moving_per_agent:.2f} μs")
    print()
    print(f"Speed difference: {moving_time / stationary_time:.1f}x slower when moving")
    print()
    print("Optimization potential:")
    savings_time = moving_time - (moving_time + stationary_time) / 2
    savings_percent = (1 - (moving_time + stationary_time) / 2 / moving_time) * 100
    print(f"  If 50% stationary: {(moving_time + stationary_time) / 2:.2f} ms (vs {moving_time:.2f} ms)")
    print(f"  Savings: {savings_time:.2f} ms ({savings_percent:.1f}%)")

    p.disconnect()

    return {
        "stationary_ms": stationary_time,
        "stationary_per_agent_us": stationary_per_agent,
        "moving_ms": moving_time,
        "moving_per_agent_us": moving_per_agent,
        "ratio": moving_time / stationary_time,
    }


def test_motion_modes(num_agents: int = 1000):
    """DIFFERENTIAL vs OMNIDIRECTIONAL モード比較"""
    print(f"\n{'='*70}")
    print(f"Test 2: Motion Mode Comparison ({num_agents} agents)")
    print(f"{'='*70}\n")

    robot_urdf = os.path.join(os.path.dirname(__file__), "../../robots/simple_cube.urdf")

    results = {}

    for mode in [MotionMode.DIFFERENTIAL, MotionMode.OMNIDIRECTIONAL]:
        if p.isConnected():
            p.disconnect()

        params = SimulationParams(gui=False, timestep=0.01)
        sim_core = MultiRobotSimulationCore(params)
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)

        agents = []
        for i in range(num_agents):
            x = (i % 100) * 0.5
            y = (i // 100) * 0.5
            spawn_params = AgentSpawnParams(
                urdf_path=robot_urdf,
                initial_pose=Pose.from_xyz(x, y, 0.1),
                motion_mode=mode,
                max_linear_vel=1.0,
                max_angular_vel=1.0,
                mass=0.0,
            )
            agent = Agent.from_params(spawn_params, sim_core=sim_core)
            # Set goal
            agent.set_goal_pose(Pose.from_xyz(x + 5.0, y + 5.0, 0.1))
            agents.append(agent)

        # Measure update time
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
    print(f"Ratio (DIFFERENTIAL/OMNIDIRECTIONAL): {diff_time / omni_time:.2f}x")

    return results


def test_with_without_goal(num_agents: int = 1000):
    """Goal設定の有無による影響"""
    print(f"\n{'='*70}")
    print(f"Test 3: Effect of Goal Setting ({num_agents} agents)")
    print(f"{'='*70}\n")

    if p.isConnected():
        p.disconnect()

    params = SimulationParams(gui=False, timestep=0.01)
    sim_core = MultiRobotSimulationCore(params)
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    robot_urdf = os.path.join(os.path.dirname(__file__), "../../robots/simple_cube.urdf")

    agents = []
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
        agents.append(agent)

    # Test 1: No goal
    times = []
    for _ in range(3):
        t0 = time.perf_counter()
        for agent in agents:
            agent.update(dt=0.01)
        elapsed = (time.perf_counter() - t0) * 1000.0
        times.append(elapsed)
    no_goal_time = sum(times) / len(times)

    # Test 2: Set goals
    for i, agent in enumerate(agents):
        x = agent.get_pose().position[0]
        y = agent.get_pose().position[1]
        agent.set_goal_pose(Pose.from_xyz(x + 5.0, y + 5.0, 0.1))

    times = []
    for _ in range(3):
        t0 = time.perf_counter()
        for agent in agents:
            agent.update(dt=0.01)
        elapsed = (time.perf_counter() - t0) * 1000.0
        times.append(elapsed)
    with_goal_time = sum(times) / len(times)

    # Test 3: After reaching goal (should stop)
    # Move agents to goal
    for agent in agents:
        if agent._goal_pose:
            agent.set_pose(agent._goal_pose)

    times = []
    for _ in range(3):
        t0 = time.perf_counter()
        for agent in agents:
            agent.update(dt=0.01)
        elapsed = (time.perf_counter() - t0) * 1000.0
        times.append(elapsed)
    reached_goal_time = sum(times) / len(times)

    print("No goal (stationary):")
    print(f"  Total: {no_goal_time:.2f} ms ({no_goal_time / num_agents * 1000:.2f} μs/agent)")
    print()
    print("With goal (moving):")
    print(f"  Total: {with_goal_time:.2f} ms ({with_goal_time / num_agents * 1000:.2f} μs/agent)")
    print()
    print("Reached goal (stopped):")
    print(f"  Total: {reached_goal_time:.2f} ms ({reached_goal_time / num_agents * 1000:.2f} μs/agent)")
    print()
    print(f"Moving overhead: {with_goal_time / no_goal_time:.1f}x")

    p.disconnect()

    return {
        "no_goal_ms": no_goal_time,
        "with_goal_ms": with_goal_time,
        "reached_goal_ms": reached_goal_time,
    }


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--agents", type=int, default=1000, help="Number of agents")
    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("Agent.update() Performance Analysis")
    print("=" * 70)

    # Run tests
    result1 = test_stationary_vs_moving(args.agents)
    result2 = test_motion_modes(args.agents)
    result3 = test_with_without_goal(args.agents)

    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY & OPTIMIZATION RECOMMENDATIONS")
    print("=" * 70)
    print()
    print(f"For {args.agents} agents per frame:")
    print()
    print("1. Current performance (all moving, DIFFERENTIAL):")
    print(f"   {result1['moving_ms']:.2f} ms ({result1['moving_per_agent_us']:.2f} μs/agent)")
    print()
    print("2. If implement early return for stationary agents:")
    print(f"   Stationary cost: {result1['stationary_per_agent_us']:.2f} μs/agent")
    print(f"   Moving cost: {result1['moving_per_agent_us']:.2f} μs/agent")
    print(f"   Speedup: {result1['ratio']:.1f}x for stationary agents")
    print()
    print("3. Potential savings (assuming 50% stationary):")
    avg_time = (result1["moving_ms"] + result1["stationary_ms"]) / 2
    savings_time = result1["moving_ms"] - avg_time
    savings_percent = (1 - avg_time / result1["moving_ms"]) * 100
    print(f"   Current: {result1['moving_ms']:.2f} ms")
    print(f"   With optimization: {avg_time:.2f} ms")
    print(f"   Savings: {savings_time:.2f} ms ({savings_percent:.1f}%)")
    print()
    print("4. Motion mode comparison:")
    print(f"   DIFFERENTIAL: {result2['DIFFERENTIAL']['total_ms']:.2f} ms")
    print(f"   OMNIDIRECTIONAL: {result2['OMNIDIRECTIONAL']['total_ms']:.2f} ms")
    ratio = result2["DIFFERENTIAL"]["total_ms"] / result2["OMNIDIRECTIONAL"]["total_ms"]
    print(f"   Ratio: {ratio:.2f}x")
    print()
    print("=" * 70)
    print()


if __name__ == "__main__":
    main()
