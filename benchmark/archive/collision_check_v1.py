#!/usr/bin/env python3
"""
collision_check_v1.py (ARCHIVED - 参考用)
衝突検出処理の詳細プロファイリングツール（旧版: Manual Implementation）

⚠️ 非推奨: このファイルは archive に移動されました。
✅ 推奨: ../profiling/collision_check.py を使用してください。

移行理由:
    この旧版は core_simulation.py の実装を手動でコピーして測定していました。
    新版（collision_check.py）は Built-in Profiling 機能（return_profiling=True）を使用し、
    以下の問題を解決しています：

    旧版の問題:
    - ❌ 実装の二重管理（約200行のコピー）
    - ❌ core_simulation.py 変更時に乖離のリスク
    - ❌ 簡略化により正確性が低下（固定 cell_size=1.0、3D のみ）

    新版の利点:
    - ✅ 実装のコピー不要（二重管理なし）
    - ✅ 常に最新の実装を測定
    - ✅ Dynamic cell size、2D/3D モード対応
    - ✅ オーバーヘッド最小（return_profiling フラグのみ）

移行ガイド:
    # 旧版（このファイル）
    python benchmark/archive/collision_check_v1.py --agents=1000 --test=manual

    # 新版（推奨）
    python benchmark/profiling/collision_check.py --agents=1000 --test=builtin

参考用として残す理由:
    - Manual Implementation の実装例として教育的価値がある
    - 実装のコピー vs Built-in Profiling の比較材料

---

概要（旧版）:
    衝突検出（sim_core.check_collisions()）は大規模シミュレーションの主要なボトルネックです。
    このツールは衝突検出を4ステップに分解して、どの部分が遅いかを特定します。

分析手法:
    1. Manual Implementation（手動測定）- デフォルト
       - 4ステップを個別に測定（オーバーヘッド最小）
       - Get AABBs: PyBullet からバウンディングボックス取得
       - Spatial Hashing: 空間グリッドの構築
       - AABB Filtering: 近接ペアの候補選定（最大のボトルネック）
       - Contact Points: 実際の衝突判定

    2. cProfile分析（--test=cprofile）
       - 関数レベルの詳細分析
       - ステップ内部の関数呼び出しを記録
       - 意外なボトルネックの発見に最適

使い方（旧版）:
    # Manual 測定（デフォルト）
    python collision_check_v1.py --agents=1000 --iterations=100

    # cProfile で詳細分析
    python collision_check_v1.py --agents=1000 --test=cprofile

    # 両方実行
    python collision_check_v1.py --agents=1000 --test=all

出力例:
    Manual Implementation:
        Get Aabbs:         0.523ms ( 10.2%)
        Spatial Hashing:   0.312ms (  6.1%)
        Aabb Filtering:    3.845ms ( 75.2%)  ← 最大のボトルネック
        Contact Points:    0.432ms (  8.5%)
        Total:             5.112ms (100.0%)

    Additional Information:
        Candidate pairs: 3456
        Actual collisions: 12
        Collision ratio: 0.3%  ← 99.7%は無駄な計算

    cProfile (AABB Filtering の詳細):
        ncalls  tottime  cumtime  関数
        384500    1.850    1.850  AABB overlap check
         27000    0.650    0.650  dict.get
         27000    0.420    0.420  tuple addition

使い分け:
    - ボトルネックのステップ特定 → Manual（デフォルト）
    - ステップ内の詳細分析 → cProfile
    - 最適化効果の検証 → Manual（オーバーヘッドなし）

関連ファイル:
    - agent_update.py: Agent.update() のプロファイリング
    - step_breakdown.py: シミュレーション全体のステップ分解
    - getaabb_performance.py: getAABB() の最適化実験
"""
import os
import sys
import time
import cProfile
import pstats
import io

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pybullet as p
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.agent import AgentSpawnParams, MotionMode
import math


def profile_collision_check(num_agents: int, num_iterations: int = 100):
    """Profile collision check in detail."""

    # Setup
    params = SimulationParams(
        gui=False,
        timestep=0.01,
        duration=0,
        target_rtf=0.0,
        physics=False,
        monitor=False,
        enable_monitor_gui=False,
    )

    sim_core = MultiRobotSimulationCore(params)
    agent_manager = AgentManager(sim_core=sim_core)

    # Spawn agents
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

    robot_urdf = os.path.join(os.path.dirname(__file__), "../../robots/simple_cube.urdf")
    agent_spawn_params = AgentSpawnParams(
        urdf_path=robot_urdf,
        motion_mode=MotionMode.OMNIDIRECTIONAL,
        max_linear_vel=2.0,
        max_angular_vel=3.0,
        mass=0.0,
    )

    print(f"Spawning {num_agents} agents...")
    agents = agent_manager.spawn_agents_grid(
        num_agents=num_agents,
        grid_params=grid_params,
        spawn_params=agent_spawn_params,
    )
    print(f"Spawned {len(agents)} agents")

    # Detailed profiling
    timings = {
        "get_aabbs": [],
        "spatial_hashing": [],
        "aabb_filtering": [],
        "contact_points": [],
        "total": [],
    }

    print(f"\nProfiling collision check ({num_iterations} iterations)...")

    # Test 1: Manual implementation (current)
    print("\n[Test 1] Manual implementation breakdown:")
    for iteration in range(num_iterations):
        t_total_start = time.perf_counter()

        # 1. Get AABBs
        t0 = time.perf_counter()
        robot_indices = list(range(len(sim_core.robot_bodies)))
        robot_body_ids = sim_core.robot_bodies
        aabbs = [p.getAABB(bid) for bid in robot_body_ids]
        t1 = time.perf_counter()
        timings["get_aabbs"].append((t1 - t0) * 1000)

        # 2. Spatial hashing (grid construction)
        t0 = time.perf_counter()
        cell_size = 1.0
        grid = {}
        index_to_cell = []
        for idx, aabb in enumerate(aabbs):
            center = [0.5 * (aabb[0][d] + aabb[1][d]) for d in range(3)]
            cell = tuple(int(center[d] // cell_size) for d in range(3))
            index_to_cell.append(cell)
            grid.setdefault(cell, []).append(idx)
        t1 = time.perf_counter()
        timings["spatial_hashing"].append((t1 - t0) * 1000)

        # 3. AABB overlap filtering
        t0 = time.perf_counter()
        neighbor_offsets = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)]
        pairs = set()
        for idx, cell in enumerate(index_to_cell):
            id_i = robot_body_ids[idx]
            for offset in neighbor_offsets:
                neighbor_cell = (cell[0] + offset[0], cell[1] + offset[1], cell[2] + offset[2])
                for jdx in grid.get(neighbor_cell, []):
                    if jdx <= idx:
                        continue
                    id_j = robot_body_ids[jdx]
                    aabb_i = aabbs[idx]
                    aabb_j = aabbs[jdx]
                    if (
                        aabb_i[1][0] < aabb_j[0][0]
                        or aabb_i[0][0] > aabb_j[1][0]
                        or aabb_i[1][1] < aabb_j[0][1]
                        or aabb_i[0][1] > aabb_j[1][1]
                        or aabb_i[1][2] < aabb_j[0][2]
                        or aabb_i[0][2] > aabb_j[1][2]
                    ):
                        continue
                    pairs.add((robot_indices[idx], robot_indices[jdx]))
        candidate_pairs = list(pairs)
        t1 = time.perf_counter()
        timings["aabb_filtering"].append((t1 - t0) * 1000)

        # 4. Contact point checks
        t0 = time.perf_counter()
        collision_pairs = []
        for i, j in candidate_pairs:
            contact_points = p.getContactPoints(sim_core.robot_bodies[i], sim_core.robot_bodies[j])
            if contact_points:
                collision_pairs.append((i, j))
        t1 = time.perf_counter()
        timings["contact_points"].append((t1 - t0) * 1000)

        t_total_end = time.perf_counter()
        timings["total"].append((t_total_end - t_total_start) * 1000)

    # Test 2: Using actual check_collisions() method
    print("\n[Test 2] Using sim_core.check_collisions():")
    actual_timings = []
    for iteration in range(num_iterations):
        # Synchronize robot_bodies like in step_once()
        sim_core.robot_bodies = [obj.body_id for obj in sim_core.sim_objects]

        t0 = time.perf_counter()
        sim_core.check_collisions()
        t1 = time.perf_counter()
        actual_timings.append((t1 - t0) * 1000)

    # Statistics
    def stats(data):
        import statistics

        return {
            "mean": statistics.mean(data),
            "median": statistics.median(data),
            "stdev": statistics.stdev(data) if len(data) > 1 else 0,
            "min": min(data),
            "max": max(data),
        }

    print("\n" + "=" * 70)
    print(f"Collision Check Breakdown for {num_agents} Agents")
    print("=" * 70)

    print("\n[Test 1 Results] Manual implementation:")
    total_mean = stats(timings["total"])["mean"]

    for component, times in timings.items():
        s = stats(times)
        percentage = (s["mean"] / total_mean) * 100 if total_mean > 0 else 0
        print(f"\n{component.replace('_', ' ').title()}:")
        print(f"  Mean:   {s['mean']:>8.3f}ms ({percentage:>5.1f}%)")
        print(f"  Median: {s['median']:>8.3f}ms")
        print(f"  StdDev: {s['stdev']:>8.3f}ms")
        print(f"  Range:  [{s['min']:>6.3f}, {s['max']:>6.3f}]ms")

    # Test 2 results
    print("\n[Test 2 Results] Using sim_core.check_collisions():")
    s = stats(actual_timings)
    print(f"  Mean:   {s['mean']:>8.3f}ms")
    print(f"  Median: {s['median']:>8.3f}ms")
    print(f"  StdDev: {s['stdev']:>8.3f}ms")
    print(f"  Range:  [{s['min']:>6.3f}, {s['max']:>6.3f}]ms")

    # Additional info
    print("\n" + "=" * 70)
    print("Additional Information:")
    print(f"  Total agents: {num_agents}")
    print(f"  Candidate pairs (last iteration): {len(candidate_pairs)}")
    print(f"  Actual collisions (last iteration): {len(collision_pairs)}")
    print(f"  Collision ratio: {len(collision_pairs) / len(candidate_pairs) * 100:.1f}%" if candidate_pairs else "N/A")
    print("=" * 70)

    # Cleanup
    p.disconnect()


def profile_collision_check_with_cprofile(num_agents: int):
    """cProfile で collision check の詳細分析"""

    # Setup
    params = SimulationParams(
        gui=False,
        timestep=0.01,
        duration=0,
        target_rtf=0.0,
        physics=False,
        monitor=False,
        enable_monitor_gui=False,
    )

    sim_core = MultiRobotSimulationCore(params)
    agent_manager = AgentManager(sim_core=sim_core)

    # Spawn agents
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

    robot_urdf = os.path.join(os.path.dirname(__file__), "../../robots/simple_cube.urdf")
    agent_spawn_params = AgentSpawnParams(
        urdf_path=robot_urdf,
        motion_mode=MotionMode.OMNIDIRECTIONAL,
        max_linear_vel=2.0,
        max_angular_vel=3.0,
        mass=0.0,
    )

    print(f"Spawning {num_agents} agents...")
    agents = agent_manager.spawn_agents_grid(
        num_agents=num_agents,
        grid_params=grid_params,
        spawn_params=agent_spawn_params,
    )
    print(f"Spawned {len(agents)} agents")

    print("\ncProfile analysis of collision check...")
    print("=" * 70)

    # Warm-up
    sim_core.robot_bodies = [obj.body_id for obj in sim_core.sim_objects]
    sim_core.check_collisions()

    # Profile
    profiler = cProfile.Profile()
    profiler.enable()

    # Run collision check multiple times
    for _ in range(10):
        sim_core.robot_bodies = [obj.body_id for obj in sim_core.sim_objects]
        sim_core.check_collisions()

    profiler.disable()

    # Results
    s = io.StringIO()
    stats = pstats.Stats(profiler, stream=s)
    stats.sort_stats("cumulative")
    stats.print_stats(40)  # Top 40 functions

    print(s.getvalue())

    print("\n" + "=" * 70)
    print("Tips:")
    print("  - Look for functions with high 'tottime' in collision-related code")
    print("  - 'ncalls' shows how many times a function is called")
    print("  - PyBullet C++ internals (getAABB, getContactPoints) show as built-ins")
    print("=" * 70)

    # Cleanup
    p.disconnect()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Profile collision check in detail")
    parser.add_argument("--agents", type=int, default=1000, help="Number of agents")
    parser.add_argument("--iterations", type=int, default=100, help="Number of iterations for manual profiling")
    parser.add_argument(
        "--test",
        choices=["manual", "cprofile", "all"],
        default="manual",
        help="Which profiling method to use",
    )
    args = parser.parse_args()

    if args.test in ["manual", "all"]:
        profile_collision_check(args.agents, args.iterations)

    if args.test in ["cprofile", "all"]:
        if args.test == "all":
            print("\n\n")
        profile_collision_check_with_cprofile(args.agents)
