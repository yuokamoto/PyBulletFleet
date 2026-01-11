#!/usr/bin/env python3
"""
collision_check.py
衝突検出処理の詳細プロファイリングツール

概要:
 import os
import sys
import time
import cProfile
import pstats
import io
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import pybullet as p
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.agent import AgentSpawnParams.check_collisions()）は大規模シミュレーションの主要なボトルネックです。
    このツールは core_simulation.py に組み込まれた Built-in Profiling 機能を使用して、
    衝突検出を4ステップに分解し、どの部分が遅いかを特定します。

分析手法:
    1. Built-in Profiling（組み込みプロファイリング）- デフォルト
       - core_simulation.py の return_profiling=True オプションを使用
       - Get AABBs: PyBullet からバウンディングボックス取得
       - Spatial Hashing: 空間グリッドの構築
       - AABB Filtering: 近接ペアの候補選定（最大のボトルネック、通常75%）
       - Contact Points: 実際の衝突判定
    
    2. cProfile分析（--test=cprofile）
       - 関数レベルの詳細分析
       - ステップ内部の関数呼び出しを記録
       - 意外なボトルネックの発見に最適

使い方:
    # Built-in profiling（デフォルト、推奨）
    python collision_check.py --agents=1000 --iterations=100
    
    # cProfile で詳細分析
    python collision_check.py --agents=1000 --test=cprofile
    
    # 両方実行
    python collision_check.py --agents=1000 --test=all

出力例:
    Built-in Profiling:
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
         27000    0.650    0.650  dict.get (grid lookup)
         27000    0.420    0.420  tuple addition (neighbor_cell)

Built-in Profiling の特徴:
    ✅ 実装のコピー不要（core_simulation.py から直接取得）
    ✅ 常に最新の実装を測定
    ✅ Dynamic cell size、2D/3D モード対応
    ✅ オーバーヘッド最小（return_profiling フラグのみ）

使い分け:
    - ボトルネックのステップ特定 → Built-in profiling（デフォルト）
    - ステップ内の詳細分析 → cProfile
    - 最適化効果の検証 → Built-in profiling（オーバーヘッドなし）

最適化のヒント:
    - AABB Filtering が 75% → 2D モードで 67%削減可能
    - Collision ratio が 0.3% → フィルタリング精度向上の余地あり
    - Dynamic cell size により自動最適化

関連ファイル:
    - agent_update.py: Agent.update() の詳細プロファイリング
    - step_breakdown.py: シミュレーション全体のステップ分解
    - ../archive/collision_check_v1.py: 旧版（Manual implementation、参考用）
"""
import os
import sys
import time
import cProfile
import pstats
import io
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import pybullet as p
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.agent import AgentSpawnParams, MotionMode


def profile_collision_check_builtin(num_agents: int, num_iterations: int = 100):
    """Built-in profiling を使用した collision check 分析"""
    
    # Ensure clean disconnect and use DIRECT mode
    if p.isConnected():
        p.disconnect()
    
    p.connect(p.DIRECT)  # Force DIRECT mode (no GUI, no X11)
    
    # Setup - Load from profiling config
    config_path = os.path.join(os.path.dirname(__file__), "profiling_config.yaml")
    params = SimulationParams.from_config(config_path)
    params.monitor = False  # Disable monitor for collision profiling

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

    # Collect profiling data
    timings = {
        "get_aabbs": [],
        "spatial_hashing": [],
        "aabb_filtering": [],
        "contact_points": [],
        "total": [],
    }

    print(f"\nProfiling collision check ({num_iterations} iterations)...")
    print("Using built-in profiling (return_profiling=True)\n")

    # Run iterations
    for iteration in range(num_iterations):
        # Synchronize robot_bodies like in step_once()
        sim_core.robot_bodies = [obj.body_id for obj in sim_core.sim_objects]
        
        # Get collision results + profiling data
        collision_pairs, iter_timings = sim_core.check_collisions(return_profiling=True)
        
        # Collect timings
        for key in timings.keys():
            if key in iter_timings:
                timings[key].append(iter_timings[key])

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
    print(f"Collision Check Breakdown for {num_agents} Agents (Built-in Profiling)")
    print("=" * 70)

    total_mean = stats(timings["total"])["mean"]

    for component, times in timings.items():
        if not times:
            continue
        s = stats(times)
        percentage = (s["mean"] / total_mean) * 100 if total_mean > 0 else 0
        print(f"\n{component.replace('_', ' ').title()}:")
        print(f"  Mean:   {s['mean']:>8.3f}ms ({percentage:>5.1f}%)")
        print(f"  Median: {s['median']:>8.3f}ms")
        print(f"  StdDev: {s['stdev']:>8.3f}ms")
        print(f"  Range:  [{s['min']:>6.3f}, {s['max']:>6.3f}]ms")

    # Additional info
    print("\n" + "=" * 70)
    print("Advantages of Built-in Profiling:")
    print("  ✅ No implementation duplication (no manual copy)")
    print("  ✅ Always measures the latest implementation")
    print("  ✅ Accurate dynamic cell size, 2D/3D mode measurement")
    print("  ✅ Minimal overhead (return_profiling flag only)")
    print("=" * 70)

    # Cleanup
    p.disconnect()


def profile_collision_check_with_cprofile(num_agents: int):
    """cProfile で collision check の詳細分析"""
    
    # Ensure clean disconnect and use DIRECT mode
    if p.isConnected():
        p.disconnect()
    
    p.connect(p.DIRECT)  # Force DIRECT mode (no GUI, no X11)
    
    # Setup - Load from profiling config
    config_path = os.path.join(os.path.dirname(__file__), "profiling_config.yaml")
    params = SimulationParams.from_config(config_path)
    params.monitor = False  # Disable monitor for collision profiling

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

    print(f"\ncProfile analysis of collision check...")
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

    parser = argparse.ArgumentParser(description="Profile collision check with built-in profiling")
    parser.add_argument("--agents", type=int, default=1000, help="Number of agents")
    parser.add_argument("--iterations", type=int, default=100, help="Number of iterations for built-in profiling")
    parser.add_argument(
        "--test",
        choices=["builtin", "cprofile", "all"],
        default="all",
        help="Which profiling method to use",
    )
    args = parser.parse_args()

    if args.test in ["builtin", "all"]:
        profile_collision_check_builtin(args.agents, args.iterations)

    if args.test in ["cprofile", "all"]:
        if args.test == "all":
            print("\n\n")
        profile_collision_check_with_cprofile(args.agents)
