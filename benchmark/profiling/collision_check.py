#!/usr/bin/env python3
"""
collision_check.py
Detailed profiling tool for collision detection pipeline.
Measures 4-step breakdown: Get AABBs, Spatial Hashing, AABB Filtering, Contact Points.

Analysis Methods:
    1. Built-in Profiling - default
       - Uses core_simulation.py's return_profiling=True option
       - Get AABBs: Retrieve bounding boxes from PyBullet
       - Spatial Hashing: Build spatial grid
       - AABB Filtering: Candidate pair selection for nearby objects (biggest bottleneck, typically 75%)
       - Contact Points: Actual collision determination

    2. cProfile analysis (--test=cprofile)
       - Function-level detailed analysis
       - Records function calls within a step
       - Best for discovering unexpected bottlenecks

Usage:
    # Built-in profiling (default, recommended)
    python collision_check.py --agents=1000 --iterations=100

    # Detailed analysis with cProfile
    python collision_check.py --agents=1000 --test=cprofile

    # Run both
    python collision_check.py --agents=1000 --test=all

Example Output:
    Built-in Profiling:
        Get Aabbs:         0.523ms ( 10.2%)
        Spatial Hashing:   0.312ms (  6.1%)
        Aabb Filtering:    3.845ms ( 75.2%)  <- biggest bottleneck
        Contact Points:    0.432ms (  8.5%)
        Total:             5.112ms (100.0%)

        Additional Information:
          Candidate pairs: 3456
          Actual collisions: 12
          Collision ratio: 0.3%  <- 99.7% are wasted computations

    cProfile (AABB Filtering details):
        ncalls  tottime  cumtime  function
        384500    1.850    1.850  AABB overlap check
         27000    0.650    0.650  dict.get (grid lookup)
         27000    0.420    0.420  tuple addition (neighbor_cell)

Features of Built-in Profiling:
    * No need to copy implementation (obtained directly from core_simulation.py)
    * Always measures the latest implementation
    * Supports dynamic cell size, 2D/3D modes
    * Minimal overhead (only the return_profiling flag)

When to Use:
    - Identifying bottleneck steps -> Built-in profiling (default)
    - Detailed analysis within a step -> cProfile
    - Verifying optimization effects -> Built-in profiling (no overhead)

Optimization Tips:
    - AABB Filtering at 75% -> 67% reduction possible with 2D mode
    - Collision ratio at 0.3% -> room for filtering accuracy improvement
    - Automatic optimization via dynamic cell size

Related Files:
    - agent_update.py: Detailed profiling of Agent.update()
    - step_breakdown.py: Simulation-wide step breakdown
    - ../archive/collision_check_v1.py: Legacy version (manual implementation, for reference)
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
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.agent import AgentSpawnParams, MotionMode
from pybullet_fleet.geometry import Pose


def profile_collision_check_builtin(num_agents: int, num_iterations: int = 100):
    """Collision check analysis using built-in profiling"""

    # Clean up any leftover connection before creating a fresh sim_core.
    if p.isConnected():
        p.disconnect()

    # Setup - Load from profiling config
    config_path = os.path.join(os.path.dirname(__file__), "profiling_config.yaml")
    params = SimulationParams.from_config(config_path)
    params.monitor = False  # Disable monitor for collision profiling
    # Disable collision inside step_once; we call check_collisions separately
    params.collision_check_frequency = 0

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

    # Set goals so agents actually move (required for _moved_this_step to be populated)
    for agent in agents:
        pos = agent.get_pose().position
        agent.set_goal_pose(Pose.from_xyz(pos[0] + 5.0, pos[1] + 5.0, pos[2]))

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
        # Step agents so they move (populates _moved_this_step)
        # collision_check_frequency=0 ensures no collision check inside step_once
        sim_core.step_once()

        # Measure collision check separately
        collision_pairs, iter_timings = sim_core.check_collisions(return_profiling=True)

        # Collect timings
        for key in timings.keys():
            if key in iter_timings:
                timings[key].append(iter_timings[key])

    # Statistics
    def stats(data):
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

    # Cleanup
    p.disconnect()


def profile_collision_check_with_cprofile(num_agents: int):
    """Detailed analysis of collision check using cProfile"""

    # Clean up any leftover connection before creating a fresh sim_core.
    if p.isConnected():
        p.disconnect()

    # Setup - Load from profiling config
    config_path = os.path.join(os.path.dirname(__file__), "profiling_config.yaml")
    params = SimulationParams.from_config(config_path)
    params.monitor = False  # Disable monitor for collision profiling
    # Disable collision inside step_once; we call check_collisions under cProfile
    params.collision_check_frequency = 0

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

    # Set goals so agents actually move
    for agent in agents:
        pos = agent.get_pose().position
        agent.set_goal_pose(Pose.from_xyz(pos[0] + 5.0, pos[1] + 5.0, pos[2]))

    num_cprofile_iterations = 50
    print(f"\ncProfile analysis of collision check ({num_cprofile_iterations} iterations)...")
    print("=" * 70)

    # Warm-up: step a few times to get agents moving
    for _ in range(3):
        sim_core.step_once()
        sim_core.check_collisions()

    # Profile: step_once updates agents, then profile check_collisions separately
    profiler = cProfile.Profile()
    profiler.enable()

    for _ in range(num_cprofile_iterations):
        sim_core.step_once()  # Move agents (populates _moved_this_step)
        sim_core.check_collisions()  # <-- This is what we want to profile

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
