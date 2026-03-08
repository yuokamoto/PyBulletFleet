#!/usr/bin/env python3
"""
agent_manager_set_goal.py
Profiling tool for AgentManager.set_goal_pose()

Overview:
    AgentManager.set_goal_pose() is the operation that sets goals for multiple agents.
    This tool uses cProfile for function-level detailed analysis to identify bottlenecks.

Measured Items:
    - agent_manager.set_goal_pose() overhead
    - agent.set_goal_pose() execution time
    - agent.set_path() internal processing
    - _init_differential_rotation_trajectory() time (biggest bottleneck)
    - _init_differential_forward_distance_trajectory() time

Usage:
    # Basic usage (1000 agents)
    python agent_manager_set_goal.py --agents=1000

    # Test with fewer agents
    python agent_manager_set_goal.py --agents=100

    # Large-scale test
    python agent_manager_set_goal.py --agents=5000

Example Output:
    ncalls  tottime  cumtime  function
      1000    0.001    0.109  agent_manager.set_goal_pose()
      1000    0.000    0.108  agent.set_goal_pose()
      1000    0.007    0.107  agent.set_path()
      1000    0.022    0.092  _init_differential_rotation_trajectory()  <- bottleneck (84%)
      1000    0.003    0.032  _init_differential_forward_distance_trajectory()

How to Read the Results:
    - tottime: Processing time of the function itself
    - cumtime: Total time including internal calls
    - cumtime - tottime = processing time of child functions

    Example: agent_manager.set_goal_pose()
      - cumtime: 0.109s (total)
      - tottime: 0.001s (own overhead, 0.9%)
      - Remaining 99.1% is in child functions (agent.set_goal_pose)

Optimization Tips:
    - _init_differential_rotation_trajectory accounts for 84%
    - Rotation matrix computation (scipy) and trajectory interpolation are expensive
    - Can be improved with caching or pre-computation

Related Files:
    - agent_update.py: Detailed profiling of Agent.update()
    - collision_check.py: Detailed profiling of collision detection
"""
import os
import sys
import time
import cProfile
import pstats
from io import StringIO

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import pybullet as p
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.agent import AgentSpawnParams, MotionMode
from pybullet_fleet.geometry import Pose


def profile_set_goal_pose(num_agents=1000):
    """Profile set_goal_pose calls."""

    # Setup
    if p.isConnected():
        p.disconnect()

    # Load from profiling config
    config_path = os.path.join(os.path.dirname(__file__), "profiling_config.yaml")
    params = SimulationParams.from_config(config_path)

    sim_core = MultiRobotSimulationCore(params)

    # Spawn agents
    manager = AgentManager(sim_core=sim_core)
    rows = (num_agents + 99) // 100
    grid_params = GridSpawnParams(
        x_min=0,
        x_max=99,
        y_min=0,
        y_max=rows - 1,
        z_min=0,
        z_max=0,
        spacing=[0.5, 0.5, 0.0],
        offset=[0.0, 0.0, 0.1],
    )

    robot_urdf = os.path.join(os.path.dirname(__file__), "../../robots/simple_cube.urdf")
    agent_spawn_params = AgentSpawnParams(
        urdf_path=robot_urdf,
        motion_mode=MotionMode.DIFFERENTIAL,
        max_linear_vel=1.0,
        max_angular_vel=1.0,
        mass=0.0,
    )

    print(f"Spawning {num_agents} agents...")
    t0 = time.perf_counter()
    agents = manager.spawn_agents_grid(
        num_agents=num_agents,
        grid_params=grid_params,
        spawn_params=agent_spawn_params,
    )
    spawn_time = time.perf_counter() - t0
    print(f"  Spawn time: {spawn_time:.3f}s")

    # Get poses
    print("\nGetting all poses...")
    t0 = time.perf_counter()
    poses = manager.get_all_poses()
    get_time = time.perf_counter() - t0
    print(f"  Get time: {get_time*1000:.2f}ms")

    # Prepare new poses
    new_poses = [Pose.from_xyz(p.position[0] + 0.1, p.position[1], p.position[2]) for p in poses]

    # Profile set_goal_pose
    print(f"\nProfiling set_goal_pose for {num_agents} agents...")

    profiler = cProfile.Profile()
    profiler.enable()

    t0 = time.perf_counter()
    for i, new_pose in enumerate(new_poses):
        manager.set_goal_pose(i, new_pose)
    set_time = time.perf_counter() - t0

    profiler.disable()

    print(f"  Set time: {set_time*1000:.2f}ms ({set_time/num_agents*1000000:.2f}µs per agent)")

    # Print profile stats
    print("\n" + "=" * 80)
    print("Profile Statistics (top 30 by cumulative time)")
    print("=" * 80)
    s = StringIO()
    ps = pstats.Stats(profiler, stream=s).sort_stats("cumulative")
    ps.print_stats(30)
    print(s.getvalue())

    # Cleanup
    p.resetSimulation()
    p.disconnect()


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--agents", type=int, default=1000, help="Number of agents")
    args = ap.parse_args()

    profile_set_goal_pose(args.agents)
