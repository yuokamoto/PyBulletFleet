#!/usr/bin/env python3
"""
Profile AgentManager.set_goal_pose() to identify bottleneck.
"""
import os
import sys
import time
import cProfile
import pstats
from io import StringIO

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

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

    params = SimulationParams(gui=False, timestep=0.01)
    sim_core = MultiRobotSimulationCore(params)
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

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

    robot_urdf = os.path.join(os.path.dirname(__file__), "../robots/simple_cube.urdf")
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
    ap.add_argument("--n", type=int, default=1000, help="Number of agents")
    args = ap.parse_args()

    profile_set_goal_pose(args.n)
