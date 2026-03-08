#!/usr/bin/env python3
"""
collision_method_comparison.py
Collision detection method comparison experiment

Overview:
    Compares multiple collision detection methods:
    1. Spatial Hashing (Current): Spatial partitioning + AABB filtering + getContactPoints
    2. Brute Force AABB: All-pairs brute force + AABB overlap + getContactPoints
    3. PyBullet getClosestPoints: PyBullet's standard getClosestPoints API
    4. getContactPoints() [No Args]: Batch retrieval of all contacts with no arguments
    5. getContactPoints(A,B) [Pairwise]: Individual getContactPoints calls for all pairs

Characteristics of Each Method:
    Spatial Hashing (Current):
        - O(N) spatial complexity, checks only neighbors
        - Narrows candidates with AABB filtering
        - Final collision determination via getContactPoints
        - Suited for large-scale simulations

    Brute Force AABB:
        - O(N^2) checks all pairs
        - Performs AABB overlap check
        - Fast for small simulations (<100 objects)
        - Simple implementation

    PyBullet getClosestPoints:
        - Closest point computation via PyBullet's C++ implementation
        - Configurable distance threshold
        - Accurate but requires all-pairs checking

    getContactPoints() [No Args]:
        - Called without arguments, retrieves all contact points in batch
        - Obtains all collision information managed internally by PyBullet
        - Only 1 API call needed (potentially most efficient)
        - O(1) API calls

    getContactPoints(A,B) [Pairwise]:
        - Individual getContactPoints calls for all pairs
        - Most reliable
        - O(N^2) API calls for all-pairs checking

Usage:
    # Compare all methods
    python collision_method_comparison.py --agents=100,500,1000 --methods=all

    # Compare specific methods only (comparison with no-args version)
    python collision_method_comparison.py --agents=1000 --methods=spatial,contact,pairwise

    # With detailed profiling
    python collision_method_comparison.py --agents=1000 --profile

Example Output:
    ===================================================================
    Collision Method Comparison
    ===================================================================

    Test Configuration:
      Agents: 100, 500, 1000
      Methods: Spatial Hashing, Brute Force, getClosestPoints, getContactPoints
      Iterations: 100 per configuration

    -------------------------------------------------------------------
    Results for 100 Agents
    -------------------------------------------------------------------

    Method                    Time (ms)    Speedup    Collisions    Accuracy
    -------------------------------------------------------------------------
    Spatial Hashing (Current)    1.234      1.00x         12         100%
    Brute Force                  0.987      1.25x         12         100%
    PyBullet getClosestPoints    2.456      0.50x         12         100%
    PyBullet getContactPoints    3.789      0.33x         12         100%

    Winner: Brute Force (best for <100 agents)

    -------------------------------------------------------------------
    Results for 500 Agents
    -------------------------------------------------------------------

    Method                    Time (ms)    Speedup    Collisions    Accuracy
    -------------------------------------------------------------------------
    Spatial Hashing (Current)    5.678      1.00x         45         100%
    Brute Force                 12.345      0.46x         45         100%
    PyBullet getClosestPoints   28.901      0.20x         45         100%
    PyBullet getContactPoints   45.678      0.12x         45         100%

    Winner: Spatial Hashing (1.85x faster than Brute Force)

    -------------------------------------------------------------------
    Results for 1000 Agents
    -------------------------------------------------------------------

    Method                    Time (ms)    Speedup    Collisions    Accuracy
    -------------------------------------------------------------------------
    Spatial Hashing (Current)   12.345      1.00x        123         100%
    Brute Force                 56.789      0.22x        123         100%
    PyBullet getClosestPoints  145.678      0.08x        123         100%
    PyBullet getContactPoints  234.567      0.05x        123         100%

    Winner: Spatial Hashing (4.6x faster than Brute Force)

    ===================================================================
    Recommendation
    ===================================================================

    - Small simulations (<100 agents): Brute Force (simpler, faster)
    - Medium simulations (100-500 agents): Spatial Hashing (balanced)
    - Large simulations (>500 agents): Spatial Hashing (best scalability)

    Spatial Hashing scales as O(N), Brute Force as O(N^2)

    For maximum accuracy, all methods detect the same collisions.
"""
import os
import sys
import time
import math
import statistics
from typing import List, Tuple, Dict, Any

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import pybullet as p
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.agent import AgentSpawnParams, MotionMode


def setup_simulation(num_agents: int):
    """Set up the simulation environment"""
    if p.isConnected():
        p.disconnect()

    config_path = os.path.join(os.path.dirname(__file__), "..", "profiling", "profiling_config.yaml")
    params = SimulationParams.from_config(config_path)
    params.monitor = False
    params.collision_check_frequency = 0  # Disable automatic collision checks

    sim_core = MultiRobotSimulationCore(params)
    agent_manager = AgentManager(sim_core=sim_core)

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

    agents = agent_manager.spawn_agents_grid(
        num_agents=num_agents,
        grid_params=grid_params,
        spawn_params=agent_spawn_params,
    )

    return sim_core, agents


def method_spatial_hashing(sim_core) -> Tuple[List, float]:
    """Method 1: Spatial Hashing (Current implementation)"""
    t0 = time.perf_counter()
    collision_pairs, _ = sim_core.check_collisions(return_profiling=False)
    elapsed = (time.perf_counter() - t0) * 1000
    return collision_pairs, elapsed


def method_brute_force(sim_core) -> Tuple[List, float]:
    """Method 2: Brute Force AABB (all pairs with AABB overlap check)"""
    t0 = time.perf_counter()

    collision_pairs = []
    objects = sim_core.sim_objects

    for i in range(len(objects)):
        for j in range(i + 1, len(objects)):
            # AABB overlap check
            aabb_a = p.getAABB(objects[i].body_id)
            aabb_b = p.getAABB(objects[j].body_id)

            # Check AABB overlap
            if (
                aabb_a[0][0] <= aabb_b[1][0]
                and aabb_a[1][0] >= aabb_b[0][0]
                and aabb_a[0][1] <= aabb_b[1][1]
                and aabb_a[1][1] >= aabb_b[0][1]
                and aabb_a[0][2] <= aabb_b[1][2]
                and aabb_a[1][2] >= aabb_b[0][2]
            ):

                # Detailed collision check
                contact_points = p.getContactPoints(objects[i].body_id, objects[j].body_id)
                if contact_points:
                    collision_pairs.append((i, j))

    elapsed = (time.perf_counter() - t0) * 1000
    return collision_pairs, elapsed


def method_get_closest_points(sim_core, distance_threshold: float = 0.01) -> Tuple[List, float]:
    """Method 3: PyBullet getClosestPoints"""
    t0 = time.perf_counter()

    collision_pairs = []
    objects = sim_core.sim_objects

    for i in range(len(objects)):
        for j in range(i + 1, len(objects)):
            closest_points = p.getClosestPoints(objects[i].body_id, objects[j].body_id, distance=distance_threshold)
            if closest_points and len(closest_points) > 0:
                # Check if distance is below threshold
                if closest_points[0][8] < distance_threshold:  # distance is at index 8
                    collision_pairs.append((i, j))

    elapsed = (time.perf_counter() - t0) * 1000
    return collision_pairs, elapsed


def method_get_contact_points_all(sim_core) -> Tuple[List, float]:
    """Method 4: PyBullet getContactPoints (no args - batch all contacts)"""
    t0 = time.perf_counter()

    # Step simulation to populate the contact buffer before querying
    p.stepSimulation()
    # Call getContactPoints() without arguments to retrieve all contact points in batch
    all_contacts = p.getContactPoints()

    # Extract body_id pairs (use set to avoid duplicates)
    collision_pairs_set = set()
    objects = sim_core.sim_objects
    body_id_to_index = {obj.body_id: i for i, obj in enumerate(objects)}

    for contact in all_contacts:
        body_a = contact[1]  # bodyUniqueIdA
        body_b = contact[2]  # bodyUniqueIdB

        # Only include objects within the simulation
        if body_a in body_id_to_index and body_b in body_id_to_index:
            idx_a = body_id_to_index[body_a]
            idx_b = body_id_to_index[body_b]
            # Normalize order and add pair
            pair = (min(idx_a, idx_b), max(idx_a, idx_b))
            collision_pairs_set.add(pair)

    collision_pairs = list(collision_pairs_set)
    elapsed = (time.perf_counter() - t0) * 1000
    return collision_pairs, elapsed


def method_get_contact_points_pairwise(sim_core) -> Tuple[List, float]:
    """Method 5: PyBullet getContactPoints (pairwise calls)"""
    t0 = time.perf_counter()

    collision_pairs = []
    objects = sim_core.sim_objects

    for i in range(len(objects)):
        for j in range(i + 1, len(objects)):
            contact_points = p.getContactPoints(objects[i].body_id, objects[j].body_id)
            if contact_points:
                collision_pairs.append((i, j))

    elapsed = (time.perf_counter() - t0) * 1000
    return collision_pairs, elapsed


def benchmark_method(method_name: str, method_func, sim_core, num_iterations: int) -> Dict[str, Any]:
    """Benchmark a specific method"""
    times = []
    collision_counts = []

    # Warm-up
    for _ in range(5):
        method_func(sim_core)

    # Benchmark
    for _ in range(num_iterations):
        collision_pairs, elapsed = method_func(sim_core)
        times.append(elapsed)
        collision_counts.append(len(collision_pairs))

    return {
        "method": method_name,
        "time_mean": statistics.mean(times),
        "time_stdev": statistics.stdev(times) if len(times) > 1 else 0,
        "time_min": min(times),
        "time_max": max(times),
        "collisions_mean": statistics.mean(collision_counts),
        "collisions_stdev": statistics.stdev(collision_counts) if len(collision_counts) > 1 else 0,
    }


def print_comparison_table(results_list: List[Dict], num_agents: int):
    """Output comparison results in table format"""
    print(f"\n{'='*70}")
    print(f"Results for {num_agents} Agents")
    print(f"{'='*70}\n")

    # Table header
    print(f"{'Method':<30} {'Time (ms)':<12} {'Speedup':<10} {'Collisions':<12} {'Accuracy'}")
    print("-" * 70)

    # Baseline (spatial hashing)
    baseline_time = results_list[0]["time_mean"]
    baseline_collisions = results_list[0]["collisions_mean"]

    # Print rows
    for result in results_list:
        speedup = baseline_time / result["time_mean"] if result["time_mean"] > 0 else 0
        accuracy = (result["collisions_mean"] / baseline_collisions * 100) if baseline_collisions > 0 else 100

        print(
            f"{result['method']:<30} "
            f"{result['time_mean']:>8.3f}    "
            f"{speedup:>6.2f}x    "
            f"{result['collisions_mean']:>8.1f}      "
            f"{accuracy:>5.1f}%"
        )

    # Winner
    fastest = min(results_list, key=lambda x: x["time_mean"])
    print(f"\nWinner: {fastest['method']}")
    if fastest != results_list[0]:
        speedup = baseline_time / fastest["time_mean"]
        print(f"  {speedup:.2f}x faster than Spatial Hashing")


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Compare collision detection methods")
    parser.add_argument("--agents", type=str, default="100,500,1000", help="Comma-separated list of agent counts")
    parser.add_argument(
        "--methods", type=str, default="all", help="Methods to test: all, spatial, brute, closest, contact, pairwise"
    )
    parser.add_argument("--iterations", type=int, default=100, help="Iterations per configuration")
    parser.add_argument("--profile", action="store_true", help="Enable detailed profiling")
    args = parser.parse_args()

    # Parse agent counts
    agent_counts = [int(x) for x in args.agents.split(",")]

    # Define methods
    all_methods = {
        "spatial": ("Spatial Hashing (Current)", method_spatial_hashing),
        "brute": ("Brute Force AABB", method_brute_force),
        "closest": ("PyBullet getClosestPoints", method_get_closest_points),
        "contact": ("getContactPoints() [No Args]", method_get_contact_points_all),
        "pairwise": ("getContactPoints(A,B) [Pairwise]", method_get_contact_points_pairwise),
    }

    # Select methods
    if args.methods == "all":
        selected_methods = all_methods
    else:
        method_keys = args.methods.split(",")
        selected_methods = {k: all_methods[k] for k in method_keys if k in all_methods}

    print("\n" + "=" * 70)
    print("Collision Method Comparison")
    print("=" * 70)
    print("\nTest Configuration:")
    print(f"  Agents: {', '.join(map(str, agent_counts))}")
    print(f"  Methods: {', '.join([v[0] for v in selected_methods.values()])}")
    print(f"  Iterations: {args.iterations} per configuration")

    # Run benchmarks
    all_results = {}

    for num_agents in agent_counts:
        print(f"\n[{agent_counts.index(num_agents)+1}/{len(agent_counts)}] Benchmarking {num_agents} agents...")

        sim_core, agents = setup_simulation(num_agents)

        results = []
        for method_key, (method_name, method_func) in selected_methods.items():
            print(f"  Testing {method_name}...")
            result = benchmark_method(method_name, method_func, sim_core, args.iterations)
            results.append(result)

        all_results[num_agents] = results

        p.disconnect()

    # Print results
    for num_agents in agent_counts:
        print_comparison_table(all_results[num_agents], num_agents)

    # Summary
    print("\n" + "=" * 70)
    print("Recommendation")
    print("=" * 70)
    print("\n- Small simulations (<100 agents): Brute Force (simpler, potentially faster)")
    print("- Medium simulations (100-500 agents): Spatial Hashing (balanced)")
    print("- Large simulations (>500 agents): Spatial Hashing (best scalability)")
    print("\nSpatial Hashing scales as O(N), Brute Force as O(N²)")
    print("\nFor maximum accuracy, all methods should detect the same collisions.")
    print("=" * 70)


if __name__ == "__main__":
    main()
