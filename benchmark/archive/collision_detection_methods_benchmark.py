#!/usr/bin/env python3
"""
Benchmark: getContactPoints vs getClosestPoints for collision detection

Compare performance of different collision detection methods:
1. getContactPoints only (current implementation)
2. getClosestPoints for kinematic pairs + getContactPoints for physics pairs
3. getClosestPoints only (all pairs)

Usage:
    python benchmark/collision_detection_methods_benchmark.py
"""

import sys
import time
from pathlib import Path
from typing import Callable, List, Tuple

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

import pybullet as p
import numpy as np

from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import SimObject


def create_test_objects(
    num_objects: int, physics_ratio: float = 0.5, spacing: float = 0.4
) -> Tuple[MultiRobotSimulationCore, List[SimObject]]:
    """
    Create test environment with mix of physics and kinematic objects.

    Args:
        num_objects: Total number of objects to create
        physics_ratio: Ratio of physics objects (0.0-1.0)
        spacing: Distance between objects

    Returns:
        Tuple of (sim_core, objects_list)
    """
    # Create simulation
    params = SimulationParams(
        gui=False,
        physics=True,
        timestep=0.01,
        target_rtf=0,  # Fast as possible
        duration=0,
        ignore_static_collision=True,
    )
    sim = MultiRobotSimulationCore(params)
    sim.setup_pybullet()
    pid = sim._client

    objects = []
    num_physics = int(num_objects * physics_ratio)

    # Arrange objects in grid
    grid_size = int(np.ceil(np.sqrt(num_objects)))

    for i in range(num_objects):
        row = i // grid_size
        col = i % grid_size
        pos = [col * spacing, row * spacing, 0.5]

        # Create cube
        is_physics = i < num_physics
        mass = 1.0 if is_physics else 0.0

        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25, 0.25, 0.25], physicsClientId=pid)
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.25, 0.25, 0.25],
            rgbaColor=[1, 0, 0, 1] if is_physics else [0, 1, 0, 1],
            physicsClientId=pid,
        )

        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=pos,
            physicsClientId=pid,
        )

        obj = SimObject(body_id=body_id, sim_core=sim)
        obj.is_kinematic = not is_physics
        objects.append(obj)

    # Step simulation once to initialize
    p.stepSimulation(physicsClientId=pid)

    return sim, objects


# ---------------------------------------------------------------------------
# Collision check strategies
# ---------------------------------------------------------------------------
# Each strategy is a callable (obj_id_i, obj_id_j, body_id_i, body_id_j) -> bool.
# They are constructed via factory functions that capture `pid` and `sim`.


def _make_contact_points_check(pid: int):
    """getContactPoints for every pair."""

    def check(_oi: int, _oj: int, bi: int, bj: int) -> bool:
        return bool(p.getContactPoints(bi, bj, physicsClientId=pid))

    return check


def _make_closest_points_check(pid: int, distance: float = 0.0):
    """getClosestPoints for every pair."""

    def check(_oi: int, _oj: int, bi: int, bj: int) -> bool:
        return bool(p.getClosestPoints(bi, bj, distance=distance, physicsClientId=pid))

    return check


def _make_hybrid_check(sim: MultiRobotSimulationCore, pid: int):
    """getContactPoints for physics-physics pairs, getClosestPoints for the rest."""
    physics_objects = sim._physics_objects

    def check(oi: int, oj: int, bi: int, bj: int) -> bool:
        if oi in physics_objects and oj in physics_objects:
            return bool(p.getContactPoints(bi, bj, physicsClientId=pid))
        return bool(p.getClosestPoints(bi, bj, distance=0.0, physicsClientId=pid))

    return check


# ---------------------------------------------------------------------------
# Generic benchmark runner
# ---------------------------------------------------------------------------


def _benchmark_method(
    sim: MultiRobotSimulationCore,
    pairs: List[Tuple[int, int]],
    check_fn: Callable[[int, int, int, int], bool],
    iterations: int = 100,
    warmup: int = 5,
) -> Tuple[float, float]:
    """
    Benchmark a collision check strategy over random pairs.

    Args:
        sim: Simulation core (used to resolve object IDs → body IDs)
        pairs: List of (obj_id_i, obj_id_j) pairs to check
        check_fn: Callable(obj_id_i, obj_id_j, body_id_i, body_id_j) -> bool
        iterations: Timed iterations
        warmup: Untimed warmup iterations

    Returns:
        Tuple of (mean_ms, stdev_ms) per iteration
    """
    objs = sim._sim_objects_dict
    # Pre-resolve to (obj_id_i, obj_id_j, body_id_i, body_id_j) tuples
    resolved = [(oi, oj, objs[oi].body_id, objs[oj].body_id) for oi, oj in pairs]

    # Warmup
    for _ in range(warmup):
        for oi, oj, bi, bj in resolved:
            check_fn(oi, oj, bi, bj)

    # Timed runs
    times = []
    for _ in range(iterations):
        t0 = time.perf_counter()
        for oi, oj, bi, bj in resolved:
            check_fn(oi, oj, bi, bj)
        times.append((time.perf_counter() - t0) * 1000)

    return float(np.mean(times)), float(np.std(times))


def run_benchmark(num_objects: int, physics_ratio: float, num_pairs: int = 50, iterations: int = 100) -> dict:
    """
    Run complete benchmark for given configuration.

    Args:
        num_objects: Number of objects to create
        physics_ratio: Ratio of physics objects (0.0-1.0)
        num_pairs: Number of random pairs to test
        iterations: Number of iterations per method

    Returns:
        Dict with benchmark results
    """
    print(f"\n{'='*70}")
    print(f"Benchmark: {num_objects} objects, {physics_ratio*100:.0f}% physics, {num_pairs} pairs")
    print(f"{'='*70}")

    # Create test environment
    sim, objects = create_test_objects(num_objects, physics_ratio)

    # Generate random pairs for testing
    np.random.seed(42)
    all_obj_ids = list(sim._sim_objects_dict.keys())
    pairs = []
    for _ in range(num_pairs):
        i, j = np.random.choice(all_obj_ids, size=2, replace=False)
        pairs.append((min(i, j), max(i, j)))

    # Count physics vs kinematic pairs
    physics_pairs = 0
    kinematic_pairs = 0
    mixed_pairs = 0

    for obj_id_i, obj_id_j in pairs:
        is_physics_i = obj_id_i in sim._physics_objects
        is_physics_j = obj_id_j in sim._physics_objects

        if is_physics_i and is_physics_j:
            physics_pairs += 1
        elif not is_physics_i and not is_physics_j:
            kinematic_pairs += 1
        else:
            mixed_pairs += 1

    print("\nPair composition:")
    print(f"  Physics-Physics:     {physics_pairs:3d} pairs ({physics_pairs/num_pairs*100:.1f}%)")
    print(f"  Kinematic-Kinematic: {kinematic_pairs:3d} pairs ({kinematic_pairs/num_pairs*100:.1f}%)")
    print(f"  Mixed:               {mixed_pairs:3d} pairs ({mixed_pairs/num_pairs*100:.1f}%)")

    # Run benchmarks
    print(f"\nRunning benchmarks ({iterations} iterations each, 5 warmup iterations)...")

    pid = sim._client
    time_contact, std_contact = _benchmark_method(sim, pairs, _make_contact_points_check(pid), iterations)
    time_closest, std_closest = _benchmark_method(sim, pairs, _make_closest_points_check(pid, distance=0.0), iterations)
    time_hybrid, std_hybrid = _benchmark_method(sim, pairs, _make_hybrid_check(sim, pid), iterations)

    # Results
    print("\nResults (average time ± stdev per iteration):")
    print(f"  1. getContactPoints only:  {time_contact:.3f} ± {std_contact:.3f} ms")
    print(f"  2. getClosestPoints only:  {time_closest:.3f} ± {std_closest:.3f} ms  ({time_closest/time_contact:5.2f}x)")
    print(f"  3. Hybrid approach:        {time_hybrid:.3f} ± {std_hybrid:.3f} ms  ({time_hybrid/time_contact:5.2f}x)")

    if time_hybrid < time_contact:
        speedup = (time_contact - time_hybrid) / time_contact * 100
        print(f"\n✅ Hybrid approach is {speedup:.1f}% faster than current implementation")
    else:
        slowdown = (time_hybrid - time_contact) / time_contact * 100
        print(f"\n❌ Hybrid approach is {slowdown:.1f}% slower than current implementation")

    # Cleanup
    try:
        p.disconnect(sim._client)
    except p.error:
        pass

    return {
        "num_objects": num_objects,
        "physics_ratio": physics_ratio,
        "num_pairs": num_pairs,
        "physics_pairs": physics_pairs,
        "kinematic_pairs": kinematic_pairs,
        "mixed_pairs": mixed_pairs,
        "time_contact": time_contact,
        "std_contact": std_contact,
        "time_closest": time_closest,
        "std_closest": std_closest,
        "time_hybrid": time_hybrid,
        "std_hybrid": std_hybrid,
    }


def main():
    """Run comprehensive benchmark suite."""
    print("=" * 70)
    print("COLLISION DETECTION METHODS BENCHMARK")
    print("=" * 70)
    print("\nComparing:")
    print("  1. getContactPoints only (current implementation)")
    print("  2. getClosestPoints only")
    print("  3. Hybrid: getClosestPoints for kinematic + getContactPoints for physics")

    results = []

    # Test different configurations
    configs = [
        # (num_objects, physics_ratio, num_pairs)
        (20, 0.5, 50),  # Small: 20 objects, 50% physics
        (50, 0.5, 100),  # Medium: 50 objects, 50% physics
        (100, 0.5, 200),  # Large: 100 objects, 50% physics
        (50, 0.2, 100),  # Mostly kinematic: 50 objects, 20% physics
        (50, 0.8, 100),  # Mostly physics: 50 objects, 80% physics
    ]

    for num_objects, physics_ratio, num_pairs in configs:
        result = run_benchmark(num_objects, physics_ratio, num_pairs, iterations=100)
        results.append(result)

    # Summary
    print(f"\n{'='*70}")
    print("SUMMARY")
    print(f"{'='*70}")
    print(f"\n{'Config':<25} {'Contact':<20} {'Closest':<20} {'Hybrid':<20} {'Winner':<10}")
    print(f"{'-'*95}")

    for r in results:
        config = f"{r['num_objects']}obj, {r['physics_ratio']*100:.0f}%phys"
        contact = f"{r['time_contact']:.2f}\u00b1{r['std_contact']:.2f}ms"
        closest = f"{r['time_closest']:.2f}\u00b1{r['std_closest']:.2f}ms"
        hybrid = f"{r['time_hybrid']:.2f}\u00b1{r['std_hybrid']:.2f}ms"

        # Determine winner
        times = [r["time_contact"], r["time_closest"], r["time_hybrid"]]
        methods = ["Contact", "Closest", "Hybrid"]
        winner = methods[times.index(min(times))]

        print(f"{config:<25} {contact:<20} {closest:<20} {hybrid:<20} {winner:<10}")

    print(f"\n{'='*70}")
    print("RECOMMENDATIONS")
    print(f"{'='*70}")

    # Calculate average speedup/slowdown
    avg_hybrid_ratio = np.mean([r["time_hybrid"] / r["time_contact"] for r in results])

    if avg_hybrid_ratio < 1.0:
        speedup = (1.0 - avg_hybrid_ratio) * 100
        print(f"\n✅ Hybrid approach is {speedup:.1f}% faster on average")
        print("\nRecommendation: Implement hybrid approach")
        print("  - Use getClosestPoints for kinematic-kinematic and kinematic-physics pairs")
        print("  - Use getContactPoints for physics-physics pairs")
    else:
        slowdown = (avg_hybrid_ratio - 1.0) * 100
        print(f"\n❌ Hybrid approach is {slowdown:.1f}% slower on average")
        print("\nRecommendation: Keep current implementation (getContactPoints only)")
        print("  - getContactPoints is already optimized and sufficient")
        print("  - Additional complexity of hybrid approach is not justified")


if __name__ == "__main__":
    main()
