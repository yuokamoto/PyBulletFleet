#!/usr/bin/env python3
"""
Test optimization ideas for collision check.
"""
import time


# Simulate the current implementation
def current_implementation(robot_bodies, structure_body_ids):
    """Current: List comprehension every time."""
    robot_indices = [i for i, bid in enumerate(robot_bodies) if bid not in structure_body_ids]
    return robot_indices


def optimized_v1(robot_bodies, structure_body_ids):
    """Optimization 1: Pre-filter using set difference."""
    non_structure_ids = set(robot_bodies) - structure_body_ids
    robot_indices = [i for i, bid in enumerate(robot_bodies) if bid in non_structure_ids]
    return robot_indices


def optimized_v2(robot_bodies, structure_body_ids):
    """Optimization 2: Single pass with direct check."""
    robot_indices = []
    for i, bid in enumerate(robot_bodies):
        if bid not in structure_body_ids:
            robot_indices.append(i)
    return robot_indices


def optimized_v3(robot_bodies, structure_body_ids_set):
    """Optimization 3: Assume structure_body_ids is already a set."""
    robot_indices = [i for i, bid in enumerate(robot_bodies) if bid not in structure_body_ids_set]
    return robot_indices


# Test data
num_robots = 1000
robot_bodies = list(range(num_robots))
structure_body_ids = set(range(500, 600))  # 100 structure bodies

iterations = 1000

print(f"Testing with {num_robots} robots, {len(structure_body_ids)} structure bodies")
print(f"Iterations: {iterations}\n")

# Verify all produce same results
r1 = current_implementation(robot_bodies, structure_body_ids)
r2 = optimized_v1(robot_bodies, structure_body_ids)
r3 = optimized_v2(robot_bodies, structure_body_ids)
r4 = optimized_v3(robot_bodies, structure_body_ids)

assert r1 == r2 == r3 == r4, "Results don't match!"
print(f"✓ All methods produce same result ({len(r1)} non-structure robots)\n")


# Benchmark
def benchmark(name, func, *args):
    t0 = time.perf_counter()
    for _ in range(iterations):
        func(*args)
    t1 = time.perf_counter()
    avg_ms = (t1 - t0) / iterations * 1000
    print(f"{name:30} {avg_ms:>8.4f}ms")
    return avg_ms


print("=" * 50)
print("Benchmark Results:")
print("=" * 50)

baseline = benchmark("Current (list comp)", current_implementation, robot_bodies, structure_body_ids)
v1 = benchmark("Optimized V1 (set diff)", optimized_v1, robot_bodies, structure_body_ids)
v2 = benchmark("Optimized V2 (for loop)", optimized_v2, robot_bodies, structure_body_ids)
v3 = benchmark("Optimized V3 (assume set)", optimized_v3, robot_bodies, structure_body_ids)

print("=" * 50)
print(f"Speedup factors (vs baseline):")
print(f"  V1: {baseline/v1:.2f}x")
print(f"  V2: {baseline/v2:.2f}x")
print(f"  V3: {baseline/v3:.2f}x")
