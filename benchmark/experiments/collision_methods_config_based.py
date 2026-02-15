"""
Config-based collision detection comparison benchmark.

Uses different config files to compare:
1. Physics OFF + CLOSEST_POINTS (kinematics mode)
2. Physics ON + CONTACT_POINTS (physics mode)
3. Physics ON + HYBRID (mixed mode)

This demonstrates the practical difference between modes in realistic scenarios.
"""

import sys
import time
import numpy as np
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import SimObject
import pybullet as p


def create_test_scene(sim: MultiRobotSimulationCore, num_objects: int = 100, physics_ratio: float = 0.3):
    """
    Create a test scene with mixed physics/kinematic objects.

    Args:
        sim: Simulation core
        num_objects: Number of objects to spawn
        physics_ratio: Ratio of physics objects (0.0-1.0)

    Returns:
        List of (SimObject, is_physics) tuples
    """
    grid_size = int(np.ceil(np.sqrt(num_objects)))
    spacing = 1.5
    object_size = 0.5

    objects = []
    for i in range(num_objects):
        row = i // grid_size
        col = i % grid_size

        x = col * spacing + np.random.uniform(-0.1, 0.1)
        y = row * spacing + np.random.uniform(-0.1, 0.1)
        z = object_size + 0.1

        # Determine if physics or kinematic
        is_physics = i < int(num_objects * physics_ratio)
        mass = 1.0 if is_physics else 0.0

        # Create object
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[object_size, object_size, object_size])
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[object_size, object_size, object_size])

        body_id = p.createMultiBody(
            baseMass=mass, baseCollisionShapeIndex=collision_shape, baseVisualShapeIndex=visual_shape, basePosition=[x, y, z]
        )

        # Add to simulation
        obj = SimObject(body_id, sim)
        objects.append((obj, is_physics))

        # Apply random velocity to physics objects
        if is_physics:
            p.resetBaseVelocity(body_id, linearVelocity=[np.random.uniform(-2.0, 2.0), np.random.uniform(-2.0, 2.0), 0])

    return objects


def move_kinematic_objects(objects, timestep: float, velocities: dict):
    """Move kinematic objects manually."""
    for obj, is_physics in objects:
        if not is_physics:
            if obj.body_id not in velocities:
                velocities[obj.body_id] = np.array([np.random.uniform(-2.0, 2.0), np.random.uniform(-2.0, 2.0), 0])

            pos, orn = p.getBasePositionAndOrientation(obj.body_id)
            new_pos = np.array(pos) + velocities[obj.body_id] * timestep
            p.resetBasePositionAndOrientation(obj.body_id, new_pos.tolist(), orn)


def run_benchmark(config_path: str, num_objects: int = 100, num_steps: int = 500):
    """
    Run benchmark with a specific config file.

    Args:
        config_path: Path to config YAML file
        num_objects: Number of objects to spawn
        num_steps: Number of simulation steps

    Returns:
        Dictionary with benchmark results
    """
    print(f"\n{'='*70}")
    print(f"Config: {Path(config_path).name}")
    print(f"{'='*70}")

    # Load config
    params = SimulationParams.from_config(config_path)
    print(f"Physics: {params.physics}")
    print(f"Collision method: {params.collision_detection_method.value}")
    print(f"Collision margin: {params.collision_margin}m")
    print(f"Objects: {num_objects}")
    print(f"Steps: {num_steps}")

    # Create simulation
    sim = MultiRobotSimulationCore(params)

    # Create test scene
    print("Creating test scene...")
    objects = create_test_scene(sim, num_objects=num_objects)

    # Kinematic velocities
    kinematic_velocities = {}

    # Warmup (100 steps)
    print("Warming up...")
    for step in range(100):
        move_kinematic_objects(objects, sim.params.timestep, kinematic_velocities)
        if params.physics:
            p.stepSimulation()

    # Count collisions after warmup
    active_pairs, _ = sim.check_collisions(return_profiling=False)
    print(f"Active collisions after warmup: {len(active_pairs)}\n")

    # Benchmark
    print("Running benchmark...")
    step_times = []
    collision_times = []
    total_collisions = []

    for step in range(num_steps):
        t_step_start = time.perf_counter()

        # Move kinematic objects
        move_kinematic_objects(objects, sim.params.timestep, kinematic_velocities)

        # Step physics (if enabled)
        if params.physics:
            p.stepSimulation()

        # Measure collision detection time
        t_col_start = time.perf_counter()
        active_pairs, _ = sim.check_collisions(return_profiling=False)
        collision_time = (time.perf_counter() - t_col_start) * 1000  # ms

        step_time = (time.perf_counter() - t_step_start) * 1000  # ms

        step_times.append(step_time)
        collision_times.append(collision_time)
        total_collisions.append(len(active_pairs))

        if step % 100 == 0:
            print(f"  Step {step}: total={step_time:.3f}ms, collision={collision_time:.3f}ms ({len(active_pairs)} collisions)")

    # Calculate statistics
    avg_step = np.mean(step_times)
    avg_collision = np.mean(collision_times)
    avg_collisions = np.mean(total_collisions)

    # Cleanup
    p.disconnect()

    print("\nResults:")
    print(f"  Average step time: {avg_step:.3f}ms")
    print(f"  Average collision time: {avg_collision:.3f}ms")
    print(f"  Average active collisions: {avg_collisions:.1f}")

    return {
        "config": Path(config_path).name,
        "physics": params.physics,
        "method": params.collision_detection_method.value,
        "margin": params.collision_margin,
        "avg_step": avg_step,
        "avg_collision": avg_collision,
        "avg_collisions": avg_collisions,
    }


def main():
    """Run config-based benchmarks."""
    print("=" * 70)
    print("CONFIG-BASED COLLISION DETECTION COMPARISON")
    print("=" * 70)
    print("\nComparing different config files:")
    print("  1. Physics OFF + CLOSEST_POINTS (kinematics)")
    print("  2. Physics ON + CONTACT_POINTS (physics)")
    print("  3. Physics ON + HYBRID (mixed)")

    # Config files to test
    configs = [
        "configs/collision_physics_off.yaml",
        "configs/collision_physics_on.yaml",
        "configs/collision_hybrid.yaml",
    ]

    # Test parameters
    num_objects = 100
    num_steps = 500

    results = []

    for config_path in configs:
        try:
            result = run_benchmark(config_path, num_objects=num_objects, num_steps=num_steps)
            results.append(result)
            time.sleep(1)  # Brief pause between tests
        except FileNotFoundError:
            print(f"\n⚠️  Config file not found: {config_path}")
            print("   Skipping this test...")
            continue

    # Print summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)

    if not results:
        print("No benchmarks completed successfully.")
        return

    print(f"\n{'Config':<40} {'Method':<15} {'Step (ms)':<12} {'Collision (ms)':<15} {'Collisions':<10}")
    print("-" * 90)

    baseline = None
    for result in results:
        if result["method"] == "contact_points" and result["physics"]:
            baseline = result["avg_collision"]

        ratio = ""
        if baseline and baseline > 0:
            ratio = f"({result['avg_collision']/baseline:.2f}x)"

        print(
            f"{result['config']:<40} {result['method']:<15} "
            f"{result['avg_step']:>10.3f}  {result['avg_collision']:>12.3f} {ratio:<6} "
            f"{result['avg_collisions']:>8.1f}"
        )

    print("\n" + "=" * 70)
    print("CONCLUSION")
    print("=" * 70)

    # Find fastest
    fastest = min(results, key=lambda r: r["avg_collision"])
    print(f"\n✅ Fastest collision detection: {fastest['method'].upper()}")
    print(f"   Config: {fastest['config']}")
    print(f"   Average time: {fastest['avg_collision']:.3f}ms")

    # Physics OFF vs ON comparison
    physics_off = [r for r in results if not r["physics"]]
    physics_on = [r for r in results if r["physics"]]

    if physics_off and physics_on:
        avg_off = np.mean([r["avg_collision"] for r in physics_off])
        avg_on = np.mean([r["avg_collision"] for r in physics_on])

        print("\n📊 Physics OFF vs ON:")
        print(f"   Physics OFF: {avg_off:.3f}ms (CLOSEST_POINTS)")
        print(f"   Physics ON:  {avg_on:.3f}ms (CONTACT_POINTS/HYBRID)")
        print(f"   Difference:  {avg_on/avg_off:.2f}x")


if __name__ == "__main__":
    main()
