"""
Realistic collision detection method comparison using actual simulation.

Compares three collision detection methods in a realistic simulation scenario:
1. CONTACT_POINTS: getContactPoints() only (current default)
2. CLOSEST_POINTS: getClosestPoints() only
3. HYBRID: getContactPoints for physics, getClosestPoints for kinematic

Uses actual PyBulletFleet simulation with mixed physics/kinematic objects,
movement, and real collision detection frequency control.
"""

import sys
import time
import numpy as np
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import SimObject
from pybullet_fleet.types import CollisionDetectionMethod
import pybullet as p


def create_simulation(
    num_objects: int = 100,
    object_size: float = 0.5,
    physics_ratio: float = 0.3,
    collision_detection_method: CollisionDetectionMethod = CollisionDetectionMethod.CONTACT_POINTS
) -> MultiRobotSimulationCore:
    """
    Create a simulation with mixed physics/kinematic objects.
    
    Args:
        num_objects: Number of objects to spawn
        object_size: Size of each object (meters)
        physics_ratio: Ratio of physics objects (0.0-1.0)
        collision_detection_method: Which collision detection method to use
    
    Returns:
        Configured simulation core
    """
    # Create simulation parameters
    params = SimulationParams(
        speed=0,  # Maximum speed (no sleep)
        timestep=1/240,
        duration=0,  # Run indefinitely
        gui=False,  # No GUI for benchmarking
        physics=True,
        monitor=False,
        collision_check_frequency=None,  # Check every step
        log_level="error",  # Suppress debug logs
        enable_profiling=True,
        collision_detection_method=collision_detection_method,
    )
    
    # Create simulation core
    sim = MultiRobotSimulationCore(params)
    
    # Spawn objects in a grid
    grid_size = int(np.ceil(np.sqrt(num_objects)))
    spacing = object_size * 2.5
    
    objects = []
    for i in range(num_objects):
        row = i // grid_size
        col = i % grid_size
        
        x = col * spacing + np.random.uniform(-0.1, 0.1)
        y = row * spacing + np.random.uniform(-0.1, 0.1)
        z = object_size + 0.1
        
        # Determine if physics or kinematic
        is_physics = (i < int(num_objects * physics_ratio))
        mass = 1.0 if is_physics else 0.0
        
        # Create object
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[object_size, object_size, object_size]
        )
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[object_size, object_size, object_size]
        )
        
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=[x, y, z]
        )
        
        # Add to simulation
        obj = SimObject(body_id, sim)
        objects.append((obj, is_physics))
    
    return sim, objects


def benchmark_collision_method(
    method: CollisionDetectionMethod,
    num_objects: int = 100,
    object_size: float = 0.5,
    physics_ratio: float = 0.3,
    num_steps: int = 500,
    movement_speed: float = 2.0
) -> dict:
    """
    Benchmark a specific collision detection method.
    
    Args:
        method: Collision detection method to test
        num_objects: Number of objects
        object_size: Size of objects
        physics_ratio: Ratio of physics objects
        num_steps: Number of simulation steps
        movement_speed: Speed of kinematic objects
    
    Returns:
        Dictionary with benchmark results
    """
    print(f"\n{'='*70}")
    print(f"Testing: {method.value.upper()}")
    print(f"{'='*70}")
    print(f"Objects: {num_objects} (physics: {int(num_objects * physics_ratio)}, "
          f"kinematic: {num_objects - int(num_objects * physics_ratio)})")
    print(f"Steps: {num_steps}")
    
    # Create simulation
    sim, objects = create_simulation(
        num_objects=num_objects,
        object_size=object_size,
        physics_ratio=physics_ratio,
        collision_detection_method=method
    )
    
    # Apply velocities to physics objects
    for obj, is_physics in objects:
        if is_physics:
            p.resetBaseVelocity(
                obj.body_id,
                linearVelocity=[
                    np.random.uniform(-movement_speed, movement_speed),
                    np.random.uniform(-movement_speed, movement_speed),
                    0
                ]
            )
    
    # Initialize kinematic velocities
    kinematic_velocities = {}
    for obj, is_physics in objects:
        if not is_physics:
            kinematic_velocities[obj.body_id] = np.array([
                np.random.uniform(-movement_speed, movement_speed),
                np.random.uniform(-movement_speed, movement_speed),
                0
            ])
    
    # Warmup (100 steps to stabilize physics)
    print("Warming up simulation...")
    for step in range(100):
        # Move kinematic objects
        for obj, is_physics in objects:
            if not is_physics:
                pos, orn = p.getBasePositionAndOrientation(obj.body_id)
                new_pos = np.array(pos) + kinematic_velocities[obj.body_id] * sim.params.timestep
                p.resetBasePositionAndOrientation(obj.body_id, new_pos.tolist(), orn)
        
        # Step physics
        p.stepSimulation()
    
    # Count active collisions after warmup
    active_pairs, _ = sim.check_collisions(return_profiling=False)
    print(f"Active collisions after warmup: {len(active_pairs)}\n")
    
    # Benchmark
    print("Running benchmark...")
    collision_times = []
    total_collisions = []
    
    for step in range(num_steps):
        # Move kinematic objects
        for obj, is_physics in objects:
            if not is_physics:
                pos, orn = p.getBasePositionAndOrientation(obj.body_id)
                new_pos = np.array(pos) + kinematic_velocities[obj.body_id] * sim.params.timestep
                p.resetBasePositionAndOrientation(obj.body_id, new_pos.tolist(), orn)
        
        # Step physics
        p.stepSimulation()
        
        # Measure collision detection time
        t0 = time.perf_counter()
        active_pairs, _ = sim.check_collisions(return_profiling=False)
        elapsed = (time.perf_counter() - t0) * 1000  # ms
        
        collision_times.append(elapsed)
        total_collisions.append(len(active_pairs))
        
        if step % 100 == 0:
            print(f"  Step {step}: {elapsed:.3f}ms ({len(active_pairs)} collisions)")
    
    # Calculate statistics
    avg_time = np.mean(collision_times)
    std_time = np.std(collision_times)
    min_time = np.min(collision_times)
    max_time = np.max(collision_times)
    avg_collisions = np.mean(total_collisions)
    
    # Cleanup
    p.disconnect()
    
    print(f"\nResults:")
    print(f"  Average time: {avg_time:.3f} ± {std_time:.3f} ms")
    print(f"  Min time: {min_time:.3f} ms")
    print(f"  Max time: {max_time:.3f} ms")
    print(f"  Average collisions: {avg_collisions:.1f}")
    
    return {
        'method': method.value,
        'avg_time': avg_time,
        'std_time': std_time,
        'min_time': min_time,
        'max_time': max_time,
        'avg_collisions': avg_collisions,
        'times': collision_times,
    }


def main():
    """Run realistic collision detection benchmarks."""
    print("="*70)
    print("REALISTIC COLLISION DETECTION METHOD COMPARISON")
    print("="*70)
    print("\nUsing actual PyBulletFleet simulation with:")
    print("  - Mixed physics/kinematic objects")
    print("  - Continuous movement")
    print("  - Real collision detection")
    print("  - Spatial hashing + AABB filtering")
    
    # Test configurations
    configs = [
        {'num_objects': 50, 'object_size': 0.5, 'physics_ratio': 0.3},
        {'num_objects': 100, 'object_size': 0.5, 'physics_ratio': 0.3},
        {'num_objects': 200, 'object_size': 0.5, 'physics_ratio': 0.3},
    ]
    
    all_results = []
    
    for config in configs:
        print(f"\n{'#'*70}")
        print(f"Configuration: {config['num_objects']} objects, "
              f"size={config['object_size']}m, "
              f"physics={config['physics_ratio']*100:.0f}%")
        print(f"{'#'*70}")
        
        config_results = []
        
        # Test each method
        for method in [
            CollisionDetectionMethod.CONTACT_POINTS,
            CollisionDetectionMethod.CLOSEST_POINTS,
            CollisionDetectionMethod.HYBRID,
        ]:
            result = benchmark_collision_method(
                method=method,
                num_objects=config['num_objects'],
                object_size=config['object_size'],
                physics_ratio=config['physics_ratio'],
                num_steps=500,
                movement_speed=2.0
            )
            config_results.append(result)
            time.sleep(1)  # Brief pause between tests
        
        all_results.append((config, config_results))
    
    # Print summary
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    
    for config, results in all_results:
        print(f"\n{config['num_objects']} objects "
              f"(physics={config['physics_ratio']*100:.0f}%, "
              f"size={config['object_size']}m):")
        
        baseline = None
        for result in results:
            if result['method'] == 'contact_points':
                baseline = result['avg_time']
            
            if baseline:
                ratio = result['avg_time'] / baseline
                print(f"  {result['method']:20s}: {result['avg_time']:6.3f} ± {result['std_time']:5.3f} ms "
                      f"({ratio:4.2f}x baseline)")
            else:
                print(f"  {result['method']:20s}: {result['avg_time']:6.3f} ± {result['std_time']:5.3f} ms")
    
    print("\n" + "="*70)
    print("CONCLUSION")
    print("="*70)
    
    # Find overall winner
    total_contact = sum(r['avg_time'] for c, results in all_results 
                       for r in results if r['method'] == 'contact_points')
    total_closest = sum(r['avg_time'] for c, results in all_results 
                       for r in results if r['method'] == 'closest_points')
    total_hybrid = sum(r['avg_time'] for c, results in all_results 
                      for r in results if r['method'] == 'hybrid')
    
    print(f"\nTotal time across all configurations:")
    print(f"  CONTACT_POINTS: {total_contact:.3f} ms")
    print(f"  CLOSEST_POINTS: {total_closest:.3f} ms ({total_closest/total_contact:.2f}x slower)")
    print(f"  HYBRID:         {total_hybrid:.3f} ms ({total_hybrid/total_contact:.2f}x slower)")
    
    if total_contact < total_closest and total_contact < total_hybrid:
        print("\n✅ WINNER: CONTACT_POINTS (current default)")
        print("   Recommendation: Keep current implementation")
    elif total_hybrid < total_contact:
        print("\n✅ WINNER: HYBRID")
        print("   Recommendation: Consider switching to hybrid mode")
    else:
        print("\n✅ WINNER: CLOSEST_POINTS")
        print("   Recommendation: Consider switching to closest_points mode")


if __name__ == "__main__":
    main()
