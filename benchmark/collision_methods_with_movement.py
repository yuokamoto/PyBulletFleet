#!/usr/bin/env python3
"""
Benchmark: Collision detection methods with moving objects and actual contacts

Tests getContactPoints vs getClosestPoints performance with:
- Moving objects (creating real contacts)
- Various object sizes (affecting collision density)
- Physics simulation (realistic scenario)

Usage:
    python benchmark/collision_methods_with_movement.py
"""

import sys
import time
from pathlib import Path
from typing import List, Tuple

sys.path.insert(0, str(Path(__file__).parent.parent))

import pybullet as p
import numpy as np


def benchmark_with_movement(
    num_objects: int = 50,
    object_size: float = 0.5,
    num_iterations: int = 100,
    movement_speed: float = 2.0
):
    """
    Benchmark collision detection methods with moving objects.
    
    Args:
        num_objects: Number of objects to spawn
        object_size: Half-extent of box objects (larger = more contacts)
        num_iterations: Number of collision checks per method
        movement_speed: Velocity magnitude for moving objects
    """
    print(f"\n{'='*80}")
    print(f"Collision Detection Benchmark - Moving Objects")
    print(f"{'='*80}")
    print(f"Objects: {num_objects}")
    print(f"Object size (half-extent): {object_size}m")
    print(f"Movement speed: {movement_speed}m/s")
    print(f"Iterations: {num_iterations}")
    print(f"{'='*80}\n")
    
    # Initialize PyBullet
    client = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1/240)
    
    # Spawn objects in a tight grid (to create contacts)
    objects = []
    spacing = object_size * 2.2  # Tight spacing for potential overlaps
    grid_size = int(np.ceil(np.sqrt(num_objects)))
    
    print("Creating objects...")
    for i in range(num_objects):
        row = i // grid_size
        col = i % grid_size
        x = col * spacing + np.random.uniform(-0.1, 0.1)  # Small random offset
        y = row * spacing + np.random.uniform(-0.1, 0.1)
        z = object_size + 0.1
        
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX, 
            halfExtents=[object_size, object_size, object_size]
        )
        visual_shape = p.createVisualShape(
            p.GEOM_BOX, 
            halfExtents=[object_size, object_size, object_size],
            rgbaColor=[0.7, 0.7, 0.7, 1.0]
        )
        
        # Mix of physics (30%) and kinematic (70%) objects
        mass = 1.0 if i % 3 == 0 else 0.0
        
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=[x, y, z]
        )
        objects.append((body_id, mass > 0))
    
    # Apply initial velocities to physics objects
    print("Applying velocities to physics objects...")
    for body_id, is_physics in objects:
        if is_physics:
            p.resetBaseVelocity(
                body_id,
                linearVelocity=[
                    np.random.uniform(-movement_speed, movement_speed),
                    np.random.uniform(-movement_speed, movement_speed),
                    0
                ]
            )
    
    # Run physics to create contacts and move kinematic objects
    print("Running physics simulation to create contacts...")
    kinematic_velocities = {}  # Track velocities for kinematic objects
    
    # Initialize kinematic velocities
    for body_id, is_physics in objects:
        if not is_physics:
            kinematic_velocities[body_id] = np.array([
                np.random.uniform(-movement_speed, movement_speed),
                np.random.uniform(-movement_speed, movement_speed),
                0
            ])
    
    for step in range(100):
        # Move kinematic objects manually
        for body_id, is_physics in objects:
            if not is_physics:
                pos, orn = p.getBasePositionAndOrientation(body_id)
                vel = kinematic_velocities[body_id]
                new_pos = [
                    pos[0] + vel[0] * (1/240),
                    pos[1] + vel[1] * (1/240),
                    pos[2]
                ]
                p.resetBasePositionAndOrientation(body_id, new_pos, orn)
        
        p.stepSimulation()
        
        # Re-apply velocities every 20 steps (keep objects moving)
        if step % 20 == 0:
            for body_id, is_physics in objects:
                if is_physics:
                    p.resetBaseVelocity(
                        body_id,
                        linearVelocity=[
                            np.random.uniform(-movement_speed, movement_speed),
                            np.random.uniform(-movement_speed, movement_speed),
                            0
                        ]
                    )
                else:
                    # Update kinematic velocities
                    kinematic_velocities[body_id] = np.array([
                        np.random.uniform(-movement_speed, movement_speed),
                        np.random.uniform(-movement_speed, movement_speed),
                        0
                    ])
    
    # Count actual contacts before benchmark
    contact_count = 0
    for i in range(len(objects)):
        for j in range(i+1, len(objects)):
            contacts = p.getContactPoints(objects[i][0], objects[j][0])
            if contacts:
                contact_count += 1
    
    print(f"Active contacts after physics: {contact_count}\n")
    
    # Generate all pairs
    pairs = [(objects[i][0], objects[j][0], objects[i][1], objects[j][1]) 
             for i in range(len(objects)) 
             for j in range(i+1, len(objects))]
    
    print(f"Total pairs to check: {len(pairs)}\n")
    
    # ========================================
    # Method 1: getContactPoints (current implementation)
    # ========================================
    print("Benchmarking Method 1: getContactPoints only...")
    times_contact = []
    
    for iteration in range(num_iterations):
        # Move kinematic objects manually
        for body_id, is_physics in objects:
            if not is_physics:
                pos, orn = p.getBasePositionAndOrientation(body_id)
                vel = kinematic_velocities[body_id]
                new_pos = [
                    pos[0] + vel[0] * (1/240),
                    pos[1] + vel[1] * (1/240),
                    pos[2]
                ]
                p.resetBasePositionAndOrientation(body_id, new_pos, orn)
        
        # Run physics step to keep physics objects moving
        p.stepSimulation()
        
        t0 = time.perf_counter()
        collision_count = 0
        
        for body_i, body_j, _, _ in pairs:
            contacts = p.getContactPoints(body_i, body_j)
            if contacts:
                collision_count += 1
        
        elapsed = (time.perf_counter() - t0) * 1000  # ms
        times_contact.append(elapsed)
        
        if iteration % 20 == 0:
            print(f"  Iteration {iteration}: {elapsed:.3f}ms ({collision_count} collisions)")
    
    avg_contact = np.mean(times_contact)
    std_contact = np.std(times_contact)
    
    # ========================================
    # Method 2: getClosestPoints only
    # ========================================
    print("\nBenchmarking Method 2: getClosestPoints only...")
    times_closest = []
    
    # Reset simulation state
    p.disconnect()
    client = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1/240)
    
    # Recreate objects with same setup
    objects = []
    for i in range(num_objects):
        row = i // grid_size
        col = i % grid_size
        x = col * spacing + np.random.uniform(-0.1, 0.1)
        y = row * spacing + np.random.uniform(-0.1, 0.1)
        z = object_size + 0.1
        
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX, 
            halfExtents=[object_size, object_size, object_size]
        )
        visual_shape = p.createVisualShape(
            p.GEOM_BOX, 
            halfExtents=[object_size, object_size, object_size]
        )
        
        mass = 1.0 if i % 3 == 0 else 0.0
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=[x, y, z]
        )
        objects.append((body_id, mass > 0))
    
    # Apply velocities and run physics
    kinematic_velocities = {}
    for body_id, is_physics in objects:
        if is_physics:
            p.resetBaseVelocity(
                body_id,
                linearVelocity=[
                    np.random.uniform(-movement_speed, movement_speed),
                    np.random.uniform(-movement_speed, movement_speed),
                    0
                ]
            )
        else:
            kinematic_velocities[body_id] = np.array([
                np.random.uniform(-movement_speed, movement_speed),
                np.random.uniform(-movement_speed, movement_speed),
                0
            ])
    
    for step in range(100):
        # Move kinematic objects
        for body_id, is_physics in objects:
            if not is_physics:
                pos, orn = p.getBasePositionAndOrientation(body_id)
                vel = kinematic_velocities[body_id]
                new_pos = [
                    pos[0] + vel[0] * (1/240),
                    pos[1] + vel[1] * (1/240),
                    pos[2]
                ]
                p.resetBasePositionAndOrientation(body_id, new_pos, orn)
        p.stepSimulation()
    
    pairs = [(objects[i][0], objects[j][0], objects[i][1], objects[j][1]) 
             for i in range(len(objects)) 
             for j in range(i+1, len(objects))]
    
    for iteration in range(num_iterations):
        # Move kinematic objects
        for body_id, is_physics in objects:
            if not is_physics:
                pos, orn = p.getBasePositionAndOrientation(body_id)
                vel = kinematic_velocities[body_id]
                new_pos = [
                    pos[0] + vel[0] * (1/240),
                    pos[1] + vel[1] * (1/240),
                    pos[2]
                ]
                p.resetBasePositionAndOrientation(body_id, new_pos, orn)
        
        p.stepSimulation()
        
        t0 = time.perf_counter()
        collision_count = 0
        
        for body_i, body_j, _, _ in pairs:
            closest = p.getClosestPoints(body_i, body_j, distance=0.0)
            if closest:
                collision_count += 1
        
        elapsed = (time.perf_counter() - t0) * 1000  # ms
        times_closest.append(elapsed)
        
        if iteration % 20 == 0:
            print(f"  Iteration {iteration}: {elapsed:.3f}ms ({collision_count} collisions)")
    
    avg_closest = np.mean(times_closest)
    std_closest = np.std(times_closest)
    
    # ========================================
    # Method 3: Hybrid (kinematic=getClosestPoints, physics=getContactPoints)
    # ========================================
    print("\nBenchmarking Method 3: Hybrid approach...")
    times_hybrid = []
    
    # Reset simulation
    p.disconnect()
    client = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1/240)
    
    # Recreate objects
    objects = []
    for i in range(num_objects):
        row = i // grid_size
        col = i % grid_size
        x = col * spacing + np.random.uniform(-0.1, 0.1)
        y = row * spacing + np.random.uniform(-0.1, 0.1)
        z = object_size + 0.1
        
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX, 
            halfExtents=[object_size, object_size, object_size]
        )
        visual_shape = p.createVisualShape(
            p.GEOM_BOX, 
            halfExtents=[object_size, object_size, object_size]
        )
        
        mass = 1.0 if i % 3 == 0 else 0.0
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=[x, y, z]
        )
        objects.append((body_id, mass > 0))
    
    kinematic_velocities = {}
    for body_id, is_physics in objects:
        if is_physics:
            p.resetBaseVelocity(
                body_id,
                linearVelocity=[
                    np.random.uniform(-movement_speed, movement_speed),
                    np.random.uniform(-movement_speed, movement_speed),
                    0
                ]
            )
        else:
            kinematic_velocities[body_id] = np.array([
                np.random.uniform(-movement_speed, movement_speed),
                np.random.uniform(-movement_speed, movement_speed),
                0
            ])
    
    for step in range(100):
        # Move kinematic objects
        for body_id, is_physics in objects:
            if not is_physics:
                pos, orn = p.getBasePositionAndOrientation(body_id)
                vel = kinematic_velocities[body_id]
                new_pos = [
                    pos[0] + vel[0] * (1/240),
                    pos[1] + vel[1] * (1/240),
                    pos[2]
                ]
                p.resetBasePositionAndOrientation(body_id, new_pos, orn)
        p.stepSimulation()
    
    pairs = [(objects[i][0], objects[j][0], objects[i][1], objects[j][1]) 
             for i in range(len(objects)) 
             for j in range(i+1, len(objects))]
    
    for iteration in range(num_iterations):
        # Move kinematic objects
        for body_id, is_physics in objects:
            if not is_physics:
                pos, orn = p.getBasePositionAndOrientation(body_id)
                vel = kinematic_velocities[body_id]
                new_pos = [
                    pos[0] + vel[0] * (1/240),
                    pos[1] + vel[1] * (1/240),
                    pos[2]
                ]
                p.resetBasePositionAndOrientation(body_id, new_pos, orn)
        
        p.stepSimulation()
        
        t0 = time.perf_counter()
        collision_count = 0
        
        for body_i, body_j, is_physics_i, is_physics_j in pairs:
            # Use getContactPoints if both are physics, else getClosestPoints
            if is_physics_i and is_physics_j:
                result = p.getContactPoints(body_i, body_j)
            else:
                result = p.getClosestPoints(body_i, body_j, distance=0.0)
            
            if result:
                collision_count += 1
        
        elapsed = (time.perf_counter() - t0) * 1000  # ms
        times_hybrid.append(elapsed)
        
        if iteration % 20 == 0:
            print(f"  Iteration {iteration}: {elapsed:.3f}ms ({collision_count} collisions)")
    
    avg_hybrid = np.mean(times_hybrid)
    std_hybrid = np.std(times_hybrid)
    
    # ========================================
    # Results summary
    # ========================================
    print(f"\n{'='*80}")
    print("RESULTS SUMMARY")
    print(f"{'='*80}")
    print(f"Configuration: {num_objects} objects, size={object_size}m, speed={movement_speed}m/s")
    print(f"Total pairs checked: {len(pairs)}")
    print(f"\nMethod 1: getContactPoints only")
    print(f"  Average: {avg_contact:.3f}ms ± {std_contact:.3f}ms")
    print(f"\nMethod 2: getClosestPoints only")
    print(f"  Average: {avg_closest:.3f}ms ± {std_closest:.3f}ms")
    print(f"  Speedup: {avg_contact/avg_closest:.2f}x {'FASTER' if avg_closest < avg_contact else 'SLOWER'}")
    print(f"\nMethod 3: Hybrid approach")
    print(f"  Average: {avg_hybrid:.3f}ms ± {std_hybrid:.3f}ms")
    print(f"  Speedup: {avg_contact/avg_hybrid:.2f}x {'FASTER' if avg_hybrid < avg_contact else 'SLOWER'}")
    print(f"\nBest method: ", end="")
    if avg_contact <= min(avg_closest, avg_hybrid):
        print("getContactPoints (current implementation) ✅")
    elif avg_closest <= avg_hybrid:
        print("getClosestPoints only")
    else:
        print("Hybrid approach")
    print(f"{'='*80}\n")
    
    p.disconnect()
    
    return {
        'contact': (avg_contact, std_contact),
        'closest': (avg_closest, std_closest),
        'hybrid': (avg_hybrid, std_hybrid)
    }


if __name__ == "__main__":
    print("\n" + "="*80)
    print("COLLISION DETECTION METHODS BENCHMARK - WITH MOVING OBJECTS")
    print("="*80)
    
    # Test 1: Small objects (0.3m) - fewer contacts
    print("\n### Test 1: Small objects (0.3m half-extent)")
    results_small = benchmark_with_movement(
        num_objects=50,
        object_size=0.3,
        num_iterations=100,
        movement_speed=1.5
    )
    
    # Test 2: Medium objects (0.5m) - moderate contacts
    print("\n### Test 2: Medium objects (0.5m half-extent)")
    results_medium = benchmark_with_movement(
        num_objects=50,
        object_size=0.5,
        num_iterations=100,
        movement_speed=2.0
    )
    
    # Test 3: Large objects (0.8m) - many contacts
    print("\n### Test 3: Large objects (0.8m half-extent)")
    results_large = benchmark_with_movement(
        num_objects=50,
        object_size=0.8,
        num_iterations=100,
        movement_speed=2.5
    )
    
    # Overall summary
    print("\n" + "="*80)
    print("OVERALL SUMMARY")
    print("="*80)
    print("\nSmall objects (0.3m):")
    print(f"  getContactPoints: {results_small['contact'][0]:.3f}ms")
    print(f"  getClosestPoints: {results_small['closest'][0]:.3f}ms ({results_small['contact'][0]/results_small['closest'][0]:.2f}x)")
    print(f"  Hybrid:           {results_small['hybrid'][0]:.3f}ms ({results_small['contact'][0]/results_small['hybrid'][0]:.2f}x)")
    
    print("\nMedium objects (0.5m):")
    print(f"  getContactPoints: {results_medium['contact'][0]:.3f}ms")
    print(f"  getClosestPoints: {results_medium['closest'][0]:.3f}ms ({results_medium['contact'][0]/results_medium['closest'][0]:.2f}x)")
    print(f"  Hybrid:           {results_medium['hybrid'][0]:.3f}ms ({results_medium['contact'][0]/results_medium['hybrid'][0]:.2f}x)")
    
    print("\nLarge objects (0.8m):")
    print(f"  getContactPoints: {results_large['contact'][0]:.3f}ms")
    print(f"  getClosestPoints: {results_large['closest'][0]:.3f}ms ({results_large['contact'][0]/results_large['closest'][0]:.2f}x)")
    print(f"  Hybrid:           {results_large['hybrid'][0]:.3f}ms ({results_large['contact'][0]/results_large['hybrid'][0]:.2f}x)")
    print("="*80 + "\n")
