#!/usr/bin/env python3
"""
Path Following Demo

Demonstrates two motion modes:
1. Omnidirectional: Robot can move in any direction without rotating first
2. Differential Drive: Robot must rotate to face target, then move forward

Both robots follow predefined paths (circle and square) with realistic
velocity and acceleration constraints.
"""

import os
import sys

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pybullet as p

from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Path, Pose


def main():
    # Create simulation
    params = SimulationParams(gui=True, timestep=1.0 / 10.0, speed=10.0)
    sim = MultiRobotSimulationCore(params)

    # Get absolute paths
    current_dir = os.path.dirname(os.path.abspath(__file__))
    mesh_path = os.path.join(current_dir, "..", "mesh", "cube.obj")
    urdf_path = os.path.join(current_dir, "..", "robots", "mobile_robot.urdf")

    # Create two robots with different motion modes
    print("Spawning robots...")

    # Robot 1: Omnidirectional (blue) - rectangular cube, follows circle
    # Use rectangular shape so orientation is visible
    # SLOW ACCELERATION for clear smooth motion visualization
    omnidirectional_params = AgentSpawnParams(
        mesh_path=mesh_path,
        initial_pose=Pose.from_xyz(-4.0, 0.0, 0.5),  # Larger circle
        mesh_scale=[0.5, 0.25, 0.3],  # Rectangular: longer in X direction
        collision_half_extents=[0.25, 0.125, 0.15],
        rgba_color=[0.2, 0.4, 1.0, 1.0],  # Blue
        mass=1.0,
        max_vel=2.0,  # Faster max speed
        max_accel=0.5,  # MUCH slower acceleration for visible effect
        motion_mode="omnidirectional",
    )
    robot_omni = Agent.from_params(omnidirectional_params, sim_core=sim)

    # Robot 2: Differential Drive (red) - mobile_robot URDF, follows square
    # URDF has visible front direction
    # SLOW ACCELERATION for clear smooth motion visualization
    differential_params = AgentSpawnParams(
        urdf_path=urdf_path,
        initial_pose=Pose.from_xyz(4.0, 0.0, 0.3),  # Larger square
        use_fixed_base=False,
        mass=1.0,
        max_vel=2.0,  # Faster max speed
        max_accel=0.5,  # MUCH slower acceleration for visible effect
        max_angular_vel=1.0,  # Slower rotation
        motion_mode="differential",
    )
    robot_diff = Agent.from_params(differential_params, sim_core=sim)

    # Create paths
    print("Creating paths...")

    # Larger circle path for omnidirectional robot (center at -4, 0)
    circle_path = Path.create_circle(center=[-4.0, 0.0], radius=3.0, num_points=32, height=0.5)

    # Larger square path for differential drive robot (center at 4, 0)
    square_path = Path.create_square(center=[4.0, 0.0], side_length=6.0, height=0.5)

    # Add debug visualization for paths
    for i in range(len(circle_path) - 1):
        p1 = circle_path[i].position
        p2 = circle_path[i + 1].position
        p.addUserDebugLine(p1, p2, [0.2, 0.4, 1.0], lineWidth=2.0, lifeTime=0)

    for i in range(len(square_path) - 1):
        p1 = square_path[i].position
        p2 = square_path[i + 1].position
        p.addUserDebugLine(p1, p2, [1.0, 0.2, 0.2], lineWidth=2.0, lifeTime=0)

    # Set paths
    print("\nRobot configurations:")
    print(f"  Omnidirectional (Blue Rectangle): Following circle path ({len(circle_path)} waypoints)")
    print(f"    - Max velocity: {robot_omni.max_vel} m/s")
    print(f"    - Max acceleration: {robot_omni.max_accel} m/s²")
    print(f"    - Total distance: {circle_path.get_total_distance():.2f} m")
    print("    - Note: Orientation stays fixed while moving in any direction")

    print(f"\n  Differential Drive (Red Mobile Robot): Following square path ({len(square_path)} waypoints)")
    print(f"    - Max velocity: {robot_diff.max_vel} m/s")
    print(f"    - Max acceleration: {robot_diff.max_accel} m/s²")
    print(f"    - Max angular velocity: {robot_diff.max_angular_vel} rad/s")
    print(f"    - Total distance: {square_path.get_total_distance():.2f} m")
    print("    - Note: Rotates to face target, then moves forward")

    robot_omni.set_path(circle_path.waypoints)
    robot_diff.set_path(square_path.waypoints)

    # Add text labels
    p.addUserDebugText(
        "Omnidirectional\n(Smooth Accel/Decel)", [-4.0, 0.0, 2.5], textColorRGB=[0.2, 0.4, 1.0], textSize=1.5, lifeTime=0
    )

    p.addUserDebugText(
        "Differential Drive\n(Rotate then Move)", [4.0, 0.0, 2.5], textColorRGB=[1.0, 0.2, 0.2], textSize=1.5, lifeTime=0
    )

    # Set camera to view both robots
    p.resetDebugVisualizerCamera(
        cameraDistance=14.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0]  # Zoom out for larger paths
    )

    print("\nSimulation started! Press Ctrl+C to exit.")
    print("=" * 60)
    print("Two-Point Interpolation Demo:")
    print("  - max_vel = 2.0 m/s")
    print("  - max_accel = 0.5 m/s² (SLOW for visible smooth acceleration)")
    print("  - Longer paths (radius=3m, side=6m) to see acceleration/deceleration")
    print("=" * 60)
    print("Watch the SMOOTH acceleration and deceleration:")
    print("  - Blue (omnidirectional): Smooth accel/decel, orientation FIXED")
    print("  - Red (differential): Smooth rotation + smooth forward motion")

    # Movement callback with velocity display
    step_counter = [0]  # Use list to allow modification in nested function

    def robot_movement_callback(sim_objects, sim_core, dt):
        """Update robot movement each step and display velocities"""
        for obj in sim_objects:
            if isinstance(obj, Agent):
                obj.update(dt)

        # Print velocities every 10 steps (1 second at 10Hz)
        step_counter[0] += 1
        if step_counter[0] % 10 == 0:
            omni_vel = robot_omni.get_velocity()
            diff_vel = robot_diff.get_velocity()
            omni_speed = np.linalg.norm(omni_vel[:2])
            diff_speed = np.linalg.norm(diff_vel[:2])

            print(
                f"[t={step_counter[0]*dt:.1f}s] Omnidirectional speed: {omni_speed:.3f} m/s | "
                f"Differential speed: {diff_speed:.3f} m/s"
            )

    # Register callback and run simulation
    sim.register_callback(robot_movement_callback, frequency=None)  # Call every step
    sim.run_simulation()


if __name__ == "__main__":
    main()
