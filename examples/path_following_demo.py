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

from pybullet_fleet.agent import Agent, AgentSpawnParams, Differential3DMode
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Path, Pose


def main():
    # Create simulation
    params = SimulationParams(gui=True, timestep=1.0 / 10.0, speed=1.0)
    sim = MultiRobotSimulationCore(params)

    # Get absolute paths
    current_dir = os.path.dirname(os.path.abspath(__file__))
    mesh_path = os.path.join(current_dir, "..", "mesh", "cube.obj")
    urdf_path = os.path.join(current_dir, "..", "robots", "mobile_robot.urdf")

    # Create robots with different motion modes
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
        max_linear_vel=2.0,  # Faster max speed
        max_linear_accel=0.5,  # MUCH slower acceleration for visible effect
        motion_mode="omnidirectional",
    )
    robot_omni = Agent.from_params(omnidirectional_params, sim_core=sim)

    # Robot 2: Differential Drive (red) - mobile_robot URDF, follows square (2D)
    # URDF has visible front direction
    # SLOW ACCELERATION for clear smooth motion visualization
    differential_params = AgentSpawnParams(
        urdf_path=urdf_path,
        initial_pose=Pose.from_xyz(4.0, 0.0, 0.3),  # Larger square
        use_fixed_base=False,
        mass=1.0,
        max_linear_vel=2.0,  # Faster max speed
        max_linear_accel=0.5,  # MUCH slower acceleration for visible effect
        max_angular_vel=1.0,  # Slower rotation
        motion_mode="differential",
    )
    robot_diff = Agent.from_params(differential_params, sim_core=sim)

    # Robot 3: Omnidirectional (green) - rectangular cube, follows 3D circle path
    omni3d_params = AgentSpawnParams(
        mesh_path=mesh_path,
        initial_pose=Pose.from_xyz(0.0, -6.0, 2.0),
        mesh_scale=[0.4, 0.2, 0.25],
        collision_half_extents=[0.2, 0.1, 0.125],
        rgba_color=[0.2, 1.0, 0.4, 1.0],  # Green
        mass=1.0,
        max_linear_vel=2.0,
        max_linear_accel=0.5,
        motion_mode="omnidirectional",
    )
    robot_omni_3d = Agent.from_params(omni3d_params, sim_core=sim)

    # Robot 4: Differential Drive with full_3d mode (yellow) - follows 3D square path
    # This robot uses straight 3D line motion after yaw alignment
    diff_full3d_params = AgentSpawnParams(
        urdf_path=urdf_path,
        initial_pose=Pose.from_xyz(0.0, 6.0, 0.3),
        use_fixed_base=False,
        mass=1.0,
        max_linear_vel=2.0,
        max_linear_accel=0.5,
        max_angular_vel=1.0,
        motion_mode="differential",
    )
    robot_diff_full3d = Agent.from_params(diff_full3d_params, sim_core=sim)
    # Set full_3d mode for straight 3D line motion
    robot_diff_full3d.differential_3d_mode = Differential3DMode.FULL_3D

    # Robot 5: Differential Drive with 2d_with_z mode (orange) - follows same 3D square path as Robot 4
    # This robot uses XY differential motion + Z interpolation for comparison with full_3d mode
    diff_2dwithz_params = AgentSpawnParams(
        urdf_path=urdf_path,
        initial_pose=Pose.from_xyz(-3.0, 6.0, 0.3),  # Offset to left of Robot 4
        use_fixed_base=False,
        mass=1.0,
        max_linear_vel=2.0,
        max_linear_accel=0.5,
        max_angular_vel=1.0,
        motion_mode="differential",
    )
    robot_diff_2dwithz = Agent.from_params(diff_2dwithz_params, sim_core=sim)
    # Keep default 2d_with_z mode (explicitly set for clarity)
    robot_diff_2dwithz.differential_3d_mode = Differential3DMode.TWO_D_WITH_Z

    # Create paths
    print("Creating paths...")

    # Larger circle path for omnidirectional robot (center at -4, 0, 0.5)
    # Default XY plane (Z+ is upward, perpendicular to circle plane)
    circle_path = Path.create_circle(center=[-4.0, 0.0, 0.5], radius=3.0, num_points=32)

    # Larger square path for differential drive robot (center at 4, 0, 0.5)
    # Default XY plane (Z+ is upward, perpendicular to square plane)
    square_path = Path.create_square(center=[4.0, 0.0, 0.5], side_length=6.0)

    # 3D circle path for the green omnidirectional robot.
    # This path is tilted using roll/pitch so the robot moves in full 3D.
    # Z+ of the robot will point perpendicular to the tilted circle plane
    circle_path_3d = Path.create_circle(
        center=[0.0, -6.0, 2.0],
        radius=2.0,
        num_points=32,
        rpy=[np.deg2rad(25.0), np.deg2rad(10.0), 0.0],  # Tilted plane
    )

    # 3D square path for full_3d differential robot with significant Z changes
    # Calculate RPY so that Z+ is perpendicular to the square plane
    # The square goes: (0,6,0.5) -> (3,6,2.5) -> (3,9,2.5) -> (0,9,0.5)
    # Two edges: v1 = (3,0,2), v2 = (0,3,0)
    # Normal vector (perpendicular to plane): v1 × v2
    v1 = np.array([3.0, 0.0, 2.0])
    v2 = np.array([0.0, 3.0, 0.0])
    normal = np.cross(v1, v2)  # Normal to the plane
    normal = normal / np.linalg.norm(normal)  # Normalize

    # Calculate roll and pitch to align Z+ with normal vector
    # normal = [nx, ny, nz], we want Z+ = [0, 0, 1] to align with normal
    # pitch = arcsin(nx), roll = arctan2(-ny, nz)
    pitch_3d = np.arcsin(np.clip(normal[0], -1.0, 1.0))
    roll_3d = np.arctan2(-normal[1], normal[2])

    square_path_3d = Path.create_square(
        center=[1.5, 7.5, 1.5],  # Center of the square
        side_length=4.24,  # sqrt((3^2 + 3^2 + 2^2)) ≈ 4.24 for similar size
        rpy=[roll_3d, pitch_3d, 0.0],
    )

    # 3D square path for 2d_with_z differential robot (same orientation, offset to left)
    square_path_3d_2dwithz = Path.create_square(
        center=[-1.5, 7.5, 1.5],  # Offset to left
        side_length=4.24,
        rpy=[roll_3d, pitch_3d, 0.0],  # Same orientation as yellow robot
    )

    # Add debug visualization for paths
    for i in range(len(circle_path) - 1):
        p1 = circle_path[i].position
        p2 = circle_path[i + 1].position
        p.addUserDebugLine(p1, p2, [0.2, 0.4, 1.0], lineWidth=2.0, lifeTime=0)

    for i in range(len(square_path) - 1):
        p1 = square_path[i].position
        p2 = square_path[i + 1].position
        p.addUserDebugLine(p1, p2, [1.0, 0.2, 0.2], lineWidth=2.0, lifeTime=0)

    # 3D circle path (green)
    for i in range(len(circle_path_3d) - 1):
        p1 = circle_path_3d[i].position
        p2 = circle_path_3d[i + 1].position
        p.addUserDebugLine(p1, p2, [0.2, 1.0, 0.4], lineWidth=2.0, lifeTime=0)

    # 3D square path (yellow/orange)
    for i in range(len(square_path_3d) - 1):
        p1 = square_path_3d[i].position
        p2 = square_path_3d[i + 1].position
        p.addUserDebugLine(p1, p2, [1.0, 0.8, 0.2], lineWidth=2.0, lifeTime=0)

    # 3D square path for 2d_with_z (orange)
    for i in range(len(square_path_3d_2dwithz) - 1):
        p1 = square_path_3d_2dwithz[i].position
        p2 = square_path_3d_2dwithz[i + 1].position
        p.addUserDebugLine(p1, p2, [1.0, 0.5, 0.0], lineWidth=2.0, lifeTime=0)

    # Set paths
    print("\nRobot configurations:")
    print(f"  Omnidirectional (Blue Rectangle): Following circle path ({len(circle_path)} waypoints)")
    print(f"    - Max velocity: {robot_omni.max_linear_vel} m/s")
    print(f"    - Max acceleration: {robot_omni.max_linear_accel} m/s²")
    print(f"    - Total distance: {circle_path.get_total_distance():.2f} m")
    print("    - Note: Orientation stays fixed while moving in any direction")

    print(f"\n  Differential Drive (Red Mobile Robot): Following square path ({len(square_path)} waypoints)")
    print(f"    - Max velocity: {robot_diff.max_linear_vel} m/s")
    print(f"    - Max acceleration: {robot_diff.max_linear_accel} m/s²")
    print(f"    - Max angular velocity: {robot_diff.max_angular_vel} rad/s")
    print(f"    - Total distance: {square_path.get_total_distance():.2f} m")
    print("    - Note: Rotates to face target, then moves forward")
    print(f"    - Mode: {robot_diff.differential_3d_mode} (yaw rotation + XY differential + Z interpolation)")

    print(f"\n  Omnidirectional 3D (Green Rectangle): Following tilted circle path ({len(circle_path_3d)} waypoints)")
    print(f"    - Max velocity: {robot_omni_3d.max_linear_vel} m/s")
    print(f"    - Max acceleration: {robot_omni_3d.max_linear_accel} m/s²")
    print(f"    - Total distance: {circle_path_3d.get_total_distance():.2f} m")
    print("    - Note: Smooth 3D motion along tilted circle")

    print(f"\n  Differential Drive Full 3D (Yellow Mobile Robot): Following 3D square path ({len(square_path_3d)} waypoints)")
    print(f"    - Max velocity: {robot_diff_full3d.max_linear_vel} m/s")
    print(f"    - Max acceleration: {robot_diff_full3d.max_linear_accel} m/s²")
    print(f"    - Max angular velocity: {robot_diff_full3d.max_angular_vel} rad/s")
    print(f"    - Total distance: {square_path_3d.get_total_distance():.2f} m")
    print(f"    - Mode: {robot_diff_full3d.differential_3d_mode} (straight 3D line motion after yaw alignment)")
    print("    - Note: Rotates to face target, then moves in straight 3D line")

    print(
        f"\n  Differential Drive 2D with Z (Orange Mobile Robot): Following 3D square path ({len(square_path_3d_2dwithz)} waypoints)"
    )
    print(f"    - Max velocity: {robot_diff_2dwithz.max_linear_vel} m/s")
    print(f"    - Max acceleration: {robot_diff_2dwithz.max_linear_accel} m/s²")
    print(f"    - Max angular velocity: {robot_diff_2dwithz.max_angular_vel} rad/s")
    print(f"    - Total distance: {square_path_3d_2dwithz.get_total_distance():.2f} m")
    print(f"    - Mode: {robot_diff_2dwithz.differential_3d_mode} (XY differential + Z interpolation)")
    print("    - Note: Rotates to face XY target, moves with XY differential + Z interpolation")

    robot_omni.set_path(circle_path.waypoints)
    robot_diff.set_path(square_path.waypoints)
    robot_omni_3d.set_path(circle_path_3d.waypoints)
    robot_diff_full3d.set_path(square_path_3d.waypoints)
    robot_diff_2dwithz.set_path(square_path_3d_2dwithz.waypoints)

    # Add text labels
    p.addUserDebugText(
        "Omnidirectional\n(Smooth Accel/Decel)", [-4.0, 0.0, 2.5], textColorRGB=[0.2, 0.4, 1.0], textSize=1.5, lifeTime=0
    )

    p.addUserDebugText(
        "Differential Drive\n(Rotate then Move)", [4.0, 0.0, 2.5], textColorRGB=[1.0, 0.2, 0.2], textSize=1.5, lifeTime=0
    )

    p.addUserDebugText(
        "Omnidirectional 3D\n(Tilted Circle)", [0.0, -6.0, 4.0], textColorRGB=[0.2, 1.0, 0.4], textSize=1.5, lifeTime=0
    )

    p.addUserDebugText(
        "Differential Full 3D\n(Straight 3D Lines)", [0.0, 7.5, 3.5], textColorRGB=[1.0, 0.8, 0.2], textSize=1.5, lifeTime=0
    )

    p.addUserDebugText(
        "Differential 2D+Z\n(XY Diff + Z Interp)", [-1.5, 7.5, 3.5], textColorRGB=[1.0, 0.5, 0.0], textSize=1.5, lifeTime=0
    )

    # Set camera to view both robots
    p.resetDebugVisualizerCamera(
        cameraDistance=18.0, cameraYaw=45, cameraPitch=-25, cameraTargetPosition=[0, 0, 1.5]  # Zoom out for all paths
    )

    print("\nSimulation started! Press Ctrl+C to exit.")
    print("=" * 60)
    print("Two-Point Interpolation Demo:")
    print("  - max_linear_vel = 2.0 m/s")
    print("  - max_linear_accel = 0.5 m/s² (SLOW for visible smooth acceleration)")
    print("  - Longer paths (radius=3m, side=6m) to see acceleration/deceleration")
    print("=" * 60)
    print("Watch the SMOOTH acceleration and deceleration:")
    print("  - Blue (omnidirectional): Smooth accel/decel, orientation FIXED (2D circle)")
    print("  - Green (omnidirectional 3D): Smooth accel/decel along tilted 3D circle")
    print("  - Red (differential 2d_with_z): Smooth rotation + smooth forward motion (2D XY + Z interpolation)")
    print("  - Yellow (differential full_3d): Smooth rotation + straight 3D line motion")
    print("  - Orange (differential 2d_with_z): XY differential + Z interpolation (compare with yellow!)")

    # Movement callback with velocity display
    step_counter = [0]  # Use list to allow modification in nested function

    def robot_movement_callback(sim_objects, sim_core, dt):
        """Update robot movement each step and display velocities"""
        # Print velocities every 10 steps (1 second at 10Hz)
        step_counter[0] += 1
        if step_counter[0] % 10 == 0:
            omni_vel = robot_omni.get_velocity()
            diff_vel = robot_diff.get_velocity()
            omni3d_vel = robot_omni_3d.get_velocity()
            diff_full3d_vel = robot_diff_full3d.get_velocity()
            diff_2dwithz_vel = robot_diff_2dwithz.get_velocity()
            omni_speed = np.linalg.norm(omni_vel[:2])
            diff_speed = np.linalg.norm(diff_vel[:2])
            omni3d_speed = np.linalg.norm(omni3d_vel)
            diff_full3d_speed = np.linalg.norm(diff_full3d_vel)
            diff_2dwithz_speed = np.linalg.norm(diff_2dwithz_vel)

            print(
                f"[t={step_counter[0]*dt:.1f}s] "
                f"Omni2D: {omni_speed:.3f} | "
                f"Omni3D: {omni3d_speed:.3f} | "
                f"Diff2D: {diff_speed:.3f} | "
                f"Full3D: {diff_full3d_speed:.3f} | "
                f"2D+Z: {diff_2dwithz_speed:.3f}"
            )

    # Register callback and run simulation
    sim.register_callback(robot_movement_callback, frequency=None)  # Call every step
    sim.run_simulation()


if __name__ == "__main__":
    main()
