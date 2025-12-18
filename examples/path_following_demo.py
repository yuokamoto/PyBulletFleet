#!/usr/bin/env python3
"""
Path Following Demo

Demonstrates two motion modes:
1. Omnidirectional: Robot can move in any direction without rotating first
2. Differential Drive: Robot must rotate to face target (3D pitch/yaw), then move forward in 3D

Five robots demonstrate:
- Blue (Omni): 2D circle path
- Red (Diff): 2D square path
- Green (Omni3D): Tilted 3D circle path
- Yellow (Diff3D): 3D square path with pitch control
- Magenta (Climb): Climbing path with auto-approach and final orientation alignment

All robots use realistic velocity and acceleration constraints.
"""

import os
import sys

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pybullet as p

from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Path, Pose
from pybullet_fleet.sim_object import ShapeParams


def main():
    # Create simulation
    params = SimulationParams(gui=True, timestep=1.0 / 10.0, speed=3.0)
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
        visual_shape=ShapeParams(
            shape_type="mesh",
            mesh_path=mesh_path,
            mesh_scale=[0.5, 0.25, 0.3],  # Rectangular: longer in X direction
            rgba_color=[0.2, 0.4, 1.0, 1.0],  # Blue
        ),
        collision_shape=ShapeParams(shape_type="box", half_extents=[0.25, 0.125, 0.15]),
        initial_pose=Pose.from_xyz(-4.0, 0.0, 0.5),  # Larger circle
        mass=0.0,  # Kinematic control (no physics simulation)
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
        mass=0.0,  # Kinematic control (no physics simulation)
        max_linear_vel=2.0,  # Faster max speed
        max_linear_accel=0.5,  # MUCH slower acceleration for visible effect
        max_angular_vel=1.0,  # Slower rotation
        motion_mode="differential",
    )
    robot_diff = Agent.from_params(differential_params, sim_core=sim)

    # Robot 3: Omnidirectional (green) - rectangular cube, follows 3D circle path
    omni3d_params = AgentSpawnParams(
        visual_shape=ShapeParams(
            shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.4, 0.2, 0.25], rgba_color=[0.2, 1.0, 0.4, 1.0]  # Green
        ),
        collision_shape=ShapeParams(shape_type="box", half_extents=[0.2, 0.1, 0.125]),
        initial_pose=Pose.from_xyz(0.0, -6.0, 2.0),
        mass=0.0,  # Kinematic control (no physics simulation)
        max_linear_vel=2.0,
        max_linear_accel=0.5,
        motion_mode="omnidirectional",
    )
    robot_omni_3d = Agent.from_params(omni3d_params, sim_core=sim)

    # Robot 4: Differential Drive (yellow) - follows 3D square path
    # Uses straight 3D line motion with pitch control
    diff_full3d_params = AgentSpawnParams(
        urdf_path=urdf_path,
        initial_pose=Pose.from_xyz(0.0, 6.0, 0.3),
        use_fixed_base=False,
        mass=0.0,  # Kinematic control (no physics simulation)
        max_linear_vel=2.0,
        max_linear_accel=0.5,
        max_angular_vel=1.0,
        motion_mode="differential",
    )
    robot_diff_full3d = Agent.from_params(diff_full3d_params, sim_core=sim)

    # Robot 5: Differential Drive (magenta/purple) - climbing path demo
    # Starts far from path, demonstrates auto-approach and final orientation alignment
    climb_params = AgentSpawnParams(
        urdf_path=urdf_path,
        initial_pose=Pose.from_xyz(-10.0, -10.0, 0.3),  # Far from path start
        use_fixed_base=False,
        mass=0.0,  # Kinematic control
        max_linear_vel=2.0,
        max_linear_accel=0.5,
        max_angular_vel=1.0,
        max_angular_accel=5.0,
        motion_mode="differential",
    )
    robot_climb = Agent.from_params(climb_params, sim_core=sim)

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

    # Climbing path for the 5th robot: horizontal → climb → horizontal
    # Simple 4-point path demonstrating auto-approach and final orientation

    # Calculate orientation for point 3 where Z+ should be perpendicular to the slope
    # Slope direction: from point 2 to point 3
    p2 = np.array([-3.0, 0.0, 0.5])
    p3 = np.array([2.0, 0.0, 3.5])
    slope_direction = p3 - p2  # [5.0, 0.0, 3.0]

    # For Z+ to be perpendicular to the slope (slope normal):
    # The slope is in XZ plane, so normal is perpendicular to slope direction
    # Normal vector: rotate slope direction by 90° in XZ plane
    # If slope direction is [dx, 0, dz], normal is [-dz, 0, dx]
    slope_normal = np.array([-slope_direction[2], 0.0, slope_direction[0]])
    slope_normal = slope_normal / np.linalg.norm(slope_normal)

    # Calculate pitch angle for Z+ to point in slope_normal direction
    # slope_normal = [nx, 0, nz], we want Z+ axis to align with this
    # pitch = arctan2(nx, nz) (rotation around Y axis)
    slope_pitch = np.arctan2(slope_normal[0], slope_normal[2])

    # Create waypoints with simple orientations
    climbing_waypoints = [
        Pose.from_euler(-8.0, 0.0, 0.5, roll=0, pitch=0, yaw=0),  # Point 1: Ground start, Z+ = world up
        Pose.from_euler(-3.0, 0.0, 0.5, roll=0, pitch=0, yaw=0),  # Point 2: Slope start, Z+ = world up
        Pose.from_euler(2.0, 0.0, 3.5, roll=0, pitch=slope_pitch, yaw=0),  # Point 3: Slope end, Z+ ⊥ slope
        Pose.from_euler(7.0, 0.0, 3.5, roll=0, pitch=0, yaw=0),  # Point 4: Elevated end, Z+ = world up
    ]

    climb_path = Path(waypoints=climbing_waypoints)

    # Visualize paths using Path.visualize() method
    print("Visualizing paths...")

    # Blue circle for omnidirectional robot
    circle_path.visualize(
        line_color=[0.2, 0.4, 1.0],
        line_width=2.0,
        show_waypoints=True,
        show_axes=True,
        axis_length=0.4,
        show_points=True,
    )

    # Red square for differential robot
    square_path.visualize(
        line_color=[1.0, 0.2, 0.2],
        line_width=2.0,
    )

    # Green 3D circle
    circle_path_3d.visualize(
        line_color=[0.2, 1.0, 0.4],
        line_width=2.0,
        show_waypoints=True,
        show_axes=True,
        axis_length=0.4,
        show_points=True,
    )

    # Yellow 3D square
    square_path_3d.visualize(
        line_color=[1.0, 0.8, 0.2],
        line_width=2.0,
        show_waypoints=True,
        show_axes=True,
        axis_length=0.4,
        show_points=True,
    )

    # Climbing path visualization with different colors for each segment
    # Segment 1: Cyan (ground start → slope start)
    p.addUserDebugLine(
        climbing_waypoints[0].position, climbing_waypoints[1].position, [0.0, 1.0, 1.0], lineWidth=3.0, lifeTime=0
    )

    # Segment 2: Magenta (slope start → slope end, diagonal climb)
    p.addUserDebugLine(
        climbing_waypoints[1].position, climbing_waypoints[2].position, [1.0, 0.2, 1.0], lineWidth=3.0, lifeTime=0
    )

    # Segment 3: Orange (slope end → elevated end)
    p.addUserDebugLine(
        climbing_waypoints[2].position, climbing_waypoints[3].position, [1.0, 0.6, 0.2], lineWidth=3.0, lifeTime=0
    )

    # Show waypoint orientations for climb path
    climb_path.visualize(
        show_lines=False,
        show_waypoints=True,
        show_axes=True,
        axis_length=0.4,
        show_points=True,
    )

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

    print(f"\n  Omnidirectional 3D (Green Rectangle): Following tilted circle path ({len(circle_path_3d)} waypoints)")
    print(f"    - Max velocity: {robot_omni_3d.max_linear_vel} m/s")
    print(f"    - Max acceleration: {robot_omni_3d.max_linear_accel} m/s²")
    print(f"    - Total distance: {circle_path_3d.get_total_distance():.2f} m")
    print("    - Note: Smooth 3D motion along tilted circle")

    print(f"\n  Differential Drive 3D (Yellow Mobile Robot): Following 3D square path ({len(square_path_3d)} waypoints)")
    print(f"    - Max velocity: {robot_diff_full3d.max_linear_vel} m/s")
    print(f"    - Max acceleration: {robot_diff_full3d.max_linear_accel} m/s²")
    print(f"    - Max angular velocity: {robot_diff_full3d.max_angular_vel} rad/s")
    print(f"    - Total distance: {square_path_3d.get_total_distance():.2f} m")
    print("    - Note: Rotates to face target (3D pitch/yaw), then moves in straight 3D line")

    print(f"\n  Climbing Path Demo (Magenta Mobile Robot): Following climbing path ({len(climb_path)} waypoints)")
    print(f"    - Max velocity: {robot_climb.max_linear_vel} m/s")
    print(f"    - Max acceleration: {robot_climb.max_linear_accel} m/s²")
    print(f"    - Max angular velocity: {robot_climb.max_angular_vel} rad/s")
    print(f"    - Total distance: {climb_path.get_total_distance():.2f} m")
    print(f"    - Starting position: {robot_climb.get_pose().position}")
    print(f"    - Path start: {climb_path.waypoints[0].position}")
    distance_to_path = np.linalg.norm(np.array(robot_climb.get_pose().position) - np.array(climb_path.waypoints[0].position))
    print(f"    - Distance to path start: {distance_to_path:.2f}m")
    print("    - Note: Auto-approach enabled, final orientation alignment enabled")
    print("    - Path: Ground (cyan) → Climb (magenta) → Elevated (orange)")

    robot_omni.set_path(circle_path.waypoints)
    robot_diff.set_path(square_path.waypoints)
    robot_omni_3d.set_path(circle_path_3d.waypoints)
    robot_diff_full3d.set_path(square_path_3d.waypoints)
    robot_climb.set_path(
        climb_path.waypoints,
        auto_approach=True,  # Automatically add approach waypoint
        final_orientation_align=True,  # Rotate to final orientation after reaching position
    )

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
        "Differential 3D\n(Straight 3D Lines)", [0.0, 7.5, 3.5], textColorRGB=[1.0, 0.8, 0.2], textSize=1.5, lifeTime=0
    )

    p.addUserDebugText(
        "Climbing Path Demo\n(Auto-Approach + Final Orientation)",
        [-2.5, 0.0, 5.0],
        textColorRGB=[1.0, 0.2, 1.0],
        textSize=1.5,
        lifeTime=0,
    )

    p.addUserDebugText(
        "Start Position\n(Far from path)",
        [-10.0, -10.0, 1.5],
        textColorRGB=[1.0, 1.0, 0.2],
        textSize=1.2,
        lifeTime=0,
    )

    # Set camera to view all 5 robots
    p.resetDebugVisualizerCamera(
        cameraDistance=25.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[-2, 0, 2]  # Zoom out for all paths
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
    print("  - Red (differential): Smooth rotation + smooth forward motion (2D square)")
    print("  - Yellow (differential 3D): Smooth 3D rotation + straight 3D line motion")
    print("  - Magenta (climb): Auto-approach → Ground → Climb → Elevated → Final orientation")

    # Movement callback with velocity display
    step_counter = [0]  # Use list to allow modification in nested function

    def robot_movement_callback(sim_core, dt):
        """Update robot movement each step and display velocities"""
        # Print velocities every 10 steps (1 second at 10Hz)
        step_counter[0] += 1
        if step_counter[0] % 10 == 0:
            omni_vel = robot_omni.get_velocity()
            diff_vel = robot_diff.get_velocity()
            omni3d_vel = robot_omni_3d.get_velocity()
            diff_full3d_vel = robot_diff_full3d.get_velocity()
            climb_vel = robot_climb.get_velocity()
            omni_speed = np.linalg.norm(omni_vel[:2])
            diff_speed = np.linalg.norm(diff_vel[:2])
            omni3d_speed = np.linalg.norm(omni3d_vel)
            diff_full3d_speed = np.linalg.norm(diff_full3d_vel)
            climb_speed = np.linalg.norm(climb_vel)

            print(
                f"[t={step_counter[0]*dt:.1f}s] "
                f"Omni2D: {omni_speed:.3f} | "
                f"Omni3D: {omni3d_speed:.3f} | "
                f"Diff2D: {diff_speed:.3f} | "
                f"Full3D: {diff_full3d_speed:.3f} | "
                f"Climb: {climb_speed:.3f}"
            )

    # Register callback and run simulation
    sim.register_callback(robot_movement_callback, frequency=None)  # Call every step
    sim.run_simulation()


if __name__ == "__main__":
    main()
