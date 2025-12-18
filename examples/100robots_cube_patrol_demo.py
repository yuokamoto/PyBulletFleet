#!/usr/bin/env python3
"""
100 Robots Cube Patrol Demo

Demonstrates 100 robots (mixed omnidirectional and differential drive) where each
robot patrols around its own 5m × 5m × 5m cube centered at its spawn position:
1. XY plane circuit (bottom level)
2. Vertical climb (Z+)
3. XY plane circuit (top level)
4. Descent (Z-)

Features:
- 100 robots spawned in a 10×10 grid (3m spacing)
- Each robot patrols a 5m×5m×5m cube around its spawn position
- Random mix of omnidirectional and differential drive (50/50)
- Random forward/backward direction for differential drive only (50/50) - tests both orientations
- All robots move in parallel, creating synchronized swarm behavior
- Color-coded by motion mode (blue=omni, red=differential)
- Realistic acceleration/deceleration constraints
"""

import os
import sys

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pybullet as p
import random

from pybullet_fleet.agent import AgentSpawnParams, MotionMode, MovementDirection
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Path, Pose


def create_cube_patrol_path(
    cube_center: list = [0.0, 0.0, 2.5],
    cube_size: float = 5.0,
) -> Path:
    """
    Create a patrol path around a cube (corners only):
    1. Bottom XY circuit (4 corners: TR → TL → BL → BR)
    2. Vertical climb from bottom BR corner directly to top BR corner
    3. Top XY circuit (4 corners: TR → TL → BL → BR)
    4. Vertical descent from top BR corner directly to bottom BR corner (loops back to start)

    Args:
        cube_center: Center of the cube [x, y, z]
        cube_size: Size of the cube (meters)

    Returns:
        Path object with 8 waypoints (4 bottom + 4 top corners only)
    """
    half_size = cube_size / 2.0
    cx, cy, cz = cube_center

    # Bottom level Z coordinate
    z_bottom = cz - half_size
    # Top level Z coordinate
    z_top = cz + half_size

    waypoints = []

    # 1. Bottom XY circuit (4 corners, counterclockwise from top-right)
    bottom_corners = [
        [cx + half_size, cy + half_size, z_bottom],  # Top-right
        [cx - half_size, cy + half_size, z_bottom],  # Top-left
        [cx - half_size, cy - half_size, z_bottom],  # Bottom-left
        [cx + half_size, cy - half_size, z_bottom],  # Bottom-right
        [cx + half_size, cy - half_size, z_top],  # Bottom-right
        [cx + half_size, cy + half_size, z_top],  # Top-right
        [cx - half_size, cy + half_size, z_top],  # Top-left
        [cx - half_size, cy - half_size, z_top],  # Bottom-left
        [cx + half_size, cy - half_size, z_bottom],  # Bottom-right
    ]

    for corner in bottom_corners:
        # Bottom level: Z+ points up (roll=0, pitch=0, yaw=0)
        waypoints.append(Pose.from_euler(corner[0], corner[1], corner[2], roll=0, pitch=0, yaw=0))

    return Path(waypoints=waypoints)


def main():
    # Seed for reproducibility
    np.random.seed(42)

    # Create simulation
    params = SimulationParams(gui=True, timestep=1.0 / 60.0, speed=5.0)
    sim = MultiRobotSimulationCore(params)

    # Get absolute path to URDF
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(current_dir, "..", "robots", "mobile_robot.urdf")

    print("=" * 70)
    print("100 Robots Cube Patrol Demo")
    print("=" * 70)
    print("Features:")
    print("  - 100 robots in 10×10 grid (3m spacing)")
    print("  - Each robot patrols 5m×5m×5m cube around its spawn position")
    print("  - Random mix: 50% omnidirectional, 50% differential drive")
    print("  - Random direction for differential only: 50% forward, 50% backward")
    print("  - Patrol: bottom XY → climb → top XY → descend → repeat")
    print("  - Color coding: Blue=omni, Red=differential")
    print("=" * 70)

    # Create cube patrol path (corners only, no intermediate points)
    cube_center = [0.0, 0.0, 2.5]
    cube_size = 5.0
    patrol_path = create_cube_patrol_path(
        cube_center=cube_center,
        cube_size=cube_size,
    )

    print("\nReference patrol path specs:")
    print(f"  - Waypoints per path: {len(patrol_path)} (corners only)")
    print(f"  - Distance per path: {patrol_path.get_total_distance():.2f} m")
    print(f"  - Cube size: {cube_size}m × {cube_size}m × {cube_size}m")
    print("  - Each robot will patrol around its spawn position")

    # Create agent manager
    manager = AgentManager(sim_core=sim)

    # Spawn 100 robots in 10×10 grid using AgentManager
    print("\nSpawning 100 robots...")

    # Grid configuration (10x10 grid, 3m spacing)
    grid_params = GridSpawnParams(
        x_min=0,
        x_max=9,  # 10 cells (0-9)
        y_min=0,
        y_max=9,  # 10 cells (0-9)
        spacing=[10.0, 10.0, 0.0],  # 10m spacing in X and Y
        offset=[-15.0, -15.0, 0.3],  # Start position
    )

    # Define two robot types with equal probability (50/50 split)
    spawn_params_omni = AgentSpawnParams(
        urdf_path=urdf_path,
        mass=0.0,  # Kinematic control
        max_linear_vel=2.0,
        max_linear_accel=1.0,
        max_angular_vel=2.0,
        max_angular_accel=5.0,
        motion_mode=MotionMode.OMNIDIRECTIONAL,
        use_fixed_base=False,
    )

    spawn_params_diff = AgentSpawnParams(
        urdf_path=urdf_path,
        mass=0.0,  # Kinematic control
        max_linear_vel=2.0,
        max_linear_accel=1.0,
        max_angular_vel=2.0,
        max_angular_accel=5.0,
        motion_mode=MotionMode.DIFFERENTIAL,
        use_fixed_base=False,
    )

    # Spawn with 50/50 probability
    spawn_params_list = [
        (spawn_params_omni, 0.5),  # 50% omnidirectional
        (spawn_params_diff, 0.5),  # 50% differential
    ]

    manager.spawn_agents_grid_mixed(
        num_agents=100,
        grid_params=grid_params,
        spawn_params_list=spawn_params_list,
    )

    # Count spawned robot types
    num_omni = sum(1 for robot in manager.objects if robot.motion_mode == MotionMode.OMNIDIRECTIONAL)
    num_diff = sum(1 for robot in manager.objects if robot.motion_mode == MotionMode.DIFFERENTIAL)

    print(f"✓ Spawned {len(manager.objects)} robots:")
    print(f"  - Omnidirectional: {num_omni} (blue)")
    print(f"  - Differential: {num_diff} (red)")

    # Set patrol path for each robot (centered at their spawn position)
    print("\nSetting patrol paths (each robot patrols 5m×5m×5m cube, corners only)...")
    num_forward = 0
    num_backward = 0
    num_omni_skipped = 0
    for robot in manager.objects:
        # Get robot's spawn position
        spawn_pos = robot.get_pose().position

        # Create individual patrol path centered at spawn position
        robot_patrol_path = create_cube_patrol_path(
            cube_center=[spawn_pos[0], spawn_pos[1], spawn_pos[2] + 2.2],  # Center cube above spawn
            cube_size=5.0,
        )

        # Randomly choose FORWARD or BACKWARD movement direction (only for differential drive)
        if robot.motion_mode == MotionMode.DIFFERENTIAL:
            direction = random.choice([MovementDirection.FORWARD, MovementDirection.BACKWARD])
            if direction == MovementDirection.FORWARD:
                num_forward += 1
            else:
                num_backward += 1
            robot.set_path(robot_patrol_path.waypoints, direction=direction)
        else:
            # Omnidirectional: direction parameter is ignored, use default
            num_omni_skipped += 1
            robot.set_path(robot_patrol_path.waypoints)

        # Visualize the path for each robot
        robot_patrol_path.visualize(
            show_lines=True,
            line_color=[0.5, 0.5, 0.5],  # Gray lines
            line_width=1.0,
            show_waypoints=True,
            show_axes=False,
            show_points=False,
            lifetime=0,  # Persistent visualization
        )

    print("✓ All robots assigned to individual patrol paths (with visualization)")
    print(f"  - Differential drive - Forward: {num_forward} robots")
    print(f"  - Differential drive - Backward: {num_backward} robots")
    print(f"  - Omnidirectional (direction N/A): {num_omni_skipped} robots")

    # Set camera to view the grid center
    # Grid center from offset + middle of the grid
    grid_center = [
        grid_params.offset[0] + (grid_params.x_max - grid_params.x_min) * grid_params.spacing[0] / 2,
        grid_params.offset[1] + (grid_params.y_max - grid_params.y_min) * grid_params.spacing[1] / 2,
        2.5,
    ]
    p.resetDebugVisualizerCamera(
        cameraDistance=60.0,  # Increased to see wider grid
        cameraYaw=45,
        cameraPitch=-35,
        cameraTargetPosition=grid_center,
    )

    # Add labels
    p.addUserDebugText(
        "100 Robots Parallel Patrol",
        [grid_center[0], grid_center[1], grid_center[2] + 5.0],
        textColorRGB=[1.0, 1.0, 0.0],
        textSize=2.0,
    )

    print("\n" + "=" * 70)
    print("Simulation started!")
    print("=" * 70)
    print("Watch 100 robots patrol in parallel:")
    print("  Each robot follows its own cube path (5m×5m×5m)")
    print("  1. Bottom XY circuit")
    print("  2. Vertical climb")
    print("  3. Top XY circuit")
    print("  4. Vertical descent")
    print("  5. Repeat...")
    print("\nPress Ctrl+C to exit")
    print("=" * 70)

    # Velocity monitoring callback
    step_counter = [0]

    def monitoring_callback(sim_core, dt):
        """Monitor and display statistics every 5 seconds"""
        step_counter[0] += 1

        # Print statistics every 300 steps (5 seconds at 60Hz)
        if step_counter[0] % 300 == 0:
            moving_count = sum(1 for robot in manager.objects if robot.is_moving)

            # Calculate average speed
            speeds = [np.linalg.norm(robot.velocity) for robot in manager.objects if robot.is_moving]
            avg_speed = np.mean(speeds) if speeds else 0.0
            max_speed = np.max(speeds) if speeds else 0.0

            print(
                f"[t={step_counter[0]*dt:.1f}s] "
                f"Moving: {moving_count}/100 | "
                f"Avg speed: {avg_speed:.2f} m/s | "
                f"Max speed: {max_speed:.2f} m/s"
            )

    # Register callback
    sim.register_callback(monitoring_callback, frequency=None)

    # Run simulation
    sim.run_simulation()


if __name__ == "__main__":
    main()
