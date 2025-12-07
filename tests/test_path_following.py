#!/usr/bin/env python3
"""
Path following demo with GUI visualization
"""

import os
import time

import pybullet as p
import pybullet_data

from pybullet_fleet.agent import Agent
from pybullet_fleet.sim_object import Path, Pose


def main():
    # Connect to PyBullet with GUI (direct connection, not via MultiRobotSimulationCore)
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)

    # Load ground plane
    plane_id = p.loadURDF("plane.urdf")

    # Get absolute path to mesh
    current_dir = os.path.dirname(os.path.abspath(__file__))
    mesh_path = os.path.join(current_dir, "mesh", "cube.obj")

    # Create robot using from_mesh directly
    print("Spawning robot...")
    robot = Agent.from_mesh(
        mesh_path=mesh_path,
        pose=Pose.from_xyz(0.0, 0.0, 0.5),
        mesh_scale=[0.5, 0.5, 0.5],
        collision_half_extents=[0.25, 0.25, 0.25],
        rgba_color=[1.0, 0.0, 0.0, 1.0],  # Red
        base_mass=1.0,
        max_linear_vel=1.0,
        max_linear_accel=2.0,
        motion_mode="omnidirectional",
    )

    print(f"Robot created with body_id: {robot.body_id}")

    # Create simple square path around origin
    print("\nCreating square path...")
    path = Path(
        [
            Pose.from_xyz(1.0, 1.0, 0.5),
            Pose.from_xyz(-1.0, 1.0, 0.5),
            Pose.from_xyz(-1.0, -1.0, 0.5),
            Pose.from_xyz(1.0, -1.0, 0.5),
            Pose.from_xyz(1.0, 1.0, 0.5),  # Back to start
        ]
    )

    print(f"Path has {len(path)} waypoints:")
    for i, waypoint in enumerate(path):
        print(f"  {i}: {waypoint.position}")

    # Visualize path
    for i in range(len(path) - 1):
        p1 = path[i].position
        p2 = path[i + 1].position
        p.addUserDebugLine(p1, p2, [0.0, 1.0, 0.0], lineWidth=3.0, lifeTime=0)

    # Add markers at waypoints
    for i, waypoint in enumerate(path):
        p.addUserDebugText(
            f"{i}",
            [waypoint.position[0], waypoint.position[1], waypoint.position[2] + 0.3],
            textColorRGB=[1.0, 1.0, 0.0],
            textSize=2.0,
            lifeTime=0,
        )

    # Set path
    robot.set_path(path.waypoints)
    print(f"\nPath set. Robot state:")
    print(f"  is_moving: {robot.is_moving}")
    print(f"  current_waypoint_index: {robot.current_waypoint_index}")
    print(f"  goal_pose: {robot.goal_pose.position if robot.goal_pose else None}")

    # Set camera
    p.resetDebugVisualizerCamera(cameraDistance=5.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

    print("\nSimulation started! Press Ctrl+C to exit.")

    # Simulation loop
    dt = 1 / 240.0
    step_count = 0
    last_print_time = time.time()

    try:
        while True:
            # Update robot
            robot.update(dt)

            # Step simulation
            p.stepSimulation()
            step_count += 1

            # Print status every second
            current_time = time.time()
            if current_time - last_print_time > 1.0:
                pos = robot.get_pose().position
                print(
                    f"[{step_count*dt:.1f}s] "
                    f"Pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), "
                    f"Waypoint: {robot.current_waypoint_index}/{len(path)}, "
                    f"Moving: {robot.is_moving}"
                )
                last_print_time = current_time

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nSimulation stopped by user")

    p.disconnect()
    print("Simulation ended")


if __name__ == "__main__":
    main()
