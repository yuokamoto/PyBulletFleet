#!/usr/bin/env python3
"""
Simplified path following test without MultiRobotSimulationCore
"""

import time
import pybullet as p
import pybullet_data

from pybullet_fleet.agent import Agent
from pybullet_fleet.geometry import Path, Pose


def main():
    # Connect to PyBullet with GUI
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load ground plane
    p.loadURDF("plane.urdf")

    # Create robot using from_mesh directly (like test_robot_movement.py)
    print("Creating robot...")
    robot = Agent.from_mesh(
        mesh_path="mesh/cube.obj",
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

    # Check robot position
    pos, orn = p.getBasePositionAndOrientation(robot.body_id)
    print(f"Initial position: {pos}")

    # Create simple square path
    print("\nCreating square path...")
    path = Path(
        [
            Pose.from_xyz(1.0, 1.0, 0.5),
            Pose.from_xyz(-1.0, 1.0, 0.5),
            Pose.from_xyz(-1.0, -1.0, 0.5),
            Pose.from_xyz(1.0, -1.0, 0.5),
            Pose.from_xyz(1.0, 1.0, 0.5),
        ]
    )

    print(f"Path has {len(path)} waypoints")

    # Visualize path
    for i in range(len(path) - 1):
        p1 = path[i].position
        p2 = path[i + 1].position
        p.addUserDebugLine(p1, p2, [0.0, 1.0, 0.0], lineWidth=3.0, lifeTime=0)

    # Set path
    robot.set_path(path.waypoints)
    print(f"\nPath set. Robot is_moving: {robot.is_moving}")

    # Set camera
    p.resetDebugVisualizerCamera(cameraDistance=5.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

    print("\nSimulation started! Press Ctrl+C to exit.")

    # Simulation loop
    dt = 1 / 240.0
    step_count = 0

    try:
        while True:
            robot.update(dt)
            p.stepSimulation()
            step_count += 1

            # Print status every second
            if step_count % 240 == 0:
                pos = robot.get_pose().position
                print(
                    f"[{step_count*dt:.1f}s] "
                    f"Pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), "
                    f"Waypoint: {robot.current_waypoint_index}/{len(path)}, "
                    f"Moving: {robot.is_moving}"
                )

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nSimulation stopped by user")

    p.disconnect()
    print("Simulation ended")


if __name__ == "__main__":
    main()
