#!/usr/bin/env python3
"""
Test path following with MultiRobotSimulationCore
"""

import os

import pybullet as p

from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Path, Pose


def main():
    # Create simulation with MultiRobotSimulationCore
    print("Creating MultiRobotSimulationCore...")
    params = SimulationParams(gui=True, timestep=1.0 / 240.0)
    sim = MultiRobotSimulationCore(params)

    print(f"Number of bodies after setup: {p.getNumBodies()}")

    # Get absolute path to mesh
    current_dir = os.path.dirname(os.path.abspath(__file__))
    mesh_path = os.path.join(current_dir, "mesh", "cube.obj")
    print(f"Mesh path: {mesh_path}")
    print(f"Mesh exists: {os.path.exists(mesh_path)}")

    # Create robot using from_params with sim_core
    print("\nSpawning robot with Agent.from_params()...")
    robot_params = AgentSpawnParams(
        mesh_path=mesh_path,
        initial_pose=Pose.from_xyz(0.0, 0.0, 0.5),
        mesh_scale=[0.5, 0.5, 0.5],
        collision_half_extents=[0.25, 0.25, 0.25],
        rgba_color=[1.0, 0.0, 0.0, 1.0],  # Red
        mass=1.0,
        max_linear_vel=1.0,
        max_linear_accel=2.0,
        motion_mode="omnidirectional",
    )
    robot = Agent.from_params(robot_params, sim_core=sim)

    print(f"✓ Robot created with body_id: {robot.body_id}")
    print(f"✓ Number of bodies after robot creation: {p.getNumBodies()}")
    print(f"✓ sim_core.sim_objects count: {len(sim.sim_objects)}")

    # Check robot position
    pos, orn = p.getBasePositionAndOrientation(robot.body_id)
    print(f"✓ Robot position: {pos}")
    print(f"✓ Robot orientation: {orn}")

    # Try to get visual shape data
    visual_data = p.getVisualShapeData(robot.body_id)
    print(f"✓ Visual shape data: {len(visual_data)} shapes")
    if visual_data:
        for i, vd in enumerate(visual_data):
            print(f"  Shape {i}: {vd}")

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

    print("\nSimulation started! Robot should be visible as a red cube.")
    print("Press Ctrl+C to exit.")

    # Movement callback (similar to 100robots_grid_demo.py)
    def robot_movement_callback(sim_objects, sim_core, dt):
        """Update robot movement each step"""
        for obj in sim_objects:
            if isinstance(obj, Agent):
                obj.update(dt)

    # Register callback and run simulation (like 100robots_grid_demo.py)
    sim.register_callback(robot_movement_callback, frequency=None)  # Call every step
    sim.run_simulation()


if __name__ == "__main__":
    main()
