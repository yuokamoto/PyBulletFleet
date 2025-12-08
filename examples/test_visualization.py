#!/usr/bin/env python3
"""
Test Path Visualization

Tests the visualize_waypoints() method for debugging path orientations.
"""

import os
import sys

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pybullet as p

from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Path


def main():
    # Create simulation
    params = SimulationParams(gui=True, timestep=1.0 / 60.0, speed=1.0)
    sim = MultiRobotSimulationCore(params)

    print("Creating paths with tilted orientations...")

    # Create a tilted square path
    v1 = np.array([3.0, 0.0, 2.0])
    v2 = np.array([0.0, 3.0, 0.0])
    normal = np.cross(v1, v2)
    normal = normal / np.linalg.norm(normal)

    pitch_3d = np.arcsin(np.clip(normal[0], -1.0, 1.0))
    roll_3d = np.arctan2(-normal[1], normal[2])

    print(f"Plane normal: {normal}")
    print(f"Plane roll: {np.rad2deg(roll_3d):.2f}°, pitch: {np.rad2deg(pitch_3d):.2f}°")

    square_path = Path.create_square(
        center=[0, 0, 1.5],
        side_length=3.0,
        rpy=[roll_3d, pitch_3d, 0.0]
    )

    # Draw path lines
    for i in range(len(square_path) - 1):
        p1 = square_path[i].position
        p2 = square_path[i + 1].position
        p.addUserDebugLine(p1, p2, [1.0, 1.0, 0.0], lineWidth=3.0, lifeTime=0)

    # Visualize waypoint orientations
    print("\nVisualizing waypoint orientations:")
    print("  Red   = X+ axis (forward direction)")
    print("  Green = Y+ axis (right direction)")
    print("  Blue  = Z+ axis (up direction, perpendicular to plane)")
    
    debug_ids = square_path.visualize_waypoints(
        show_axes=True,
        axis_length=0.5,
        show_points=True,
        point_size=0.08,
        lifetime=0
    )
    
    print(f"\nCreated {len(debug_ids)} debug visualization items")
    print("\nCheck that:")
    print("  1. Blue (Z+) axes point in the same direction at all waypoints")
    print("  2. Red (X+) axes point along the path direction")
    print("  3. All three axes are perpendicular at each waypoint")

    # Set camera
    p.resetDebugVisualizerCamera(
        cameraDistance=8.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 1.5]
    )

    print("\nSimulation running. Press Ctrl+C to exit.")
    
    # Run simulation (no robots, just visualization)
    try:
        while True:
            sim.step_once()
    except KeyboardInterrupt:
        print("\nExiting...")


if __name__ == "__main__":
    main()
