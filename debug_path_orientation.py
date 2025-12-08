#!/usr/bin/env python3
"""Debug script to check path orientations"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "."))

from pybullet_fleet.sim_object import Path

# Create the same 3D square as in the demo
v1 = np.array([3.0, 0.0, 2.0])
v2 = np.array([0.0, 3.0, 0.0])
normal = np.cross(v1, v2)
normal = normal / np.linalg.norm(normal)

pitch_3d = np.arcsin(np.clip(normal[0], -1.0, 1.0))
roll_3d = np.arctan2(-normal[1], normal[2])

print(f"Normal vector: {normal}")
print(f"Roll: {np.rad2deg(roll_3d):.2f}°, Pitch: {np.rad2deg(pitch_3d):.2f}°")

square_path_3d = Path.create_square(
    center=[1.5, 7.5, 1.5], side_length=4.24, rpy=[roll_3d, pitch_3d, 0.0]
)

print(f"\nSquare path waypoints:")
for i, wp in enumerate(square_path_3d.waypoints):
    # Convert quaternion to euler
    quat = wp.orientation
    # Using PyBullet's convention
    import pybullet as p

    euler = p.getEulerFromQuaternion(quat)
    print(f"  Waypoint {i}: pos={wp.position}, euler(r,p,y)={[np.rad2deg(e) for e in euler]}")

    # Calculate direction to next waypoint
    if i < len(square_path_3d.waypoints) - 1:
        next_wp = square_path_3d.waypoints[i + 1]
        direction = np.array(next_wp.position) - np.array(wp.position)
        direction_xy = direction[:2]
        distance_xy = np.linalg.norm(direction_xy)
        distance_3d = np.linalg.norm(direction)

        if distance_xy > 1e-6:
            desired_yaw = np.arctan2(direction_xy[1], direction_xy[0])
        else:
            desired_yaw = 0.0

        if distance_3d > 1e-6:
            desired_pitch = np.arctan2(direction[2], distance_xy)
        else:
            desired_pitch = 0.0

        print(f"    -> Direction to next: {direction}")
        print(
            f"    -> Calculated yaw: {np.rad2deg(desired_yaw):.2f}°, pitch: {np.rad2deg(desired_pitch):.2f}°"
        )
