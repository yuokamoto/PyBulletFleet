#!/usr/bin/env python3
"""
Test agent movement and path following
"""

import os
import sys
import numpy as np

# Add PyBulletFleet to path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

import pybullet as p
import pybullet_data
from pybullet_fleet.agent import Agent
from pybullet_fleet.sim_object import Pose


def test_agent_basic_movement():
    """Test basic agent movement without GUI"""
    print("=== Testing Agent Basic Movement ===")

    # Initialize PyBullet (no GUI)
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    try:
        # Create omnidirectional robot
        print("Creating omnidirectional robot...")
        robot_omni = Agent.from_mesh(
            mesh_path=os.path.join(current_dir, "mesh", "cube.obj"),
            pose=Pose.from_xyz(0, 0, 0.5),
            base_mass=1.0,
            max_linear_vel=1.0,
            max_linear_accel=2.0,
            motion_mode="omnidirectional",
            use_fixed_base=False,
        )
        print(f"✓ Created robot: body_id={robot_omni.body_id}")

        # Test initial state
        initial_pose = robot_omni.get_pose()
        print(f"Initial position: {initial_pose.position}")
        print(f"Initial moving state: {robot_omni.is_moving}")

        # Set a goal
        goal = Pose.from_xyz(2, 0, 0.5)
        robot_omni.set_goal_pose(goal)
        print(f"✓ Goal set to: {goal.position}")
        print(f"Robot moving: {robot_omni.is_moving}")

        # Simulate movement
        dt = 1 / 240.0  # 240 Hz
        max_steps = 1000  # Increased to give more time

        for step in range(max_steps):
            # Update robot
            robot_omni.update(dt)
            p.stepSimulation()

            # Check progress every 50 steps
            if step % 50 == 0:
                current_pose = robot_omni.get_pose()
                distance_to_goal = np.linalg.norm(np.array(current_pose.position) - np.array(goal.position))
                print(
                    f"Step {step:3d}: pos=({current_pose.position[0]:.2f}, "
                    f"{current_pose.position[1]:.2f}, {current_pose.position[2]:.2f}), "
                    f"dist={distance_to_goal:.3f}, moving={robot_omni.is_moving}"
                )

            # Check if reached goal
            if not robot_omni.is_moving:
                print(f"✓ Robot reached goal at step {step}")
                break

        else:
            print(f"⚠ Robot did not reach goal within {max_steps} steps")

        # Final position
        final_pose = robot_omni.get_pose()
        # Use XY-plane distance (same as agent uses for goal detection)
        final_distance = np.linalg.norm(np.array(final_pose.position[:2]) - np.array(goal.position[:2]))
        print(f"Final position: {final_pose.position}")
        print(f"Final XY distance to goal: {final_distance:.3f}")

        if final_distance < 0.02:  # 2cm threshold
            print("✓✓✓ SUCCESS: Robot reached goal!")
            return True
        else:
            print("✗✗✗ FAILED: Robot did not reach goal")
            return False

    except Exception as e:
        print(f"✗ ERROR: {e}")
        import traceback

        traceback.print_exc()
        return False

    finally:
        p.disconnect()


def test_agent_path_following():
    """Test path following functionality"""
    print("\n=== Testing Agent Path Following ===")

    # Initialize PyBullet (no GUI)
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    try:
        # Create differential drive robot
        print("Creating differential drive robot...")
        robot_diff = Agent.from_mesh(
            mesh_path=os.path.join(current_dir, "mesh", "cube.obj"),
            pose=Pose.from_xyz(0, 0, 0.5),
            base_mass=1.0,
            max_linear_vel=1.0,
            max_linear_accel=2.0,
            motion_mode="differential",
            use_fixed_base=False,
        )
        print(f"✓ Created robot: body_id={robot_diff.body_id}")

        # Create simple path (square)
        path = [Pose.from_xyz(1, 0, 0.5), Pose.from_xyz(1, 1, 0.5), Pose.from_xyz(0, 1, 0.5), Pose.from_xyz(0, 0, 0.5)]

        robot_diff.set_path(path)
        print(f"✓ Path set with {len(path)} waypoints")
        print(f"Robot moving: {robot_diff.is_moving}")
        print(f"Current waypoint: {robot_diff.current_waypoint_index}")
        if robot_diff.goal_pose:
            print(f"Current goal: {robot_diff.goal_pose.position}")

        # Simulate path following
        dt = 1 / 240.0
        max_steps = 5000  # Increased for differential drive

        for step in range(max_steps):
            robot_diff.update(dt)
            p.stepSimulation()

            # Check progress every 100 steps
            if step % 100 == 0:
                current_pose = robot_diff.get_pose()
                print(
                    f"Step {step:4d}: pos=({current_pose.position[0]:.2f}, "
                    f"{current_pose.position[1]:.2f}, {current_pose.position[2]:.2f}), "
                    f"waypoint={robot_diff.current_waypoint_index}/{len(path)}, "
                    f"moving={robot_diff.is_moving}"
                )

            # Check if path complete
            if not robot_diff.is_moving and robot_diff.path is None:
                print(f"✓ Path completed at step {step}")
                break

        else:
            print(f"⚠ Path not completed within {max_steps} steps")

        final_pose = robot_diff.get_pose()
        print(f"Final position: {final_pose.position}")
        print(f"Final waypoint: {robot_diff.current_waypoint_index}")

        # Success if completed all waypoints
        if robot_diff.path is None:
            print("✓✓✓ SUCCESS: Path following completed!")
            return True
        else:
            print(f"✗✗✗ FAILED: Path following incomplete (at waypoint {robot_diff.current_waypoint_index}/{len(path)})")
            return False

    except Exception as e:
        print(f"✗ ERROR: {e}")
        import traceback

        traceback.print_exc()
        return False

    finally:
        p.disconnect()


if __name__ == "__main__":
    print("=" * 60)
    print("Agent Movement Test Suite")
    print("=" * 60)

    result1 = test_agent_basic_movement()
    result2 = test_agent_path_following()

    print("\n" + "=" * 60)
    print("Test Results Summary")
    print("=" * 60)
    print(f"Basic Movement Test: {'✓ PASS' if result1 else '✗ FAIL'}")
    print(f"Path Following Test: {'✓ PASS' if result2 else '✗ FAIL'}")
    print("=" * 60)

    sys.exit(0 if (result1 and result2) else 1)
