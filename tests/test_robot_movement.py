#!/usr/bin/env python3
"""
Tests for agent movement and goal-seeking behavior.

This test verifies:
- Agent creation from mesh
- Goal pose setting
- Agent basic properties

Note: Full movement tests require CoreSimulation and are covered in integration tests.
"""

import os

import numpy as np
import pybullet as p
import pybullet_data
import pytest

from pybullet_fleet.agent import Agent
from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import ShapeParams


@pytest.fixture
def pybullet_env():
    """Setup and teardown PyBullet environment"""
    # Connect to PyBullet in DIRECT mode (no GUI)
    physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    # Load ground plane
    plane_id = p.loadURDF("plane.urdf")

    yield physics_client, plane_id

    # Cleanup
    p.disconnect()


class TestAgentCreation:
    """Test agent creation and basic properties"""

    def test_agent_creation_from_mesh(self, pybullet_env):
        """Test creating agent from mesh file"""
        physics_client, _ = pybullet_env

        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        robot = Agent.from_mesh(
            visual_shape=ShapeParams(
                shape_type="mesh",
                mesh_path=mesh_path,
                mesh_scale=[0.3, 0.3, 0.3],
                rgba_color=[1.0, 0.0, 0.0, 1.0],
            ),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.15, 0.15, 0.15]),
            pose=Pose.from_xyz(0.0, 0.0, 0.5),
            mass=1.0,
            max_linear_vel=1.0,
            max_linear_accel=2.0,
            motion_mode="omnidirectional",
        )

        # Verify robot was created
        assert robot.body_id >= 0
        assert robot.mass == 1.0
        # max_linear_vel is a numpy array
        assert np.allclose(robot.max_linear_vel, [1.0, 1.0, 1.0])

        # Check initial position
        pos, orn = p.getBasePositionAndOrientation(robot.body_id)
        assert abs(pos[0] - 0.0) < 0.1
        assert abs(pos[1] - 0.0) < 0.1
        assert abs(pos[2] - 0.5) < 0.1

    def test_agent_properties(self, pybullet_env):
        """Test agent properties after creation"""
        physics_client, _ = pybullet_env

        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        robot = Agent.from_mesh(
            visual_shape=ShapeParams(
                shape_type="mesh",
                mesh_path=mesh_path,
                mesh_scale=[0.3, 0.3, 0.3],
                rgba_color=[0.0, 1.0, 0.0, 1.0],
            ),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.15, 0.15, 0.15]),
            pose=Pose.from_xyz(1.0, 2.0, 0.5),
            mass=2.5,
            max_linear_vel=0.5,
            max_linear_accel=1.0,
            motion_mode="omnidirectional",
        )

        # Check properties
        assert robot.mass == 2.5
        assert robot.motion_mode == "omnidirectional"
        assert np.allclose(robot.max_linear_vel, [0.5, 0.5, 0.5])
        assert np.allclose(robot.max_linear_accel, [1.0, 1.0, 1.0])

    def test_agent_different_motion_modes(self, pybullet_env):
        """Test creating agents with different motion modes"""
        physics_client, _ = pybullet_env

        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")

        # Omnidirectional
        robot1 = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0.0, 0.0, 0.5),
            mass=1.0,
            motion_mode="omnidirectional",
        )
        assert robot1.motion_mode.value == "omnidirectional"

        # Differential drive
        robot2 = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(1.0, 0.0, 0.5),
            mass=1.0,
            motion_mode="differential",
        )
        assert robot2.motion_mode.value == "differential"


def manual_test():
    """Manual test with GUI for visual verification (not run by pytest)"""
    import time

    # Connect to PyBullet with GUI
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    # Load ground plane
    plane_id = p.loadURDF("plane.urdf")

    # Create robot
    print("Creating robot...")
    mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
    robot = Agent.from_mesh(
        visual_shape=ShapeParams(
            shape_type="mesh",
            mesh_path=mesh_path,
            mesh_scale=[0.3, 0.3, 0.3],
            rgba_color=[1.0, 0.0, 0.0, 1.0],
        ),
        collision_shape=ShapeParams(shape_type="box", half_extents=[0.15, 0.15, 0.15]),
        pose=Pose.from_xyz(0.0, 0.0, 0.5),
        mass=1.0,
        max_linear_vel=1.0,
        max_linear_accel=2.0,
        motion_mode="omnidirectional",
    )

    print(f"Robot created with body_id: {robot.body_id}")

    # Set camera
    p.resetDebugVisualizerCamera(cameraDistance=3.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

    print("\nNote: For full movement tests, use CoreSimulation")
    print("See examples/robot_demo.py for complete movement demonstration")

    # Simulation loop
    dt = 1.0 / 240.0
    for i in range(2000):
        p.stepSimulation()

        if i % 240 == 0:
            pos, _ = p.getBasePositionAndOrientation(robot.body_id)
            print(f"[{i*dt:.1f}s] Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")

        time.sleep(dt)

    print("\nTest complete!")
    p.disconnect()


if __name__ == "__main__":
    # Run manual test with GUI
    manual_test()
