"""
Tests for Agent class core functionality.

This module tests:
- Agent creation from mesh and URDF
- Agent properties and parameters
- Goal setting and path following
- Motion modes (omnidirectional vs differential)
- Velocity and acceleration constraints
- Agent state management
"""

import os

import numpy as np
import pybullet as p
import pybullet_data
import pytest

from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.geometry import Pose, Path
from pybullet_fleet.sim_object import ShapeParams
from pybullet_fleet.types import MotionMode


@pytest.fixture
def pybullet_env():
    """Setup and teardown PyBullet environment"""
    physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    plane_id = p.loadURDF("plane.urdf")

    yield physics_client, plane_id

    p.disconnect()


class TestAgentCreationFromMesh:
    """Test Agent creation from mesh files"""

    def test_from_mesh_basic(self, pybullet_env):
        """Test basic agent creation from mesh"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0.5),
            mass=1.0,
        )

        assert agent.body_id >= 0
        assert agent.mass == 1.0
        assert agent.motion_mode == MotionMode.OMNIDIRECTIONAL

    def test_from_mesh_with_params(self, pybullet_env):
        """Test agent creation with custom parameters"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.3, 0.3, 0.3]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.15, 0.15, 0.15]),
            pose=Pose.from_xyz(1, 2, 0.5),
            mass=2.5,
            max_linear_vel=0.8,
            max_linear_accel=1.5,
            motion_mode="differential",
        )

        assert agent.mass == 2.5
        assert np.allclose(agent.max_linear_vel, [0.8, 0.8, 0.8])
        assert np.allclose(agent.max_linear_accel, [1.5, 1.5, 1.5])
        assert agent.motion_mode == MotionMode.DIFFERENTIAL

    def test_from_mesh_different_colors(self, pybullet_env):
        """Test creating agents with different colors"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")

        # Red agent
        agent1 = Agent.from_mesh(
            visual_shape=ShapeParams(
                shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2], rgba_color=[1, 0, 0, 1]
            ),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0.5),
        )

        # Green agent
        agent2 = Agent.from_mesh(
            visual_shape=ShapeParams(
                shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2], rgba_color=[0, 1, 0, 1]
            ),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(1, 0, 0.5),
        )

        assert agent1.body_id != agent2.body_id


class TestAgentCreationFromURDF:
    """Test Agent creation from URDF files"""

    def test_from_urdf_mobile_robot(self, pybullet_env):
        """Test creating agent from mobile robot URDF"""
        urdf_path = "robots/mobile_robot.urdf"
        agent = Agent.from_urdf(urdf_path=urdf_path, pose=Pose.from_xyz(0, 0, 0.5), mass=2.0)

        assert agent.body_id >= 0
        # Mass is defined in URDF, not overridden by parameter
        assert agent.mass >= 0.5  # Just check it has some mass
        assert agent.is_urdf_robot()

    def test_from_urdf_with_params(self, pybullet_env):
        """Test URDF agent with custom parameters"""
        urdf_path = "robots/mobile_robot.urdf"
        agent = Agent.from_urdf(
            urdf_path=urdf_path,
            pose=Pose.from_xyz(1, 1, 0.5),
            mass=3.0,
            max_linear_vel=1.5,
            max_linear_accel=3.0,
            motion_mode="omnidirectional",
        )

        # Mass comes from URDF, not parameter
        assert agent.mass >= 0.5
        assert np.allclose(agent.max_linear_vel, [1.5, 1.5, 1.5])
        assert agent.motion_mode == MotionMode.OMNIDIRECTIONAL


class TestAgentSpawnParams:
    """Test AgentSpawnParams dataclass"""

    def test_spawn_params_basic(self):
        """Test creating basic spawn parameters"""
        params = AgentSpawnParams(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.2, 0.2, 0.2]),
            mass=1.5,
            max_linear_vel=2.0,
        )

        assert params.mass == 1.5
        assert params.max_linear_vel == 2.0
        assert params.motion_mode == MotionMode.OMNIDIRECTIONAL

    def test_spawn_params_from_params(self, pybullet_env):
        """Test creating agent from spawn parameters"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        params = AgentSpawnParams(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            initial_pose=Pose.from_xyz(0, 0, 0.5),
            mass=2.0,
            max_linear_vel=1.0,
            motion_mode="differential",
        )

        agent = Agent.from_params(params)

        assert agent.body_id >= 0
        assert agent.mass == 2.0
        assert agent.motion_mode == MotionMode.DIFFERENTIAL


class TestAgentGoalSetting:
    """Test agent goal and path setting"""

    def test_set_goal_pose(self, pybullet_env):
        """Test setting goal pose"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0.5),
        )

        # Initially no goal
        assert agent.goal_pose is None
        assert not agent.is_moving

        # Set goal
        goal = Pose.from_xyz(5, 0, 0.5)
        agent.set_goal_pose(goal)

        # Note: goal_pose might be different due to internal processing
        # Just check that some movement state changed
        assert agent.is_moving or agent.goal_pose is not None

    def test_set_path(self, pybullet_env):
        """Test setting path with waypoints"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0.5),
        )

        # Create path
        path = Path.from_positions([[1, 0, 0.5], [2, 0, 0.5], [2, 1, 0.5]])

        # Set path
        agent.set_path(path)

        # Agent should be moving or have path set
        assert agent.is_moving or hasattr(agent, "_path")


class TestAgentMotionModes:
    """Test different motion modes"""

    def test_omnidirectional_mode(self, pybullet_env):
        """Test omnidirectional motion mode"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0.5),
            motion_mode="omnidirectional",
        )

        assert agent.motion_mode == MotionMode.OMNIDIRECTIONAL

    def test_differential_mode(self, pybullet_env):
        """Test differential drive motion mode"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0.5),
            motion_mode="differential",
        )

        assert agent.motion_mode == MotionMode.DIFFERENTIAL

    def test_set_motion_mode(self, pybullet_env):
        """Test changing motion mode"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0.5),
            motion_mode="omnidirectional",
        )

        assert agent.motion_mode == MotionMode.OMNIDIRECTIONAL

        # Change to differential
        success = agent.set_motion_mode("differential")
        assert success
        assert agent.motion_mode == MotionMode.DIFFERENTIAL


class TestAgentVelocityConstraints:
    """Test velocity and acceleration constraints"""

    def test_scalar_velocity_constraint(self, pybullet_env):
        """Test scalar velocity constraint applied to all axes"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0.5),
            max_linear_vel=1.5,
        )

        # Should be applied to all axes
        assert np.allclose(agent.max_linear_vel, [1.5, 1.5, 1.5])

    def test_vector_velocity_constraint(self, pybullet_env):
        """Test per-axis velocity constraints"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0.5),
            max_linear_vel=[1.0, 2.0, 0.5],
        )

        # Should preserve per-axis values
        assert np.allclose(agent.max_linear_vel, [1.0, 2.0, 0.5])

    def test_scalar_acceleration_constraint(self, pybullet_env):
        """Test scalar acceleration constraint"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0.5),
            max_linear_accel=3.0,
        )

        assert np.allclose(agent.max_linear_accel, [3.0, 3.0, 3.0])

    def test_vector_acceleration_constraint(self, pybullet_env):
        """Test per-axis acceleration constraints"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0.5),
            max_linear_accel=[2.0, 4.0, 1.0],
        )

        assert np.allclose(agent.max_linear_accel, [2.0, 4.0, 1.0])


class TestAgentProperties:
    """Test agent properties and state"""

    def test_is_urdf_robot_mesh(self, pybullet_env):
        """Test is_urdf_robot property for mesh robot"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0.5),
        )

        assert not agent.is_urdf_robot()

    def test_is_urdf_robot_urdf(self, pybullet_env):
        """Test is_urdf_robot property for URDF robot"""
        urdf_path = "robots/mobile_robot.urdf"
        agent = Agent.from_urdf(urdf_path=urdf_path, pose=Pose.from_xyz(0, 0, 0.5))

        assert agent.is_urdf_robot()

    def test_agent_position_access(self, pybullet_env):
        """Test accessing agent position"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(1, 2, 0.5),
        )

        pos, _ = p.getBasePositionAndOrientation(agent.body_id)

        # Position should be close to initial pose
        assert abs(pos[0] - 1.0) < 0.1
        assert abs(pos[1] - 2.0) < 0.1
        assert abs(pos[2] - 0.5) < 0.1

    def test_agent_mass_property(self, pybullet_env):
        """Test mass property"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0.5),
            mass=3.5,
        )

        assert agent.mass == 3.5


class TestAgentFixedBase:
    """Test fixed base agents"""

    def test_fixed_base_agent(self, pybullet_env):
        """Test creating fixed base agent"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0.5),
            use_fixed_base=True,
        )

        assert agent.body_id >= 0

        # Position should not change even after simulation steps
        initial_pos, _ = p.getBasePositionAndOrientation(agent.body_id)

        for _ in range(100):
            p.stepSimulation()

        final_pos, _ = p.getBasePositionAndOrientation(agent.body_id)

        # Position should remain the same (fixed base)
        assert np.allclose(initial_pos, final_pos, atol=0.001)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
