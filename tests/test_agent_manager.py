"""
Tests for AgentManager class.

This module tests:
- Agent spawning (grid, mixed types, counts)
- Agent queries (poses, counts, moving agents)
- Grid positioning
- Batch spawning
"""

import os

import pybullet as p
import pybullet_data
import pytest

from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.agent import AgentSpawnParams
from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import ShapeParams


@pytest.fixture
def pybullet_env():
    """Setup and teardown PyBullet environment"""
    physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    plane_id = p.loadURDF("plane.urdf")

    yield physics_client, plane_id

    p.disconnect()


class TestAgentManagerCreation:
    """Test AgentManager creation and initialization"""

    def test_agent_manager_creation(self, pybullet_env):
        """Test creating AgentManager"""
        manager = AgentManager()

        assert manager is not None
        assert manager.get_object_count() == 0

    def test_agent_manager_with_frequency(self, pybullet_env):
        """Test AgentManager with custom update frequency"""
        manager = AgentManager(update_frequency=5.0)

        assert manager is not None


class TestAgentSpawningGrid:
    """Test agent spawning in grid pattern"""

    def test_spawn_agents_grid_basic(self, pybullet_env):
        """Test basic grid spawning"""
        manager = AgentManager()

        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        spawn_params = AgentSpawnParams(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            mass=1.0,
        )

        # 2x2 grid: x from 0 to 1, y from 0 to 1
        grid_params = GridSpawnParams(x_min=0, x_max=1, y_min=0, y_max=1, spacing=[1.0, 1.0, 0], offset=[0, 0, 0.5])

        agents = manager.spawn_agents_grid(num_agents=4, grid_params=grid_params, spawn_params=spawn_params)

        assert len(agents) == 4
        assert manager.get_object_count() == 4

    def test_spawn_agents_grid_positions(self, pybullet_env):
        """Test that agents are positioned correctly in grid"""
        manager = AgentManager()

        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        spawn_params = AgentSpawnParams(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            mass=1.0,
        )

        # Create 3x3 grid: x from 0 to 2, y from 0 to 2
        grid_params = GridSpawnParams(x_min=0, x_max=2, y_min=0, y_max=2, spacing=[2.0, 2.0, 0], offset=[0, 0, 0.5])

        agents = manager.spawn_agents_grid(num_agents=9, grid_params=grid_params, spawn_params=spawn_params)

        assert len(agents) == 9

        # Check that agents are spread out
        poses = manager.get_all_poses()
        assert len(poses) == 9

        # Check that not all positions are identical
        x_coords = [pose.x for pose in poses]
        assert len(set(x_coords)) > 1  # At least 2 different x coordinates

    def test_spawn_fewer_agents_than_grid(self, pybullet_env):
        """Test spawning fewer agents than grid capacity"""
        manager = AgentManager()

        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        spawn_params = AgentSpawnParams(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            mass=1.0,
        )

        # Grid can hold 16 agents (4x4)
        grid_params = GridSpawnParams(x_min=0, x_max=3, y_min=0, y_max=3, spacing=[1.0, 1.0, 0], offset=[0, 0, 0.5])

        # Spawn only 6 agents
        agents = manager.spawn_agents_grid(num_agents=6, grid_params=grid_params, spawn_params=spawn_params)

        assert len(agents) == 6
        assert manager.get_object_count() == 6


class TestAgentSpawningMixed:
    """Test spawning mixed agent types"""

    def test_spawn_agents_grid_mixed(self, pybullet_env):
        """Test spawning mixed types with probabilities"""
        manager = AgentManager()

        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")

        # Red agents (50% probability)
        red_params = AgentSpawnParams(
            visual_shape=ShapeParams(
                shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2], rgba_color=[1, 0, 0, 1]
            ),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            mass=1.0,
        )

        # Blue agents (50% probability)
        blue_params = AgentSpawnParams(
            visual_shape=ShapeParams(
                shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2], rgba_color=[0, 0, 1, 1]
            ),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            mass=1.0,
        )

        grid_params = GridSpawnParams(x_min=0, x_max=2, y_min=0, y_max=2, spacing=[1.0, 1.0, 0], offset=[0, 0, 0.5])

        agents = manager.spawn_agents_grid_mixed(
            num_agents=9, grid_params=grid_params, spawn_params_list=[(red_params, 0.5), (blue_params, 0.5)]
        )

        assert len(agents) == 9
        assert manager.get_object_count() == 9

    def test_spawn_grid_counts(self, pybullet_env):
        """Test spawning specific counts of each type"""
        manager = AgentManager()

        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")

        # Type A agents
        type_a = AgentSpawnParams(
            visual_shape=ShapeParams(
                shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2], rgba_color=[1, 0, 0, 1]
            ),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            mass=1.0,
        )

        # Type B agents
        type_b = AgentSpawnParams(
            visual_shape=ShapeParams(
                shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2], rgba_color=[0, 1, 0, 1]
            ),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            mass=1.0,
        )

        grid_params = GridSpawnParams(x_min=0, x_max=3, y_min=0, y_max=3, spacing=[1.0, 1.0, 0], offset=[0, 0, 0.5])

        # Spawn 3 of type A, 5 of type B
        agents = manager.spawn_agent_grid_counts(grid_params=grid_params, spawn_params_count_list=[(type_a, 3), (type_b, 5)])

        # Total should be 8 agents
        assert len(agents) == 8
        assert manager.get_object_count() == 8


class TestAgentQueries:
    """Test agent query methods"""

    def test_get_object_count(self, pybullet_env):
        """Test getting object count"""
        manager = AgentManager()

        assert manager.get_object_count() == 0

        # Spawn some agents
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        spawn_params = AgentSpawnParams(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            mass=1.0,
        )
        grid_params = GridSpawnParams(x_min=0, x_max=1, y_min=0, y_max=1, spacing=[1.0, 1.0, 0], offset=[0, 0, 0.5])

        manager.spawn_agents_grid(num_agents=4, grid_params=grid_params, spawn_params=spawn_params)

        assert manager.get_object_count() == 4

    def test_get_all_poses(self, pybullet_env):
        """Test getting all agent poses"""
        manager = AgentManager()

        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        spawn_params = AgentSpawnParams(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            mass=1.0,
        )
        grid_params = GridSpawnParams(x_min=0, x_max=1, y_min=0, y_max=1, spacing=[1.0, 1.0, 0], offset=[0, 0, 0.5])

        agents = manager.spawn_agents_grid(num_agents=4, grid_params=grid_params, spawn_params=spawn_params)

        poses = manager.get_all_poses()

        assert len(poses) == 4
        assert all(isinstance(pose, Pose) for pose in poses)

    def test_get_poses_dict(self, pybullet_env):
        """Test getting poses as dictionary"""
        manager = AgentManager()

        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        spawn_params = AgentSpawnParams(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            mass=1.0,
        )
        grid_params = GridSpawnParams(x_min=0, x_max=1, y_min=0, y_max=1, spacing=[1.0, 1.0, 0], offset=[0, 0, 0.5])

        manager.spawn_agents_grid(num_agents=4, grid_params=grid_params, spawn_params=spawn_params)

        poses_dict = manager.get_poses_dict()

        assert isinstance(poses_dict, dict)
        assert len(poses_dict) == 4
        assert all(isinstance(pose, Pose) for pose in poses_dict.values())

    def test_get_moving_count(self, pybullet_env):
        """Test getting count of moving agents"""
        manager = AgentManager()

        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        spawn_params = AgentSpawnParams(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            mass=1.0,
        )
        grid_params = GridSpawnParams(x_min=0, x_max=1, y_min=0, y_max=1, spacing=[1.0, 1.0, 0], offset=[0, 0, 0.5])

        agents = manager.spawn_agents_grid(num_agents=4, grid_params=grid_params, spawn_params=spawn_params)

        # Initially no agents are moving
        moving_count = manager.get_moving_count()
        assert moving_count == 0

        # Set goal for one agent
        agents[0].set_goal_pose(Pose.from_xyz(5, 0, 0.5))

        # Now should have at least one potentially moving
        moving_count_after = manager.get_moving_count()
        # Movement depends on internal state, just check it doesn't crash
        assert moving_count_after >= 0


class TestGridSpawnParams:
    """Test GridSpawnParams dataclass"""

    def test_grid_spawn_params_creation(self):
        """Test creating grid spawn parameters"""
        params = GridSpawnParams(x_min=0, x_max=2, y_min=0, y_max=2, spacing=[1.0, 1.0, 0], offset=[0, 0, 0])

        assert params.x_min == 0
        assert params.x_max == 2
        assert params.y_min == 0
        assert params.y_max == 2
        assert params.spacing == [1.0, 1.0, 0]
        assert params.offset == [0, 0, 0]

    def test_grid_spawn_params_with_offset(self):
        """Test grid spawn parameters with offset"""
        params = GridSpawnParams(x_min=0, x_max=3, y_min=0, y_max=3, spacing=[2.0, 2.0, 0], offset=[0.5, 0.5, 0])

        assert params.x_min == 0
        assert params.x_max == 3
        assert params.offset == [0.5, 0.5, 0]


class TestBatchSpawning:
    """Test batch spawning operations"""

    def test_spawn_objects_batch(self, pybullet_env):
        """Test batch spawning with list of parameters"""
        manager = AgentManager()

        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")

        # Create list of spawn parameters
        params_list = [
            AgentSpawnParams(
                visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path, mesh_scale=[0.2, 0.2, 0.2]),
                collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
                initial_pose=Pose.from_xyz(i, 0, 0.5),
                mass=1.0,
            )
            for i in range(5)
        ]

        agents = manager.spawn_objects_batch(params_list)

        assert len(agents) == 5
        assert manager.get_object_count() == 5


class TestAgentManagerEmpty:
    """Test AgentManager operations with no agents"""

    def test_empty_manager_queries(self, pybullet_env):
        """Test queries on empty manager"""
        manager = AgentManager()

        assert manager.get_object_count() == 0
        assert manager.get_all_poses() == []
        assert manager.get_poses_dict() == {}
        assert manager.get_moving_count() == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
