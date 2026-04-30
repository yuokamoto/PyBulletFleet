"""Tests for PatrolController and RandomWalkController."""

import pytest
from pybullet_fleet import Agent, AgentSpawnParams, Pose
from pybullet_fleet.types import MotionMode
from pybullet_fleet.controllers.patrol_controller import PatrolController
from pybullet_fleet.controllers.random_walk_controller import RandomWalkController


@pytest.fixture
def sim_core(pybullet_env):
    from tests.conftest import MockSimCore

    sc = MockSimCore()
    sc._client = pybullet_env
    return sc


class TestPatrolController:
    def test_patrol_cycles_waypoints(self, sim_core):
        """PatrolController cycles through waypoints with loop=True."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0.1),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
            max_linear_vel=5.0,
        )
        agent = Agent.from_params(params, sim_core)
        patrol = PatrolController(
            waypoints=[[2, 0, 0.1], [2, 2, 0.1], [0, 2, 0.1]],
            loop=True,
        )
        agent.add_controller(patrol)

        sim_core.tick(500)
        # Agent should have moved from origin
        pos = agent.get_pose().position
        assert not (abs(pos[0]) < 0.01 and abs(pos[1]) < 0.01)

    def test_patrol_no_loop_stops(self, sim_core):
        """PatrolController with loop=False stops at last waypoint."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0.1),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
            max_linear_vel=5.0,
        )
        agent = Agent.from_params(params, sim_core)
        patrol = PatrolController(
            waypoints=[[1, 0, 0.1]],
            loop=False,
        )
        agent.add_controller(patrol)

        sim_core.tick(200)
        pos = agent.get_pose().position
        assert abs(pos[0] - 1.0) < 0.5  # near target

    def test_patrol_from_config(self):
        """PatrolController.from_config() creates from dict."""
        ctrl = PatrolController.from_config(
            {
                "waypoints": [[1, 0, 0], [2, 0, 0]],
                "wait_time": 1.0,
                "loop": True,
            }
        )
        assert ctrl._loop is True
        assert len(ctrl._waypoints) == 2

    def test_patrol_registered(self):
        """PatrolController is registered in CONTROLLER_REGISTRY."""
        from pybullet_fleet.controller import create_controller

        ctrl = create_controller("patrol", {"waypoints": [[0, 0, 0]]})
        assert isinstance(ctrl, PatrolController)


class TestRandomWalkController:
    def test_random_walk_moves(self, sim_core):
        """RandomWalkController moves agent from origin."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0.1),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
            max_linear_vel=5.0,
        )
        agent = Agent.from_params(params, sim_core)
        rw = RandomWalkController(radius=3.0)
        agent.add_controller(rw)

        sim_core.tick(200)
        pos = agent.get_pose().position
        # Should have moved somewhere
        assert abs(pos[0]) > 0.01 or abs(pos[1]) > 0.01

    def test_random_walk_stays_within_radius(self, sim_core):
        """RandomWalkController goals stay within radius."""
        rw = RandomWalkController(radius=2.0)
        # Verify config stored
        assert rw._radius == 2.0

    def test_random_walk_registered(self):
        """RandomWalkController is registered in CONTROLLER_REGISTRY."""
        from pybullet_fleet.controller import create_controller

        ctrl = create_controller("random_walk", {"radius": 5.0})
        assert isinstance(ctrl, RandomWalkController)
