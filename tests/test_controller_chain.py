"""Tests for Agent controller chain."""

import pytest
from pybullet_fleet import Agent, AgentSpawnParams, Pose
from pybullet_fleet.controller import Controller
from pybullet_fleet.types import MotionMode


class MockController(Controller):
    def __init__(self):
        self.calls = []

    def compute(self, agent, dt):
        self.calls.append(("compute", dt))
        return False


@pytest.fixture
def sim_core(pybullet_env):
    from tests.conftest import MockSimCore

    sc = MockSimCore()
    sc._client = pybullet_env
    return sc


class TestControllerChain:
    def test_set_controller_backward_compat(self, sim_core):
        """set_controller() replaces base controller (index 0)."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
        )
        agent = Agent.from_params(params, sim_core)
        new_ctrl = MockController()
        agent.set_controller(new_ctrl)
        assert agent.controller is new_ctrl
        assert len(agent._controllers) == 1

    def test_add_controller_appends(self, sim_core):
        """add_controller() appends to chain."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
        )
        agent = Agent.from_params(params, sim_core)
        high_ctrl = MockController()
        agent.add_controller(high_ctrl)
        assert len(agent._controllers) == 2
        assert agent._controllers[-1] is high_ctrl

    def test_chain_execution_order(self, sim_core):
        """High-level controller executes before low-level (reversed order)."""
        call_order = []

        class LowCtrl(Controller):
            def compute(self, agent, dt):
                call_order.append("low")
                return False

        class HighCtrl(Controller):
            def compute(self, agent, dt):
                call_order.append("high")
                return False

        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
        )
        agent = Agent.from_params(params, sim_core)
        agent.set_controller(LowCtrl())
        agent.add_controller(HighCtrl())

        agent.update(0.01)
        assert call_order == ["high", "low"]

    def test_remove_controller(self, sim_core):
        """remove_controller() removes non-base controller."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
        )
        agent = Agent.from_params(params, sim_core)
        high_ctrl = MockController()
        agent.add_controller(high_ctrl)
        agent.remove_controller(high_ctrl)
        assert len(agent._controllers) == 1

    def test_controller_property_returns_base(self, sim_core):
        """controller property returns controllers[0] (base)."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
        )
        agent = Agent.from_params(params, sim_core)
        base = agent.controller
        assert base is agent._controllers[0]
