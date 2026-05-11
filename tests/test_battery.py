"""Tests for Agent battery simulation (BatteryPlugin).

Tests cover:
- BatteryPlugin defaults and custom values
- Battery drain while moving
- Battery charge via set_charging()
- Idle drain
- No battery when plugin is absent
- BatteryPlugin from AgentSpawnParams.from_dict()
- Battery SOC clamping [0.0, 1.0]
"""

import os

import pybullet as p
import pybullet_data
import pytest

from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.plugins.battery_plugin import BatteryPlugin
from pybullet_fleet.geometry import Pose
from pybullet_fleet.types import MotionMode
from tests.conftest import MockSimCore

MOBILE_URDF = "robots/mobile_robot.urdf"


@pytest.fixture
def pybullet_env():
    """Headless PyBullet DIRECT session."""
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.loadURDF("plane.urdf")
    yield client
    p.disconnect()


@pytest.fixture
def sim_core(pybullet_env):
    """MockSimCore with physics disabled."""
    sc = MockSimCore(physics=False)
    sc._client = pybullet_env
    return sc


def _make_agent(sim_core, **battery_kwargs):
    """Create a mobile agent with optional battery plugin kwargs."""
    params = AgentSpawnParams(
        urdf_path=MOBILE_URDF,
        initial_pose=Pose.from_xyz(0, 0, 0.1),
        motion_mode=MotionMode.OMNIDIRECTIONAL,
        max_linear_vel=2.0,
    )
    agent = Agent.from_params(params, sim_core)
    if battery_kwargs:
        agent.add_plugin(BatteryPlugin(agent, **battery_kwargs))
    return agent


# -------------------------------------------------------------------
# BatteryPlugin defaults
# -------------------------------------------------------------------


class TestBatteryPluginDefaults:
    """Tests for BatteryPlugin default attribute values."""

    def test_defaults(self, sim_core):
        agent = _make_agent(sim_core, initial_soc=1.0)
        bp = agent.battery_plugin
        assert bp.soc == 1.0
        assert bp.discharge_rate == 0.001
        assert bp.charge_rate == 0.005
        assert bp.idle_rate == 0.0

    def test_custom_values(self, sim_core):
        agent = _make_agent(
            sim_core,
            initial_soc=0.5,
            discharge_rate=0.01,
            charge_rate=0.02,
            idle_rate=0.001,
        )
        bp = agent.battery_plugin
        assert bp.soc == 0.5
        assert bp.discharge_rate == 0.01
        assert bp.charge_rate == 0.02
        assert bp.idle_rate == 0.001


# -------------------------------------------------------------------
# Agent battery properties
# -------------------------------------------------------------------


class TestAgentBattery:
    """Tests for battery properties on Agent."""

    def test_no_battery_defaults_to_1(self, sim_core):
        """Agent without battery plugin always reports SOC=1.0."""
        agent = _make_agent(sim_core)
        assert agent.battery_soc == 1.0
        assert agent.battery_plugin is None
        assert agent.is_charging is False

    def test_initial_soc(self, sim_core):
        """Agent's initial SOC matches plugin's initial_soc."""
        agent = _make_agent(sim_core, initial_soc=0.75)
        assert agent.battery_soc == pytest.approx(0.75)
        assert agent.battery_plugin is not None

    def test_charging_flag(self, sim_core):
        """set_charging toggles is_charging."""
        agent = _make_agent(sim_core, initial_soc=1.0)
        assert agent.is_charging is False
        agent.set_charging(True)
        assert agent.is_charging is True
        agent.set_charging(False)
        assert agent.is_charging is False

    def test_set_charging_noop_without_plugin(self, sim_core):
        """set_charging is a no-op when no battery plugin."""
        agent = _make_agent(sim_core)
        agent.set_charging(True)
        assert agent.is_charging is False  # unchanged


# -------------------------------------------------------------------
# Battery drain/charge during update
# -------------------------------------------------------------------


class TestBatteryUpdate:
    """Tests for battery drain and charge during Agent.update()."""

    def test_drain_while_moving(self, sim_core):
        """Battery drains when agent is moving."""
        agent = _make_agent(sim_core, initial_soc=1.0, discharge_rate=0.1)

        # Set a goal so the agent starts moving
        agent.set_goal_pose(Pose.from_xyz(10, 0, 0.1))
        sim_core.tick(10)  # 10 steps

        assert agent.battery_soc < 1.0

    def test_no_drain_when_idle_rate_zero(self, sim_core):
        """Battery does not drain when idle and idle_rate=0."""
        agent = _make_agent(sim_core, initial_soc=0.8, discharge_rate=0.1, idle_rate=0.0)
        # Don't set any goal — agent is idle
        sim_core.tick(100)
        assert agent.battery_soc == pytest.approx(0.8)

    def test_idle_drain(self, sim_core):
        """Battery drains when idle with non-zero idle_rate."""
        agent = _make_agent(sim_core, initial_soc=1.0, idle_rate=0.01)
        # Agent is idle — no goal set
        sim_core.tick(10)
        assert agent.battery_soc < 1.0

    def test_charge_increases_soc(self, sim_core):
        """Battery charges when set_charging(True) is active."""
        agent = _make_agent(sim_core, initial_soc=0.5, charge_rate=0.1)
        agent.set_charging(True)
        sim_core.tick(10)
        assert agent.battery_soc > 0.5

    def test_soc_clamp_at_zero(self, sim_core):
        """SOC never goes below 0.0."""
        agent = _make_agent(sim_core, initial_soc=0.001, discharge_rate=1.0)
        agent.set_goal_pose(Pose.from_xyz(10, 0, 0.1))
        sim_core.tick(100)
        assert agent.battery_soc == pytest.approx(0.0)

    def test_soc_clamp_at_one(self, sim_core):
        """SOC never goes above 1.0."""
        agent = _make_agent(sim_core, initial_soc=0.99, charge_rate=1.0)
        agent.set_charging(True)
        sim_core.tick(100)
        assert agent.battery_soc == pytest.approx(1.0)

    def test_no_battery_update_without_plugin(self, sim_core):
        """Agent without battery plugin always stays at 1.0 regardless of movement."""
        agent = _make_agent(sim_core)
        agent.set_goal_pose(Pose.from_xyz(10, 0, 0.1))
        sim_core.tick(100)
        assert agent.battery_soc == 1.0

    def test_charge_rate_accuracy(self, sim_core):
        """Verify linear charge rate over known time period."""
        dt = sim_core._dt  # timestep
        charge_rate = 0.1  # SOC/s
        agent = _make_agent(sim_core, initial_soc=0.5, charge_rate=charge_rate)
        agent.set_charging(True)

        n_steps = 10
        sim_core.tick(n_steps)
        expected_soc = 0.5 + charge_rate * dt * n_steps
        assert agent.battery_soc == pytest.approx(expected_soc, abs=1e-6)

    def test_discharge_rate_accuracy(self, sim_core):
        """Verify linear discharge rate over known time period."""
        dt = sim_core._dt  # timestep
        discharge_rate = 0.1  # SOC/s
        agent = _make_agent(sim_core, initial_soc=1.0, discharge_rate=discharge_rate)
        # Set unreachable goal so agent stays moving for all steps
        agent.set_goal_pose(Pose.from_xyz(1000, 0, 0.1))

        n_steps = 10
        sim_core.tick(n_steps)
        expected_soc = 1.0 - discharge_rate * dt * n_steps
        assert agent.battery_soc == pytest.approx(expected_soc, abs=1e-6)


# -------------------------------------------------------------------
# AgentSpawnParams.from_dict with battery plugin
# -------------------------------------------------------------------


class TestBatteryFromDict:
    """Tests for parsing plugins from YAML-like dicts."""

    def test_from_dict_with_battery_plugin(self, sim_core):
        """Battery plugin configured via plugins list in dict."""
        config = {
            "name": "test_bot",
            "urdf_path": MOBILE_URDF,
            "plugins": [
                {
                    "type": "battery",
                    "config": {
                        "initial_soc": 0.6,
                        "discharge_rate": 0.002,
                        "charge_rate": 0.01,
                        "idle_rate": 0.0001,
                    },
                }
            ],
        }
        params = AgentSpawnParams.from_dict(config)
        assert params.plugins is not None
        agent = Agent.from_params(params, sim_core)
        assert agent.battery_soc == pytest.approx(0.6)
        bp = agent.battery_plugin
        assert bp is not None
        assert bp.discharge_rate == 0.002
        assert bp.charge_rate == 0.01
        assert bp.idle_rate == 0.0001

    def test_from_dict_without_battery(self):
        config = {"name": "test_bot", "urdf_path": MOBILE_URDF}
        params = AgentSpawnParams.from_dict(config)
        assert not params.plugins
