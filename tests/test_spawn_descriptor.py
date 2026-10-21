"""An agent's stored spawn descriptor is its own copy, built once per spawn.

The descriptor keeps the controller's construction config as plain data, a
fresh dict from ``ControllerParams.as_config()`` (``asdict`` copies nested
values), so it needs no second deep copy. Parameter introspection for spawn
factories and controller configs is cached per callable.
"""

from __future__ import annotations

from typing import Any, Dict, cast

import pybullet as p
import pytest

from pybullet_fleet import Agent, AgentSpawnParams, MotionMode, MultiRobotSimulationCore, Pose, SimulationParams
from pybullet_fleet import config_utils


@pytest.fixture
def sim():
    sc = MultiRobotSimulationCore(SimulationParams(gui=False, monitor=False, physics=False, monitor_log_interval=0))
    sc.initialize_simulation()
    yield sc
    p.disconnect(sc.client)


CONTROLLER = {"type": "omni", "max_linear_vel": [1.0, 0.5, 0.0], "max_linear_accel": 2.0}


def _spawn(sim, name):
    params = AgentSpawnParams(
        name=name,
        urdf_path="robots/simple_cube.urdf",
        initial_pose=Pose.from_xyz(0.0, 0.0, 0.1),
        motion_mode=MotionMode.OMNIDIRECTIONAL,
        controller=dict(CONTROLLER, max_linear_vel=list(CONTROLLER["max_linear_vel"])),
    )
    return Agent.from_params(params, sim_core=sim)


def _descriptor_controller(agent) -> Dict[str, Any]:
    """The controller config an agent's spawn descriptor keeps (a plain dict)."""
    return cast(Dict[str, Any], cast(AgentSpawnParams, agent._checkpoint_spawn_params).controller)


def test_a_stored_descriptor_is_independent(sim):
    """Mutating one agent's stored controller config changes neither the other
    agent's nor the live controller's limits."""
    first = _spawn(sim, "first")
    second = _spawn(sim, "second")
    stored = _descriptor_controller(first)
    assert stored["max_linear_vel"] == [1.0, 0.5, 0.0]

    stored["max_linear_vel"][0] = 99.0
    stored["max_linear_accel"] = 99.0

    assert _descriptor_controller(second)["max_linear_vel"] == [1.0, 0.5, 0.0]
    assert _descriptor_controller(second)["max_linear_accel"] == 2.0
    assert list(first.controller.params.max_linear_vel) == [1.0, 0.5, 0.0]
    assert first.controller.params.max_linear_accel == 2.0


def test_parameter_names_are_introspected_once_per_callable(monkeypatch):
    calls = []
    real = config_utils.inspect.signature

    def counting(function):
        calls.append(function)
        return real(function)

    config_utils._parameter_names.cache_clear()
    monkeypatch.setattr(config_utils.inspect, "signature", counting)

    def factory(cls, name, pose=None, *, mass=0.0):  # a stand-in factory
        return None

    for _ in range(5):
        assert config_utils._parameter_names(factory) == frozenset({"name", "pose", "mass"})
    assert calls == [factory]
    config_utils._parameter_names.cache_clear()
