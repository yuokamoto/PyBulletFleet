"""Tests for BridgeNode — main simulation node.

These tests require ROS 2.  Run inside Docker or with a sourced ROS workspace.
"""

from types import SimpleNamespace
from unittest.mock import MagicMock

import tempfile

import pytest
import yaml

ros_msgs = pytest.importorskip("rclpy", reason="ROS 2 (rclpy) not available")


def test_step_once_increments_sim_time():
    """step_once() advances the step count and sim_time.

    BridgeNode drives the sim by calling ``step_once()`` once per ROS step
    (its daemon thread runs ``run_simulation()`` on the same core), so verify
    the stepping primitive the bridge depends on.
    """
    from pybullet_fleet import MultiRobotSimulationCore, SimulationParams

    sim = MultiRobotSimulationCore(SimulationParams(gui=False, monitor=False, physics=False))
    sim.initialize_simulation()
    assert sim._step_count == 0
    sim.step_once()
    assert sim._step_count == 1
    sim.step_once()
    assert sim.sim_time > 0


def test_agent_spawn_params_from_dict():
    """AgentSpawnParams.from_dict is usable for config-driven spawning."""
    from pybullet_fleet.agent import AgentSpawnParams

    result = AgentSpawnParams.from_dict({"name": "robot0", "urdf_path": "robots/mobile_robot.urdf", "pose": [0, 0, 0.05]})
    assert result.name == "robot0"
    assert result.urdf_path == "robots/mobile_robot.urdf"


def test_load_yaml_config_with_robots():
    """load_yaml_config reads YAML and extracts robots list."""
    from pybullet_fleet.config_utils import load_yaml_config

    config = {
        "simulation": {"gui": False, "physics": False},
        "entities": [
            {"name": "robot0", "urdf_path": "robots/mobile_robot.urdf", "pose": [0, 0, 0.05]},
            {"name": "cube0", "urdf_path": "robots/simple_cube.urdf", "pose": [2, 0, 0.5]},
        ],
    }
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        yaml.dump(config, f)
        config_path = f.name

    result = load_yaml_config(config_path)
    assert len(result["entities"]) == 2


def test_register_robot_handler_passes_interface_config_to_robot_handler_subclass():
    """RobotHandler subclasses receive the standard per-robot group config."""
    from pybullet_fleet.types import MotionMode
    from pybullet_fleet_ros.bridge_node import BridgeNode
    from pybullet_fleet_ros.interface_config import PerRobotApiConfig
    from pybullet_fleet_ros.robot_handler import RobotHandler

    class CustomRobotHandler(RobotHandler):
        def __init__(self, node, agent, tf_broadcaster=None, interface_config=None):
            self.node = node
            self.agent = agent
            self.tf_broadcaster = tf_broadcaster
            self.interface_config = interface_config

        def destroy(self):
            pass

    cfg = PerRobotApiConfig(command_topics=False)
    bridge = BridgeNode.__new__(BridgeNode)
    bridge._api_config = SimpleNamespace(per_robot_api=cfg)
    bridge._handler_map = {}
    bridge._handlers = {}
    bridge._tf_broadcaster = object()
    bridge.handler_class = CustomRobotHandler

    agent = MagicMock()
    agent.name = "robot0"
    agent.object_id = 1
    agent._controller = object()
    agent._motion_mode = MotionMode.DIFFERENTIAL

    BridgeNode._register_robot_handler(bridge, agent)

    handler = bridge._handlers[agent.object_id][0]
    assert isinstance(handler, CustomRobotHandler)
    assert handler.interface_config is cfg
    assert handler.tf_broadcaster is bridge._tf_broadcaster
