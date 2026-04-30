"""Tests for BridgeNode — main simulation node.

These tests require ROS 2.  Run inside Docker or with a sourced ROS workspace.
"""

import tempfile

import pytest
import yaml

ros_msgs = pytest.importorskip("rclpy", reason="ROS 2 (rclpy) not available")


def test_ros_simulation_core_overrides_run_simulation():
    """ROSSimulationCore.run_simulation() initializes but does not block."""
    from pybullet_fleet import SimulationParams
    from pybullet_fleet_ros.bridge_node import ROSSimulationCore

    sim = ROSSimulationCore(SimulationParams(gui=False, monitor=False, target_rtf=0))
    # run_simulation should NOT block (it just calls initialize_simulation)
    sim.run_simulation(duration=0)
    assert sim._step_count == 0


def test_step_once_increments_sim_time():
    """step_once() increments simulation step count.

    Note: sim_time is updated at the *start* of step_once() using the previous _step_count,
    so after one call sim_time is still 0.0.  After two calls, sim_time > 0.
    We verify via _step_count (incremented at end of each step).
    """
    from pybullet_fleet import SimulationParams
    from pybullet_fleet_ros.bridge_node import ROSSimulationCore

    sim = ROSSimulationCore(SimulationParams(gui=False, monitor=False, physics=False))
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
