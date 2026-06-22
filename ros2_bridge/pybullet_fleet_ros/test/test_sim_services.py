"""Tests for SimServices — simulation_interfaces implementation.

These tests require ROS 2 message types and are designed to run inside Docker.
"""

from unittest.mock import MagicMock, patch

import pytest

ros_msgs = pytest.importorskip("simulation_interfaces.srv", reason="simulation_interfaces not available")


@pytest.fixture
def sim_services():
    """Create SimServices with mock node, sim, and bridge.

    Patches ActionServer to avoid requiring a real rclpy Node (MagicMock is not
    compatible with the pybind11 ActionServer constructor).
    """
    from pybullet_fleet_ros.sim_services import SimServices

    mock_node = MagicMock()
    mock_sim = MagicMock()
    mock_bridge = MagicMock()

    with patch("pybullet_fleet_ros.sim_services.ActionServer"):
        services = SimServices(mock_node, mock_sim, mock_bridge)
    return services, mock_node, mock_sim, mock_bridge


def test_get_simulator_features(sim_services):
    """GetSimulatorFeatures returns supported features."""
    from simulation_interfaces.srv import GetSimulatorFeatures

    services, _, _, _ = sim_services
    request = GetSimulatorFeatures.Request()
    response = GetSimulatorFeatures.Response()

    result = services._get_features(request, response)
    assert result is not None
    assert len(response.features.features) > 0


def test_get_entities_returns_agents(sim_services):
    """GetEntities returns list of all agents."""
    from simulation_interfaces.srv import GetEntities

    services, _, mock_sim, _ = sim_services

    agent1 = MagicMock()
    agent1.name = "robot0"
    agent1.object_id = 0
    agent2 = MagicMock()
    agent2.name = "robot1"
    agent2.object_id = 1
    mock_sim.sim_objects = [agent1, agent2]

    request = GetEntities.Request()
    response = GetEntities.Response()

    services._get_entities(request, response)
    assert len(response.entities) == 2


def test_step_simulation(sim_services):
    """StepSimulation calls sim.step_once()."""
    from simulation_interfaces.srv import StepSimulation

    services, _, mock_sim, _ = sim_services
    request = StepSimulation.Request()
    request.steps = 5
    response = StepSimulation.Response()

    services._step_sim(request, response)
    assert mock_sim.step_once.call_count == 5


def test_pause_resume(sim_services):
    """SetSimulationState can pause and resume."""
    from simulation_interfaces.msg import SimulationState
    from simulation_interfaces.srv import SetSimulationState

    services, _, mock_sim, _ = sim_services

    # Pause
    request = SetSimulationState.Request()
    request.state.state = SimulationState.STATE_PAUSED
    response = SetSimulationState.Response()
    services._set_sim_state(request, response)
    mock_sim.pause.assert_called_once()

    # Resume
    request.state.state = SimulationState.STATE_PLAYING
    response = SetSimulationState.Response()
    services._set_sim_state(request, response)
    mock_sim.resume.assert_called_once()
