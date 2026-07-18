"""Unit tests for RMF client factory and fleet ROS client."""

import math
from unittest.mock import MagicMock

import pytest

pytest.importorskip("pybullet_fleet_rmf.fleet_clients", reason="ROS 2 / RMF message dependencies not available")

from geometry_msgs.msg import Point, Pose, Quaternion
from pybullet_fleet_msgs.msg import FleetState, RobotState3D


def _node_with_clients(*clients):
    node = MagicMock()
    node.create_subscription.side_effect = lambda *args, **kwargs: MagicMock()
    node.create_client.side_effect = list(clients)
    return node


def _fleet_state(robot_name="tinyRobot1", x=1.0, y=2.0, yaw=0.0, battery_soc=0.5):
    state = FleetState()
    robot = RobotState3D()
    robot.name = robot_name
    robot.pose = Pose(
        position=Point(x=x, y=y, z=0.0),
        orientation=Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)),
    )
    robot.has_battery_soc = True
    robot.battery_soc = battery_soc
    state.robots = [robot]
    return state


def test_client_factory_defaults_to_per_robot_ros(mock_node):
    from pybullet_fleet_rmf.fleet_clients import PerRobotRosClientFactory, create_rmf_client_factory
    from pybullet_fleet_rmf.robot_client_api import RobotClientAPI

    factory = create_rmf_client_factory("", mock_node)
    assert isinstance(factory, PerRobotRosClientFactory)
    assert isinstance(factory.robot("tinyRobot1"), RobotClientAPI)


def test_client_factory_creates_fleet_ros_client():
    from pybullet_fleet_rmf.fleet_clients import RosFleetClient, create_rmf_client_factory

    nav_client = MagicMock()
    node = _node_with_clients(nav_client)

    factory = create_rmf_client_factory("fleet_ros", node)

    assert isinstance(factory, RosFleetClient)
    node.create_subscription.assert_called_once()
    node.create_client.assert_called_once()
    assert node.create_client.call_args.args[1] == "/fleet/navigate"


def test_client_factory_rejects_aliases(mock_node):
    from pybullet_fleet_rmf.fleet_clients import create_rmf_client_factory

    for mode in ("per_robot", "legacy", "legacy_ros", "fleet", "ros_fleet"):
        with pytest.raises(ValueError):
            create_rmf_client_factory(mode, mock_node)


def test_ros_fleet_client_reads_shared_fleet_state():
    from pybullet_fleet_rmf.fleet_clients import RosFleetClient

    nav_client = MagicMock()
    node = _node_with_clients(nav_client, MagicMock(), MagicMock(), MagicMock())
    fleet = RosFleetClient(node, map_name="L1")
    state_cb = node.create_subscription.call_args.args[2]
    robot = fleet.robot("tinyRobot1")

    state_cb(_fleet_state(x=1.5, y=-2.0, yaw=0.25, battery_soc=0.42))
    data = robot.get_data()

    assert data is not None
    assert data.map == "L1"
    assert data.position[0] == pytest.approx(1.5)
    assert data.position[1] == pytest.approx(-2.0)
    assert data.position[2] == pytest.approx(0.25)
    assert data.battery_soc == pytest.approx(0.42)


def test_ros_fleet_client_tracks_map_name_per_robot():
    from pybullet_fleet_rmf.fleet_clients import RosFleetClient

    nav_client = MagicMock()
    node = _node_with_clients(nav_client, MagicMock(), MagicMock(), MagicMock(), MagicMock(), MagicMock(), MagicMock())
    fleet = RosFleetClient(node, map_name="L1")
    state_cb = node.create_subscription.call_args.args[2]
    first = fleet.robot("tinyRobot1")
    second = fleet.robot("tinyRobot2")

    first.set_map_name("L2")
    state = FleetState()
    state.robots = [
        _fleet_state("tinyRobot1").robots[0],
        _fleet_state("tinyRobot2").robots[0],
    ]
    state_cb(state)

    assert first.get_data().map == "L2"
    assert second.get_data().map == "L1"


def test_ros_fleet_robot_client_navigates_through_fleet_service():
    from pybullet_fleet_rmf.fleet_clients import RosFleetClient

    future = MagicMock()
    nav_client = MagicMock()
    nav_client.service_is_ready.return_value = True
    nav_client.call_async.return_value = future
    node = _node_with_clients(nav_client, MagicMock(), MagicMock(), MagicMock())
    fleet = RosFleetClient(node, map_name="L1")
    state_cb = node.create_subscription.call_args.args[2]
    robot = fleet.robot("tinyRobot1")
    state_cb(_fleet_state(x=0.0, y=0.0))

    assert robot.navigate(12, [1.0, 2.0, 0.3], "L1") is True

    nav_client.call_async.assert_called_once()
    req = nav_client.call_async.call_args.args[0]
    assert req.command_id == "12"
    assert req.source == "rmf"
    assert len(req.goals_2d) == 1
    assert req.goals_2d[0].name == "tinyRobot1"
    assert list(req.goals_2d[0].position) == pytest.approx([1.0, 2.0])
    assert req.goals_2d[0].yaw == pytest.approx(0.3)
    future.add_done_callback.assert_called_once()


def test_ros_fleet_robot_client_marks_navigation_complete_from_state():
    from pybullet_fleet_rmf.fleet_clients import RosFleetClient

    nav_client = MagicMock()
    nav_client.service_is_ready.return_value = True
    nav_client.call_async.return_value = MagicMock()
    node = _node_with_clients(nav_client, MagicMock(), MagicMock(), MagicMock())
    fleet = RosFleetClient(node, map_name="L1")
    state_cb = node.create_subscription.call_args.args[2]
    robot = fleet.robot("tinyRobot1")

    state_cb(_fleet_state(x=0.0, y=0.0))
    assert robot.navigate(4, [1.0, 0.0, 0.0], "L1") is True
    assert not robot.get_data().is_command_completed(4)

    state_cb(_fleet_state(x=1.0, y=0.0))
    assert robot.get_data().is_command_completed(4)
