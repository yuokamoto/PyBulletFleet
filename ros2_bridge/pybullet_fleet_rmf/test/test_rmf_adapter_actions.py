"""Unit tests for RMF adapter custom action handling."""

from unittest.mock import MagicMock

import pytest

pytest.importorskip("pybullet_fleet_rmf.fleet_adapter", reason="ROS 2 / RMF not available")


def test_unknown_execute_action_warns_and_finishes():
    from pybullet_fleet_rmf.fleet_adapter import RobotAdapter

    node = MagicMock()
    logger = node.get_logger.return_value
    api = MagicMock()
    execution = MagicMock()
    adapter = RobotAdapter(
        name="tinyRobot1",
        configuration=MagicMock(compatible_chargers=["tinyRobot1_charger"]),
        node=node,
        api=api,
        fleet_handle=MagicMock(),
    )

    adapter.execute_action("inspect", {"zone": "A"}, execution)

    logger.warning.assert_called_once()
    assert "not mapped to a PyBulletFleet action yet" in logger.warning.call_args.args[0]
    execution.finished.assert_called_once()
    assert adapter.execution is None
    api.execute_action.assert_not_called()


def test_other_robot_charger_waypoint_does_not_start_charging(monkeypatch):
    from pybullet_fleet_rmf.fleet_adapter import RobotAdapter

    node = MagicMock()
    api = MagicMock()
    api.navigate.return_value = True
    api.start_charge.return_value = True
    execution = MagicMock()
    destination = MagicMock()
    destination.position = [1.0, 2.0, 0.0]
    destination.map = "L1"
    destination.speed_limit = None
    destination.name = "tinyRobot1_charger"
    data = MagicMock()
    data.is_command_completed.return_value = True

    adapter = RobotAdapter(
        name="tinyRobot2",
        configuration=MagicMock(compatible_chargers=["tinyRobot2_charger"]),
        node=node,
        api=api,
        fleet_handle=MagicMock(),
    )
    monkeypatch.setattr(adapter, "_is_charger_waypoint", lambda _: True)

    adapter.navigate(destination, execution)
    adapter.update(MagicMock(), data)

    api.start_charge.assert_not_called()
    execution.finished.assert_called_once()
    assert adapter.execution is None


def test_charger_navigation_starts_charging_after_arrival(monkeypatch):
    from pybullet_fleet_rmf.fleet_adapter import RobotAdapter

    node = MagicMock()
    api = MagicMock()
    api.run_commands_inline = True
    api.navigate.return_value = True
    api.start_charge.return_value = True
    execution = MagicMock()
    destination = MagicMock()
    destination.position = [1.0, 2.0, 0.0]
    destination.map = "L1"
    destination.speed_limit = None
    destination.name = "tinyRobot1_charger"
    data = MagicMock()
    data.is_command_completed.return_value = True

    adapter = RobotAdapter(
        name="tinyRobot1",
        configuration=MagicMock(compatible_chargers=["tinyRobot1_charger"]),
        node=node,
        api=api,
        fleet_handle=MagicMock(),
    )
    monkeypatch.setattr(adapter, "_is_charger_waypoint", lambda _: True)

    adapter.navigate(destination, execution)

    api.navigate.assert_called_once_with(1, destination.position, "L1", None)
    api.start_charge.assert_not_called()
    execution.finished.assert_not_called()

    adapter.update(MagicMock(), data)

    api.start_charge.assert_called_once_with(2)
    execution.finished.assert_called_once()
    assert adapter.execution is None


def test_non_charger_navigation_clears_pending_charger_command(monkeypatch):
    from pybullet_fleet_rmf.fleet_adapter import RobotAdapter

    node = MagicMock()
    api = MagicMock()
    api.navigate.return_value = True
    api.start_charge.return_value = True
    execution = MagicMock()
    data = MagicMock()
    data.is_command_completed.return_value = True

    charger_destination = MagicMock()
    charger_destination.position = [1.0, 2.0, 0.0]
    charger_destination.map = "L1"
    charger_destination.speed_limit = None
    charger_destination.name = "tinyRobot1_charger"

    normal_destination = MagicMock()
    normal_destination.position = [2.0, 3.0, 0.0]
    normal_destination.map = "L1"
    normal_destination.speed_limit = None
    normal_destination.name = "lounge"

    adapter = RobotAdapter(
        name="tinyRobot1",
        configuration=MagicMock(compatible_chargers=["tinyRobot1_charger"]),
        node=node,
        api=api,
        fleet_handle=MagicMock(),
    )
    monkeypatch.setattr(adapter, "_is_charger_waypoint", lambda _: False)

    adapter.navigate(charger_destination, execution)
    assert adapter._pending_charger_cmd == ("tinyRobot1_charger", 2)

    adapter.navigate(normal_destination, execution)
    assert adapter._pending_charger_cmd is None

    adapter.update(MagicMock(), data)

    api.start_charge.assert_not_called()
