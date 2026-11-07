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


def test_robots_are_added_to_rmf_one_at_a_time(monkeypatch):
    """Concurrent first updates add one robot at a time, each to completion (#152)."""
    import threading
    import time

    from pybullet_fleet_rmf import fleet_adapter
    from pybullet_fleet_rmf.fleet_adapter import RobotAdapter

    monkeypatch.setattr(fleet_adapter.rmf_easy, "RobotState", MagicMock())
    lock = threading.Lock()
    in_flight = []  # robots added whose registration RMF has not finished
    overlaps = []

    class Handle:
        def __init__(self, name):
            self.name = name
            self.polls = 0

        def more(self):
            # RMF finishes the registration on the third poll.
            self.polls += 1
            if self.polls < 3:
                return None
            with lock:
                if self.name in in_flight:
                    in_flight.remove(self.name)
            return MagicMock()

    def add_robot(name, state, configuration, callbacks):
        with lock:
            if in_flight:
                overlaps.append((name, list(in_flight)))
            in_flight.append(name)
        time.sleep(0.01)
        return Handle(name)

    fleet_handle = MagicMock()
    fleet_handle.add_robot.side_effect = add_robot
    robots = [
        RobotAdapter(
            name=f"robot{i}",
            configuration=MagicMock(),
            node=MagicMock(),
            api=MagicMock(),
            fleet_handle=fleet_handle,
        )
        for i in range(4)
    ]

    threads = [threading.Thread(target=fleet_adapter._update_robot_once, args=(robot,)) for robot in robots]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=5)

    assert fleet_handle.add_robot.call_count == 4
    assert overlaps == []
    assert all(robot.update_handle is not None for robot in robots)
    assert all(robot.update_handle.polls == 3 for robot in robots)


def test_adding_a_robot_rmf_never_finishes_times_out_with_a_warning(monkeypatch):
    from pybullet_fleet_rmf import fleet_adapter
    from pybullet_fleet_rmf.fleet_adapter import RobotAdapter

    monkeypatch.setattr(fleet_adapter.rmf_easy, "RobotState", MagicMock())
    monkeypatch.setattr(fleet_adapter, "_REGISTRATION_TIMEOUT_S", 0.05)
    handle = MagicMock()
    handle.more.return_value = None
    node = MagicMock()
    robot = RobotAdapter(
        name="robot0",
        configuration=MagicMock(),
        node=node,
        api=MagicMock(),
        fleet_handle=MagicMock(**{"add_robot.return_value": handle}),
    )

    fleet_adapter._update_robot_once(robot)

    assert robot.update_handle is handle
    node.get_logger.return_value.warning.assert_called_once()
    assert "robot0" in node.get_logger.return_value.warning.call_args.args[0]
