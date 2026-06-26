"""Unit tests for RobotClientAPI (RMF EasyFullControl ↔ bridge client).

Mock-based; require ROS 2 message types. Run inside Docker or a sourced ROS 2
workspace.
"""

import math
from unittest.mock import MagicMock

import pytest

# importorskip the module under test: it pulls rclpy + all the ROS/RMF message
# deps used below, so a single guard covers them all (skips outside a ROS env).
pytest.importorskip("pybullet_fleet_rmf.robot_client_api", reason="ROS 2 / RMF not available")

from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState


def _api(mock_node, name="tinyRobot1"):
    from pybullet_fleet_rmf.robot_client_api import RobotClientAPI

    return RobotClientAPI(name, mock_node, map_name="L1")


def _odom_at(api, x, y, yaw):
    """Build an Odometry message at a pose (used to feed the API a first odom,
    since get_data() returns None until one arrives)."""
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation = api._yaw_to_quat(yaw)
    return msg


def test_update_data_is_command_completed():
    from pybullet_fleet_rmf.robot_client_api import RobotUpdateData

    d = RobotUpdateData(map="L1", position=[0.0, 0.0, 0.0], battery_soc=1.0, last_completed_cmd_id=5)
    assert d.is_command_completed(5)
    assert d.is_command_completed(3)
    assert not d.is_command_completed(6)


def test_yaw_quat_roundtrip(mock_node):
    api = _api(mock_node)
    for yaw in (0.0, 0.5, -1.2, math.pi / 2):
        q = api._yaw_to_quat(yaw)
        assert api._quat_to_yaw(q) == pytest.approx(yaw, abs=1e-9)


def test_get_data_none_before_odom(mock_node):
    # Registering at (0,0,0) before odom would place the robot off the nav graph.
    assert _api(mock_node).get_data() is None


def test_odom_callback_updates_position(mock_node):
    api = _api(mock_node)
    msg = Odometry()
    msg.pose.pose.position.x = 1.5
    msg.pose.pose.position.y = -2.0
    msg.pose.pose.orientation = api._yaw_to_quat(0.5)
    api._odom_callback(msg)

    d = api.get_data()
    assert d is not None
    assert d.position[0] == pytest.approx(1.5)
    assert d.position[1] == pytest.approx(-2.0)
    assert d.position[2] == pytest.approx(0.5, abs=1e-9)
    assert d.map == "L1"
    assert d.battery_soc == 1.0  # fallback until a BatteryState arrives


def test_battery_soc_from_battery_state(mock_node):
    api = _api(mock_node)
    api._odom_callback(_odom_at(api, 0.0, 0.0, 0.0))
    bat = BatteryState()
    bat.percentage = 0.42
    api._battery_callback(bat)
    assert api.get_data().battery_soc == pytest.approx(0.42)


def test_set_map_name_reflected_in_data(mock_node):
    api = _api(mock_node)
    api.set_map_name("L2")
    api._odom_callback(_odom_at(api, 0.0, 0.0, 0.0))
    assert api.get_data().map == "L2"


def test_navigate_sends_goal_and_sets_cmd_id(mock_node):
    api = _api(mock_node)
    api._nav_client.server_is_ready.return_value = True
    assert api.navigate(cmd_id=7, position=[1.0, 2.0, 0.3], map_name="L1") is True
    assert api._active_cmd_id == 7
    api._nav_client.send_goal_async.assert_called_once()


def test_navigate_false_when_server_unavailable(mock_node):
    api = _api(mock_node)
    api._nav_client.server_is_ready.return_value = False
    api._nav_client.wait_for_server.return_value = False
    assert api.navigate(8, [0.0, 0.0, 0.0], "L1") is False
    api._nav_client.send_goal_async.assert_not_called()


def test_stop_cancels_active_goal(mock_node):
    api = _api(mock_node)
    gh = MagicMock()
    api._active_goal_handle = gh
    assert api.stop() is True
    gh.cancel_goal_async.assert_called_once()
    assert api._active_goal_handle is None


def test_stop_when_idle_is_noop(mock_node):
    assert _api(mock_node).stop() is True  # cancelling when idle is fine


def test_on_nav_complete_marks_completed_on_success(mock_node):
    api = _api(mock_node)
    api._active_cmd_id = 9
    future = MagicMock()
    future.result.return_value = MagicMock(status=GoalStatus.STATUS_SUCCEEDED)
    api._on_nav_complete(future)
    assert api._last_completed_cmd_id == 9
    assert api._active_goal_handle is None


def test_on_nav_complete_failure_does_not_mark_completed(mock_node):
    api = _api(mock_node)
    api._active_cmd_id = 11
    future = MagicMock()
    future.result.return_value = MagicMock(status=GoalStatus.STATUS_ABORTED)
    api._on_nav_complete(future)
    assert api._last_completed_cmd_id == 0  # unchanged
