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
        configuration=MagicMock(),
        node=node,
        api=api,
        fleet_handle=MagicMock(),
    )

    adapter.execute_action("inspect", {"zone": "A"}, execution)

    logger.warn.assert_called_once()
    assert "not mapped to a PyBulletFleet action yet" in logger.warn.call_args.args[0]
    execution.finished.assert_called_once()
    assert adapter.execution is None
    api.execute_action.assert_not_called()
