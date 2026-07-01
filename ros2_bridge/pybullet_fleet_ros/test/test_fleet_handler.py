"""Tests for the Pattern 2 fleet-level ROS handler."""

from unittest.mock import MagicMock

import pytest

pytest.importorskip("pybullet_fleet_msgs", reason="ROS 2 message package not available")
pytest.importorskip("std_msgs", reason="ROS 2 std_msgs not available")

from builtin_interfaces.msg import Time
from pybullet_fleet.geometry import Pose
from pybullet_fleet_msgs.msg import RobotGoal

from pybullet_fleet_ros.fleet_handler import FleetHandler


def _agent(name="robot0", pose=None):
    agent = MagicMock()
    agent.name = name
    agent.get_pose.return_value = pose or Pose.from_yaw(10.0, 20.0, 0.05, 0.0)
    agent.velocity = [1.0, 2.0, 0.0]
    agent.angular_velocity = 0.3
    agent.is_moving = True
    agent.battery_soc = 0.75
    agent.is_charging = False
    return agent


def test_fleet_navigate_applies_rmf_offset_and_preserves_agent_z(mock_node):
    mock_node.rmf_frame_offset = (100.0, 200.0)
    agent = _agent(pose=Pose.from_yaw(10.0, 20.0, 0.05, 0.0))
    sim = MagicMock()
    sim.agents = [agent]
    handler = FleetHandler(mock_node, sim)

    applied = handler._set_fleet_goals([RobotGoal(name="robot0", x=110.0, y=220.0, yaw=1.2)])

    assert applied == 1
    goal = agent.set_goal_pose.call_args.args[0]
    assert goal.x == pytest.approx(10.0)
    assert goal.y == pytest.approx(20.0)
    assert goal.z == pytest.approx(0.05)
    assert goal.yaw == pytest.approx(1.2)


def test_fleet_states_publish_map_frame_coordinates(mock_node):
    mock_node.rmf_frame_offset = (100.0, 200.0)
    agent = _agent(pose=Pose.from_yaw(10.0, 20.0, 0.05, 0.4))
    sim = MagicMock()
    sim.agents = [agent]
    handler = FleetHandler(mock_node, sim)

    handler.post_step(dt=0.1, stamp=Time(sec=1))

    msg = handler._states_pub.publish.call_args.args[0]
    assert msg.header.frame_id == "map"
    assert len(msg.robots) == 1
    state = msg.robots[0]
    assert state.name == "robot0"
    assert state.x == pytest.approx(110.0)
    assert state.y == pytest.approx(220.0)
    assert state.yaw == pytest.approx(0.4)
