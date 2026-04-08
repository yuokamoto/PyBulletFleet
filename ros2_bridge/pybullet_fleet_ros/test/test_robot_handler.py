"""Tests for RobotHandler — per-robot ROS interface management.

These tests use mocks exclusively and require ROS message types.
Run inside Docker or with a sourced ROS workspace.
"""

import pytest

ros_msgs = pytest.importorskip("geometry_msgs.msg", reason="ROS 2 messages not available")


def test_robot_handler_creates_publishers(mock_node, mock_agent):
    """RobotHandler creates odom, joint_states publishers for a mobile robot."""
    from pybullet_fleet_ros.robot_handler import RobotHandler

    RobotHandler(mock_node, mock_agent)

    topic_names = [call[0][1] for call in mock_node.create_publisher.call_args_list]
    assert "/test_robot/odom" in topic_names
    assert "/test_robot/joint_states" in topic_names


def test_robot_handler_creates_cmd_vel_sub(mock_node, mock_agent):
    """RobotHandler subscribes to cmd_vel."""
    from pybullet_fleet_ros.robot_handler import RobotHandler

    RobotHandler(mock_node, mock_agent)

    sub_topics = [call[0][1] for call in mock_node.create_subscription.call_args_list]
    assert "/test_robot/cmd_vel" in sub_topics


def test_cmd_vel_calls_velocity_controller(mock_node, mock_agent):
    """cmd_vel message calls OmniController.set_velocity() (not agent directly)."""
    from unittest.mock import MagicMock

    from geometry_msgs.msg import Twist

    from pybullet_fleet.geometry import Pose
    from pybullet_fleet_ros.robot_handler import RobotHandler

    mock_agent.get_pose.return_value = Pose.from_xyz(0.0, 0.0, 0.0)  # yaw=0 (facing +X)
    mock_vel_ctrl = MagicMock()

    handler = RobotHandler(mock_node, mock_agent, vel_controller=mock_vel_ctrl)

    twist = Twist()
    twist.linear.x = 1.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.z = 0.0
    handler._cmd_vel_cb(twist)
    handler.apply_cmd_vel()

    mock_vel_ctrl.set_velocity.assert_called_once()
    args = mock_vel_ctrl.set_velocity.call_args
    assert args[1]["vx"] == pytest.approx(1.0)
    assert args[1]["vy"] == pytest.approx(0.0)


def test_apply_cmd_vel_without_controller_is_safe(mock_node, mock_agent):
    """apply_cmd_vel with no OmniController does not crash."""
    from geometry_msgs.msg import Twist

    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(mock_node, mock_agent, vel_controller=None)

    twist = Twist()
    twist.linear.x = 1.0
    handler._cmd_vel_cb(twist)
    handler.apply_cmd_vel()  # should not raise


def test_publish_state_publishes_odom(mock_node, mock_agent):
    """publish_state sends Odometry message."""
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(mock_node, mock_agent)

    from pybullet_fleet_ros.conversions import sim_time_to_ros_time

    stamp = sim_time_to_ros_time(1.0)
    handler.publish_state(stamp)

    assert handler._odom_pub.publish.called


def test_destroy_cleans_up(mock_node, mock_agent):
    """destroy() removes publishers and subscribers."""
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(mock_node, mock_agent)
    handler.destroy()

    mock_node.destroy_publisher.assert_called()
    mock_node.destroy_subscription.assert_called()
