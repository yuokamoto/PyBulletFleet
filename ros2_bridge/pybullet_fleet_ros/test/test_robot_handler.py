"""Tests for RobotHandler — per-robot ROS interface management.

These tests use mocks exclusively and require ROS message types.
Run inside Docker or with a sourced ROS workspace.
"""

import pytest

ros_msgs = pytest.importorskip("geometry_msgs.msg", reason="ROS 2 messages not available")

from builtin_interfaces.msg import Time as TimeMsg


def test_robot_handler_creates_publishers(mock_node, mock_agent):
    """RobotHandler creates odom, joint_states publishers for a mobile robot."""
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(mock_node, mock_agent)

    topic_names = [call[0][1] for call in mock_node.create_publisher.call_args_list]
    assert "/test_robot/odom" in topic_names
    assert "/test_robot/joint_states" in topic_names
    assert any(group.__class__.__name__ == "StatePublisherHandler" for group in handler._interface_groups)


def test_robot_handler_creates_cmd_vel_sub(mock_node, mock_agent):
    """RobotHandler subscribes to cmd_vel."""
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(mock_node, mock_agent)

    sub_topics = [call[0][1] for call in mock_node.create_subscription.call_args_list]
    assert "/test_robot/cmd_vel" in sub_topics
    assert any(group.__class__.__name__ == "CommandTopicHandler" for group in handler._interface_groups)


def test_robot_handler_creates_all_interface_groups(mock_node, mock_agent):
    """RobotHandler remains a facade over all per-robot interface groups."""
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(mock_node, mock_agent)

    group_names = [group.__class__.__name__ for group in handler._interface_groups]
    assert group_names == [
        "StatePublisherHandler",
        "TfPublisherHandler",
        "CommandTopicHandler",
        "NavigationActionHandler",
        "ExecuteActionHandler",
        "ServiceHandler",
    ]


def test_robot_handler_honors_state_and_tf_only_config(mock_node, mock_agent):
    """RobotHandler can create only publish-side per-robot groups."""
    from pybullet_fleet_ros.interface_config import PerRobotApiConfig
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(
        mock_node,
        mock_agent,
        interface_config=PerRobotApiConfig(
            state_publishers=True,
            tf=True,
            command_topics=False,
            services=False,
            actions=False,
        ),
    )

    group_names = [group.__class__.__name__ for group in handler._interface_groups]
    assert group_names == ["StatePublisherHandler", "TfPublisherHandler"]
    assert mock_node.create_publisher.called
    assert not mock_node.create_subscription.called
    assert not mock_node.create_service.called


def test_robot_handler_honors_command_only_config(mock_node, mock_agent):
    """RobotHandler can create only command topic subscriptions."""
    from pybullet_fleet_ros.interface_config import PerRobotApiConfig
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(
        mock_node,
        mock_agent,
        interface_config=PerRobotApiConfig(
            state_publishers=False,
            tf=False,
            command_topics=True,
            services=False,
            actions=False,
        ),
    )

    group_names = [group.__class__.__name__ for group in handler._interface_groups]
    sub_topics = [call[0][1] for call in mock_node.create_subscription.call_args_list]
    assert group_names == ["CommandTopicHandler"]
    assert "/test_robot/cmd_vel" in sub_topics
    assert not mock_node.create_publisher.called
    assert not mock_node.create_service.called


def test_robot_handler_honors_disabled_config(mock_node, mock_agent):
    """RobotHandler creates no groups when per-robot API is disabled."""
    from pybullet_fleet_ros.interface_config import PerRobotApiConfig
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(
        mock_node,
        mock_agent,
        interface_config=PerRobotApiConfig(enabled=False),
    )

    assert handler._interface_groups == []
    assert not mock_node.create_publisher.called
    assert not mock_node.create_subscription.called
    assert not mock_node.create_service.called
    handler.pre_step()
    handler.post_step()
    handler.destroy()


def test_robot_handler_honors_no_groups_config(mock_node, mock_agent):
    """RobotHandler creates no groups when every per-robot group is disabled."""
    from pybullet_fleet_ros.interface_config import PerRobotApiConfig
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(
        mock_node,
        mock_agent,
        interface_config=PerRobotApiConfig(
            state_publishers=False,
            tf=False,
            command_topics=False,
            services=False,
            actions=False,
        ),
    )

    assert handler._interface_groups == []
    assert not mock_node.create_publisher.called
    assert not mock_node.create_subscription.called
    assert not mock_node.create_service.called


def test_cmd_vel_calls_velocity_controller(mock_node, mock_agent):
    """cmd_vel message calls OmniController.set_velocity() (not agent directly)."""
    from unittest.mock import MagicMock

    from geometry_msgs.msg import Twist

    from pybullet_fleet.geometry import Pose
    from pybullet_fleet_ros.robot_handler import RobotHandler

    mock_agent.get_pose.return_value = Pose.from_xyz(0.0, 0.0, 0.0)  # yaw=0 (facing +X)
    mock_motion_ctrl = MagicMock()
    mock_agent._controller = mock_motion_ctrl

    handler = RobotHandler(mock_node, mock_agent)

    twist = Twist()
    twist.linear.x = 1.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.z = 0.0
    handler._cmd_vel_cb(twist)
    handler.pre_step(dt=0.01, stamp=TimeMsg(sec=1, nanosec=0))

    mock_motion_ctrl.set_velocity.assert_called_once()
    args = mock_motion_ctrl.set_velocity.call_args
    assert args[1]["vx"] == pytest.approx(1.0)
    assert args[1]["vy"] == pytest.approx(0.0)


def test_pre_step_without_controller_is_safe(mock_node, mock_agent):
    """pre_step with no controller does not crash."""
    from geometry_msgs.msg import Twist

    from pybullet_fleet_ros.robot_handler import RobotHandler

    mock_agent._controller = None
    handler = RobotHandler(mock_node, mock_agent)

    twist = Twist()
    twist.linear.x = 1.0
    handler._cmd_vel_cb(twist)
    handler.pre_step(dt=0.01, stamp=TimeMsg(sec=1, nanosec=0))  # should not raise


def test_execute_action_topic_uses_shared_robot_action_command(mock_node, mock_agent):
    """execute_action topic reuses RobotActionCommand inside ExecuteActionGoal."""
    from pybullet_fleet_msgs.msg import ExecuteActionGoal, RobotActionCommand

    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(mock_node, mock_agent)
    msg = ExecuteActionGoal()
    msg.command = RobotActionCommand(
        name="test_robot",
        action_type="wait",
        action_params_json='{"duration": 0.1}',
    )

    handler._execute_actions.execute_action_topic_cb(msg)

    mock_agent.add_action.assert_called_once()


def test_execute_action_topic_rejects_mismatched_robot_name(mock_node, mock_agent):
    """execute_action topic rejects commands targeted at another robot."""
    from pybullet_fleet_msgs.msg import ExecuteActionGoal, RobotActionCommand

    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(mock_node, mock_agent)
    msg = ExecuteActionGoal()
    msg.command = RobotActionCommand(
        name="other_robot",
        action_type="wait",
        action_params_json='{"duration": 0.1}',
    )

    handler._execute_actions.execute_action_topic_cb(msg)

    mock_agent.add_action.assert_not_called()


def test_post_step_publishes_odom(mock_node, mock_agent):
    """post_step sends Odometry message."""
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(mock_node, mock_agent)
    handler.post_step(dt=0.01, stamp=TimeMsg(sec=1, nanosec=0))

    assert handler._odom_pub.publish.called


def test_destroy_cleans_up(mock_node, mock_agent):
    """destroy() removes publishers and subscribers."""
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(mock_node, mock_agent)
    handler.destroy()

    mock_node.destroy_publisher.assert_called()
    mock_node.destroy_subscription.assert_called()
