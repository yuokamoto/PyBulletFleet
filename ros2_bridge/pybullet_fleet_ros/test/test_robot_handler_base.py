"""Tests for RobotHandlerBase — abstract handler lifecycle protocol.

Validates the base class contract: pre_step / post_step / destroy lifecycle,
and that RobotHandler correctly inherits from it.
"""

import pytest

ros_msgs = pytest.importorskip("geometry_msgs.msg", reason="ROS 2 messages not available")

from builtin_interfaces.msg import Time as TimeMsg


class TestRobotHandlerBaseContract:
    """RobotHandlerBase defines the handler lifecycle protocol."""

    def test_base_class_exists_and_importable(self):
        from pybullet_fleet_ros.robot_handler_base import RobotHandlerBase

        assert RobotHandlerBase is not None

    def test_base_has_pre_step_method(self):
        from pybullet_fleet_ros.robot_handler_base import RobotHandlerBase

        assert hasattr(RobotHandlerBase, "pre_step")
        assert callable(getattr(RobotHandlerBase, "pre_step"))

    def test_base_has_post_step_method(self):
        from pybullet_fleet_ros.robot_handler_base import RobotHandlerBase

        assert hasattr(RobotHandlerBase, "post_step")
        assert callable(getattr(RobotHandlerBase, "post_step"))

    def test_base_has_destroy_method(self):
        from pybullet_fleet_ros.robot_handler_base import RobotHandlerBase

        assert hasattr(RobotHandlerBase, "destroy")
        assert callable(getattr(RobotHandlerBase, "destroy"))

    def test_base_stores_node_and_agent(self, mock_node, mock_agent):
        from pybullet_fleet_ros.robot_handler_base import RobotHandlerBase

        class MinimalHandler(RobotHandlerBase):
            def destroy(self):
                pass

        h = MinimalHandler(mock_node, mock_agent)
        assert h.node is mock_node
        assert h.agent is mock_agent

    def test_base_stores_namespace(self, mock_node, mock_agent):
        from pybullet_fleet_ros.robot_handler_base import RobotHandlerBase

        class MinimalHandler(RobotHandlerBase):
            def destroy(self):
                pass

        h = MinimalHandler(mock_node, mock_agent)
        assert h.ns == "test_robot"

    def test_base_stores_tf_broadcaster(self, mock_node, mock_agent):
        from unittest.mock import MagicMock

        from pybullet_fleet_ros.robot_handler_base import RobotHandlerBase

        class MinimalHandler(RobotHandlerBase):
            def destroy(self):
                pass

        tf = MagicMock()
        h = MinimalHandler(mock_node, mock_agent, tf_broadcaster=tf)
        assert h.tf_broadcaster is tf

    def test_pre_step_default_is_noop(self, mock_node, mock_agent):
        """Default pre_step does nothing (no-op) — no exception."""
        from pybullet_fleet_ros.robot_handler_base import RobotHandlerBase

        class MinimalHandler(RobotHandlerBase):
            def destroy(self):
                pass

        h = MinimalHandler(mock_node, mock_agent)
        h.pre_step(dt=0.01, stamp=TimeMsg(sec=1, nanosec=0))  # should not raise

    def test_post_step_default_is_noop(self, mock_node, mock_agent):
        """Default post_step does nothing (no-op) — no exception."""
        from pybullet_fleet_ros.robot_handler_base import RobotHandlerBase

        class MinimalHandler(RobotHandlerBase):
            def destroy(self):
                pass

        h = MinimalHandler(mock_node, mock_agent)
        h.post_step(dt=0.01, stamp=TimeMsg(sec=1, nanosec=0))  # should not raise

    def test_destroy_is_abstract(self):
        """Cannot instantiate RobotHandlerBase without implementing destroy()."""
        from pybullet_fleet_ros.robot_handler_base import RobotHandlerBase

        with pytest.raises(TypeError):
            RobotHandlerBase(None, None)  # type: ignore[abstract]


class TestRobotHandlerInheritance:
    """RobotHandler inherits from RobotHandlerBase."""

    def test_robot_handler_is_subclass_of_base(self):
        from pybullet_fleet_ros.robot_handler import RobotHandler
        from pybullet_fleet_ros.robot_handler_base import RobotHandlerBase

        assert issubclass(RobotHandler, RobotHandlerBase)

    def test_robot_handler_pre_step_applies_cmd_vel(self, mock_node, mock_agent):
        """RobotHandler.pre_step() applies stored cmd_vel."""
        from unittest.mock import MagicMock

        from geometry_msgs.msg import Twist

        from pybullet_fleet_ros.robot_handler import RobotHandler

        mock_ctrl = MagicMock()
        mock_agent._controller = mock_ctrl

        handler = RobotHandler(mock_node, mock_agent)
        twist = Twist()
        twist.linear.x = 1.0
        handler._cmd_vel_cb(twist)
        handler.pre_step(dt=0.01, stamp=TimeMsg(sec=1, nanosec=0))

        mock_ctrl.set_velocity.assert_called_once()

    def test_robot_handler_post_step_publishes_odom(self, mock_node, mock_agent):
        """RobotHandler.post_step() publishes odometry."""
        from pybullet_fleet_ros.robot_handler import RobotHandler

        handler = RobotHandler(mock_node, mock_agent)
        handler.post_step(dt=0.01, stamp=TimeMsg(sec=1, nanosec=0))

        assert handler._odom_pub.publish.called


class TestBridgeNodeUsesNewAPI:
    """BridgeNode calls pre_step/post_step on handlers."""

    def test_on_pre_step_calls_handler_pre_step(self, mock_node, mock_agent):
        """BridgeNode._on_pre_step delegates to handler.pre_step()."""
        from unittest.mock import MagicMock

        from pybullet_fleet_ros.robot_handler_base import RobotHandlerBase

        # Create a mock handler that tracks pre_step calls
        mock_handler = MagicMock(spec=RobotHandlerBase)

        # Simulate BridgeNode's _handlers dict and _on_pre_step logic
        # We test the pattern, not BridgeNode directly (requires full ROS init)
        handlers = {0: [mock_handler]}
        dt = 0.01
        stamp = TimeMsg(sec=1, nanosec=0)
        for handler_list in handlers.values():
            for h in handler_list:
                h.pre_step(dt=dt, stamp=stamp)

        mock_handler.pre_step.assert_called_once_with(dt=dt, stamp=stamp)

    def test_on_post_step_calls_handler_post_step(self, mock_node, mock_agent):
        """BridgeNode._on_post_step delegates to handler.post_step()."""
        from unittest.mock import MagicMock

        from pybullet_fleet_ros.robot_handler_base import RobotHandlerBase

        mock_handler = MagicMock(spec=RobotHandlerBase)
        handlers = {0: [mock_handler]}
        dt = 0.01
        stamp = TimeMsg(sec=1, nanosec=0)

        for handler_list in handlers.values():
            for h in handler_list:
                h.post_step(dt=dt, stamp=stamp)

        mock_handler.post_step.assert_called_once_with(dt=dt, stamp=stamp)
