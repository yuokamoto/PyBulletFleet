"""Test fixtures for pybullet_fleet_ros tests."""

from unittest.mock import MagicMock, patch

import pytest

from pybullet_fleet.geometry import Pose
from pybullet_fleet.types import MotionMode


@pytest.fixture(autouse=True)
def _patch_action_server():
    """Stub out rclpy's ActionServer for mock-node unit tests.

    ``rclpy.action.ActionServer`` is a pybind11 C-binding that validates its
    ``node`` argument and rejects a ``MagicMock`` (TypeError). Handlers
    (RobotHandler, SimServices) import it lazily inside their setup methods, so
    patching it at the source — ``rclpy.action.ActionServer`` — lets a handler
    be constructed with a mock node. No-op when rclpy is unavailable (those
    tests ``importorskip`` and are skipped anyway).
    """
    try:
        import rclpy.action  # noqa: F401
    except ImportError:
        yield
        return
    with patch("rclpy.action.ActionServer"):
        yield


@pytest.fixture
def mock_node():
    """Mock rclpy.node.Node with publisher/subscriber factory methods."""
    node = MagicMock()
    node.get_logger.return_value = MagicMock()

    def make_pub(*args, **kwargs):
        pub = MagicMock()
        pub.topic_name = args[1] if len(args) > 1 else "unknown"
        return pub

    node.create_publisher.side_effect = make_pub

    def make_sub(*args, **kwargs):
        sub = MagicMock()
        sub.topic_name = args[1] if len(args) > 1 else "unknown"
        return sub

    node.create_subscription.side_effect = make_sub

    def make_service(*args, **kwargs):
        svc = MagicMock()
        return svc

    node.create_service.side_effect = make_service

    return node


@pytest.fixture
def mock_agent():
    """Mock Agent with basic attributes for a mobile robot."""
    agent = MagicMock()
    agent.name = "test_robot"
    agent.object_id = 0
    agent.body_id = 0
    agent.is_moving = False
    agent.get_pose.return_value = Pose.from_xyz(0.0, 0.0, 0.0)
    agent.velocity = [0.0, 0.0, 0.0]
    agent.angular_velocity = 0.0
    agent.get_num_joints.return_value = 0
    agent.get_all_joints_state.return_value = []
    agent.get_all_joints_state_by_name.return_value = {}
    agent.spawn_params = MagicMock()
    agent.spawn_params.motion_mode = MotionMode.OMNIDIRECTIONAL
    agent.spawn_params.use_fixed_base = False
    agent.ik_params = None
    agent._controller = None
    # Idle state so post_step()/_publish_status() take the "no active goal" path
    # instead of dereferencing auto-generated MagicMock attributes.
    agent.goal_pose = None
    agent.path = []
    agent.current_waypoint_index = 0
    agent.get_current_action.return_value = None
    agent._action_queue = []
    # No battery by default, so RobotHandler skips the battery publisher and
    # post_step() doesn't dereference MagicMock battery_soc/is_charging.
    agent.battery_plugin = None
    return agent
