"""Test fixtures for pybullet_fleet_rmf unit tests.

Mock-based; require ROS 2 message types, so they ``importorskip`` and are
skipped outside a sourced ROS 2 workspace (e.g. the core pytest env).
"""

from unittest.mock import MagicMock, patch

import pytest


@pytest.fixture(autouse=True)
def _patch_action_client():
    """Stub the ActionClient bound in robot_client_api.

    ``rclpy.action.ActionClient`` is a pybind11 C-binding that rejects a
    ``MagicMock`` node. ``robot_client_api`` imports it at module scope, so we
    patch the bound name there (cf. test_sim_services patching ActionServer).
    No-op when the module can't be imported (ROS unavailable → tests skip).
    """
    try:
        import pybullet_fleet_rmf.robot_client_api  # noqa: F401
    except Exception:
        yield
        return
    with patch("pybullet_fleet_rmf.robot_client_api.ActionClient"):
        yield


@pytest.fixture
def mock_node():
    """Mock rclpy.node.Node with the factory methods the handlers call.

    A real Node needs ``rclpy.init()`` and a running context; the handlers only
    touch the node's factory methods + logger during construction, so a mock is
    enough for fast, runtime-free unit tests.

    Note ``side_effect=lambda ...: MagicMock()`` rather than ``return_value``:
    - ``return_value = MagicMock()`` would hand back the *same* object every call,
      so every publisher/subscription/client would be one shared mock.
    - ``side_effect`` runs the lambda *per call*, returning a *fresh* mock each
      time — mirroring a real node (each pub/sub is its own object). Tests rely on
      this to assert against a specific resource (e.g. DoorHandler._shared_pub,
      RobotClientAPI's separate odom vs battery subscriptions).
    """
    node = MagicMock()
    node.get_logger.return_value = MagicMock()  # so get_logger().info(...) is a no-op
    node.create_subscription.side_effect = lambda *a, **k: MagicMock()
    node.create_publisher.side_effect = lambda *a, **k: MagicMock()
    node.create_client.side_effect = lambda *a, **k: MagicMock()
    node.create_service.side_effect = lambda *a, **k: MagicMock()
    return node


@pytest.fixture
def node_with_clock():
    """``mock_node`` whose ``get_clock().now().to_msg()`` returns a real ``Time``.

    Handlers that stamp typed ROS messages (DoorState / LiftState / Dispenser-
    Result …) assign ``node.get_clock().now().to_msg()`` to a ``builtin_interfaces/
    Time`` field, and rosidl validates the field type — a bare MagicMock would
    raise. We import Time lazily so this conftest stays importable without ROS
    (the tests ``importorskip`` and are skipped there anyway).
    """
    from builtin_interfaces.msg import Time as TimeMsg

    node = MagicMock()
    node.get_logger.return_value = MagicMock()
    node.create_subscription.side_effect = lambda *a, **k: MagicMock()
    node.create_publisher.side_effect = lambda *a, **k: MagicMock()
    node.create_client.side_effect = lambda *a, **k: MagicMock()
    node.create_service.side_effect = lambda *a, **k: MagicMock()
    node.get_clock.return_value.now.return_value.to_msg.return_value = TimeMsg()
    return node
