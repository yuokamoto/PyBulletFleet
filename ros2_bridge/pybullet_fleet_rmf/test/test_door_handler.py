"""Unit tests for DoorHandler (RMF door protocol ↔ Door device)."""

from unittest.mock import MagicMock

import pytest

pytest.importorskip("pybullet_fleet_rmf.door_handler", reason="ROS 2 / RMF not available")

from rmf_door_msgs.msg import DoorMode, DoorRequest

from pybullet_fleet.types import DoorState as SimDoorState


@pytest.fixture(autouse=True)
def _reset_door_class_state():
    """DoorHandler keeps class-level shared pub/sub + instance map; reset around each test."""
    from pybullet_fleet_rmf.door_handler import DoorHandler

    DoorHandler._instances.clear()
    DoorHandler._shared_pub = None
    DoorHandler._shared_sub = None
    yield
    DoorHandler._instances.clear()
    DoorHandler._shared_pub = None
    DoorHandler._shared_sub = None


def _door_agent(name="L1_door1", state=SimDoorState.CLOSED):
    from pybullet_fleet.devices.door import Door

    agent = MagicMock(spec=Door)  # spec makes isinstance(agent, Door) True
    agent.name = name
    agent.door_state = state
    agent.sim_core = MagicMock()
    agent.sim_core.sim_time = 0.0
    return agent


@pytest.fixture
def make_door(node_with_clock):
    """Factory: build a DoorHandler on the shared clock-equipped mock node."""
    from pybullet_fleet_rmf.door_handler import DoorHandler

    def _make(name="L1_door1", state=SimDoorState.CLOSED):
        return DoorHandler(node_with_clock, _door_agent(name, state))

    return _make


def test_rejects_non_door_agent(node_with_clock):
    from pybullet_fleet_rmf.door_handler import DoorHandler

    with pytest.raises(TypeError):
        DoorHandler(node_with_clock, MagicMock(name="not_a_door"))


def test_registers_instance_and_shared_resources(make_door):
    from pybullet_fleet_rmf.door_handler import DoorHandler

    h = make_door("L1_door1")
    assert DoorHandler._instances["L1_door1"] is h
    assert DoorHandler._shared_pub is not None
    assert DoorHandler._shared_sub is not None


def test_open_request_calls_request_open(make_door):
    h = make_door("L1_door1")
    req = DoorRequest()
    req.door_name = "L1_door1"
    req.requested_mode = DoorMode(value=DoorMode.MODE_OPEN)
    h._handle_request(req)
    h._door.request_open.assert_called_once()
    h._door.request_close.assert_not_called()


def test_close_request_calls_request_close(make_door):
    h = make_door("L1_door1")
    req = DoorRequest()
    req.door_name = "L1_door1"
    req.requested_mode = DoorMode(value=DoorMode.MODE_CLOSED)
    h._handle_request(req)
    h._door.request_close.assert_called_once()


def test_on_request_dispatches_by_name(make_door):
    from pybullet_fleet_rmf.door_handler import DoorHandler

    a = make_door("L1_door1")
    b = make_door("L1_door2")
    req = DoorRequest()
    req.door_name = "L1_door2"
    req.requested_mode = DoorMode(value=DoorMode.MODE_OPEN)
    DoorHandler._on_request(req)
    b._door.request_open.assert_called_once()
    a._door.request_open.assert_not_called()


def test_on_request_unknown_door_is_noop(make_door):
    from pybullet_fleet_rmf.door_handler import DoorHandler

    h = make_door("L1_door1")
    req = DoorRequest()
    req.door_name = "ghost_door"
    req.requested_mode = DoorMode(value=DoorMode.MODE_OPEN)
    DoorHandler._on_request(req)  # must not raise
    h._door.request_open.assert_not_called()


def test_publish_state_maps_open_and_closed(make_door):
    from pybullet_fleet_rmf.door_handler import DoorHandler

    h = make_door("L1_door1", state=SimDoorState.OPEN)
    h._publish_state()
    published = DoorHandler._shared_pub.publish.call_args[0][0]
    assert published.current_mode.value == DoorMode.MODE_OPEN
    assert published.door_name == "L1_door1"

    h._door.door_state = SimDoorState.CLOSED
    h._publish_state()
    assert DoorHandler._shared_pub.publish.call_args[0][0].current_mode.value == DoorMode.MODE_CLOSED


def test_destroy_tears_down_shared_when_last(make_door):
    from pybullet_fleet_rmf.door_handler import DoorHandler

    a = make_door("L1_door1")
    b = make_door("L1_door2")
    a.destroy()
    assert "L1_door1" not in DoorHandler._instances
    assert DoorHandler._shared_pub is not None  # b still alive
    b.destroy()
    assert DoorHandler._shared_pub is None and DoorHandler._shared_sub is None
