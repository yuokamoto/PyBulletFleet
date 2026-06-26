"""Unit tests for LiftHandler (RMF lift protocol ↔ Elevator device)."""

from unittest.mock import MagicMock

import pytest

pytest.importorskip("rmf_lift_msgs.msg", reason="ROS 2 / RMF messages not available")

from builtin_interfaces.msg import Time as TimeMsg
from rmf_lift_msgs.msg import LiftRequest, LiftState


@pytest.fixture(autouse=True)
def _reset_lift_class_state():
    from pybullet_fleet_rmf.lift_handler import LiftHandler

    LiftHandler._instances.clear()
    LiftHandler._shared_pub = None
    LiftHandler._shared_sub = None
    yield
    LiftHandler._instances.clear()
    LiftHandler._shared_pub = None
    LiftHandler._shared_sub = None


def _elevator(name="Lift1", floors=("L1", "L2", "L3"), current="L1", moving=False):
    from pybullet_fleet.devices.elevator import Elevator

    e = MagicMock(spec=Elevator)
    e.name = name
    e.available_floors = list(floors)
    e.current_floor = current
    e.target_floor = current
    e.is_moving = moving
    e.sim_core = MagicMock()
    e.sim_core.sim_time = 0.0
    return e


def _node_with_clock():
    node = MagicMock()
    node.create_publisher.side_effect = lambda *a, **k: MagicMock()
    node.create_subscription.side_effect = lambda *a, **k: MagicMock()
    node.get_clock.return_value.now.return_value.to_msg.return_value = TimeMsg()
    return node


def _make(elevator=None):
    from pybullet_fleet_rmf.lift_handler import LiftHandler

    return LiftHandler(_node_with_clock(), elevator or _elevator())


def _request(lift_name, dest, rtype=LiftRequest.REQUEST_AGV_MODE, session="s1"):
    req = LiftRequest()
    req.lift_name = lift_name
    req.destination_floor = dest
    req.request_type = rtype
    req.session_id = session
    return req


def test_rejects_non_elevator_agent():
    from pybullet_fleet_rmf.lift_handler import LiftHandler

    with pytest.raises(TypeError):
        LiftHandler(_node_with_clock(), MagicMock(name="not_an_elevator"))


def test_registers_instance_and_shared_resources():
    from pybullet_fleet_rmf.lift_handler import LiftHandler

    h = _make()
    assert LiftHandler._instances["Lift1"] is h
    assert LiftHandler._shared_pub is not None and LiftHandler._shared_sub is not None


def test_request_to_other_floor_drives_elevator():
    h = _make(_elevator(current="L1"))
    h._handle_request(_request("Lift1", "L3"))
    h._elevator.request_floor.assert_called_once_with("L3")
    assert h._session_id == "s1"
    assert h._door_state == LiftState.DOOR_CLOSED


def test_request_to_same_floor_opens_doors_without_moving():
    h = _make(_elevator(current="L2"))
    h._handle_request(_request("Lift1", "L2"))
    h._elevator.request_floor.assert_not_called()
    assert h._door_state == LiftState.DOOR_OPEN


def test_request_unknown_floor_does_not_drive():
    h = _make(_elevator(current="L1", floors=("L1", "L2")))
    h._handle_request(_request("Lift1", "L9"))
    h._elevator.request_floor.assert_not_called()


def test_end_session_clears_state():
    h = _make()
    h._session_id = "s1"
    h._handle_request(_request("Lift1", "", rtype=LiftRequest.REQUEST_END_SESSION))
    assert h._session_id == ""
    assert h._door_state == LiftState.DOOR_CLOSED


def test_on_request_dispatches_by_name():
    from pybullet_fleet_rmf.lift_handler import LiftHandler

    a = _make(_elevator(name="Lift1"))
    b = _make(_elevator(name="Lift2"))
    LiftHandler._on_request(_request("Lift2", "L3"))
    b._elevator.request_floor.assert_called_once_with("L3")
    a._elevator.request_floor.assert_not_called()


def test_publish_state_fields():
    from pybullet_fleet_rmf.lift_handler import LiftHandler

    h = _make(_elevator(current="L2", moving=False))
    h._publish_state()
    msg = LiftHandler._shared_pub.publish.call_args[0][0]
    assert msg.lift_name == "Lift1"
    assert msg.current_floor == "L2"
    assert msg.motion_state == LiftState.MOTION_STOPPED


def test_post_step_opens_doors_on_arrival():
    h = _make(_elevator(current="L3", moving=False))
    h._session_id = "s1"
    h._door_state = LiftState.DOOR_CLOSED
    h.post_step(0.1, TimeMsg())
    assert h._door_state == LiftState.DOOR_OPEN


def test_destroy_tears_down_shared_when_last():
    from pybullet_fleet_rmf.lift_handler import LiftHandler

    a = _make(_elevator(name="Lift1"))
    b = _make(_elevator(name="Lift2"))
    a.destroy()
    assert LiftHandler._shared_pub is not None
    b.destroy()
    assert LiftHandler._shared_pub is None and LiftHandler._shared_sub is None
