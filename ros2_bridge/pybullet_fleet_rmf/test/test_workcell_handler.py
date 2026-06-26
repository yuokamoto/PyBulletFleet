"""Unit tests for WorkcellHandler (RMF dispenser/ingestor protocol bridge).

The handler is the thin ROS layer; simulation logic lives in WorkcellPlugin,
which is mocked here.
"""

from unittest.mock import MagicMock

import pytest

# importorskip the module under test: it pulls all the RMF message deps used
# below, so a single guard covers them all (skips outside a ROS env).
pytest.importorskip("pybullet_fleet_rmf.workcell_handler", reason="ROS 2 / RMF not available")

from builtin_interfaces.msg import Time as TimeMsg
from rmf_dispenser_msgs.msg import DispenserRequest, DispenserResult, DispenserState
from rmf_fleet_msgs.msg import FleetState, RobotState


def _node():
    node = MagicMock()
    node.create_subscription.side_effect = lambda *a, **k: MagicMock()
    node.create_publisher.side_effect = lambda *a, **k: MagicMock()
    node.get_clock.return_value.now.return_value.to_msg.return_value = TimeMsg()
    return node


def _agent(name):
    a = MagicMock()
    a.name = name
    return a


def _handler(agents=()):
    from pybullet_fleet_rmf.workcell_handler import WorkcellHandler

    sim_core = MagicMock()
    sim_core.agents = list(agents)
    h = WorkcellHandler(_node(), sim_core)
    # on_init() (which registers the real plugin) isn't called in unit tests; inject a mock.
    h._plugin = MagicMock()
    h._plugin.pending_actions = {}
    return h


def _disp_request(guid="g1", target="dispenser1", fleet="tinyRobot"):
    req = DispenserRequest()
    req.request_guid = guid
    req.target_guid = target
    req.transporter_type = fleet
    return req


def test_pre_register_infers_type_by_name():
    h = _handler()
    h._pre_register_workcells({"overrides": {"coke_dispenser": {}, "coke_ingestor": {}}})
    assert "coke_dispenser" in h._dispensers
    assert "coke_ingestor" in h._ingestors


def test_pre_register_explicit_type_wins():
    h = _handler()
    h._pre_register_workcells({"overrides": {"weird_name": {"type": "ingestor"}}})
    assert "weird_name" in h._ingestors
    assert "weird_name" not in h._dispensers


def test_fleet_state_caches_candidates():
    a, b = _agent("tinyRobot1"), _agent("tinyRobot2")
    h = _handler(agents=[a, b, _agent("other")])
    fs = FleetState()
    fs.name = "tinyRobot"
    rs1, rs2 = RobotState(), RobotState()
    rs1.name, rs2.name = "tinyRobot1", "tinyRobot2"
    fs.robots = [rs1, rs2]
    h._on_fleet_state(fs)
    candidates = h._get_fleet_candidates("tinyRobot")
    assert {c.name for c in candidates} == {"tinyRobot1", "tinyRobot2"}


def test_dispenser_request_delegates_to_plugin():
    robot = _agent("tinyRobot1")
    h = _handler(agents=[robot])
    h._fleet_robots["tinyRobot"] = ["tinyRobot1"]
    h._plugin.find_nearest_robot.return_value = robot
    h._plugin.dispense.return_value = ("item", "pick_action")

    h._on_dispenser_request(_disp_request())

    h._plugin.dispense.assert_called_once()
    # ACKNOWLEDGED published; workcell auto-registered and BUSY
    assert "dispenser1" in h._dispensers
    statuses = [c[0][0].status for c in h._dispenser_result_pub.publish.call_args_list]
    assert DispenserResult.ACKNOWLEDGED in statuses


def test_dispenser_request_dedup_replays_cached_result():
    h = _handler()
    h._past_requests["d:g1"] = DispenserResult.SUCCESS
    h._on_dispenser_request(_disp_request(guid="g1"))
    # Replays cached result, does not dispense again
    h._plugin.dispense.assert_not_called()
    published = h._dispenser_result_pub.publish.call_args[0][0]
    assert published.status == DispenserResult.SUCCESS


def test_dispenser_request_no_robot_instant_success():
    h = _handler(agents=[])  # no candidates
    h._plugin.find_nearest_robot.return_value = None
    h._on_dispenser_request(_disp_request())
    statuses = [c[0][0].status for c in h._dispenser_result_pub.publish.call_args_list]
    assert DispenserResult.SUCCESS in statuses
    assert h._dispensers["dispenser1"].mode == DispenserState.IDLE  # reset after instant success
    h._plugin.dispense.assert_not_called()


def test_post_step_publishes_states_after_interval():
    h = _handler()
    h._dispensers["d1"] = _winfo("d1")
    h.post_step(sim_time=10.0)  # first call (>interval from -1.0)
    assert h._dispenser_state_pub.publish.called


def _winfo(name):
    from pybullet_fleet_rmf.workcell_handler import _WorkcellInfo

    return _WorkcellInfo(name)
