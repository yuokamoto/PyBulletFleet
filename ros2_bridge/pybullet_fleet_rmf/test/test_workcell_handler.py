"""Unit tests for WorkcellHandler (RMF dispenser/ingestor protocol bridge).

The handler is the thin ROS layer; simulation logic lives in WorkcellPlugin,
which is mocked here.
"""

from unittest.mock import MagicMock

import pytest

# importorskip the module under test: it pulls all the RMF message deps used
# below, so a single guard covers them all (skips outside a ROS env).
pytest.importorskip("pybullet_fleet_rmf.workcell_handler", reason="ROS 2 / RMF not available")

from rmf_dispenser_msgs.msg import DispenserRequest, DispenserResult, DispenserState
from rmf_fleet_msgs.msg import FleetState, RobotState
from rmf_ingestor_msgs.msg import IngestorRequest, IngestorResult, IngestorState


def _agent(name):
    a = MagicMock()
    a.name = name
    return a


@pytest.fixture
def make_handler(node_with_clock):
    """Factory: build a WorkcellHandler with a mocked WorkcellPlugin injected.

    ``on_init()`` (which registers the real plugin) is not called in unit tests,
    so we attach a mock plugin directly.
    """
    from pybullet_fleet_rmf.workcell_handler import WorkcellHandler

    def _make(agents=()):
        sim_core = MagicMock()
        sim_core.agents = list(agents)
        h = WorkcellHandler(node_with_clock, sim_core)
        h._plugin = MagicMock()
        h._plugin.pending_actions = {}
        return h

    return _make


def _disp_request(guid="g1", target="dispenser1", fleet="tinyRobot"):
    req = DispenserRequest()
    req.request_guid = guid
    req.target_guid = target
    req.transporter_type = fleet
    return req


def _ing_request(guid="g1", target="ingestor1", fleet="tinyRobot"):
    req = IngestorRequest()
    req.request_guid = guid
    req.target_guid = target
    req.transporter_type = fleet
    return req


def test_pre_register_infers_type_by_name(make_handler):
    h = make_handler()
    h._pre_register_workcells({"overrides": {"coke_dispenser": {}, "coke_ingestor": {}}})
    assert "coke_dispenser" in h._dispensers
    assert "coke_ingestor" in h._ingestors


def test_pre_register_explicit_type_wins(make_handler):
    h = make_handler()
    h._pre_register_workcells({"overrides": {"weird_name": {"type": "ingestor"}}})
    assert "weird_name" in h._ingestors
    assert "weird_name" not in h._dispensers


def test_fleet_state_caches_candidates(make_handler):
    a, b = _agent("tinyRobot1"), _agent("tinyRobot2")
    h = make_handler(agents=[a, b, _agent("other")])
    fs = FleetState()
    fs.name = "tinyRobot"
    rs1, rs2 = RobotState(), RobotState()
    rs1.name, rs2.name = "tinyRobot1", "tinyRobot2"
    fs.robots = [rs1, rs2]
    h._on_fleet_state(fs)
    candidates = h._get_fleet_candidates("tinyRobot")
    assert {c.name for c in candidates} == {"tinyRobot1", "tinyRobot2"}


def test_dispenser_request_delegates_to_plugin(make_handler):
    robot = _agent("tinyRobot1")
    h = make_handler(agents=[robot])
    h._fleet_robots["tinyRobot"] = ["tinyRobot1"]
    h._plugin.find_nearest_robot.return_value = robot
    h._plugin.dispense.return_value = ("item", "pick_action")

    h._on_dispenser_request(_disp_request())

    h._plugin.dispense.assert_called_once()
    assert "dispenser1" in h._dispensers
    statuses = [c[0][0].status for c in h._dispenser_result_pub.publish.call_args_list]
    assert DispenserResult.ACKNOWLEDGED in statuses


def test_dispenser_request_dedup_replays_cached_result(make_handler):
    h = make_handler()
    h._past_requests["d:g1"] = DispenserResult.SUCCESS
    h._on_dispenser_request(_disp_request(guid="g1"))
    h._plugin.dispense.assert_not_called()
    published = h._dispenser_result_pub.publish.call_args[0][0]
    assert published.status == DispenserResult.SUCCESS


def test_dispenser_request_no_robot_fails_by_default(make_handler):
    # Gazebo TeleportDispenser behaviour: no robot at the workcell -> FAILED.
    h = make_handler(agents=[])  # no candidates
    h._plugin.find_nearest_robot.return_value = None
    h._on_dispenser_request(_disp_request())
    statuses = [c[0][0].status for c in h._dispenser_result_pub.publish.call_args_list]
    assert DispenserResult.FAILED in statuses
    assert DispenserResult.SUCCESS not in statuses
    assert h._dispensers["dispenser1"].mode == DispenserState.IDLE  # reset after the failure
    h._plugin.dispense.assert_not_called()


def test_dispenser_request_no_robot_success_when_configured(make_handler):
    # workcell.fail_on_no_robot: false -> report SUCCESS so RMF doesn't stall.
    h = make_handler(agents=[])
    h._fail_on_no_robot = False
    h._plugin.find_nearest_robot.return_value = None
    h._on_dispenser_request(_disp_request())
    statuses = [c[0][0].status for c in h._dispenser_result_pub.publish.call_args_list]
    assert DispenserResult.SUCCESS in statuses
    assert DispenserResult.FAILED not in statuses


def test_ingestor_request_delegates_to_plugin(make_handler):
    robot = _agent("tinyRobot1")
    h = make_handler(agents=[robot])
    h._fleet_robots["tinyRobot"] = ["tinyRobot1"]
    h._plugin.find_nearest_carrier.return_value = robot  # ingestor uses find_nearest_carrier
    h._plugin.ingest.return_value = "drop_action"

    h._on_ingestor_request(_ing_request())

    h._plugin.ingest.assert_called_once()
    assert "ingestor1" in h._ingestors
    statuses = [c[0][0].status for c in h._ingestor_result_pub.publish.call_args_list]
    assert IngestorResult.ACKNOWLEDGED in statuses


def test_ingestor_request_dedup_replays_cached_result(make_handler):
    h = make_handler()
    h._past_requests["i:g1"] = IngestorResult.SUCCESS  # note the "i:" prefix
    h._on_ingestor_request(_ing_request(guid="g1"))
    h._plugin.ingest.assert_not_called()
    assert h._ingestor_result_pub.publish.call_args[0][0].status == IngestorResult.SUCCESS


def test_ingestor_request_no_carrier_fails_by_default(make_handler):
    # Gazebo TeleportIngestor behaviour: no carrier robot -> FAILED.
    h = make_handler(agents=[])
    h._plugin.find_nearest_carrier.return_value = None
    h._on_ingestor_request(_ing_request())
    statuses = [c[0][0].status for c in h._ingestor_result_pub.publish.call_args_list]
    assert IngestorResult.FAILED in statuses
    assert IngestorResult.SUCCESS not in statuses
    assert h._ingestors["ingestor1"].mode == IngestorState.IDLE
    h._plugin.ingest.assert_not_called()


def test_ingestor_request_no_carrier_success_when_configured(make_handler):
    h = make_handler(agents=[])
    h._fail_on_no_robot = False
    h._plugin.find_nearest_carrier.return_value = None
    h._on_ingestor_request(_ing_request())
    statuses = [c[0][0].status for c in h._ingestor_result_pub.publish.call_args_list]
    assert IngestorResult.SUCCESS in statuses
    assert IngestorResult.FAILED not in statuses


def test_dispenser_pending_uses_prefixed_key(make_handler):
    # An in-flight dispenser action is keyed "d:<guid>"; a re-sent dispenser
    # request with the same guid is treated as pending (ACK, no re-dispense).
    h = make_handler()
    h._plugin.pending_actions = {"d:g1": object()}
    h._on_dispenser_request(_disp_request(guid="g1"))
    h._plugin.dispense.assert_not_called()
    statuses = [c[0][0].status for c in h._dispenser_result_pub.publish.call_args_list]
    assert DispenserResult.ACKNOWLEDGED in statuses


def test_ingestor_not_blocked_by_dispenser_with_same_guid(make_handler):
    # Same raw guid across dispenser+ingestor must not collide: a pending
    # dispenser ("d:g1") must not make an ingestor request ("i:g1") look pending.
    robot = _agent("tinyRobot1")
    h = make_handler(agents=[robot])
    h._fleet_robots["tinyRobot"] = ["tinyRobot1"]
    h._plugin.pending_actions = {"d:g1": object()}  # dispenser g1 in flight
    h._plugin.find_nearest_carrier.return_value = robot
    h._plugin.ingest.return_value = "drop_action"
    h._on_ingestor_request(_ing_request(guid="g1"))
    h._plugin.ingest.assert_called_once()  # not blocked by the dispenser's pending entry


def test_post_step_publishes_states_after_interval(make_handler):
    from pybullet_fleet_rmf.workcell_handler import _WorkcellInfo

    h = make_handler()
    h._dispensers["d1"] = _WorkcellInfo("d1")
    h._ingestors["i1"] = _WorkcellInfo("i1")
    h.post_step(sim_time=10.0)  # first call (>interval from -1.0)
    assert h._dispenser_state_pub.publish.called
    assert h._ingestor_state_pub.publish.called
