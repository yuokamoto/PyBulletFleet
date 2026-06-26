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


def test_dispenser_request_no_robot_instant_success(make_handler):
    h = make_handler(agents=[])  # no candidates
    h._plugin.find_nearest_robot.return_value = None
    h._on_dispenser_request(_disp_request())
    statuses = [c[0][0].status for c in h._dispenser_result_pub.publish.call_args_list]
    assert DispenserResult.SUCCESS in statuses
    assert h._dispensers["dispenser1"].mode == DispenserState.IDLE  # reset after instant success
    h._plugin.dispense.assert_not_called()


def test_post_step_publishes_states_after_interval(make_handler):
    from pybullet_fleet_rmf.workcell_handler import _WorkcellInfo

    h = make_handler()
    h._dispensers["d1"] = _WorkcellInfo("d1")
    h.post_step(sim_time=10.0)  # first call (>interval from -1.0)
    assert h._dispenser_state_pub.publish.called
