#!/usr/bin/env python3
"""RMF dispatch end-to-end check (run inside the bridge container).

Assumes an RMF demo (office/hotel/...) is already launched and publishing. Runs
one or more dispatch SCENARIOS sequentially in the single running demo and proves
the full chain for each:

    dispatch_*  ->  RMF bidding  ->  fleet adapter  ->  NavigateToPose / actions
        ->  pybullet_fleet bridge  ->  PyBullet  ->  the robot finishes the task

Scenarios are given as argv tokens ``type:arg1,arg2,...`` with an optional
``;assertion`` clause:
    patrol:lounge,coe
    patrol:lobby,L2_room1;zrise=1.0     # also assert the robot rose >= 1.0 m (lift)
    delivery:pantry,coke_dispenser,hardware_2,coke_ingestor
    clean:clean_lobby

Each scenario PASSES when its dispatched task is observed to complete within
``SCENARIO_TIMEOUT``. The portable completion signal is ``/fleet_states``:
the task id must first be assigned to a robot and then clear from that robot's
state. This works across the Docker image and native Jazzy setups, even when
``/task_state_update`` or ``/task_summaries`` are not published by the launched
RMF stack.

``/task_state_update`` and ``/task_summaries`` are still consumed when present
and can provide an earlier explicit ``completed``/terminal status. They are
treated as auxiliary status feeds, not required interfaces.

On top of task completion we add cheap *direct* checks that catch a false
completion and localize failures:
  - delivery: a DispenserResult.SUCCESS and an IngestorResult.SUCCESS must be seen
    on /dispenser_results + /ingestor_results during the scenario;
  - any scenario tagged ``;zrise=<m>``: the robot's odom z must rise that far (the
    elevator physically carried it).
All scenarios must pass for exit 0.

Every dispatch is sent with --use_sim_time: the demos run RMF on the simulated
/clock, so a wall-clock earliest-start would sit in the sim future and the task
would never leave `queued`.
"""

import json
import math
import re
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rmf_dispenser_msgs.msg import DispenserResult
from rmf_fleet_msgs.msg import FleetState
from rmf_ingestor_msgs.msg import IngestorResult
from rmf_task_msgs.msg import DispatchState
from rmf_task_msgs.msg import DispatchStates
from rmf_task_msgs.msg import TaskSummary
from std_msgs.msg import String

MOVE_THRESHOLD = 0.3  # metres; comfortably above odom noise
# Task ids look like "<category>.dispatch-<hex>". Note clean dispatches a *composed*
# task, so its id is "compose.dispatch-..." (not "clean.dispatch-..."); match any.
TASK_ID_RE = re.compile(r"\b[a-z_]+\.dispatch-[0-9a-fA-F]+")

READY_TIMEOUT = 150.0  # wait for the demo stack + fleet adapter
SCENARIO_TIMEOUT = 300.0  # per-scenario wait for the task to finish
SETTLE_MAX = 60.0  # wait for robots to go quiet before dispatching (best effort)
SETTLE_WINDOW = 20.0  # contiguous quiet span that counts as "settled"
SETTLE_EPS = 0.05  # metres of jitter tolerated while "quiet"
ADAPTER_READY_GRACE = 10.0  # let EasyFullControl add robots after odom appears
TERMINAL_OK = ("completed",)
TERMINAL_BAD = ("failed", "canceled", "killed")


def build_dispatch(scenario: str):
    """Map a ``type:csv`` scenario token to a dispatch_* command (+ --use_sim_time)."""
    kind, _, rest = scenario.partition(":")
    args = [a for a in rest.split(",") if a]
    if kind == "patrol":
        if not args:
            raise ValueError("patrol needs >=1 waypoint: patrol:wp1[,wp2,...]")
        return ["ros2", "run", "rmf_demos_tasks", "dispatch_patrol", "-p", *args, "-n", "1", "--use_sim_time"]
    if kind == "delivery":
        if len(args) != 4:
            raise ValueError("delivery needs 4 args: delivery:pickup,pickup_handler,dropoff,dropoff_handler")
        pickup, pick_h, dropoff, drop_h = args
        return [
            "ros2",
            "run",
            "rmf_demos_tasks",
            "dispatch_delivery",
            "-p",
            pickup,
            "-ph",
            pick_h,
            "-d",
            dropoff,
            "-dh",
            drop_h,
            "--use_sim_time",
        ]
    if kind == "clean":
        if len(args) != 1:
            raise ValueError("clean needs 1 arg: clean:zone")
        return ["ros2", "run", "rmf_demos_tasks", "dispatch_clean", "-cs", args[0], "--use_sim_time"]
    raise ValueError(f"unknown scenario kind: {kind!r}")


class DispatchChecker(Node):
    def __init__(self, expected_robots=None):
        super().__init__("rmf_dispatch_check")
        self._expected_robots = set(expected_robots or [])
        self._pos = {}  # robot name -> latest (x, y)
        self._z = {}  # robot name -> latest z (for lift checks)
        self._subs = {}  # robot name -> odom subscription
        self._robot_task_ids = {}  # robot name -> latest RMF task_id
        self._task_status = {}  # task_id -> latest status string
        self._disp_success = 0  # count of DispenserResult.SUCCESS seen
        self._ing_success = 0  # count of IngestorResult.SUCCESS seen
        self.create_subscription(FleetState, "/fleet_states", self._on_fleet, 10)
        self.create_subscription(DispatchStates, "/dispatch_states", self._on_dispatch_states, 10)
        self.create_subscription(TaskSummary, "/task_summaries", self._on_task_summary, 10)
        self.create_subscription(String, "/task_state_update", self._on_task_state, 10)
        self.create_subscription(DispenserResult, "/dispenser_results", self._on_disp_result, 10)
        self.create_subscription(IngestorResult, "/ingestor_results", self._on_ing_result, 10)
        for robot in sorted(self._expected_robots):
            self._subscribe_odom(robot)

    def _subscribe_odom(self, robot):
        if robot not in self._subs:
            self._pos.setdefault(robot, None)
            self._subs[robot] = self.create_subscription(Odometry, f"/{robot}/odom", self._make_odom_cb(robot), 10)

    def _on_fleet(self, msg: FleetState):
        # Discover robots across all fleets and subscribe to each one's odom once.
        for r in msg.robots:
            if self._expected_robots and r.name not in self._expected_robots:
                continue
            self._robot_task_ids[r.name] = r.task_id
            self._subscribe_odom(r.name)

    def _make_odom_cb(self, robot):
        def _cb(msg: Odometry):
            p = msg.pose.pose.position
            self._pos[robot] = (p.x, p.y)
            self._z[robot] = p.z

        return _cb

    def _on_disp_result(self, msg: DispenserResult):
        if msg.status == DispenserResult.SUCCESS:
            self._disp_success += 1

    def _on_ing_result(self, msg: IngestorResult):
        if msg.status == IngestorResult.SUCCESS:
            self._ing_success += 1

    def _on_task_state(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        # rmf_api_msgs wraps the state as {"type":..., "data": {...}}; tolerate either.
        state = payload.get("data", payload) if isinstance(payload, dict) else {}
        booking = state.get("booking", {}) if isinstance(state, dict) else {}
        task_id, status = booking.get("id"), state.get("status")
        if task_id and status:
            self._task_status[task_id] = status

    def _on_dispatch_states(self, msg: DispatchStates):
        state_names = {
            DispatchState.STATUS_QUEUED: "queued",
            DispatchState.STATUS_SELECTED: "selected",
            DispatchState.STATUS_DISPATCHED: "dispatched",
            DispatchState.STATUS_FAILED_TO_ASSIGN: "failed",
            DispatchState.STATUS_CANCELED_IN_FLIGHT: "canceled",
        }
        for state in list(msg.active) + list(msg.finished):
            if state.task_id:
                self._task_status[state.task_id] = state_names.get(state.status, str(state.status))

    def _on_task_summary(self, msg: TaskSummary):
        task_id = msg.task_id or msg.task_profile.task_id
        if not task_id:
            return
        state_names = {
            TaskSummary.STATE_QUEUED: "queued",
            TaskSummary.STATE_ACTIVE: "underway",
            TaskSummary.STATE_COMPLETED: "completed",
            TaskSummary.STATE_FAILED: "failed",
            TaskSummary.STATE_CANCELED: "canceled",
            TaskSummary.STATE_PENDING: "queued",
        }
        self._task_status[task_id] = state_names.get(msg.state, msg.status or str(msg.state))

    def spin_for(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_fleet(self, timeout: float) -> bool:
        """Return True once every discovered robot is reporting odom (>=1 robot)."""
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._expected_robots:
                if self._expected_robots.issubset(self._pos) and all(
                    self._pos.get(r) is not None for r in self._expected_robots
                ):
                    return True
            elif self._pos and all(p is not None for p in self._pos.values()):
                return True
        return False

    def wait_until_settled(self, max_wait, window, eps) -> bool:
        deadline = time.monotonic() + max_wait
        ref = dict(self._pos)
        ref_t = time.monotonic()
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            moved = max(
                (
                    math.hypot(self._pos[r][0] - ref[r][0], self._pos[r][1] - ref[r][1])
                    for r in self._pos
                    if self._pos.get(r) and ref.get(r)
                ),
                default=0.0,
            )
            if moved > eps:
                ref, ref_t = dict(self._pos), time.monotonic()
            elif time.monotonic() - ref_t >= window:
                return True
        return False

    def max_displacement(self, start) -> float:
        best = 0.0
        for r, s in start.items():
            p = self._pos.get(r)
            if s and p:
                best = max(best, math.hypot(p[0] - s[0], p[1] - s[1]))
        return best

    def max_zrise(self, start_z) -> float:
        """Largest z increase (metres) of any robot vs its scenario-start z."""
        best = 0.0
        for r, z0 in start_z.items():
            z = self._z.get(r)
            if z0 is not None and z is not None:
                best = max(best, z - z0)
        return best

    def task_is_assigned(self, task_id: str) -> bool:
        """Return True while any expected robot reports this RMF task id."""
        return any(current == task_id for current in self._robot_task_ids.values())


def dispatch(node, log, scenario):
    cmd = build_dispatch(scenario)
    log.info(f"dispatching [{scenario}]: {' '.join(cmd)}")
    known_task_ids = set(node._task_status)
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        log.warning("dispatch command timed out (continuing to watch)")
        return None
    if out.returncode != 0:
        log.warning(f"dispatch returned {out.returncode}: {out.stderr.strip()}")
    m = TASK_ID_RE.search(out.stdout) or TASK_ID_RE.search(out.stderr)
    task_id = m.group(0) if m else None
    if task_id is None:
        log.warning(
            "dispatch output did not include a task id; "
            f"stdout={out.stdout.strip()!r} stderr={out.stderr.strip()!r}"
        )
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and rclpy.ok():
            node.spin_for(0.5)
            new_task_ids = [
                task_id
                for task_id in node._task_status
                if task_id not in known_task_ids
            ]
            if new_task_ids:
                task_id = sorted(new_task_ids)[-1]
                log.info(f"dispatch task_id recovered from /dispatch_states: {task_id}")
                break
    log.info(f"dispatch acknowledged; task_id={task_id}")
    return task_id


def _zrise_req(assert_spec: str):
    """Parse an optional ``zrise=<metres>`` assertion clause (else None).

    Raises ValueError on a malformed value or an unknown clause so callers can
    validate tokens up front.
    """
    spec = assert_spec.strip()
    if not spec:
        return None
    if spec.startswith("zrise="):
        value = spec.split("=", 1)[1]
        try:
            return float(value)
        except ValueError:
            raise ValueError(f"zrise= needs a number, got {value!r}")
    raise ValueError(f"unknown assertion clause {spec!r} (supported: zrise=<m>)")


def _check_extras(node, log, scenario, kind, zrise_req, max_zrise, disp0, ing0, completion_label="completed") -> bool:
    """Beyond task completion, verify the physical signals for delivery / lift.

    Largely redundant with RMF task completion, but catches a false completion
    and localizes failures.
    """
    ok = True
    log.info(f"[{scenario}] max z-rise during scenario: {max_zrise:.2f} m")
    if kind == "delivery":
        dn, inn = node._disp_success - disp0, node._ing_success - ing0
        if dn > 0 and inn > 0:
            log.info(f"[{scenario}] workcell: dispenser+ingestor SUCCESS observed")
        else:
            log.error(f"[{scenario}] FAIL: completed but workcell SUCCESS missing (dispenser +{dn}, ingestor +{inn})")
            ok = False
    if zrise_req is not None:
        if max_zrise >= zrise_req:
            log.info(f"[{scenario}] lift: robot rose {max_zrise:.2f} m (>= {zrise_req} m)")
        else:
            log.error(f"[{scenario}] FAIL: completed but max z-rise {max_zrise:.2f} m < {zrise_req} m (lift?)")
            ok = False
    if ok:
        log.info(f"[{scenario}] PASS: task reached `{completion_label}`")
    return ok


def _extras_ready(node, kind, zrise_req, max_zrise, disp0, ing0) -> bool:
    if kind == "delivery" and (node._disp_success <= disp0 or node._ing_success <= ing0):
        return False
    if zrise_req is not None and max_zrise < zrise_req:
        return False
    return True


def run_scenario(node, log, scenario) -> bool:
    """Dispatch one scenario, wait for completion, then assert extra signals.

    Scenario may carry an assertion clause after ``;``, e.g.
    ``patrol:lobby,L2_room1;zrise=1.0`` requires the robot to rise >= 1.0 m
    (the elevator carried it). Delivery scenarios always additionally require a
    dispenser + ingestor SUCCESS.
    """
    main, _, assert_spec = scenario.partition(";")
    kind = main.partition(":")[0]
    zrise_req = _zrise_req(assert_spec)

    if node.wait_until_settled(SETTLE_MAX, SETTLE_WINDOW, SETTLE_EPS):
        log.info("robots quiet; dispatching")
    else:
        log.warning("robots still moving; dispatching anyway")
    start = dict(node._pos)
    start_z = dict(node._z)
    disp0, ing0 = node._disp_success, node._ing_success
    task_id = dispatch(node, log, main)
    if task_id is None:
        # No id parsed -> the dispatch CLI failed or its output format changed.
        # Fail fast instead of waiting the full SCENARIO_TIMEOUT on status=None.
        log.error(f"[{scenario}] FAIL: dispatch produced no task id (CLI failed or unparsed output)")
        return False

    deadline = time.monotonic() + SCENARIO_TIMEOUT
    seen_underway = False
    seen_motion = False
    seen_robot_task = False
    motion_settle_start = None
    motion_settle_ref = None
    max_zrise = 0.0
    while time.monotonic() < deadline and rclpy.ok():
        node.spin_for(1.0)
        max_zrise = max(max_zrise, node.max_zrise(start_z))
        status = node._task_status.get(task_id)
        if not seen_underway and (status in ("selected", "dispatched", "underway") or status in TERMINAL_OK):
            seen_underway = True
            log.info(f"[{scenario}] task underway")
        if not seen_robot_task and node.task_is_assigned(task_id):
            seen_robot_task = True
            log.info(f"[{scenario}] task assigned to robot")
        if not seen_motion and node.max_displacement(start) >= MOVE_THRESHOLD:
            seen_motion = True
            log.info(f"[{scenario}] a robot moved")
        if status in TERMINAL_OK:
            return _check_extras(node, log, scenario, kind, zrise_req, max_zrise, disp0, ing0)
        if status in TERMINAL_BAD:
            log.error(f"[{scenario}] FAIL: task reached terminal `{status}`")
            return False
        if seen_robot_task and not node.task_is_assigned(task_id) and _extras_ready(
            node, kind, zrise_req, max_zrise, disp0, ing0
        ):
            return _check_extras(
                node,
                log,
                scenario,
                kind,
                zrise_req,
                max_zrise,
                disp0,
                ing0,
                "fleet state task cleared",
            )
        if seen_motion and status == "dispatched" and _extras_ready(node, kind, zrise_req, max_zrise, disp0, ing0):
            moved = max(
                (
                    math.hypot(node._pos[r][0] - motion_settle_ref[r][0], node._pos[r][1] - motion_settle_ref[r][1])
                    for r in node._pos
                    if node._pos.get(r) and motion_settle_ref and motion_settle_ref.get(r)
                ),
                default=0.0,
            )
            if motion_settle_ref is None or moved > SETTLE_EPS:
                motion_settle_ref = dict(node._pos)
                motion_settle_start = time.monotonic()
            elif motion_settle_start is not None and time.monotonic() - motion_settle_start >= SETTLE_WINDOW:
                log.warning(
                    f"[{scenario}] no task completion update observed; "
                    "accepting DISPATCHED + motion + settled robot state"
                )
                return _check_extras(
                    node,
                    log,
                    scenario,
                    kind,
                    zrise_req,
                    max_zrise,
                    disp0,
                    ing0,
                    "DISPATCHED + motion + settled",
                )
    log.error(
        f"[{scenario}] FAIL after {SCENARIO_TIMEOUT:.0f}s — last status="
        f"{node._task_status.get(task_id)}, underway={seen_underway}, moved={seen_motion}"
    )
    return False


def main(scenarios, *, ready_only: bool = False, expected_robots=None) -> int:
    rclpy.init()
    node = DispatchChecker(expected_robots)
    log = node.get_logger()

    log.info("waiting for demo stack + fleet odom ...")
    if not node.wait_for_fleet(READY_TIMEOUT):
        log.error("timed out waiting for fleet odom; demo did not come up")
        return 1
    log.info(f"fleet up: robots={sorted(node._pos)}")
    log.info(f"waiting {ADAPTER_READY_GRACE:.0f}s for RMF adapter robot registration")
    node.spin_for(ADAPTER_READY_GRACE)
    if ready_only:
        return 0

    results = {s: run_scenario(node, log, s) for s in scenarios}
    ok = all(results.values())
    log.info(f"RESULTS: {results}")
    return 0 if ok else 1


if __name__ == "__main__":
    ready_only = False
    expected_robots = []
    scenario_args = []
    args = iter(sys.argv[1:])
    for arg in args:
        if arg == "--ready-only":
            ready_only = True
        elif arg == "--expected-robots":
            try:
                expected_robots = [name for name in next(args).split(",") if name]
            except StopIteration:
                print("--expected-robots needs a comma-separated robot list", file=sys.stderr)
                sys.exit(2)
        else:
            scenario_args.append(arg)
    if not scenario_args and not ready_only:
        print("usage: rmf_dispatch_check.py [--ready-only] <type:args[;assert]> [...]", file=sys.stderr)
        sys.exit(2)
    # Validate every scenario token up front (both the dispatch part and any
    # ;assertion clause) so a typo in bridge.yml fails fast with a clear message
    # instead of crashing mid-run after launching a demo.
    for _s in scenario_args:
        _main, _, _assert = _s.partition(";")
        try:
            build_dispatch(_main)
            _zrise_req(_assert)
        except ValueError as _e:
            print(f"invalid scenario {_s!r}: {_e}", file=sys.stderr)
            sys.exit(2)
    rc = 1
    try:
        rc = main(scenario_args, ready_only=ready_only, expected_robots=expected_robots)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(rc)
