#!/usr/bin/env python3
"""Check an already-running ROS bridge fleet API scale scenario."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from pybullet_fleet_msgs.msg import FleetNavigate as FleetNavigateMsg
from pybullet_fleet_msgs.msg import FleetState, RobotGoal2D, TransportTiming
from pybullet_fleet_msgs.srv import FleetNavigate as FleetNavigateSrv
from rosgraph_msgs.msg import Clock
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from simulation_interfaces.srv import GetEntitiesStates

from ros_check_utils import RosCheckNode

try:
    from pybullet_fleet_ros.interface_config import FLEET_STATE_QOS_PRESETS
except ModuleNotFoundError:
    _BRIDGE_SOURCE = Path(__file__).resolve().parents[1] / "ros2_bridge/pybullet_fleet_ros"
    if not _BRIDGE_SOURCE.is_dir():
        raise
    sys.path.insert(0, str(_BRIDGE_SOURCE))
    from pybullet_fleet_ros.interface_config import FLEET_STATE_QOS_PRESETS


@dataclass(frozen=True)
class MotionReport:
    ok: bool
    elapsed: float
    moved: int
    first_moved: float | None
    p50_moved: float | None
    p90_moved: float | None
    p99_moved: float | None
    all_moved: float | None
    missing: list[str]
    final_positions: dict[str, tuple[float, float]]


def _fleet_state_qos(config: dict) -> QoSProfile:
    reliability = ReliabilityPolicy.RELIABLE if config["reliability"] == "reliable" else ReliabilityPolicy.BEST_EFFORT
    history = HistoryPolicy.KEEP_LAST if config["history"] == "keep_last" else HistoryPolicy.KEEP_ALL
    durability = DurabilityPolicy.VOLATILE if config["durability"] == "volatile" else DurabilityPolicy.TRANSIENT_LOCAL
    return QoSProfile(history=history, depth=config["depth"], reliability=reliability, durability=durability)


class FleetScaleClient(RosCheckNode):
    def __init__(self, *, state_qos: dict) -> None:
        super().__init__("pybullet_fleet_scale_check")
        self.names: set[str] = set()
        self.positions: dict[str, tuple[float, float]] = {}
        self._clock_samples: list[tuple[float, float]] = []
        self._latest_sim_time: float | None = None
        self._state_probe_by_stamp: dict[tuple[int, int], TransportTiming] = {}
        self._pending_state_receives: dict[tuple[int, int], int] = {}
        self._fleet_state_wall_delays: list[float] = []
        self._fleet_state_payloads: list[tuple[int, int]] = []
        self._fleet_state_probe_count = 0
        self._navigate_probe_by_id: dict[str, TransportTiming] = {}
        self._navigate_sent_ns: dict[str, int] = {}
        self._navigate_received_ns: dict[str, int] = {}
        self._navigate_topic_probe_by_id: dict[str, TransportTiming] = {}
        self._navigate_topic_sent_ns: dict[str, int] = {}
        self._navigate_topic_completed_ids: set[str] = set()
        self._navigate_command_sequence = 0
        self._navigate_wall_delays: dict[str, list[float]] = {
            "request_to_bridge": [],
            "bridge_processing": [],
            "bridge_to_client": [],
            "round_trip": [],
        }
        self._navigate_topic_wall_delays: dict[str, list[float]] = {
            "publish_to_bridge": [],
            "bridge_processing": [],
        }
        self.create_subscription(
            FleetState,
            "/fleet/states",
            self._on_fleet_state,
            _fleet_state_qos(state_qos),
        )
        self.create_subscription(TransportTiming, "/fleet/transport_timing", self._on_transport_timing, 10)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.navigate = self.create_client(FleetNavigateSrv, "/fleet/navigate")
        self.navigate_pub = self.create_publisher(FleetNavigateMsg, "/fleet/navigate", 10)
        self.get_entities_states = self.create_client(GetEntitiesStates, "/sim/get_entities_states")

    def _on_fleet_state(self, msg: FleetState) -> None:
        receive_wall_ns = time.monotonic_ns()
        self.names.update(robot.name for robot in msg.robots)
        for robot in msg.robots:
            self.positions[robot.name] = (robot.pose.position.x, robot.pose.position.y)
        stamp_key = _time_msg_key(msg.header.stamp)
        self._pending_state_receives[stamp_key] = receive_wall_ns
        self._match_state_probe(stamp_key)

    def _on_transport_timing(self, msg: TransportTiming) -> None:
        if msg.channel == "/fleet/states":
            self._fleet_state_probe_count += 1
            stamp_key = _time_msg_key(msg.source_sim_time)
            self._state_probe_by_stamp[stamp_key] = msg
            self._match_state_probe(stamp_key)
        elif msg.channel == "/fleet/navigate":
            self._navigate_probe_by_id[msg.correlation_id] = msg
            self._match_navigate_probe(msg.correlation_id)
        elif msg.channel == "/fleet/navigate_topic":
            self._navigate_topic_probe_by_id[msg.correlation_id] = msg
            self._match_navigate_topic_probe(msg.correlation_id)

    def _match_state_probe(self, stamp_key: tuple[int, int]) -> None:
        probe = self._state_probe_by_stamp.get(stamp_key)
        receive_ns = self._pending_state_receives.get(stamp_key)
        if probe is None or receive_ns is None:
            return
        if receive_ns >= probe.source_monotonic_ns:
            self._fleet_state_wall_delays.append((receive_ns - probe.source_monotonic_ns) / 1e9)
            self._fleet_state_payloads.append((int(probe.item_count), int(probe.payload_bytes)))
        self._state_probe_by_stamp.pop(stamp_key, None)
        self._pending_state_receives.pop(stamp_key, None)

    def _match_navigate_probe(self, command_id: str) -> None:
        probe = self._navigate_probe_by_id.get(command_id)
        send_ns = self._navigate_sent_ns.get(command_id)
        receive_ns = self._navigate_received_ns.get(command_id)
        if probe is None or send_ns is None or receive_ns is None:
            return
        if probe.source_monotonic_ns >= send_ns:
            self._navigate_wall_delays["request_to_bridge"].append((probe.source_monotonic_ns - send_ns) / 1e9)
        if probe.end_monotonic_ns >= probe.source_monotonic_ns:
            self._navigate_wall_delays["bridge_processing"].append((probe.end_monotonic_ns - probe.source_monotonic_ns) / 1e9)
        if receive_ns >= probe.end_monotonic_ns:
            self._navigate_wall_delays["bridge_to_client"].append((receive_ns - probe.end_monotonic_ns) / 1e9)
        self._navigate_probe_by_id.pop(command_id, None)

    def _match_navigate_topic_probe(self, command_id: str) -> None:
        probe = self._navigate_topic_probe_by_id.get(command_id)
        send_ns = self._navigate_topic_sent_ns.get(command_id)
        if probe is None or send_ns is None:
            return
        if probe.source_monotonic_ns >= send_ns:
            self._navigate_topic_wall_delays["publish_to_bridge"].append((probe.source_monotonic_ns - send_ns) / 1e9)
        if probe.end_monotonic_ns >= probe.source_monotonic_ns:
            self._navigate_topic_wall_delays["bridge_processing"].append(
                (probe.end_monotonic_ns - probe.source_monotonic_ns) / 1e9
            )
        self._navigate_topic_completed_ids.add(command_id)
        self._navigate_topic_probe_by_id.pop(command_id, None)
        self._navigate_topic_sent_ns.pop(command_id, None)

    def _on_clock(self, msg: Clock) -> None:
        sim_time = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        self._latest_sim_time = sim_time
        self._clock_samples.append((time.perf_counter(), sim_time))

    def call_navigate(self, goals: list[RobotGoal2D], timeout: float) -> FleetNavigateSrv.Response:
        request = FleetNavigateSrv.Request()
        self._stamp_command(request)
        self._navigate_command_sequence += 1
        request.command_id = f"scale-nav-{self._navigate_command_sequence}"
        request.source = "fleet-scale-check"
        request.goals_2d = goals
        client_publish_ns = time.monotonic_ns()
        self._navigate_sent_ns[request.command_id] = client_publish_ns
        response = self.call_service(self.navigate, request, timeout)
        client_receive_ns = time.monotonic_ns()
        self._navigate_wall_delays["round_trip"].append((client_receive_ns - client_publish_ns) / 1e9)
        self._navigate_received_ns[request.command_id] = client_receive_ns
        self._match_navigate_probe(request.command_id)
        return response

    def publish_fleet_navigate(self, goals: list[RobotGoal2D], timeout: float) -> tuple[str, int]:
        match_start = time.perf_counter()
        while time.perf_counter() - match_start < timeout:
            matched = self.navigate_pub.get_subscription_count()
            if matched > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        else:
            matched = self.navigate_pub.get_subscription_count()

        request = FleetNavigateMsg()
        self._stamp_command(request)
        self._navigate_command_sequence += 1
        request.command_id = f"scale-nav-topic-{self._navigate_command_sequence}"
        request.source = "fleet-scale-check"
        request.goals_2d = goals
        self._navigate_topic_sent_ns[request.command_id] = time.monotonic_ns()
        self.navigate_pub.publish(request)
        return request.command_id, matched

    def wait_for_navigate_topic_probe(self, command_id: str, timeout: float) -> bool:
        return self.spin_until(lambda: command_id in self._navigate_topic_completed_ids, timeout)

    def _stamp_command(self, msg) -> None:
        msg.header.frame_id = "odom"
        if self._latest_sim_time is not None:
            seconds = int(self._latest_sim_time)
            nanoseconds = int(round((self._latest_sim_time - seconds) * 1e9))
            if nanoseconds >= 1_000_000_000:
                seconds += 1
                nanoseconds -= 1_000_000_000
            msg.header.stamp.sec = seconds
            msg.header.stamp.nanosec = nanoseconds
        else:
            msg.header.stamp = self.get_clock().now().to_msg()

    def publish_per_robot_goals(
        self,
        goals: list[tuple[str, PoseStamped]],
        timeout: float,
        repeats: int,
        batch_size: int,
    ) -> tuple[float, int]:
        publishers = [self.create_publisher(PoseStamped, f"/{name}/goal_pose", 10) for name, _ in goals]
        match_start = time.perf_counter()
        while time.perf_counter() - match_start < timeout:
            matched = sum(1 for publisher in publishers if publisher.get_subscription_count() > 0)
            if matched == len(publishers):
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        else:
            matched = sum(1 for publisher in publishers if publisher.get_subscription_count() > 0)

        start = time.perf_counter()
        for _ in range(repeats):
            for index, (publisher, (_, goal)) in enumerate(zip(publishers, goals), start=1):
                publisher.publish(goal)
                if batch_size > 0 and index % batch_size == 0:
                    rclpy.spin_once(self, timeout_sec=0.01)
            rclpy.spin_once(self, timeout_sec=0.05)
        return time.perf_counter() - start, matched

    def poll_entity_positions(self, timeout: float) -> dict[str, tuple[float, float]]:
        response = self.call_service(self.get_entities_states, GetEntitiesStates.Request(), timeout)
        return {
            name: (state.pose.position.x, state.pose.position.y) for name, state in zip(response.entities, response.states)
        }

    def measure_rtf(self, *, warmup: float, duration: float, timeout: float) -> tuple[float, float, float]:
        if not self.spin_until(lambda: bool(self._clock_samples), timeout):
            raise RuntimeError("no /clock samples received")

        if warmup > 0.0:
            self.spin_for(warmup)

        self._clock_samples.clear()
        start_wall = time.perf_counter()
        self.spin_for(duration)
        end_wall = time.perf_counter()

        if len(self._clock_samples) < 2:
            raise RuntimeError("insufficient /clock samples for RTF measurement")

        first_wall, first_sim = self._clock_samples[0]
        last_wall, last_sim = self._clock_samples[-1]
        wall_delta = max(last_wall - first_wall, end_wall - start_wall, 1e-9)
        sim_delta = last_sim - first_sim
        if sim_delta <= 0.0:
            raise RuntimeError(f"/clock did not advance during RTF measurement: sim_delta={sim_delta:.6f}s")
        return sim_delta / wall_delta, sim_delta, wall_delta


def _robot_name(index: int) -> str:
    return f"robot_{index}"


def _make_fleet_goals(robot_count: int) -> list[RobotGoal2D]:
    side = int(math.ceil(math.sqrt(robot_count)))
    goals = []
    for index in range(robot_count):
        start_x = float(index % side) * 2.0
        start_y = float(index // side) * 2.0
        goals.append(
            RobotGoal2D(
                name=_robot_name(index),
                position=[start_x + 0.5, start_y],
                yaw=0.0,
                z=0.05,
            )
        )
    return goals


def _make_per_robot_goals(robot_count: int) -> list[tuple[str, PoseStamped]]:
    side = int(math.ceil(math.sqrt(robot_count)))
    goals = []
    for index in range(robot_count):
        start_x = float(index % side) * 2.0
        start_y = float(index // side) * 2.0
        name = _robot_name(index)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = start_x + 0.5
        goal.pose.position.y = start_y
        goal.pose.position.z = 0.05
        goal.pose.orientation.w = 1.0
        goals.append((name, goal))
    return goals


def _target_x_by_name(robot_count: int) -> dict[str, float]:
    side = int(math.ceil(math.sqrt(robot_count)))
    return {f"robot_{index}": float(index % side) * 2.0 + 0.5 for index in range(robot_count)}


def _wait_for_motion_started(
    node: FleetScaleClient,
    robot_count: int,
    timeout: float,
    *,
    prefer_fleet_state: bool,
) -> MotionReport:
    """Measure how long each robot takes to move after the relevant command boundary."""
    expected_targets = _target_x_by_name(robot_count)
    threshold = 0.01
    start = time.perf_counter()
    moved_at: dict[str, float] = {}
    last_positions: dict[str, tuple[float, float]] = {}

    def record_moved_from_positions(positions: dict[str, tuple[float, float]], elapsed: float) -> None:
        for name, target_x in expected_targets.items():
            pos = positions.get(name)
            if pos is not None and pos[0] >= target_x - 0.5 + threshold and name not in moved_at:
                moved_at[name] = elapsed

    def percentile(values: list[float], ratio: float) -> float | None:
        if not values:
            return None
        index = min(len(values) - 1, math.ceil(len(values) * ratio) - 1)
        return values[index]

    while time.perf_counter() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        if prefer_fleet_state:
            last_positions = dict(node.positions)
        else:
            try:
                remaining = max(timeout - (time.perf_counter() - start), 0.1)
                last_positions = node.poll_entity_positions(timeout=remaining)
            except RuntimeError:
                pass

        elapsed = time.perf_counter() - start
        record_moved_from_positions(last_positions, elapsed)
        if len(moved_at) == robot_count:
            break

    elapsed = time.perf_counter() - start
    moved_times = sorted(moved_at.values())
    missing = sorted(set(expected_targets) - set(moved_at))
    return MotionReport(
        ok=not missing,
        elapsed=elapsed,
        moved=len(moved_at),
        first_moved=moved_times[0] if moved_times else None,
        p50_moved=percentile(moved_times, 0.50),
        p90_moved=percentile(moved_times, 0.90),
        p99_moved=percentile(moved_times, 0.99),
        all_moved=moved_times[-1] if len(moved_times) == robot_count else None,
        missing=missing,
        final_positions=last_positions,
    )


def _format_latency(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.3f}s"


def _time_msg_key(stamp) -> tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


def _print_transport_report(node: FleetScaleClient) -> None:
    """Report probe-based same-host ROS transport timing."""

    def summary(values: list[float]) -> str:
        if not values:
            return "samples=0"
        ordered = sorted(values)

        def percentile(ratio: float) -> float:
            index = min(len(ordered) - 1, math.ceil(len(ordered) * ratio) - 1)
            return ordered[index]

        return (
            f"samples={len(values)}, min={ordered[0]:.6f}s, "
            f"p50={percentile(0.50):.6f}s, p90={percentile(0.90):.6f}s, "
            f"p99={percentile(0.99):.6f}s, max={ordered[-1]:.6f}s"
        )

    print(f"WALL /fleet/states publish-to-receive: {summary(node._fleet_state_wall_delays)}")
    if node._fleet_state_payloads:
        counts = sorted({count for count, _ in node._fleet_state_payloads})
        sizes = [size for _, size in node._fleet_state_payloads]
        print(f"FleetState payload: robots={counts}, bytes={min(sizes)}..{max(sizes)}")
    if node._fleet_state_probe_count:
        delivered = len(node._fleet_state_wall_delays)
        print(
            f"FleetState probe matches: {delivered}/{node._fleet_state_probe_count} "
            f"({delivered / node._fleet_state_probe_count:.1%})"
        )
    for label, values in node._navigate_wall_delays.items():
        print(f"WALL /fleet/navigate {label.replace('_', '-')}: {summary(values)}")
    for label, values in node._navigate_topic_wall_delays.items():
        print(f"WALL /fleet/navigate topic {label.replace('_', '-')}: {summary(values)}")
    print(
        "NOTE: timing uses a separate probe message and same-host monotonic clocks; "
        "cross-host measurements require synchronized clocks."
    )


def _measure_transport(node: FleetScaleClient, warmup: float, duration: float) -> None:
    print(f"Measuring probe-based transport timing for {duration:.1f}s after {warmup:.1f}s warmup...")
    node.spin_for(warmup)
    node._fleet_state_wall_delays.clear()
    node._fleet_state_payloads.clear()
    node._state_probe_by_stamp.clear()
    node._pending_state_receives.clear()
    node._fleet_state_probe_count = 0
    node.spin_for(duration)


def _format_missing_position(report: MotionReport, name: str) -> str:
    pos = report.final_positions.get(name)
    if pos is None:
        return "missing"
    return f"({pos[0]:.3f}, {pos[1]:.3f})"


def _print_motion_report(label: str, report: MotionReport, robot_count: int, *, motion_boundary: str) -> None:
    print(
        f"{label}: motion after {motion_boundary} moved={report.moved}/{robot_count}, "
        f"first={_format_latency(report.first_moved)}, "
        f"p50={_format_latency(report.p50_moved)}, "
        f"p90={_format_latency(report.p90_moved)}, "
        f"p99={_format_latency(report.p99_moved)}, "
        f"all={_format_latency(report.all_moved)}, "
        f"elapsed={report.elapsed:.3f}s"
    )
    if report.missing:
        samples = [f"{name}@{_format_missing_position(report, name)}" for name in report.missing[:10]]
        print(f"{label}: robots that did not move={len(report.missing)}; sample={samples}")


def _wait_for_fleet_state(node: FleetScaleClient, robot_count: int, timeout: float) -> set[str] | None:
    expected = {_robot_name(i) for i in range(robot_count)}
    print(f"Waiting for /fleet/states with {robot_count} robots...")
    if not node.spin_until(lambda: expected.issubset(node.names), timeout):
        missing = sorted(expected - node.names)
        print(f"FAIL: fleet state missing {len(missing)} robots; first missing={missing[:10]}")
        return None
    return expected


def _check_fleet_navigate_service(
    node: FleetScaleClient,
    robot_count: int,
    timeout: float,
    verify_motion: bool,
    repeats: int,
) -> int:
    expected = _wait_for_fleet_state(node, robot_count, timeout)
    if expected is None:
        return 1

    goals = _make_fleet_goals(robot_count)
    start = time.perf_counter()
    for attempt in range(1, repeats + 1):
        response = node.call_navigate(goals, timeout)
        accepted = set(response.ack.accepted_names)
        if accepted != expected or response.ack.rejected_names:
            print(
                "FAIL: /fleet/navigate ack mismatch "
                f"attempt={attempt}/{repeats}, accepted={len(accepted)}, "
                f"rejected={list(response.ack.rejected_names)[:10]}"
            )
            return 1
    elapsed = time.perf_counter() - start
    print(f"PASS: /fleet/navigate accepted {robot_count} robots in {repeats} " f"request(s), total={elapsed:.3f}s")
    if verify_motion:
        report = _wait_for_motion_started(
            node,
            robot_count,
            timeout,
            prefer_fleet_state=True,
        )
        _print_motion_report("PASS" if report.ok else "FAIL", report, robot_count, motion_boundary="ack")
        if not report.ok:
            return 1
    return 0


def _check_fleet_navigate_topic(
    node: FleetScaleClient,
    robot_count: int,
    timeout: float,
    verify_motion: bool,
    repeats: int,
    measure_transport: bool,
) -> int:
    if _wait_for_fleet_state(node, robot_count, timeout) is None:
        return 1

    goals = _make_fleet_goals(robot_count)
    start = time.perf_counter()
    for attempt in range(1, repeats + 1):
        command_id, matched = node.publish_fleet_navigate(goals, timeout)
        if matched == 0:
            print("FAIL: /fleet/navigate topic publisher did not match any subscriptions")
            return 1
        if measure_transport and not node.wait_for_navigate_topic_probe(command_id, timeout):
            print(f"FAIL: no transport probe for /fleet/navigate topic attempt {attempt}/{repeats}")
            return 1
    elapsed = time.perf_counter() - start
    print(
        f"PASS: published /fleet/navigate topic command for {robot_count} robots in {repeats} "
        f"request(s), total={elapsed:.3f}s"
    )
    if verify_motion:
        report = _wait_for_motion_started(
            node,
            robot_count,
            timeout,
            prefer_fleet_state=True,
        )
        _print_motion_report(
            "PASS" if report.ok else "FAIL",
            report,
            robot_count,
            motion_boundary="command publication",
        )
        if not report.ok:
            return 1
    return 0


def _check_per_robot_goal_pose_topics(
    node: FleetScaleClient,
    robot_count: int,
    timeout: float,
    verify_motion: bool,
    *,
    prefer_fleet_state: bool,
    publish_repeats: int,
    publish_batch_size: int,
) -> int:
    expected = {_robot_name(i) for i in range(robot_count)}
    topics = {f"/{name}/goal_pose" for name in expected}
    print(f"Waiting for {len(topics)} per-robot goal_pose subscriptions...")
    if not node.spin_until(lambda: topics.issubset(node.present_topics()), timeout):
        missing = sorted(topics - node.present_topics())
        print(f"FAIL: per-robot topics missing {len(missing)}; first missing={missing[:10]}")
        return 1
    elapsed, matched = node.publish_per_robot_goals(
        _make_per_robot_goals(robot_count),
        timeout,
        publish_repeats,
        publish_batch_size,
    )
    if matched != robot_count:
        print(f"FAIL: only {matched}/{robot_count} per-robot publishers matched subscriptions")
        return 1
    print(f"PASS: published {robot_count} per-robot goal_pose commands " f"({publish_repeats} repeat(s)) in {elapsed:.3f}s")
    if verify_motion:
        report = _wait_for_motion_started(
            node,
            robot_count,
            timeout,
            prefer_fleet_state=prefer_fleet_state,
        )
        _print_motion_report(
            "PASS" if report.ok else "FAIL",
            report,
            robot_count,
            motion_boundary="command publication",
        )
        if not report.ok:
            return 1
    return 0


def _measure_rtf(node: FleetScaleClient, warmup: float, duration: float, timeout: float) -> int:
    print(f"Measuring /clock RTF for {duration:.1f}s after {warmup:.1f}s warmup...")
    try:
        rtf, sim_delta, wall_delta = node.measure_rtf(warmup=warmup, duration=duration, timeout=timeout)
    except RuntimeError as exc:
        print(f"FAIL: {exc}")
        return 1
    print(f"PASS: max RTF {rtf:.2f}x (sim={sim_delta:.3f}s, wall={wall_delta:.3f}s)")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, default=100, help="Number of robots expected in the running bridge")
    parser.add_argument("--timeout", type=float, default=60.0, help="Readiness and service timeout")
    parser.add_argument(
        "--interface-mode",
        choices=["fleet", "hybrid", "per_robot"],
        default="fleet",
        help="ROS interfaces to create",
    )
    parser.add_argument(
        "--command-interface",
        choices=["fleet", "fleet_topic", "per_robot", "all"],
        default="fleet",
        help=(
            "Advanced measurement option: command path exercised by the checker. "
            "Usually match --interface-mode; mismatches are for hybrid overhead/debug comparisons."
        ),
    )
    parser.add_argument("--no-verify-motion", action="store_true", help="Skip waiting for commanded robots to move")
    parser.add_argument(
        "--per-robot-publish-repeats",
        type=int,
        default=3,
        help="Number of times to publish each per-robot goal command",
    )
    parser.add_argument(
        "--per-robot-publish-batch-size",
        type=int,
        default=0,
        help="Spin after each batch of per-robot publishes; 0 publishes one full burst per repeat",
    )
    parser.add_argument("--measure-rtf", action="store_true", help="Measure observed /clock RTF after command checks")
    parser.add_argument(
        "--fleet-service-repeats",
        type=int,
        default=1,
        help="Number of full-fleet /fleet/navigate service requests to measure",
    )
    parser.add_argument(
        "--fleet-topic-repeats",
        type=int,
        default=1,
        help="Number of full-fleet /fleet/navigate topic commands to measure",
    )
    parser.add_argument(
        "--measure-transport",
        action="store_true",
        help="Report probe-based same-host transport timing",
    )
    parser.add_argument(
        "--transport-warmup",
        type=float,
        default=1.0,
        help="Wall seconds to spin before transport timing measurement",
    )
    parser.add_argument(
        "--transport-duration",
        type=float,
        default=10.0,
        help="Wall seconds to collect FleetState transport samples",
    )
    parser.add_argument("--rtf-warmup", type=float, default=1.0, help="Wall seconds to spin before RTF measurement")
    parser.add_argument("--rtf-duration", type=float, default=10.0, help="Wall seconds to measure observed /clock RTF")
    parser.add_argument(
        "--state-qos-preset",
        choices=sorted(FLEET_STATE_QOS_PRESETS),
        default="fleet_state_reliable",
        help="Evaluated FleetState QoS profile",
    )
    parser.add_argument(
        "--state-qos-reliability",
        choices=["reliable", "best_effort"],
        default=None,
        help="Override QoS reliability for /fleet/states",
    )
    parser.add_argument(
        "--state-qos-history",
        choices=["keep_last", "keep_all"],
        default=None,
        help="Override QoS history policy for /fleet/states",
    )
    parser.add_argument("--state-qos-depth", type=int, default=None, help="Override QoS depth for /fleet/states")
    parser.add_argument(
        "--state-qos-durability",
        choices=["volatile", "transient_local"],
        default=None,
        help="Override QoS durability for /fleet/states",
    )
    args = parser.parse_args()

    if args.command_interface in {"fleet", "fleet_topic"} and args.interface_mode == "per_robot":
        parser.error("--command-interface fleet/fleet_topic requires --interface-mode fleet or hybrid")
    if args.command_interface == "per_robot" and args.interface_mode == "fleet":
        parser.error("--command-interface per_robot requires --interface-mode per_robot or hybrid")
    if args.command_interface == "all" and args.interface_mode != "hybrid":
        parser.error("--command-interface all requires --interface-mode hybrid")
    if args.fleet_service_repeats < 1:
        parser.error("--fleet-service-repeats must be at least 1")
    if args.fleet_topic_repeats < 1:
        parser.error("--fleet-topic-repeats must be at least 1")
    if args.state_qos_depth is not None and args.state_qos_depth < 1:
        parser.error("--state-qos-depth must be at least 1")

    preset = FLEET_STATE_QOS_PRESETS[args.state_qos_preset]
    state_qos = {
        "reliability": preset.reliability.name.lower(),
        "history": preset.history.name.lower(),
        "depth": preset.depth,
        "durability": preset.durability.name.lower(),
    }
    for key in ("reliability", "history", "depth", "durability"):
        value = getattr(args, f"state_qos_{key}")
        if value is not None:
            state_qos[key] = value

    rclpy.init()
    node = FleetScaleClient(state_qos=state_qos)
    try:
        print(
            "[runtime] checking ROS fleet scale endpoints: "
            f"robots={args.robots}, interface_mode={args.interface_mode}, command_interface={args.command_interface}"
        )
        if args.command_interface in {"fleet", "all"}:
            rc = _check_fleet_navigate_service(
                node,
                args.robots,
                args.timeout,
                not args.no_verify_motion,
                args.fleet_service_repeats,
            )
            if rc != 0:
                return rc
        if args.command_interface == "fleet_topic":
            rc = _check_fleet_navigate_topic(
                node,
                args.robots,
                args.timeout,
                not args.no_verify_motion,
                args.fleet_topic_repeats,
                args.measure_transport,
            )
            if rc != 0:
                return rc
        if args.command_interface in {"per_robot", "all"}:
            rc = _check_per_robot_goal_pose_topics(
                node,
                args.robots,
                args.timeout,
                not args.no_verify_motion,
                prefer_fleet_state=args.interface_mode == "hybrid",
                publish_repeats=args.per_robot_publish_repeats,
                publish_batch_size=args.per_robot_publish_batch_size,
            )
            if rc != 0:
                return rc
        if args.measure_rtf:
            rc = _measure_rtf(node, args.rtf_warmup, args.rtf_duration, args.timeout)
            if rc != 0:
                return rc
        if args.measure_transport:
            _measure_transport(node, args.transport_warmup, args.transport_duration)
            _print_transport_report(node)
        return 0
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == "__main__":
    sys.exit(main())
