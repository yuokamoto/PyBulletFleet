#!/usr/bin/env python3
"""Check an already-running ROS bridge fleet API scale scenario."""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from pybullet_fleet_msgs.msg import FleetNavigate as FleetNavigateMsg
from pybullet_fleet_msgs.msg import FleetState, RobotGoal2D
from pybullet_fleet_msgs.srv import FleetNavigate as FleetNavigateSrv
from rosgraph_msgs.msg import Clock
from simulation_interfaces.srv import GetEntitiesStates

from ros_check_utils import RosCheckNode


class FleetScaleClient(RosCheckNode):
    def __init__(self) -> None:
        super().__init__("pybullet_fleet_scale_check")
        self.names: set[str] = set()
        self.positions: dict[str, tuple[float, float]] = {}
        self._clock_samples: list[tuple[float, float]] = []
        self.create_subscription(FleetState, "/fleet/states", self._on_fleet_state, 10)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.navigate = self.create_client(FleetNavigateSrv, "/fleet/navigate")
        self.navigate_pub = self.create_publisher(FleetNavigateMsg, "/fleet/navigate", 10)
        self.get_entities_states = self.create_client(GetEntitiesStates, "/sim/get_entities_states")

    def _on_fleet_state(self, msg: FleetState) -> None:
        self.names.update(robot.name for robot in msg.robots)
        for robot in msg.robots:
            self.positions[robot.name] = (robot.pose.position.x, robot.pose.position.y)

    def _on_clock(self, msg: Clock) -> None:
        sim_time = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        self._clock_samples.append((time.perf_counter(), sim_time))

    def call_navigate(self, goals: list[RobotGoal2D], timeout: float) -> FleetNavigateSrv.Response:
        request = FleetNavigateSrv.Request()
        self._stamp_command(request)
        request.command_id = "scale-nav"
        request.source = "fleet-scale-check"
        request.goals_2d = goals
        return self.call_service(self.navigate, request, timeout)

    def publish_fleet_navigate(self, goals: list[RobotGoal2D], timeout: float) -> tuple[float, int]:
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
        request.command_id = "scale-nav-topic"
        request.source = "fleet-scale-check"
        request.goals_2d = goals
        start = time.perf_counter()
        self.navigate_pub.publish(request)
        rclpy.spin_once(self, timeout_sec=0.05)
        return time.perf_counter() - start, matched

    def _stamp_command(self, msg) -> None:
        msg.header.frame_id = "odom"
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
) -> tuple[bool, float, int]:
    """Wait until every robot has moved measurably toward its x-offset goal."""
    expected_targets = _target_x_by_name(robot_count)
    threshold = 0.01
    start = time.perf_counter()
    last_count = 0

    def count_moved_from_positions(positions: dict[str, tuple[float, float]]) -> int:
        moved = 0
        for name, target_x in expected_targets.items():
            pos = positions.get(name)
            if pos is not None and pos[0] >= target_x - 0.5 + threshold:
                moved += 1
        return moved

    while time.perf_counter() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        if prefer_fleet_state:
            last_count = count_moved_from_positions(node.positions)
        else:
            try:
                last_count = count_moved_from_positions(node.poll_entity_positions(timeout=5.0))
            except RuntimeError:
                last_count = 0
        if last_count == robot_count:
            return True, time.perf_counter() - start, last_count
    return False, time.perf_counter() - start, last_count


def _wait_for_fleet_state(node: FleetScaleClient, robot_count: int, timeout: float) -> set[str] | None:
    expected = {_robot_name(i) for i in range(robot_count)}
    print(f"Waiting for /fleet/states with {robot_count} robots...")
    if not node.spin_until(lambda: expected.issubset(node.names), timeout):
        missing = sorted(expected - node.names)
        print(f"FAIL: fleet state missing {len(missing)} robots; first missing={missing[:10]}")
        return None
    return expected


def _check_fleet_navigate_service(node: FleetScaleClient, robot_count: int, timeout: float, verify_motion: bool) -> int:
    expected = _wait_for_fleet_state(node, robot_count, timeout)
    if expected is None:
        return 1

    goals = _make_fleet_goals(robot_count)
    start = time.perf_counter()
    response = node.call_navigate(goals, timeout)
    elapsed = time.perf_counter() - start
    accepted = set(response.ack.accepted_names)
    if accepted != expected or response.ack.rejected_names:
        print(
            "FAIL: /fleet/navigate ack mismatch "
            f"accepted={len(accepted)}, rejected={list(response.ack.rejected_names)[:10]}"
        )
        return 1
    print(f"PASS: /fleet/navigate accepted {len(accepted)} robots in {elapsed:.3f}s")
    if verify_motion:
        ok, motion_elapsed, moved = _wait_for_motion_started(
            node,
            robot_count,
            timeout,
            prefer_fleet_state=True,
        )
        if not ok:
            print(f"FAIL: only {moved}/{robot_count} robots moved after fleet command in {motion_elapsed:.3f}s")
            return 1
        print(f"PASS: {moved} fleet-commanded robots started moving in {motion_elapsed:.3f}s")
    return 0


def _check_fleet_navigate_topic(node: FleetScaleClient, robot_count: int, timeout: float, verify_motion: bool) -> int:
    if _wait_for_fleet_state(node, robot_count, timeout) is None:
        return 1

    goals = _make_fleet_goals(robot_count)
    elapsed, matched = node.publish_fleet_navigate(goals, timeout)
    if matched == 0:
        print("FAIL: /fleet/navigate topic publisher did not match any subscriptions")
        return 1
    print(f"PASS: published /fleet/navigate topic command for {robot_count} robots in {elapsed:.3f}s")
    if verify_motion:
        ok, motion_elapsed, moved = _wait_for_motion_started(
            node,
            robot_count,
            timeout,
            prefer_fleet_state=True,
        )
        if not ok:
            print(f"FAIL: only {moved}/{robot_count} robots moved after fleet topic command in {motion_elapsed:.3f}s")
            return 1
        print(f"PASS: {moved} fleet-topic-commanded robots started moving in {motion_elapsed:.3f}s")
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
        ok, motion_elapsed, moved = _wait_for_motion_started(
            node,
            robot_count,
            timeout,
            prefer_fleet_state=prefer_fleet_state,
        )
        if not ok:
            print(f"FAIL: only {moved}/{robot_count} robots moved after per-robot commands in {motion_elapsed:.3f}s")
            return 1
        print(f"PASS: {moved} per-robot-commanded robots started moving in {motion_elapsed:.3f}s")
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
        help="Command path to exercise; fleet uses /fleet/navigate service, fleet_topic uses the topic",
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
    parser.add_argument("--rtf-warmup", type=float, default=1.0, help="Wall seconds to spin before RTF measurement")
    parser.add_argument("--rtf-duration", type=float, default=10.0, help="Wall seconds to measure observed /clock RTF")
    args = parser.parse_args()

    if args.command_interface in {"fleet", "fleet_topic"} and args.interface_mode == "per_robot":
        parser.error("--command-interface fleet/fleet_topic requires --interface-mode fleet or hybrid")
    if args.command_interface == "per_robot" and args.interface_mode == "fleet":
        parser.error("--command-interface per_robot requires --interface-mode per_robot or hybrid")
    if args.command_interface == "all" and args.interface_mode != "hybrid":
        parser.error("--command-interface all requires --interface-mode hybrid")

    rclpy.init()
    node = FleetScaleClient()
    try:
        print(
            "[runtime] checking ROS fleet scale endpoints: "
            f"robots={args.robots}, interface_mode={args.interface_mode}, command_interface={args.command_interface}"
        )
        if args.command_interface in {"fleet", "all"}:
            rc = _check_fleet_navigate_service(node, args.robots, args.timeout, not args.no_verify_motion)
            if rc != 0:
                return rc
        if args.command_interface == "fleet_topic":
            rc = _check_fleet_navigate_topic(node, args.robots, args.timeout, not args.no_verify_motion)
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
        return 0
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == "__main__":
    sys.exit(main())
