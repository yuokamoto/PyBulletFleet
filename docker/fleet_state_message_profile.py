#!/usr/bin/env python3
"""Microbenchmark the Python-to-ROS FleetState conversion path.

Run inside the bridge image so the generated ``pybullet_fleet_msgs`` bindings
match the ROS environment under test::

    docker compose -f docker/docker-compose.yaml run --rm --no-deps \
      -v "$(pwd)/docker:/docker:ro" bridge \
      python3 /docker/fleet_state_message_profile.py --robots 1000 --cprofile

This intentionally excludes simulation state collection and DDS delivery. It
isolates the work currently recorded as ``fleet_state_message`` and separates
the message construction from ROS serialization.
"""

from __future__ import annotations

import argparse
import cProfile
import gc
import pstats
import statistics
import time
from io import StringIO
from typing import Callable

from rclpy.serialization import serialize_message
from std_msgs.msg import Header

from pybullet_fleet.states import RobotState3D
from pybullet_fleet_ros.fleet_ros_interface import fleet_state_to_msg, robot_state3d_to_msg
from pybullet_fleet_msgs.msg import FleetState, RobotState3D as RobotState3DMsg


def _states(robot_count: int) -> tuple[RobotState3D, ...]:
    return tuple(
        RobotState3D(
            name=f"robot_{index:04d}",
            object_id=index,
            position=(float(index % 50), float(index // 50), 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            linear_velocity=(0.2, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.1),
            is_moving=True,
            battery_soc=0.8,
            is_charging=False,
        )
        for index in range(robot_count)
    )


def _measure(label: str, operation: Callable[[], object], iterations: int) -> None:
    # The output is deliberately retained until the operation returns, matching
    # the allocation and ownership pattern used by the bridge publisher.
    samples_ms: list[float] = []
    gc_was_enabled = gc.isenabled()
    gc.disable()
    try:
        for _ in range(iterations):
            started = time.perf_counter_ns()
            operation()
            samples_ms.append((time.perf_counter_ns() - started) / 1_000_000)
    finally:
        if gc_was_enabled:
            gc.enable()

    ordered = sorted(samples_ms)
    p90_index = min(len(ordered) - 1, int(len(ordered) * 0.9))
    print(
        f"{label}: median={statistics.median(samples_ms):.3f}ms, "
        f"mean={statistics.mean(samples_ms):.3f}ms, p90={ordered[p90_index]:.3f}ms"
    )


def _robot_state3d_to_msg_in_place(state: RobotState3D) -> RobotState3DMsg:
    """Candidate conversion which reuses generated-message nested defaults.

    ROS 2's generated Python constructors eagerly create default nested
    messages even when replacements are supplied. Mutating those defaults
    avoids constructing a second Pose/Twist tree per robot.
    """
    msg = RobotState3DMsg()
    msg.name = state.name
    msg.object_id = int(state.object_id)
    position = msg.pose.position
    position.x = state.position[0]
    position.y = state.position[1]
    position.z = state.position[2]
    orientation = msg.pose.orientation
    orientation.x = state.orientation[0]
    orientation.y = state.orientation[1]
    orientation.z = state.orientation[2]
    orientation.w = state.orientation[3]
    linear = msg.twist.linear
    linear.x = state.linear_velocity[0]
    linear.y = state.linear_velocity[1]
    linear.z = state.linear_velocity[2]
    angular = msg.twist.angular
    angular.x = state.angular_velocity[0]
    angular.y = state.angular_velocity[1]
    angular.z = state.angular_velocity[2]
    msg.is_moving = bool(state.is_moving)
    msg.has_battery_soc = state.battery_soc is not None
    msg.battery_soc = float(state.battery_soc or 0.0)
    msg.has_is_charging = state.is_charging is not None
    msg.is_charging = bool(state.is_charging)
    return msg


def _fleet_state_to_msg_in_place(states: tuple[RobotState3D, ...]) -> FleetState:
    """Build a FleetState with the candidate per-robot conversion."""
    msg = FleetState()
    msg.header = Header(frame_id="odom")
    msg.robots = [_robot_state3d_to_msg_in_place(state) for state in states]
    return msg


def _print_cprofile(operation: Callable[[], object], iterations: int) -> None:
    profiler = cProfile.Profile()
    profiler.enable()
    for _ in range(iterations):
        operation()
    profiler.disable()

    stream = StringIO()
    pstats.Stats(profiler, stream=stream).sort_stats("cumulative").print_stats(20)
    print("\nTop cProfile functions by cumulative time:")
    print(stream.getvalue().rstrip())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, default=1000, help="Number of RobotState3D entries")
    parser.add_argument("--iterations", type=int, default=50, help="Timed conversion repetitions")
    parser.add_argument("--warmup", type=int, default=5, help="Untimed conversion repetitions")
    parser.add_argument("--cprofile", action="store_true", help="Print cProfile output for full FleetState conversion")
    args = parser.parse_args()
    if args.robots < 1 or args.iterations < 1 or args.warmup < 0:
        parser.error("--robots and --iterations must be positive; --warmup must be non-negative")

    states = _states(args.robots)

    def to_robot_messages() -> list[RobotState3DMsg]:
        return [robot_state3d_to_msg(state) for state in states]

    def to_fleet_message() -> FleetState:
        return fleet_state_to_msg(states)

    def to_candidate_fleet_message() -> FleetState:
        return _fleet_state_to_msg_in_place(states)

    def serialize_fleet_message() -> bytes:
        return serialize_message(fleet_state_to_msg(states))

    for _ in range(args.warmup):
        serialize_fleet_message()

    if serialize_message(to_fleet_message()) != serialize_message(to_candidate_fleet_message()):
        raise RuntimeError("in-place conversion candidate does not match the current FleetState wire encoding")

    print(f"FleetState message microbenchmark: robots={args.robots}, iterations={args.iterations}")
    _measure("RobotState3D to per-robot ROS messages", to_robot_messages, args.iterations)
    _measure("Full FleetState conversion", to_fleet_message, args.iterations)
    _measure("Candidate in-place FleetState conversion", to_candidate_fleet_message, args.iterations)
    _measure("Full conversion plus serialize_message", serialize_fleet_message, args.iterations)
    if args.cprofile:
        _print_cprofile(to_fleet_message, args.iterations)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
