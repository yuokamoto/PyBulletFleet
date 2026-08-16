#!/usr/bin/env python3
"""Profile the transport-neutral FleetStateProvider collection path.

This benchmark models the cached kinematic-agent path used by the ROS scale
checks. It intentionally excludes ROS message construction and DDS transport.

Usage::

    python benchmark/profiling/fleet_state_collection.py --robots 1000 --cprofile
    python benchmark/profiling/fleet_state_collection.py --robots 1000 --selected 100
"""

from __future__ import annotations

import argparse
import cProfile
import pstats
import statistics
import time
from dataclasses import dataclass
from io import StringIO

import numpy as np

from pybullet_fleet.fleet_api import FleetStateProvider
from pybullet_fleet.geometry import Pose


@dataclass
class _CachedAgent:
    name: str
    object_id: int
    pose: Pose
    _current_velocity: np.ndarray
    _current_angular_velocity: float = 0.1
    is_moving: bool = True
    battery_soc: float | None = 0.8
    is_charging: bool | None = False

    def get_pose(self) -> Pose:
        return self.pose

    @property
    def velocity(self) -> np.ndarray:
        """Match Agent.velocity, which returns a defensive NumPy copy."""
        return self._current_velocity.copy()

    @property
    def angular_velocity(self) -> float:
        return self._current_angular_velocity


@dataclass
class _Simulation:
    agents: tuple[_CachedAgent, ...]


def _make_provider(robot_count: int) -> FleetStateProvider:
    agents = tuple(
        _CachedAgent(
            name=f"robot_{index:04d}",
            object_id=index,
            pose=Pose.from_xyz(float(index % 50), float(index // 50), 0.0),
            _current_velocity=np.array((0.2, 0.0, 0.0)),
        )
        for index in range(robot_count)
    )
    return FleetStateProvider(_Simulation(agents))


def _measure(provider: FleetStateProvider, names: frozenset[str] | None, iterations: int) -> None:
    samples_ms: list[float] = []
    for _ in range(iterations):
        started = time.perf_counter_ns()
        states = provider.get_states_3d(names=names)
        samples_ms.append((time.perf_counter_ns() - started) / 1_000_000)
    ordered = sorted(samples_ms)
    p90_index = min(len(ordered) - 1, int(len(ordered) * 0.9))
    print(
        f"FleetState collection ({len(states)} states): "
        f"median={statistics.median(samples_ms):.3f}ms, "
        f"mean={statistics.mean(samples_ms):.3f}ms, p90={ordered[p90_index]:.3f}ms"
    )


def _print_cprofile(provider: FleetStateProvider, names: frozenset[str] | None, iterations: int) -> None:
    profiler = cProfile.Profile()
    profiler.enable()
    for _ in range(iterations):
        provider.get_states_3d(names=names)
    profiler.disable()

    stream = StringIO()
    pstats.Stats(profiler, stream=stream).sort_stats("cumulative").print_stats(20)
    print("\nTop cProfile functions by cumulative time:")
    print(stream.getvalue().rstrip())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, default=1000, help="Total cached agents")
    parser.add_argument("--selected", type=int, help="Selected agents; defaults to all")
    parser.add_argument("--iterations", type=int, default=100, help="Timed collection repetitions")
    parser.add_argument("--cprofile", action="store_true", help="Print cProfile output")
    args = parser.parse_args()
    if args.robots < 1 or args.iterations < 1:
        parser.error("--robots and --iterations must be positive")
    if args.selected is not None and not 1 <= args.selected <= args.robots:
        parser.error("--selected must be between 1 and --robots")

    provider = _make_provider(args.robots)
    names = None
    if args.selected is not None:
        names = frozenset(f"robot_{index:04d}" for index in range(args.selected))
    _measure(provider, names, args.iterations)
    if args.cprofile:
        _print_cprofile(provider, names, args.iterations)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
