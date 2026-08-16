#!/usr/bin/env python3
"""Measure a generated fleet-scale YAML without constructing a ROS bridge."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from pybullet_fleet import MultiRobotSimulationCore
from pybullet_fleet.config_utils import load_yaml_config


def _run_for_wall_time(sim: MultiRobotSimulationCore, seconds: float) -> tuple[float, int]:
    start_wall = time.monotonic()
    start_sim = sim.sim_time
    deadline = start_wall + seconds
    steps = 0
    while time.monotonic() < deadline:
        sim.step_once()
        steps += 1
    return sim.sim_time - start_sim, steps


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, help="Generated bridge-scale YAML")
    parser.add_argument("--warmup", type=float, default=1.0, help="Wall-clock warm-up seconds")
    parser.add_argument("--duration", type=float, default=6.0, help="Wall-clock measurement seconds")
    args = parser.parse_args()

    if args.warmup < 0 or args.duration <= 0:
        parser.error("--warmup must be non-negative and --duration must be positive")

    sim = MultiRobotSimulationCore.from_dict(load_yaml_config(args.config))
    sim.initialize_simulation()
    if args.warmup:
        _run_for_wall_time(sim, args.warmup)
    start_wall = time.monotonic()
    sim_elapsed, steps = _run_for_wall_time(sim, args.duration)
    wall_elapsed = time.monotonic() - start_wall
    print(
        "CORE_ONLY_RTF "
        f"sim={sim_elapsed:.3f}s wall={wall_elapsed:.3f}s steps={steps} rtf={sim_elapsed / wall_elapsed:.2f}x"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
