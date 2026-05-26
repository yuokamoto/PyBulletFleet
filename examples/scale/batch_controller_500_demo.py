#!/usr/bin/env python3
"""
batch_controller_500_demo.py — Showcase BatchOmni + BatchDifferential at scale.

Spawns 500 robots in a grid and drives them with one of the batch controllers
introduced in Phase B of the two-phase-step refactor. Measures step time vs the
per-agent baseline so reviewers can see the speedup live.

The example mirrors the public APIs documented in
``pybullet_fleet/controllers/batch_*.py`` and the perf benchmarks under
``benchmark/batch_*_perf.py``.

Usage
-----

Default (BatchOmniController, headless, collision on)::

    python3 examples/scale/batch_controller_500_demo.py

Differential drive flavor with GUI::

    python3 examples/scale/batch_controller_500_demo.py --mode diff --gui

Skip the comparison run (only batched)::

    python3 examples/scale/batch_controller_500_demo.py --no-compare
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

# Allow running directly from a checkout without installing.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import pybullet as p  # noqa: E402

from pybullet_fleet import (  # noqa: E402
    Agent,
    AgentSpawnParams,
    MotionMode,
    MultiRobotSimulationCore,
    Pose,
    SimulationParams,
)
from pybullet_fleet.action import MoveAction  # noqa: E402
from pybullet_fleet.controllers import (  # noqa: E402
    BatchDifferentialController,
    BatchOmniController,
)
from pybullet_fleet.geometry import Path  # noqa: E402
from pybullet_fleet.types import CollisionMode, SpatialHashCellSizeMode  # noqa: E402


def _make_sim(gui: bool, collision_freq: int) -> MultiRobotSimulationCore:
    params = SimulationParams(
        gui=gui,
        monitor=False,
        physics=False,
        timestep=1.0 / 60.0,
        collision_check_frequency=collision_freq,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.0,
        log_level="warning",
    )
    return MultiRobotSimulationCore(params)


def _spawn_grid(sim: MultiRobotSimulationCore, n: int, motion_mode: MotionMode, spacing: float = 2.0) -> list[Agent]:
    side = int(np.ceil(np.sqrt(n)))
    agents: list[Agent] = []
    for i in range(n):
        r, c = divmod(i, side)
        agents.append(
            Agent.from_params(
                AgentSpawnParams(
                    urdf_path="robots/simple_cube.urdf",
                    initial_pose=Pose.from_xyz(c * spacing, r * spacing, 0.1),
                    motion_mode=motion_mode,
                    collision_mode=CollisionMode.NORMAL_3D,
                    max_linear_vel=1.5,
                    max_linear_accel=2.0,
                    max_angular_vel=2.0,
                    max_angular_accel=4.0,
                ),
                sim_core=sim,
            )
        )
    return agents


def _omni_waypoints(start: Pose, n_wp: int = 5, leg: float = 0.3) -> list[Pose]:
    return [Pose.from_xyz(start.x + i * leg, start.y, start.z) for i in range(1, n_wp + 1)]


def _diff_waypoints(start: Pose, n_wp: int = 3, leg: float = 0.4) -> list[Pose]:
    out: list[Pose] = []
    for i in range(1, n_wp + 1):
        dx = leg * i
        dy = leg * (1 if i % 2 else -1)
        out.append(Pose.from_xyz(start.x + dx, start.y + dy, start.z))
    return out


def _run_steps(sim: MultiRobotSimulationCore, steps: int, label: str) -> float:
    """Return mean ms/step over ``steps`` after a 5-step warmup."""
    for _ in range(5):
        sim.step_once()
    samples = []
    t0 = time.perf_counter()
    for _ in range(steps):
        s = time.perf_counter()
        sim.step_once()
        samples.append((time.perf_counter() - s) * 1000.0)
    wall = time.perf_counter() - t0
    mean_ms = sum(samples) / len(samples)
    print(f"  {label}: {steps} steps, wall={wall:.2f}s, mean={mean_ms:.3f} ms/step")
    return mean_ms


def run_batched(args, motion_mode) -> float:
    sim = _make_sim(gui=args.gui, collision_freq=args.collision_freq)
    agents = _spawn_grid(sim, args.n, motion_mode)
    if motion_mode == MotionMode.OMNIDIRECTIONAL:
        bc = BatchOmniController()
        wp_fn = _omni_waypoints
    else:
        bc = BatchDifferentialController()
        wp_fn = _diff_waypoints
    for a in agents:
        bc.register_agent(a)
        bc.set_path(a, wp_fn(a.get_pose()))
    try:
        return _run_steps(sim, args.steps, f"batched   ({type(bc).__name__})")
    finally:
        p.disconnect(sim.client)


def run_per_agent(args, motion_mode) -> float:
    sim = _make_sim(gui=False, collision_freq=args.collision_freq)
    agents = _spawn_grid(sim, args.n, motion_mode)
    wp_fn = _omni_waypoints if motion_mode == MotionMode.OMNIDIRECTIONAL else _diff_waypoints
    for a in agents:
        a.add_action(MoveAction(path=Path(waypoints=wp_fn(a.get_pose()))))
    try:
        return _run_steps(sim, args.steps, f"per-agent ({motion_mode.value})")
    finally:
        p.disconnect(sim.client)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--n", type=int, default=500, help="Number of robots")
    ap.add_argument("--steps", type=int, default=600, help="Sim steps to measure")
    ap.add_argument(
        "--mode",
        choices=["omni", "diff"],
        default="omni",
        help="Which batch controller to demo",
    )
    ap.add_argument(
        "--collision-freq",
        type=int,
        default=60,
        help="collision_check_frequency (default 60 = every step at 60 Hz)",
    )
    ap.add_argument("--gui", action="store_true", help="Open the GUI (batched run only)")
    ap.add_argument(
        "--no-compare",
        action="store_true",
        help="Skip the per-agent baseline run",
    )
    args = ap.parse_args()

    motion_mode = MotionMode.OMNIDIRECTIONAL if args.mode == "omni" else MotionMode.DIFFERENTIAL
    print(
        f"\n=== {args.mode.upper()} batch demo  "
        f"(n={args.n}, steps={args.steps}, collision_freq={args.collision_freq}) ===\n"
    )

    if args.no_compare:
        run_batched(args, motion_mode)
        return

    per_agent_ms = run_per_agent(args, motion_mode)
    batch_ms = run_batched(args, motion_mode)
    if batch_ms > 0:
        print(f"\n  speedup (per-agent / batch): {per_agent_ms / batch_ms:.2f}x\n")


if __name__ == "__main__":
    main()
