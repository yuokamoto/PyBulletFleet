"""Quick perf comparison: OmniController vs BatchOmniController at scale.

Spawns N agents on a grid, gives each a single MoveAction / batch path of
length 5, and measures wall time + per-step phase1_update / phase2_pose_flush
over a fixed step budget.

Run::

    python3 benchmark/batch_omni_perf.py --n 500 --steps 600
"""

from __future__ import annotations

import argparse
import statistics
import time

import numpy as np

from pybullet_fleet import (
    Agent,
    AgentSpawnParams,
    MotionMode,
    MultiRobotSimulationCore,
    Pose,
    SimulationParams,
)
from pybullet_fleet.action import MoveAction
from pybullet_fleet.controllers import BatchOmniController
from pybullet_fleet.geometry import Path
from pybullet_fleet.types import CollisionMode, SpatialHashCellSizeMode


def _make_sim(collision_freq: int = 0) -> MultiRobotSimulationCore:
    params = SimulationParams(
        gui=False,
        monitor=False,
        physics=False,
        timestep=1.0 / 60.0,
        collision_check_frequency=collision_freq,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.0,
        log_level="warning",
        enable_time_profiling=False,  # we collect via return_profiling=True per step
    )
    return MultiRobotSimulationCore(params)


def _spawn_grid(sim: MultiRobotSimulationCore, n: int, spacing: float = 2.0) -> list[Agent]:
    side = int(np.ceil(np.sqrt(n)))
    agents = []
    for i in range(n):
        r, c = divmod(i, side)
        a = Agent.from_params(
            AgentSpawnParams(
                urdf_path="robots/simple_cube.urdf",
                initial_pose=Pose.from_xyz(c * spacing, r * spacing, 0.1),
                motion_mode=MotionMode.OMNIDIRECTIONAL,
                collision_mode=CollisionMode.NORMAL_3D,
                max_linear_vel=1.5,
                max_linear_accel=2.0,
            ),
            sim_core=sim,
        )
        agents.append(a)
    return agents


def _build_waypoints(start: Pose, n_wp: int = 5, leg: float = 0.3) -> list[Pose]:
    return [Pose.from_xyz(start.x + i * leg, start.y, start.z) for i in range(1, n_wp + 1)]


def bench_per_agent(n: int, steps: int, collision_freq: int) -> dict:
    sim = _make_sim(collision_freq)
    agents = _spawn_grid(sim, n)
    for a in agents:
        wps = _build_waypoints(a.get_pose())
        a.add_action(MoveAction(path=Path(waypoints=wps)))
    return _run(sim, steps, label=f"per-agent OmniController, n={n}, col_freq={collision_freq}")


def bench_batch(n: int, steps: int, collision_freq: int) -> dict:
    sim = _make_sim(collision_freq)
    agents = _spawn_grid(sim, n)
    bc = BatchOmniController()
    for a in agents:
        bc.register_agent(a)
        wps = _build_waypoints(a.get_pose())
        bc.set_path(a, wps)
    return _run(sim, steps, label=f"BatchOmniController, n={n}")


def _run(sim: MultiRobotSimulationCore, steps: int, *, label: str) -> dict:
    # Warm up
    for _ in range(5):
        sim.step_once()

    step_times: list[float] = []
    # phase name -> list of samples (one per step)
    collected: dict[str, list[float]] = {}
    t0 = time.perf_counter()
    for _ in range(steps):
        s = time.perf_counter()
        result = sim.step_once(return_profiling=True) or {}
        step_times.append((time.perf_counter() - s) * 1000.0)
        for key, value in result.items():
            if key == "collision_breakdown":
                if isinstance(value, dict):
                    for sub, sub_val in value.items():
                        collected.setdefault(f"col.{sub}", []).append(sub_val)
                continue
            collected.setdefault(key, []).append(value)
    t1 = time.perf_counter()

    profile_stats = {}
    for key, samples in collected.items():
        if not samples:
            continue
        profile_stats[key] = (
            statistics.mean(samples),
            statistics.median(samples),
            sorted(samples)[int(0.95 * (len(samples) - 1))],
        )

    import pybullet as p

    p.disconnect(sim.client)
    return {
        "label": label,
        "wall_s": t1 - t0,
        "steps": steps,
        "mean_ms": statistics.mean(step_times),
        "p50_ms": statistics.median(step_times),
        "p95_ms": sorted(step_times)[int(0.95 * len(step_times))],
        "max_ms": max(step_times),
        "phases": profile_stats,
    }


_PHASE_KEYS = [
    "phase1_update",
    "agent_update",
    "callbacks",
    "phase2_pose_flush",
    "step_simulation",
    "phase3_aabb_grid_flush",
    "collision_check",
    "total",
]


def _print(r: dict) -> None:
    print(f"\n  {r['label']}")
    print(
        f"    wall={r['wall_s']:.2f}s  "
        f"step mean={r['mean_ms']:.3f}ms  p50={r['p50_ms']:.3f}  "
        f"p95={r['p95_ms']:.3f}  max={r['max_ms']:.3f}"
    )
    print("    phase                     mean       p50       p95")
    # Show all phases that were measured (not just the well-known ones).
    seen = set(_PHASE_KEYS)
    for key in _PHASE_KEYS:
        if key not in r["phases"]:
            continue
        m, p50, p95 = r["phases"][key]
        print(f"      {key:<22}  {m:7.3f}ms {p50:7.3f}ms {p95:7.3f}ms")
    for key in sorted(r["phases"]):
        if key in seen:
            continue
        m, p50, p95 = r["phases"][key]
        if m < 0.001:  # skip noise
            continue
        print(f"      {key:<22}  {m:7.3f}ms {p50:7.3f}ms {p95:7.3f}ms")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=500)
    ap.add_argument("--steps", type=int, default=600)
    ap.add_argument(
        "--collision-freq",
        type=int,
        default=60,
        help="collision_check_frequency (default 60 = every step at 60 Hz; 0 = disabled)",
    )
    args = ap.parse_args()

    print(
        f"\n=== Batch vs per-agent OmniController perf  "
        f"(n={args.n}, steps={args.steps}, collision_freq={args.collision_freq}) ===\n"
    )
    a = bench_per_agent(args.n, args.steps, args.collision_freq)
    _print(a)
    b = bench_batch(args.n, args.steps, args.collision_freq)
    _print(b)
    speedup = a["mean_ms"] / b["mean_ms"] if b["mean_ms"] > 0 else float("inf")
    print(f"\n  speedup (per-agent / batch) by mean step time: {speedup:.2f}x\n")


if __name__ == "__main__":
    main()
