"""Batch vs per-agent controller micro-benchmark.

Spawns N agents on a grid, gives each a short repeating path, and measures
wall time + per-step phase costs over a fixed step budget.

Omni agents follow a straight-line path (no rotation needed); differential
agents follow a zigzag path that forces a ROTATE at every waypoint — the
workload most representative of the batch controller's extra slerp computation.

Run (default: diff, 500 agents, collision on)::

    python3 benchmark/batch_perf.py

Omni mode::

    python3 benchmark/batch_perf.py --mode omni

No collision (isolates controller speedup)::

    python3 benchmark/batch_perf.py --collision-freq 0

GUI visualisation (batch only)::

    python3 benchmark/batch_perf.py --gui
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pybullet as p

from pybullet_fleet import (
    Agent,
    AgentSpawnParams,
    MotionMode,
    MultiRobotSimulationCore,
    Pose,
    SimulationParams,
)
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.types import CollisionMode, SpatialHashCellSizeMode


def _make_sim(collision_freq: int, gui: bool = False) -> MultiRobotSimulationCore:
    return MultiRobotSimulationCore(
        SimulationParams(
            gui=gui,
            monitor=False,
            physics=False,
            timestep=0.1,
            collision_check_frequency=collision_freq,
            spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
            spatial_hash_cell_size=2.0,
            log_level="warning",
            enable_time_profiling=False,
        )
    )



def _make_grid_params(n: int, spacing: float = 2.0) -> GridSpawnParams:
    side = int(np.ceil(np.sqrt(n)))
    return GridSpawnParams(
        x_min=0, x_max=side - 1, y_min=0, y_max=side - 1,
        spacing=[spacing, spacing, 0.0], offset=[0.0, 0.0, 0.1],
    )


def _make_spawn_params(mode: str) -> AgentSpawnParams:
    ctrl: dict = {"max_linear_vel": 1.5, "max_linear_accel": 2.0}
    if mode == "diff":
        ctrl["max_angular_vel"] = 2.0
        ctrl["max_angular_accel"] = 4.0
    motion_mode = MotionMode.OMNIDIRECTIONAL if mode == "omni" else MotionMode.DIFFERENTIAL
    return AgentSpawnParams(
        urdf_path="robots/simple_cube.urdf",
        motion_mode=motion_mode,
        collision_mode=CollisionMode.NORMAL_3D,
        controller=ctrl,
    )


def _build_waypoints(start: Pose, mode: str) -> list[Pose]:
    if mode == "omni":
        # Straight line: no rotation needed, tests pure translation throughput.
        return [Pose.from_xyz(start.x + i * 0.3, start.y, start.z) for i in range(1, 6)]
    # Diff: zigzag forces a ROTATE→FORWARD cycle at every waypoint.
    leg = 0.4
    return [
        Pose.from_xyz(start.x + leg * i, start.y + leg * (1 if i % 2 else -1), start.z)
        for i in range(1, 4)
    ]


def bench_per_agent(n: int, steps: int, collision_freq: int, mode: str) -> dict:
    sim = _make_sim(collision_freq)
    mgr = AgentManager(sim_core=sim)
    agents = mgr.spawn_agents_grid(n, _make_grid_params(n), _make_spawn_params(mode))
    ctrl_name = "OmniController" if mode == "omni" else "DifferentialController"
    for a in agents:
        a.set_path(_build_waypoints(a.get_pose(), mode))
    return _run(sim, steps, label=f"per-agent {ctrl_name}, n={n}, col_freq={collision_freq}")


def bench_batch(n: int, steps: int, collision_freq: int, mode: str, gui: bool = False) -> dict:
    sim = _make_sim(collision_freq, gui=gui)
    batch_controller = "batch_omni" if mode == "omni" else "batch_differential"
    ctrl_name = "BatchOmniController" if mode == "omni" else "BatchDifferentialController"
    mgr = AgentManager(sim_core=sim, batch_controller=batch_controller)
    agents = mgr.spawn_agents_grid(n, _make_grid_params(n), _make_spawn_params(mode))
    bc = mgr.batch_controller
    for a in agents:
        bc.set_path(a, _build_waypoints(a.get_pose(), mode))
    if gui:
        _run_gui(sim, agents)
        return {}
    return _run(sim, steps, label=f"{ctrl_name}, n={n}")


def _run_gui(sim: MultiRobotSimulationCore, agents: list[Agent]) -> None:
    """Auto-fit camera and run until window closed."""
    positions = [a.get_pose().position for a in agents]
    xs = [pos[0] for pos in positions]
    ys = [pos[1] for pos in positions]
    cx = (min(xs) + max(xs)) / 2.0
    cy = (min(ys) + max(ys)) / 2.0
    span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    p.resetDebugVisualizerCamera(
        cameraDistance=span * 0.7,
        cameraYaw=45,
        cameraPitch=-35,
        cameraTargetPosition=[cx, cy, span * 0.1],
        physicsClientId=sim.client,
    )
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=sim.client)
    sim.run_simulation(duration=None)


def _run(sim: MultiRobotSimulationCore, steps: int, *, label: str) -> dict:
    for _ in range(5):
        sim.step_once()

    step_times: list[float] = []
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

    profile_stats: dict[str, tuple[float, float, float]] = {
        key: (
            statistics.mean(samples),
            statistics.median(samples),
            sorted(samples)[int(0.95 * (len(samples) - 1))],
        )
        for key, samples in collected.items()
        if samples
    }
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
        print(f"      {key:<22}  {m:7.3f}ms {p50:7.3f}ms {p95:7.3f}ms")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=500, help="Number of agents (default: 500)")
    ap.add_argument("--steps", type=int, default=600, help="Measurement steps (default: 600)")
    ap.add_argument(
        "--mode",
        choices=["omni", "diff"],
        default="diff",
        help="omni (BatchOmniController) or diff (BatchDifferentialController, default)",
    )
    ap.add_argument(
        "--collision-freq",
        type=int,
        default=60,
        help="collision_check_frequency (default 60; 0 = disabled)",
    )
    ap.add_argument("--gui", action="store_true", help="Open the GUI (batch run only)")
    args = ap.parse_args()

    ctrl_name = "OmniController" if args.mode == "omni" else "DifferentialController"
    print(
        f"\n=== Batch vs per-agent {ctrl_name} perf  "
        f"(n={args.n}, steps={args.steps}, collision_freq={args.collision_freq}) ===\n"
    )
    if args.gui:
        batch_name = "BatchOmniController" if args.mode == "omni" else "BatchDifferentialController"
        print(f"{batch_name} — GUI  (n={args.n})\nClose the window to exit.\n")
        bench_batch(args.n, args.steps, args.collision_freq, args.mode, gui=True)
    else:
        a = bench_per_agent(args.n, args.steps, args.collision_freq, args.mode)
        _print(a)
        b = bench_batch(args.n, args.steps, args.collision_freq, args.mode)
        _print(b)
        speedup = a["mean_ms"] / b["mean_ms"] if b["mean_ms"] > 0 else float("inf")
        print(f"\n  speedup (per-agent / batch) by mean step time: {speedup:.2f}x\n")


if __name__ == "__main__":
    main()
