#!/usr/bin/env python3
"""
batch_controller_500_demo.py — AgentManager batch controller example.

Spawns N robots on a grid and drives them with a shared batch controller
via AgentManager.  Both omnidirectional and differential modes are supported.

Key API shown
-------------
- AgentManager(sim_core=sim, batch_controller="batch_omni")
- mgr.spawn_agents_grid(n, grid_params, spawn_params)
- agent.set_path(waypoints)   # auto-forwarded to the batch controller
- sim.run_simulation(duration=...)

For performance measurement (batch vs per-agent timing), see:
    benchmark/batch_perf.py
    benchmark/run_benchmark.py --compare per_agent batch_omni

Usage
-----
Default (omni, 500 robots, GUI)::

    python3 examples/scale/batch_controller_500_demo.py

Differential drive, 200 robots, custom robot::

    python3 examples/scale/batch_controller_500_demo.py --mode diff --n 200 --robot husky

Headless::

    python3 examples/scale/batch_controller_500_demo.py --no-gui --duration 10
"""
from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from pybullet_fleet import (  # noqa: E402
    AgentSpawnParams,
    MotionMode,
    MultiRobotSimulationCore,
    Pose,
    SimulationParams,
)
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams  # noqa: E402
from pybullet_fleet.robot_models import resolve_model  # noqa: E402
from pybullet_fleet.types import CollisionMode, SpatialHashCellSizeMode  # noqa: E402


def _make_sim(gui: bool) -> MultiRobotSimulationCore:
    return MultiRobotSimulationCore(
        SimulationParams(
            target_rtf=0,
            gui=gui,
            monitor=gui,
            enable_monitor_gui=gui,
            physics=False,
            timestep=0.1,
            spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
            spatial_hash_cell_size=2.0,
            log_level="warning",
        )
    )


def _omni_waypoints(start: Pose) -> list[Pose]:
    """Spiral-ish path: right → up → left → down → back."""
    x, y, z = start.x, start.y, start.z
    leg = 0.5
    return [
        Pose.from_xyz(x + leg * 4, y,           z),
        Pose.from_xyz(x + leg * 4, y + leg * 4, z),
        Pose.from_xyz(x,           y + leg * 4, z),
        Pose.from_xyz(x,           y,           z),
        Pose.from_xyz(x + leg * 2, y + leg * 2, z),
        Pose.from_xyz(x + leg * 4, y,           z),
        Pose.from_xyz(x,           y,           z),
    ]


def _diff_waypoints(start: Pose) -> list[Pose]:
    """Zigzag path forcing repeated ROTATE→FORWARD cycles."""
    x, y, z = start.x, start.y, start.z
    leg = 0.4
    return [
        Pose.from_xyz(x + leg,     y + leg,     z),
        Pose.from_xyz(x + leg * 2, y,           z),
        Pose.from_xyz(x + leg * 3, y + leg,     z),
        Pose.from_xyz(x + leg * 4, y,           z),
        Pose.from_xyz(x + leg * 3, y - leg,     z),
        Pose.from_xyz(x + leg * 2, y,           z),
        Pose.from_xyz(x,           y,           z),
    ]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=500, help="Number of robots (default: 500)")
    ap.add_argument("--robot", default="simple_cube",
                    help="Robot name or URDF path (default: simple_cube)")
    ap.add_argument("--mode", choices=["omni", "diff"], default="omni",
                    help="Batch controller type: omni (default) or diff")
    ap.add_argument("--duration", type=float, default=None,
                    help="Simulation duration in seconds (default: run until window closed)")
    ap.add_argument("--no-gui", action="store_true", help="Disable the PyBullet GUI")
    args = ap.parse_args()

    gui = not args.no_gui
    motion_mode = MotionMode.OMNIDIRECTIONAL if args.mode == "omni" else MotionMode.DIFFERENTIAL
    batch_controller = "batch_omni" if args.mode == "omni" else "batch_differential"
    wp_fn = _omni_waypoints if args.mode == "omni" else _diff_waypoints
    urdf_path = resolve_model(args.robot)

    sim = _make_sim(gui=gui)

    # Create an AgentManager with a shared batch controller for all agents.
    mgr = AgentManager(sim_core=sim, batch_controller=batch_controller)

    side = int(math.ceil(math.sqrt(args.n)))
    agents = mgr.spawn_agents_grid(
        num_agents=args.n,
        grid_params=GridSpawnParams(
            x_min=0, x_max=side - 1,
            y_min=0, y_max=side - 1,
            spacing=[2.0, 2.0, 0.0],
            offset=[0.0, 0.0, 0.1],
        ),
        spawn_params=AgentSpawnParams(
            urdf_path=urdf_path,
            motion_mode=motion_mode,
            collision_mode=CollisionMode.NORMAL_3D,
            controller={
                "max_linear_vel": 1.5,
                "max_linear_accel": 2.0,
                "max_angular_vel": 2.0,
                "max_angular_accel": 4.0,
            },
        ),
    )

    # agent.set_path() auto-forwards to the batch controller when one is active.
    for a in agents:
        a.set_path(wp_fn(a.get_pose()))

    print(
        f"[{args.n} robots | mode={args.mode} | "
        f"controller={type(mgr.batch_controller).__name__}]"
    )

    if gui:
        sim.setup_camera()

    sim.run_simulation(duration=args.duration)


if __name__ == "__main__":
    main()
