#!/usr/bin/env python3
"""
100 Robots Cube Patrol Demo

Demonstrates 100 robots (mixed omnidirectional and differential drive) where each
robot patrols around its own 5m × 5m × 5m cube centered at its spawn position:
1. XY plane circuit (bottom level)
2. Vertical climb (Z+)
3. XY plane circuit (top level)
4. Descent (Z-)

Features:
- 100 robots spawned in a 10×10 grid (10m spacing)
- Each robot patrols a 5m×5m×5m cube around its spawn position
- Random mix of omnidirectional and differential drive (50/50)
- Random forward/backward direction for differential drive (50/50)
- All robots move in parallel, creating synchronized swarm behavior

Spawn is config-driven: two named managers (omni_fleet / diff_fleet) are declared
with shared fleet_controller defaults and batch controllers, then entity grids are
routed to each manager by name.  Path assignment is done in Python after spawn.

Controller options:
  --controller batch      (default) vectorised batch controllers, auto-registered
  --controller per_agent  legacy per-agent controllers

Performance comparison::

    python3 benchmark/batch_perf.py --mode diff --n 300
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np
import pybullet as p
import random

from pybullet_fleet.agent import MotionMode, MovementDirection
from pybullet_fleet.config_utils import load_yaml_config, merge_configs
from pybullet_fleet.core_simulation import MultiRobotSimulationCore
from pybullet_fleet.geometry import Path, Pose
from pybullet_fleet.robot_models import resolve_model


parser = argparse.ArgumentParser(description="100 Robots Cube Patrol Demo")
parser.add_argument("--robot", default="husky", help="Robot name or URDF path")
parser.add_argument("--duration", type=float, default=None, help="Simulation duration in seconds (default: run forever)")
parser.add_argument(
    "--controller",
    choices=["batch", "per_agent"],
    default="batch",
    help="Controller type: 'batch' (default) or 'per_agent'",
)
_args = parser.parse_args()


def create_cube_patrol_path(cube_center: list, cube_size: float = 5.0) -> Path:
    """9-waypoint patrol: bottom XY circuit → climb → top XY circuit → descend."""
    half = cube_size / 2.0
    cx, cy, cz = cube_center
    z_bot = cz - half + 0.3
    z_top = cz + half
    corners = [
        [cx + half, cy + half, z_bot],
        [cx - half, cy + half, z_bot],
        [cx - half, cy - half, z_bot],
        [cx + half, cy - half, z_bot],
        [cx + half, cy - half, z_top],
        [cx + half, cy + half, z_top],
        [cx - half, cy + half, z_top],
        [cx - half, cy - half, z_top],
        [cx + half, cy - half, z_bot],
    ]
    return Path(waypoints=[Pose.from_euler(x, y, z, 0, 0, 0) for x, y, z in corners])


def main():
    random.seed(42)

    urdf_path = resolve_model(_args.robot)
    print(f"Using robot: {_args.robot} -> {urdf_path}")

    _BASE_CONFIG = os.path.join(os.path.dirname(__file__), "..", "..", "config", "config.yaml")
    _OVERRIDES = {
        "simulation": {
            "target_rtf": 5,
            "enable_time_profiling": True,
            "camera": {
                "camera_mode": "auto",
                "camera_view_type": "perspective",
                "camera_auto_scale": 0.5,
            },
        }
    }

    # Shared controller params applied to every agent in each fleet via
    # fleet_controller.  Per-agent overrides (if any) take precedence.
    _fleet_ctrl = {
        "max_linear_vel": 2.0,
        "max_linear_accel": 1.0,
        "max_angular_vel": 2.0,
        "max_angular_accel": 5.0,
    }
    use_batch = _args.controller == "batch"

    # Config-driven setup: declare two named managers, each with a shared
    # fleet_controller and (optionally) a batch controller, then route entity
    # grids to each manager by name.
    cfg = merge_configs(load_yaml_config(_BASE_CONFIG), _OVERRIDES)
    cfg["managers"] = [
        {
            "name": "omni_fleet",
            "fleet_controller": _fleet_ctrl,
            **({"batch_controller": "batch_omni"} if use_batch else {}),
        },
        {
            "name": "diff_fleet",
            "fleet_controller": _fleet_ctrl,
            **({"batch_controller": "batch_differential"} if use_batch else {}),
        },
    ]
    cfg["entities"] = [
        {
            "urdf_path": urdf_path,
            "mass": 0.0,
            "use_fixed_base": False,
            "motion_mode": "omnidirectional",
            "manager": "omni_fleet",
            "grid": {
                "x_min": 0,
                "x_max": 4,
                "y_min": 0,
                "y_max": 9,
                "spacing": [10.0, 10.0, 0.0],
                "offset": [-15.0, -15.0, 0.3],
            },
        },
        {
            "urdf_path": urdf_path,
            "mass": 0.0,
            "use_fixed_base": False,
            "motion_mode": "differential",
            "manager": "diff_fleet",
            "grid": {
                "x_min": 5,
                "x_max": 9,
                "y_min": 0,
                "y_max": 9,
                "spacing": [10.0, 10.0, 0.0],
                "offset": [-15.0, -15.0, 0.3],
            },
        },
    ]

    sim = MultiRobotSimulationCore.from_dict(cfg)
    omni_manager = sim.get_manager("omni_fleet")
    diff_manager = sim.get_manager("diff_fleet")

    print("=" * 70)
    print("100 Robots Cube Patrol Demo")
    print("=" * 70)
    if use_batch:
        print("[INFO] Batch controllers active (via AgentManager batch_controller)")
    else:
        print("[INFO] Per-agent controllers")

    all_robots = omni_manager.objects + diff_manager.objects
    num_omni = len(omni_manager.objects)
    num_diff = len(diff_manager.objects)
    print(f"✓ Spawned {len(all_robots)} robots: omni={num_omni}, diff={num_diff}")

    num_fwd = num_bwd = num_omni_count = 0
    for robot in all_robots:
        spawn_pos = robot.get_pose().position
        path = create_cube_patrol_path(cube_center=[spawn_pos[0], spawn_pos[1], spawn_pos[2] + 2.5])
        path.visualize(
            show_lines=True,
            line_color=[0.5, 0.5, 0.5],
            line_width=1.0,
            show_waypoints=True,
            show_axes=False,
            show_points=False,
            lifetime=0,
        )
        if robot.motion_mode == MotionMode.DIFFERENTIAL:
            d = random.choice([MovementDirection.FORWARD, MovementDirection.BACKWARD])
            robot.set_path(path.waypoints, direction=d)
            if d == MovementDirection.FORWARD:
                num_fwd += 1
            else:
                num_bwd += 1
        else:
            robot.set_path(path.waypoints)
            num_omni_count += 1

    print("✓ Patrol paths assigned")
    print(f"  omni={num_omni_count}  diff fwd={num_fwd}  diff bwd={num_bwd}")

    sim.setup_camera()

    p.addUserDebugText(
        "100 Robots Parallel Patrol",
        [-15.0 + 45.0, -15.0 + 45.0, 7.5],
        textColorRGB=[1.0, 1.0, 0.0],
        textSize=2.0,
    )

    step_counter = [0]

    def monitoring_callback(sim_core, dt):
        step_counter[0] += 1
        if step_counter[0] % 300 == 0:
            moving = sum(1 for r in all_robots if r.is_moving)
            speeds = [np.linalg.norm(r.velocity) for r in all_robots if r.is_moving]
            avg = np.mean(speeds) if speeds else 0.0
            print(f"[t={step_counter[0]*dt:.1f}s] " f"moving={moving}/100  avg_speed={avg:.2f}m/s")

    sim.register_callback(monitoring_callback, frequency=None)
    sim.run_simulation(duration=_args.duration)


if __name__ == "__main__":
    main()
