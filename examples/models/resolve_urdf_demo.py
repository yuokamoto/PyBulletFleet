#!/usr/bin/env python3
"""
resolve_urdf_demo.py — URDF resolution patterns demo.

Demonstrates the three ways to obtain a URDF path via ``resolve_urdf()``:

1. **By name (registry lookup)** — ``resolve_urdf("panda")``
   Resolves through KNOWN_MODELS tiers: local → pybullet_data → ros → robot_descriptions.
2. **By direct path** — ``resolve_urdf("robots/arm_robot.urdf")``
   Any string containing ``/`` or ending in ``.urdf`` / ``.sdf`` is returned as-is.
3. **List all models** — ``list_all_models()``
   Enumerates every registered model with availability info.

After resolving, the demo spawns the robot with ``Agent.from_urdf()`` and
prints the auto-detected ``RobotProfile``.

Usage::

    # Pattern 1: resolve by name (tier 0 — local)
    python resolve_urdf_demo.py --robot arm_robot

    # Pattern 1: resolve by name (tier 1 — pybullet_data)
    python resolve_urdf_demo.py --robot panda

    # Pattern 1: resolve by name (tier 3 — robot_descriptions)
    python resolve_urdf_demo.py --robot tiago

    # Pattern 2: resolve by direct file path
    python resolve_urdf_demo.py --robot robots/mobile_robot.urdf

    # Pattern 3: list all models
    python resolve_urdf_demo.py --list
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from pybullet_fleet.robot_models import (
    KNOWN_MODELS,
    auto_detect_profile,
    list_all_models,
    resolve_urdf,
)

parser = argparse.ArgumentParser(description="URDF resolution patterns demo")
parser.add_argument("--robot", default="panda", help="Robot name or URDF path (default: panda)")
parser.add_argument("--list", action="store_true", help="List all available models and exit")
parser.add_argument(
    "--gui",
    default="true",
    choices=["true", "false"],
    help="Enable PyBullet GUI (default: true)",
)
args = parser.parse_args()

# ---------------------------------------------------------------------------
# Pattern 3: list all models
# ---------------------------------------------------------------------------
if args.list:
    models = list_all_models()
    print(f"\n{'Name':<22} {'Tier':<20} {'Available':<10} {'Path / Error'}")
    print("-" * 100)
    for name, info in models.items():
        available = "yes" if info["available"] else "no"
        detail = info.get("path", info.get("error", ""))
        print(f"{name:<22} {info['tier']:<20} {available:<10} {detail}")
    print()
    sys.exit(0)

# ---------------------------------------------------------------------------
# Pattern 1 & 2: resolve URDF
#
# Note: Agent.from_urdf() calls resolve_urdf() internally, so in normal
# usage you can simply write Agent.from_urdf(urdf_path="panda", ...).
# This demo calls resolve_urdf() explicitly to show the resolution logic.
# ---------------------------------------------------------------------------
name_or_path = args.robot

# Determine resolution pattern for display
is_direct_path = os.sep in name_or_path or "/" in name_or_path or name_or_path.endswith((".urdf", ".sdf"))
if is_direct_path:
    pattern = "direct path"
    tier = "—"
else:
    entry = KNOWN_MODELS.get(name_or_path)
    tier = entry.tier if entry else "unknown"
    pattern = f"name → {tier}"

urdf_path = resolve_urdf(name_or_path)
print(f"\nResolution pattern: {pattern}")
print(f"  Input:  '{name_or_path}'")
print(f"  Output: {urdf_path}")

if not os.path.isfile(urdf_path):
    print(f"ERROR: File not found: {urdf_path}")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Spawn and introspect
# ---------------------------------------------------------------------------
from pybullet_fleet.agent import Agent  # noqa: E402
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams  # noqa: E402
from pybullet_fleet.geometry import Pose  # noqa: E402

use_gui = args.gui.lower() == "true"

sim = MultiRobotSimulationCore(SimulationParams(gui=use_gui, physics=False, monitor=False, target_rtf=10))

agent = Agent.from_urdf(
    urdf_path=urdf_path,
    pose=Pose.from_xyz(0, 0, 0),
    use_fixed_base=True,
    sim_core=sim,
)

profile = auto_detect_profile(agent.body_id, sim.client)
print(f"\nRobotProfile for '{name_or_path}':")
print(f"  robot_type:       {profile.robot_type}")
print(f"  ee_link_name:     {profile.ee_link_name}")
print(f"  num_joints:       {profile.num_joints}")
print(f"  movable_joints:   {profile.movable_joint_names}")
print(f"  lower_limits:     {[f'{x:.2f}' for x in profile.joint_lower_limits]}")
print(f"  upper_limits:     {[f'{x:.2f}' for x in profile.joint_upper_limits]}")
print(f"  max_velocities:   {[f'{x:.1f}' for x in profile.joint_max_velocities]}")

print(f"\nSpawned '{name_or_path}' with {agent.get_num_joints()} joints")

sim.setup_camera(
    camera_config={
        "camera_mode": "manual",
        "camera_distance": 2.0,
        "camera_yaw": 45,
        "camera_pitch": -30,
        "camera_target": [0, 0, 0.3],
    }
)

if use_gui:
    print("Running simulation... Close the window or press Ctrl+C to stop.\n")
    sim.run_simulation()
else:
    print("Headless mode — exiting.")
