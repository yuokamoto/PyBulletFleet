#!/usr/bin/env python3
"""
model_catalog_demo.py — Visual catalog of all available robot models.

Spawns every model in KNOWN_MODELS on a grid with floating name labels,
using the standard PyBulletFleet API (``MultiRobotSimulationCore``).

Usage::

    python model_catalog_demo.py               # all available
    python model_catalog_demo.py --tier local
    python model_catalog_demo.py --tier pybullet_data
    python model_catalog_demo.py --tier ros
    python model_catalog_demo.py --columns 5 --spacing 2.5
    python model_catalog_demo.py --floor samurai
    python model_catalog_demo.py --gui false    # headless dry-run
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import pybullet as p

from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Pose
from pybullet_fleet.robot_models import (
    KNOWN_MODELS,
    auto_detect_profile,
    list_all_models,
    resolve_urdf,
)

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

parser = argparse.ArgumentParser(description="Robot model catalog")
parser.add_argument(
    "--tier",
    default=None,
    choices=["local", "pybullet_data", "ros", "robot_descriptions"],
    help="Filter to one tier (default: all)",
)
parser.add_argument("--columns", type=int, default=4, help="Grid columns (default: 4)")
parser.add_argument("--spacing", type=float, default=3.0, help="Grid spacing metres (default: 3.0)")
parser.add_argument(
    "--floor",
    default="plain",
    choices=["samurai", "plain", "checkerboard"],
    help="Floor style (default: plain)",
)
parser.add_argument(
    "--gui",
    default="true",
    choices=["true", "false"],
    help="Enable PyBullet GUI (default: true)",
)
args = parser.parse_args()

# ---------------------------------------------------------------------------
# Discover models
# ---------------------------------------------------------------------------

print("\nScanning available models...")
all_models = list_all_models()

if args.tier:
    all_models = {k: v for k, v in all_models.items() if v["tier"] == args.tier}

# De-duplicate by rel_path (different names pointing to same URDF)
seen_rel = set()
deduped = {}
for name, info in all_models.items():
    entry = KNOWN_MODELS.get(name)
    rel = entry.rel_path if entry else name
    if rel in seen_rel:
        continue
    seen_rel.add(rel)
    deduped[name] = info
all_models = deduped

available = {k: v for k, v in all_models.items() if v["available"]}
unavailable = {k: v for k, v in all_models.items() if not v["available"]}

print(f"  Available: {len(available)}")
if unavailable:
    print(f"  Unavailable ({len(unavailable)}):")
    for name, info in unavailable.items():
        hint = KNOWN_MODELS[name].install_hint if name in KNOWN_MODELS else ""
        print(f"    - {name} [{info['tier']}] -> {hint}")
print()

# Sort by tier then name
_TIER_ORDER = {"local": 0, "pybullet_data": 1, "ros": 2, "robot_descriptions": 3}

# Skip models that are not useful in a visual catalog
_CATALOG_SKIP = {"plane"}  # huge ground-plane mesh causes Z-fighting with floor

sorted_avail = sorted(
    [n for n in available if n not in _CATALOG_SKIP],
    key=lambda n: (_TIER_ORDER.get(available[n]["tier"], 9), n),
)
sorted_unavail = sorted(
    [n for n in unavailable if n not in _CATALOG_SKIP],
    key=lambda n: (_TIER_ORDER.get(unavailable[n]["tier"], 9), n),
)
all_display = sorted_avail + sorted_unavail

if not all_display:
    print("No models to display.")
    sys.exit(0)

# ---------------------------------------------------------------------------
# Tier colours (for floating labels)
# ---------------------------------------------------------------------------

_TIER_COLORS = {
    "local": [0.2, 0.6, 1.0],
    "pybullet_data": [0.2, 0.8, 0.2],
    "ros": [1.0, 0.6, 0.0],
    "robot_descriptions": [0.8, 0.2, 0.8],
}

# ---------------------------------------------------------------------------
# Simulation setup
# ---------------------------------------------------------------------------

use_gui = args.gui.lower() == "true"
cols = args.columns
spacing = args.spacing

sim = MultiRobotSimulationCore(
    SimulationParams(
        gui=use_gui,
        physics=False,
        monitor=False,
        enable_shadows=False,
        enable_floor=False,  # We handle floor ourselves below
    )
)
client = sim.client

# Load floor based on --floor option
if args.floor == "plain":
    floor_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[100, 100, 0.002],
        rgbaColor=[0.85, 0.85, 0.85, 0.3],
        physicsClientId=client,
    )
    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=floor_visual,
        basePosition=[0, 0, 0.0],
        physicsClientId=client,
    )
elif args.floor == "samurai":
    p.loadURDF("samurai.urdf", physicsClientId=client)
elif args.floor == "checkerboard":
    p.loadURDF("plane.urdf", physicsClientId=client)

# ---------------------------------------------------------------------------
# Spawn models inside batch_spawn() for optimised rendering
# ---------------------------------------------------------------------------

print(f"Spawning {len(all_display)} models in {cols}-column grid (spacing={spacing}m)...\n")

spawned = 0
with sim.batch_spawn():
    for idx, name in enumerate(all_display):
        row = idx // cols
        col = idx % cols
        x = col * spacing
        y = -row * spacing

        entry = KNOWN_MODELS.get(name)
        tier = entry.tier if entry else "?"
        color = _TIER_COLORS.get(tier, [1.0, 1.0, 1.0])

        # Unavailable → placeholder label only
        if name in unavailable:
            hint = entry.install_hint if entry else "not installed"
            p.addUserDebugText(
                name,
                [x, y, 0.8],
                textColorRGB=[0.6, 0.0, 0.0],
                textSize=1.2,
                physicsClientId=client,
            )
            print(f"  [{tier:<20}] {name:<22} NOT INSTALLED -> {hint}")
            continue

        try:
            urdf_path = resolve_urdf(name)
        except FileNotFoundError as e:
            print(f"  [{tier}] {name}: SKIP ({e})")
            continue

        try:
            # SDF models require raw PyBullet (no Agent wrapper)
            if urdf_path.endswith(".sdf"):
                body_ids = p.loadSDF(urdf_path, physicsClientId=client)
                if not body_ids:
                    raise RuntimeError(f"SDF loaded 0 bodies: {urdf_path}")
                for bid in body_ids:
                    p.resetBasePositionAndOrientation(bid, [x, y, 0.0], [0, 0, 0, 1], physicsClientId=client)
                primary_body = body_ids[0]
            else:
                agent = Agent.from_urdf(
                    urdf_path=urdf_path,
                    pose=Pose.from_xyz(x, y, 0),
                    mass=0.0,
                    use_fixed_base=True,
                    name=name,
                    sim_core=sim,
                )
                primary_body = agent.body_id
            spawned += 1

            # Introspect the live body (no temp body, no removeBody)
            profile = auto_detect_profile(primary_body, client)

            # Label above model
            p.addUserDebugText(
                name,
                [x, y, 1.5],
                textColorRGB=color,
                textSize=1.2,
                physicsClientId=client,
            )
            print(f"  [{tier:<20}] {name:<22} type={profile.robot_type:<20} pos=({x:.0f}, {y:.0f})")
        except Exception as e:
            print(f"  [{tier:<20}] {name:<22} FAILED: {e}")

# ---------------------------------------------------------------------------
# Camera — overview of entire grid
# ---------------------------------------------------------------------------

total_rows = math.ceil(len(all_display) / cols)
cx = (cols - 1) * spacing / 2.0
cy = -(total_rows - 1) * spacing / 2.0
dist = max(total_rows, cols) * spacing * 0.9

sim.setup_camera(
    camera_config={
        "camera_mode": "manual",
        "camera_distance": max(dist, 8.0),
        "camera_yaw": 0,
        "camera_pitch": -60,
        "camera_target": [cx, cy, 0.0],
    }
)

print(f"\nSpawned {spawned}/{len(all_display)} models.")
if use_gui:
    print("Close the PyBullet window to exit.\n")
print()
print("Legend:")
print("  \033[34mlocal\033[0m              = project robots/ directory")
print("  \033[32mpybullet_data\033[0m      = bundled with pybullet (always available)")
print("  \033[33mros\033[0m                = ROS description packages (apt install)")
print("  \033[35mrobot_descriptions\033[0m = robot_descriptions pip package")
print("  \033[31mNOT INSTALLED\033[0m      = missing package (install hint shown)")
print()
sys.stdout.flush()

# ---------------------------------------------------------------------------
# Run simulation (keeps GUI window alive; headless exits immediately)
# ---------------------------------------------------------------------------

if use_gui:
    sim.run_simulation()
else:
    print("Headless mode — exiting.")
