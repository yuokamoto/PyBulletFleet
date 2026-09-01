#!/usr/bin/env python3
"""OpenUSD static warehouse GUI demo.

Run with the optional OpenUSD dependency installed:

    pip install -e '.[usd]'
    python pybullet_fleet/examples/models/usd_warehouse_demo.py

Use ``--usd /path/to/localized_warehouse.usd`` to load a locally packaged Isaac
or other OpenUSD warehouse.  The bundled scene keeps this example runnable
without redistributing any third-party assets.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

# Prefer this source checkout when the script is run directly. Without this,
# ``python3 pybullet_fleet/examples/...`` can accidentally import an older
# globally installed pybullet_fleet package instead of the code beside it.
_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pybullet as p

from pybullet_fleet import MultiRobotSimulationCore, SimulationParams, UsdImportOptions, load_usd_world


def _default_stage() -> Path:
    return Path(__file__).resolve().parents[1] / "assets" / "usd_simple_warehouse.usda"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", type=Path, default=_default_stage(), help="Local USD/USDZ warehouse stage")
    parser.add_argument("--headless", action="store_true", help="Load then exit without opening PyBullet GUI")
    parser.add_argument(
        "--lighting-controls",
        action="store_true",
        help="Show PyBullet GUI sliders for main-light position and shadow-map settings",
    )
    parser.add_argument(
        "--texture-brightness",
        type=float,
        default=1.15,
        help="Texture brightness multiplier for the PyBullet GUI (default: 1.15)",
    )
    args = parser.parse_args()
    if args.headless and args.lighting_controls:
        parser.error("--lighting-controls requires the PyBullet GUI; omit --headless")

    lighting_config = (
        {
            "light_position": [8.0, -8.0, 15.0],
            "shadow_map_world_size": 20,
            "shadow_map_resolution": 2048,
            "enable_controls": True,
        }
        if args.lighting_controls
        else None
    )
    sim = MultiRobotSimulationCore(
        SimulationParams(
            gui=not args.headless,
            monitor=not args.headless,
            enable_floor=False,
            lighting_config=lighting_config,
        )
    )
    try:
        report = load_usd_world(
            args.usd,
            sim_core=sim,
            options=UsdImportOptions(texture_brightness=args.texture_brightness),
        )
    except (FileNotFoundError, ImportError, ValueError) as exc:
        p.disconnect(sim.client)
        parser.error(str(exc))
    print(f"Loaded {report.objects_created} USD meshes from {report.source_stage}")
    print(f"Normalized with {report.stage_to_pbf_transform}")
    for diagnostic in report.diagnostics:
        print(f"{diagnostic.severity}: {diagnostic.code}: {diagnostic.message}")

    if args.headless:
        p.disconnect(sim.client)
        return

    sim.setup_camera()
    if args.lighting_controls:
        print("Use the PyBullet Parameters panel to adjust Light X/Y/Z and shadow-map settings.")
    print("USD warehouse is visible in PyBullet. Close the window or Ctrl-C to exit.")
    sim.run_simulation()


if __name__ == "__main__":
    main()
