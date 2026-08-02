#!/usr/bin/env python3
"""Prefetch Gazebo Fuel assets used by PyBulletFleet RMF demo worlds."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import quote

SCENARIOS = ("office", "hotel", "clinic", "airport_terminal", "campus", "battle_royale")


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _package_share(package: str) -> Path:
    result = subprocess.run(["ros2", "pkg", "prefix", package], check=True, capture_output=True, text=True)
    return Path(result.stdout.splitlines()[0].strip()) / "share" / package


def _resource_paths(world: Path) -> list[Path]:
    paths = [world.parent / "models", _package_share("rmf_demos_assets") / "models"]
    paths.extend(Path(path) for path in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(":"))
    return paths


def _model_is_local(model_name: str, world: Path) -> bool:
    return any((path / model_name / "model.sdf").is_file() for path in _resource_paths(world) if path)


def _fuel_uris(world: Path) -> set[str]:
    root = ET.parse(world).getroot()
    direct_fuel_uris = {
        element.text.strip()
        for element in root.iter()
        if _local_name(element.tag) == "uri" and element.text and element.text.strip().startswith("https://fuel.")
    }
    model_uris = {
        element.text.strip()[len("model://") :]
        for element in root.iter()
        if _local_name(element.tag) == "uri" and element.text and element.text.strip().startswith("model://")
    }
    return direct_fuel_uris | {
        f"https://fuel.gazebosim.org/1.0/OpenRobotics/models/{quote(model_name)}"
        for model_name in model_uris
        if not _model_is_local(model_name, world)
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=SCENARIOS, action="append", help="Prefetch one scenario; repeat as needed")
    parser.add_argument("--dry-run", action="store_true", help="List assets without downloading them")
    args = parser.parse_args()

    if not shutil.which("gz"):
        raise RuntimeError("The Gazebo 'gz' CLI is not available in this shell")

    maps_dir = _package_share("rmf_demos_maps") / "maps"
    scenarios = args.scenario or list(SCENARIOS)
    worlds = [maps_dir / scenario / f"{scenario}.world" for scenario in scenarios]
    missing = [world for world in worlds if not world.is_file()]
    if missing:
        raise FileNotFoundError(f"RMF world file not found: {missing[0]}")

    fuel_uris = sorted({uri for world in worlds for uri in _fuel_uris(world)})
    print(f"Scenarios: {', '.join(scenarios)}")
    print(f"Fuel models to fetch: {len(fuel_uris)}")
    for index, uri in enumerate(fuel_uris, start=1):
        action = "Would download" if args.dry_run else "Downloading"
        print(f"[{index}/{len(fuel_uris)}] {action}: {uri}", flush=True)
        if args.dry_run:
            continue
        subprocess.run(["gz", "fuel", "download", "-u", uri], check=True)

    if not args.dry_run:
        print("RMF demo assets are ready. Start or restart a demo launch.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
