#!/usr/bin/env python3
"""Generate a temporary ROS bridge config for fleet scale checks."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import yaml

DEFAULT_TEMPLATE_CANDIDATES = [
    Path(__file__).resolve().parents[1] / "ros2_bridge/pybullet_fleet_ros/config/bridge_fleet_scale.yaml",
    Path("/rmf_demos_ws/install/pybullet_fleet_ros/share/pybullet_fleet_ros/config/bridge_fleet_scale.yaml"),
    Path("/rmf_demos_ws/src/pybullet_fleet_ros/config/bridge_fleet_scale.yaml"),
    Path("/opt/pybullet_fleet/ros2_bridge/pybullet_fleet_ros/config/bridge_fleet_scale.yaml"),
]


def _parse_groups(raw: str) -> set[str]:
    if raw == "all":
        return {"state_publishers", "tf", "command_topics", "services", "actions"}
    if raw == "default":
        return {"state_publishers", "tf", "command_topics"}
    if raw == "none":
        return set()
    valid = {"state_publishers", "tf", "command_topics", "services", "actions"}
    groups = {part.strip() for part in raw.split(",") if part.strip()}
    unknown = groups - valid
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown per-robot group(s): {sorted(unknown)}")
    return groups


def _default_template() -> Path:
    for path in DEFAULT_TEMPLATE_CANDIDATES:
        if path.exists():
            return path
    candidates = ", ".join(str(path) for path in DEFAULT_TEMPLATE_CANDIDATES)
    raise FileNotFoundError(f"bridge_fleet_scale.yaml not found; checked: {candidates}")


def generate_bridge_config(
    robot_count: int,
    output_path: Path,
    *,
    template_path: Path,
    gui: bool,
    target_rtf: float,
    interface_mode: str,
    per_robot_groups: set[str],
) -> None:
    side = int(math.ceil(math.sqrt(robot_count)))
    fleet_enabled = interface_mode in {"fleet", "hybrid"}
    per_robot_enabled = interface_mode in {"per_robot", "hybrid"}

    with template_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    config.setdefault("simulation", {})
    config["simulation"]["gui"] = gui
    config["simulation"]["target_rtf"] = target_rtf

    config["fleet_api"] = {
        "enabled": fleet_enabled,
        "states": fleet_enabled,
        "navigate": fleet_enabled,
        "joint_command": False,
    }
    config["per_robot_api"] = {
        "enabled": per_robot_enabled,
        "state_publishers": per_robot_enabled and "state_publishers" in per_robot_groups,
        "tf": per_robot_enabled and "tf" in per_robot_groups,
        "command_topics": per_robot_enabled and "command_topics" in per_robot_groups,
        "services": per_robot_enabled and "services" in per_robot_groups,
        "actions": per_robot_enabled and "actions" in per_robot_groups,
    }

    entity = config["entities"][0]
    entity.setdefault("grid", {})
    entity["grid"]["count"] = robot_count
    entity["grid"]["columns"] = side
    output_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, default=100, help="Number of robots to spawn")
    parser.add_argument("--gui", action="store_true", help="Set generated bridge config simulation.gui=true")
    parser.add_argument("--target-rtf", type=float, default=0.0, help="Simulation target RTF")
    parser.add_argument(
        "--interface-mode",
        choices=["fleet", "hybrid", "per_robot"],
        default="fleet",
        help="ROS interfaces to create",
    )
    parser.add_argument("--template", type=Path, default=None, help="Base bridge_fleet_scale.yaml template")
    parser.add_argument("--config-out", type=Path, required=True, help="Write generated bridge config to this path")
    parser.add_argument(
        "--per-robot-groups",
        type=_parse_groups,
        default=_parse_groups("default"),
        help="Comma-separated per-robot groups for per_robot/hybrid configs, or default/all/none",
    )
    args = parser.parse_args()

    template_path = args.template if args.template is not None else _default_template()
    generate_bridge_config(
        args.robots,
        args.config_out,
        template_path=template_path,
        gui=args.gui,
        target_rtf=args.target_rtf,
        interface_mode=args.interface_mode,
        per_robot_groups=args.per_robot_groups,
    )
    print(f"[config] wrote generated bridge config: {args.config_out} (template={template_path})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
