"""Generate bridge configurations for the user-facing Fleet ROS demo."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml


SUPPORTED_ROBOT_MODELS = ("simple_cube", "mobile_robot", "tb3_burger", "tb3_waffle")


def write_fleet_demo_config(
    template_path: Path,
    output_path: Path,
    *,
    robots: int,
    robot_model: str,
    gui: bool,
    target_rtf: float,
) -> None:
    """Write a fleet-only bridge configuration derived from *template_path*."""
    if robots < 1:
        raise ValueError("robots must be at least 1")
    if robot_model not in SUPPORTED_ROBOT_MODELS:
        raise ValueError(f"unknown robot model: {robot_model}")

    with template_path.open(encoding="utf-8") as stream:
        config: dict[str, Any] = yaml.safe_load(stream) or {}

    config.setdefault("simulation", {})
    config["simulation"]["gui"] = gui
    config["simulation"]["target_rtf"] = target_rtf
    config["fleet_api"] = {
        "enabled": True,
        "states": True,
        "navigate": True,
        "joint_command": False,
    }
    config["per_robot_api"] = {
        "enabled": False,
        "state_publishers": False,
        "tf": False,
        "command_topics": False,
        "services": False,
        "actions": False,
    }

    entity = config["entities"][0]
    fleet_controller = config["managers"][0]["fleet_controller"]
    _apply_robot_model(entity, fleet_controller, robot_model)
    entity.setdefault("grid", {})
    entity["grid"]["count"] = robots
    entity["grid"]["columns"] = int(math.ceil(math.sqrt(robots)))

    output_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")


def _apply_robot_model(entity: dict[str, Any], fleet_controller: dict[str, Any], robot_model: str) -> None:
    if robot_model == "simple_cube":
        entity["urdf_path"] = "robots/simple_cube.urdf"
        fleet_controller["type"] = "batch_omni"
        entity["controller"] = {
            "type": "omni",
            "max_linear_vel": 2.0,
            "max_angular_vel": 3.0,
        }
        entity.setdefault("grid", {})["spacing"] = [2.0, 2.0, 0.0]
        entity["grid"]["offset"] = [0.0, 0.0, 0.05]
    elif robot_model == "mobile_robot":
        entity["urdf_path"] = "robots/mobile_robot.urdf"
        fleet_controller["type"] = "batch_omni"
        entity["controller"] = {
            "type": "omni",
            "max_linear_vel": 2.0,
            "max_linear_accel": 5.0,
        }
        entity.setdefault("grid", {})["spacing"] = [2.0, 2.0, 0.0]
        entity["grid"]["offset"] = [0.0, 0.0, 0.3]
    else:
        model_name = "turtlebot3_burger" if robot_model == "tb3_burger" else "turtlebot3_waffle"
        entity["urdf_path"] = model_name
        fleet_controller["type"] = "batch_differential"
        entity["controller"] = {
            "type": "differential",
            "max_linear_vel": 0.22,
            "max_linear_accel": 2.5,
            "max_angular_vel": 2.84,
            "max_angular_accel": 10.0,
        }
        entity.setdefault("grid", {})["spacing"] = [1.0, 1.0, 0.0]
        entity["grid"]["offset"] = [0.0, 0.0, 0.01]
