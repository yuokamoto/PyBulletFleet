"""Unit tests for the installed fleet demo configuration generator."""

from pathlib import Path

import yaml

from pybullet_fleet_ros.fleet_demo_config import write_fleet_demo_config


TEMPLATE = Path(__file__).parents[1] / "config" / "bridge_fleet_scale.yaml"


def test_writes_fleet_only_tb3_config(tmp_path):
    output_path = tmp_path / "fleet_demo.yaml"

    write_fleet_demo_config(
        TEMPLATE,
        output_path,
        robots=10,
        robot_model="tb3_burger",
        gui=True,
        target_rtf=1.0,
    )

    config = yaml.safe_load(output_path.read_text(encoding="utf-8"))
    assert config["simulation"] == {"gui": True, "physics": False, "target_rtf": 1.0}
    assert config["fleet_api"]["enabled"] is True
    assert config["per_robot_api"]["enabled"] is False
    assert config["managers"][0]["fleet_controller"]["type"] == "batch_differential"
    assert config["entities"][0]["urdf_path"] == "turtlebot3_burger"
    assert config["entities"][0]["grid"]["count"] == 10
    assert config["entities"][0]["grid"]["columns"] == 4
