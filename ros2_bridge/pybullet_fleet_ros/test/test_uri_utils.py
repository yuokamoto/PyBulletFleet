"""Tests for ROS package URI resolution helpers."""

from pathlib import Path

import pytest

pytest.importorskip("ament_index_python", reason="ROS 2 ament index not available")


def test_resolve_package_uri_to_installed_share_path():
    from pybullet_fleet_ros.uri_utils import resolve_uri

    resolved = resolve_uri("package://pybullet_fleet_ros/config/bridge_nav.yaml")

    assert resolved.endswith("/pybullet_fleet_ros/config/bridge_nav.yaml")
    assert Path(resolved).is_file()


def test_resolve_package_uris_recursively():
    from pybullet_fleet_ros.uri_utils import resolve_package_uris

    config = {
        "world": {"world_file": "package://pybullet_fleet_ros/config/bridge_nav.yaml"},
        "entities": [
            {"name": "robot0", "urdf_path": "robots/mobile_robot.urdf"},
            {"name": "robot1", "metadata": ("package://pybullet_fleet_ros/config/default_bridge.yaml",)},
        ],
    }

    resolved = resolve_package_uris(config)

    assert resolved["world"]["world_file"].endswith("/pybullet_fleet_ros/config/bridge_nav.yaml")
    assert resolved["entities"][0]["urdf_path"] == "robots/mobile_robot.urdf"
    assert resolved["entities"][1]["metadata"][0].endswith("/pybullet_fleet_ros/config/default_bridge.yaml")
