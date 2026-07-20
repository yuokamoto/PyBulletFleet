"""Tests for RMF launch-time bridge config composition."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip("launch", reason="ROS 2 launch not available")


def _load_pybullet_common_launch():
    path = Path(__file__).resolve().parents[1] / "launch" / "pybullet_common.launch.py"
    spec = importlib.util.spec_from_file_location("pybullet_common_launch", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_bridge_config_for_fleet_ros_enables_required_fleet_api(tmp_path):
    module = _load_pybullet_common_launch()
    config_path = tmp_path / "bridge.yaml"
    config_path.write_text("simulation: {}\n")

    result = module._bridge_config_for_client_mode(
        config_yaml=str(config_path),
        rmf_adapters=yaml.safe_dump([{"config_file": "/tmp/fleet.yaml", "nav_graph": "/tmp/nav.yaml"}]),
        client_mode="fleet_ros",
    )

    assert result != str(config_path)
    generated = yaml.safe_load(Path(result).read_text())
    assert generated == {
        "simulation": {},
        "fleet_api": {
            "enabled": True,
            "states": True,
            "navigate": True,
            "stop": True,
            "attach": True,
        },
    }


def test_bridge_config_for_per_robot_ros_does_not_append_rmf_plugin(tmp_path):
    module = _load_pybullet_common_launch()
    config_path = tmp_path / "bridge.yaml"
    config_path.write_text("simulation: {}\n")

    result = module._bridge_config_for_client_mode(
        config_yaml=str(config_path),
        rmf_adapters=yaml.safe_dump([{"config_file": "/tmp/fleet.yaml", "nav_graph": "/tmp/nav.yaml"}]),
        client_mode="per_robot_ros",
    )

    assert result != str(config_path)
    generated = yaml.safe_load(Path(result).read_text())
    assert generated == {"simulation": {}}


def test_bridge_config_resolves_package_uris(tmp_path, monkeypatch):
    module = _load_pybullet_common_launch()

    def fake_resolve(value):
        if isinstance(value, str):
            return value.replace("package://rmf_demos_maps", "/opt/ros/jazzy/share/rmf_demos_maps").replace(
                "package://rmf_demos_assets", "/opt/ros/jazzy/share/rmf_demos_assets"
            )
        if isinstance(value, list):
            return [fake_resolve(item) for item in value]
        if isinstance(value, dict):
            return {key: fake_resolve(item) for key, item in value.items()}
        return value

    monkeypatch.setattr(module, "resolve_package_uris", fake_resolve)
    config_path = tmp_path / "bridge.yaml"
    config_path.write_text(
        yaml.safe_dump(
            {
                "simulation": {"world_file": "package://rmf_demos_maps/maps/office/office.world"},
                "robots": [
                    {
                        "name": "tinyRobot1",
                        "sdf_path": "package://rmf_demos_assets/models/TinyRobot/model.sdf",
                    }
                ],
            }
        )
    )

    result = module._bridge_config_for_client_mode(
        config_yaml=str(config_path),
        rmf_adapters=yaml.safe_dump([{"config_file": "/tmp/fleet.yaml", "nav_graph": "/tmp/nav.yaml"}]),
        client_mode="fleet_ros",
    )

    generated = yaml.safe_load(Path(result).read_text())
    assert generated["simulation"]["world_file"].startswith("/opt/ros/jazzy/share/rmf_demos_maps/")
    assert generated["robots"][0]["sdf_path"].startswith("/opt/ros/jazzy/share/rmf_demos_assets/")


def test_bridge_config_leaves_non_package_paths_unchanged(tmp_path):
    module = _load_pybullet_common_launch()
    config_path = tmp_path / "bridge.yaml"
    config_path.write_text(
        yaml.safe_dump(
            {
                "simulation": {"world_file": "/tmp/custom_maps/maps/office/office.world"},
            }
        )
    )

    result = module._bridge_config_for_client_mode(
        config_yaml=str(config_path),
        rmf_adapters=yaml.safe_dump([{"config_file": "/tmp/fleet.yaml", "nav_graph": "/tmp/nav.yaml"}]),
        client_mode="fleet_ros",
    )

    generated = yaml.safe_load(Path(result).read_text())
    assert generated["simulation"]["world_file"] == "/tmp/custom_maps/maps/office/office.world"


def test_bridge_config_for_python_fleet_appends_single_rmf_plugin(tmp_path):
    module = _load_pybullet_common_launch()
    config_path = tmp_path / "bridge.yaml"
    config_path.write_text(
        yaml.safe_dump(
            {
                "simulation": {"gui": False},
                "bridge_plugins": [{"class": "existing.Plugin", "config": {}}],
                "rmf_frame_offset": [22000.0, 31500.0],
            }
        )
    )

    result = module._bridge_config_for_client_mode(
        config_yaml=str(config_path),
        rmf_adapters=yaml.safe_dump([{"config_file": "/tmp/fleet.yaml", "nav_graph": "/tmp/nav.yaml"}]),
        client_mode="python_fleet",
        server_uri="ws://localhost:8000/_internal",
        use_sim_time=True,
    )

    assert result != str(config_path)
    generated = yaml.safe_load(Path(result).read_text())
    assert generated["simulation"] == {"gui": False}
    assert generated["bridge_plugins"][0]["class"] == "existing.Plugin"
    plugin = generated["bridge_plugins"][1]
    assert plugin["class"] == module.RMF_ADAPTER_PLUGIN_CLASS
    assert plugin["config"] == {
        "config_file": "/tmp/fleet.yaml",
        "nav_graph": "/tmp/nav.yaml",
        "client_mode": "python_fleet",
        "server_uri": "ws://localhost:8000/_internal",
        "use_sim_time": True,
        "rmf_frame_offset": [22000.0, 31500.0],
    }


def test_bridge_config_for_python_fleet_appends_multiple_rmf_plugins(tmp_path):
    module = _load_pybullet_common_launch()
    config_path = tmp_path / "bridge.yaml"
    config_path.write_text("simulation: {}\n")

    result = module._bridge_config_for_client_mode(
        config_yaml=str(config_path),
        client_mode="python_fleet",
        rmf_adapters=yaml.safe_dump(
            [
                {
                    "config_file": "/tmp/tiny.yaml",
                    "nav_graph": "/tmp/tiny_nav.yaml",
                },
                {
                    "config_file": "/tmp/delivery.yaml",
                    "nav_graph": "/tmp/delivery_nav.yaml",
                },
                {
                    "config_file": "/tmp/cleaner.yaml",
                    "nav_graph": "/tmp/cleaner_nav.yaml",
                },
            ]
        ),
    )

    generated = yaml.safe_load(Path(result).read_text())
    plugins = [entry for entry in generated["bridge_plugins"] if entry["class"] == module.RMF_ADAPTER_PLUGIN_CLASS]
    assert [plugin["config"]["config_file"] for plugin in plugins] == [
        "/tmp/tiny.yaml",
        "/tmp/delivery.yaml",
        "/tmp/cleaner.yaml",
    ]
    assert all(plugin["config"]["client_mode"] == "python_fleet" for plugin in plugins)


def test_rmf_adapters_requires_at_least_one_adapter():
    module = _load_pybullet_common_launch()

    with pytest.raises(ValueError, match="rmf_adapters must contain at least one adapter"):
        module._rmf_adapters_from_yaml("[]")


def test_append_in_process_rmf_plugin_skips_same_fleet_duplicate():
    module = _load_pybullet_common_launch()
    config = {
        "bridge_plugins": [
            {
                "class": module.RMF_ADAPTER_PLUGIN_CLASS,
                "config": {"config_file": "/tmp/fleet.yaml", "nav_graph": "/tmp/nav.yaml"},
            }
        ]
    }

    updated = module._append_in_process_rmf_plugin(
        config,
        config_file="/tmp/fleet.yaml",
        nav_graph="/tmp/nav.yaml",
        client_mode="python_fleet",
    )

    assert updated["bridge_plugins"] == config["bridge_plugins"]


def test_append_in_process_rmf_plugin_allows_different_fleets():
    module = _load_pybullet_common_launch()
    config = {
        "bridge_plugins": [
            {
                "class": module.RMF_ADAPTER_PLUGIN_CLASS,
                "config": {"config_file": "/tmp/tiny.yaml", "nav_graph": "/tmp/tiny_nav.yaml"},
            }
        ]
    }

    updated = module._append_in_process_rmf_plugin(
        config,
        config_file="/tmp/delivery.yaml",
        nav_graph="/tmp/delivery_nav.yaml",
        client_mode="python_fleet",
    )

    assert len(updated["bridge_plugins"]) == 2
    assert updated["bridge_plugins"][1]["config"]["config_file"] == "/tmp/delivery.yaml"
