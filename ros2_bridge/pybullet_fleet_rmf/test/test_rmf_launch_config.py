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


def test_bridge_config_for_non_python_fleet_returns_original_path(tmp_path):
    module = _load_pybullet_common_launch()
    config_path = tmp_path / "bridge.yaml"
    config_path.write_text("simulation: {}\n")

    result = module._bridge_config_for_client_mode(
        config_yaml=str(config_path),
        fleet_config="/tmp/fleet.yaml",
        nav_graph="/tmp/nav.yaml",
        client_mode="fleet_ros",
    )

    assert result == str(config_path)


def test_bridge_config_for_python_fleet_appends_rmf_plugin(tmp_path):
    module = _load_pybullet_common_launch()
    config_path = tmp_path / "bridge.yaml"
    config_path.write_text(
        yaml.safe_dump(
            {
                "simulation": {"gui": False},
                "bridge_plugins": [{"class": "existing.Plugin", "config": {}}],
            }
        )
    )

    result = module._bridge_config_for_client_mode(
        config_yaml=str(config_path),
        fleet_config="/tmp/fleet.yaml",
        nav_graph="/tmp/nav.yaml",
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
    }


def test_append_in_process_rmf_plugin_skips_duplicates():
    module = _load_pybullet_common_launch()
    config = {
        "bridge_plugins": [
            {
                "class": module.RMF_ADAPTER_PLUGIN_CLASS,
                "config": {"config_file": "/tmp/old.yaml"},
            }
        ]
    }

    updated = module._append_in_process_rmf_plugin(
        config,
        config_file="/tmp/new.yaml",
        client_mode="python_fleet",
    )

    assert updated["bridge_plugins"] == config["bridge_plugins"]

