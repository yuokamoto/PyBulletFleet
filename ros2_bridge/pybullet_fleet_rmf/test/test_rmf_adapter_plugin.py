"""Unit tests for the in-process RMF adapter bridge plugin."""

from unittest.mock import MagicMock

import pytest

pytest.importorskip("pybullet_fleet_rmf.rmf_adapter_plugin", reason="ROS 2 / RMF dependencies not available")


def test_rmf_adapter_bridge_plugin_starts_runtime_with_sim_core(monkeypatch):
    from pybullet_fleet_rmf.rmf_adapter_plugin import RmfAdapterBridgePlugin

    node = MagicMock()
    sim_core = MagicMock()
    runtime = MagicMock()
    calls = []

    def fake_start_adapter_runtime(**kwargs):
        calls.append(kwargs)
        return runtime

    monkeypatch.setattr(
        "pybullet_fleet_rmf.rmf_adapter_plugin.start_adapter_runtime",
        fake_start_adapter_runtime,
    )

    plugin = RmfAdapterBridgePlugin(
        node,
        sim_core,
        config_file="/tmp/fleet.yaml",
        nav_graph="/tmp/nav.yaml",
        client_mode="python_fleet",
        use_sim_time=True,
        server_uri="ws://localhost:8000/_internal",
    )

    plugin.on_init()

    assert plugin.runtime is runtime
    assert calls == [
        {
            "node": node,
            "config_path": "/tmp/fleet.yaml",
            "nav_graph_path": "/tmp/nav.yaml",
            "use_sim_time": True,
            "client_mode": "python_fleet",
            "sim_core": sim_core,
            "server_uri": "ws://localhost:8000/_internal",
        }
    ]


def test_rmf_adapter_bridge_plugin_raises_when_runtime_fails(monkeypatch):
    from pybullet_fleet_rmf.rmf_adapter_plugin import RmfAdapterBridgePlugin

    monkeypatch.setattr(
        "pybullet_fleet_rmf.rmf_adapter_plugin.start_adapter_runtime",
        lambda **kwargs: None,
    )

    plugin = RmfAdapterBridgePlugin(MagicMock(), MagicMock(), config_file="/tmp/fleet.yaml")

    with pytest.raises(RuntimeError, match="in-process RMF adapter"):
        plugin.on_init()

