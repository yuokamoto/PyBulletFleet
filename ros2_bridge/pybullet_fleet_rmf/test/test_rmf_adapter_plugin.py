"""Unit tests for the in-process RMF adapter bridge plugin."""

from unittest.mock import MagicMock

import pytest

pytest.importorskip("pybullet_fleet_rmf.rmf_adapter_plugin", reason="ROS 2 / RMF dependencies not available")


def test_planner_cache_reset_size_uses_default():
    from pybullet_fleet_rmf.fleet_adapter import DEFAULT_PLANNER_CACHE_RESET_SIZE, _planner_cache_reset_size

    assert _planner_cache_reset_size({}) == DEFAULT_PLANNER_CACHE_RESET_SIZE


def test_planner_cache_reset_size_accepts_config_value():
    from pybullet_fleet_rmf.fleet_adapter import _planner_cache_reset_size

    assert _planner_cache_reset_size({"planner_cache_reset_size": "100"}) == 100


def test_planner_cache_reset_size_rejects_invalid_values():
    from pybullet_fleet_rmf.fleet_adapter import DEFAULT_PLANNER_CACHE_RESET_SIZE, _planner_cache_reset_size

    assert _planner_cache_reset_size({"planner_cache_reset_size": "bad"}) == DEFAULT_PLANNER_CACHE_RESET_SIZE
    assert _planner_cache_reset_size({"planner_cache_reset_size": 0}) == 1


def test_create_rmf_subscriptions_includes_lift_state_map_updates():
    from rmf_lift_msgs.msg import LiftState

    from pybullet_fleet_rmf.fleet_adapter import create_rmf_subscriptions

    node = MagicMock()
    node.create_publisher.return_value = MagicMock()
    node.create_subscription.side_effect = lambda *args, **kwargs: MagicMock()

    api = MagicMock()
    robot = MagicMock()
    robot.api = api
    robots = {"tinyRobot1": robot}
    fleet_handle = MagicMock()
    fleet_handle.more.return_value.fleet_name = "tinyRobot"

    subscriptions = create_rmf_subscriptions(node, robots, fleet_handle)

    assert len(subscriptions) == 4
    lift_call = node.create_subscription.call_args_list[-1]
    assert lift_call.args[0] is LiftState
    assert lift_call.args[1] == "lift_states"

    msg = LiftState()
    msg.session_id = "tinyRobot/tinyRobot1"
    msg.current_floor = "L2"
    lift_call.args[2](msg)

    api.set_map_name.assert_called_once_with("L2")


def test_rmf_adapter_runtime_shutdown_destroys_connections():
    from pybullet_fleet_rmf.fleet_adapter import RmfAdapterRuntime

    node = MagicMock()
    update_thread = MagicMock()
    update_thread.is_alive.return_value = False
    stop_event = MagicMock()
    sub_a = MagicMock()
    sub_b = MagicMock()
    runtime = RmfAdapterRuntime(
        node=node,
        adapter=MagicMock(),
        robots={},
        update_thread=update_thread,
        stop_event=stop_event,
        connections=[sub_a, sub_b],
    )

    runtime.shutdown()

    stop_event.set.assert_called_once()
    node.destroy_subscription.assert_any_call(sub_a)
    node.destroy_subscription.assert_any_call(sub_b)
    assert runtime.connections == []


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
            "rmf_frame_offset": (0.0, 0.0),
        }
    ]


def test_rmf_adapter_bridge_plugin_destroy_shuts_down_runtime(monkeypatch):
    from pybullet_fleet_rmf.rmf_adapter_plugin import RmfAdapterBridgePlugin

    runtime = MagicMock()
    monkeypatch.setattr(
        "pybullet_fleet_rmf.rmf_adapter_plugin.start_adapter_runtime",
        lambda **kwargs: runtime,
    )
    plugin = RmfAdapterBridgePlugin(MagicMock(), MagicMock(), config_file="/tmp/fleet.yaml")

    plugin.on_init()
    plugin.destroy()

    runtime.shutdown.assert_called_once()
    assert plugin.runtime is None


def test_rmf_adapter_bridge_plugin_raises_when_runtime_fails(monkeypatch):
    from pybullet_fleet_rmf.rmf_adapter_plugin import RmfAdapterBridgePlugin

    monkeypatch.setattr(
        "pybullet_fleet_rmf.rmf_adapter_plugin.start_adapter_runtime",
        lambda **kwargs: None,
    )

    plugin = RmfAdapterBridgePlugin(MagicMock(), MagicMock(), config_file="/tmp/fleet.yaml")

    with pytest.raises(RuntimeError, match="in-process RMF adapter"):
        plugin.on_init()
