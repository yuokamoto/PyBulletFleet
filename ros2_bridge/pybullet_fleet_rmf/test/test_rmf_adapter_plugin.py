"""Unit tests for the in-process RMF adapter bridge plugin."""

from types import SimpleNamespace
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


def test_robot_state_update_frequency_uses_default():
    from pybullet_fleet_rmf.fleet_adapter import (
        DEFAULT_ROBOT_STATE_UPDATE_FREQUENCY,
        _robot_state_update_frequency,
    )

    assert _robot_state_update_frequency({}) == DEFAULT_ROBOT_STATE_UPDATE_FREQUENCY


def test_robot_state_update_frequency_accepts_config_value():
    from pybullet_fleet_rmf.fleet_adapter import _robot_state_update_frequency

    assert _robot_state_update_frequency({"robot_state_update_frequency": "20"}) == 20.0


def test_robot_state_update_frequency_rejects_invalid_values():
    from pybullet_fleet_rmf.fleet_adapter import (
        DEFAULT_ROBOT_STATE_UPDATE_FREQUENCY,
        _robot_state_update_frequency,
    )

    assert _robot_state_update_frequency({"robot_state_update_frequency": "bad"}) == DEFAULT_ROBOT_STATE_UPDATE_FREQUENCY
    assert _robot_state_update_frequency({"robot_state_update_frequency": ""}) == DEFAULT_ROBOT_STATE_UPDATE_FREQUENCY
    assert _robot_state_update_frequency({"robot_state_update_frequency": 0}) == DEFAULT_ROBOT_STATE_UPDATE_FREQUENCY
    assert _robot_state_update_frequency({"robot_state_update_frequency": -1}) == DEFAULT_ROBOT_STATE_UPDATE_FREQUENCY


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
    adapter = MagicMock()
    sub_a = MagicMock()
    sub_b = MagicMock()
    runtime = RmfAdapterRuntime(
        node=node,
        adapter=adapter,
        robots={},
        update_thread=update_thread,
        stop_event=stop_event,
        connections=[sub_a, sub_b],
    )

    runtime.shutdown()

    stop_event.set.assert_called_once()
    adapter.stop.assert_called_once()
    node.destroy_subscription.assert_any_call(sub_a)
    node.destroy_subscription.assert_any_call(sub_b)
    assert runtime.connections == []


def test_rmf_adapter_runtime_shutdown_continues_when_adapter_stop_fails():
    from pybullet_fleet_rmf.fleet_adapter import RmfAdapterRuntime

    node = MagicMock()
    update_thread = MagicMock()
    update_thread.is_alive.return_value = False
    stop_event = MagicMock()
    adapter = MagicMock()
    adapter.stop.side_effect = RuntimeError("stop failed")
    sub = MagicMock()
    runtime = RmfAdapterRuntime(
        node=node,
        adapter=adapter,
        robots={},
        update_thread=update_thread,
        stop_event=stop_event,
        connections=[sub],
    )

    runtime.shutdown()

    adapter.stop.assert_called_once()
    node.destroy_subscription.assert_called_once_with(sub)
    assert runtime.connections == []


def test_fleet_adapter_main_shuts_down_runtime(monkeypatch, tmp_path):
    from pybullet_fleet_rmf import fleet_adapter

    config = tmp_path / "fleet.yaml"
    config.write_text("rmf_fleet:\n  name: tinyRobot\n", encoding="utf-8")

    node = MagicMock()
    runtime = MagicMock()
    executor = MagicMock()
    executor.spin.side_effect = KeyboardInterrupt()
    fake_rclpy = SimpleNamespace(
        init=MagicMock(),
        shutdown=MagicMock(),
        utilities=SimpleNamespace(remove_ros_args=lambda argv: argv),
        node=SimpleNamespace(Node=MagicMock(return_value=node)),
        executors=SimpleNamespace(SingleThreadedExecutor=MagicMock(return_value=executor)),
    )
    monkeypatch.setattr(fleet_adapter, "rclpy", fake_rclpy)
    monkeypatch.setattr(fleet_adapter, "start_adapter_runtime", MagicMock(return_value=runtime))

    fleet_adapter.main(["fleet_adapter", "-c", str(config)])

    runtime.shutdown.assert_called_once()
    node.destroy_node.assert_called_once()
    executor.shutdown.assert_called_once()
    fake_rclpy.shutdown.assert_called_once()


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
