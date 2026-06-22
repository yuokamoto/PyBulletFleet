"""Tests for BridgePlugin — registry, from_config, lifecycle.

No ROS 2 dependency.

Validates:
- Constructor protocol: (node, sim_core) — config via from_config
- Lifecycle hooks: on_init, post_step, on_reset, destroy (all no-op by default)
- Registry auto-registration via _registry_name + __init_subclass__
- from_config introspection (kwargs matching)
- create_bridge_plugin_from_entry (type: / class: resolution)
"""

from unittest.mock import MagicMock

import pytest

from pybullet_fleet_ros.bridge_plugin import (
    BridgePlugin,
    _registry,
    create_bridge_plugin,
    create_bridge_plugin_from_entry,
    list_bridge_plugins,
    register_bridge_plugin,
)


# ---- Fixtures ----


@pytest.fixture(autouse=True)
def _clear_bridge_registry():
    """Clear registry before each test to avoid cross-contamination."""
    saved = dict(_registry._registry)
    yield
    _registry._registry.clear()
    _registry._registry.update(saved)


# ---- Concrete stubs ----


class _StubPlugin(BridgePlugin):
    """Minimal concrete implementation for testing."""

    def __init__(self, node, sim_core, **kwargs):
        super().__init__(node, sim_core)
        self.init_kwargs = dict(kwargs)
        self.post_step_calls = []
        self.destroyed = False
        self.init_called = False
        self.reset_called = False

    def on_init(self) -> None:
        self.init_called = True

    def post_step(self, sim_time: float) -> None:
        self.post_step_calls.append(sim_time)

    def on_reset(self) -> None:
        self.reset_called = True

    def destroy(self) -> None:
        self.destroyed = True


class _ConfigurablePlugin(BridgePlugin):
    """Accepts typed config kwargs for from_config tests."""

    def __init__(self, node, sim_core, rate: float = 1.0, enabled: bool = True):
        super().__init__(node, sim_core)
        self.rate = rate
        self.enabled = enabled


# ---- Tests ----


class TestBridgePluginContract:
    """BridgePlugin base can be instantiated (no-op defaults)."""

    def test_base_instantiates_directly(self):
        """BridgePlugin itself is not abstract — no-op defaults."""
        plugin = BridgePlugin(MagicMock(), MagicMock())
        assert plugin is not None

    def test_concrete_subclass_instantiates(self):
        """Complete subclass can be instantiated."""
        plugin = _StubPlugin(MagicMock(), MagicMock())
        assert plugin is not None


class TestBridgePluginInit:
    """Constructor stores node, sim_core."""

    def test_stores_node(self):
        node = MagicMock()
        plugin = _StubPlugin(node, MagicMock())
        assert plugin.node is node

    def test_stores_sim_core(self):
        sim_core = MagicMock()
        plugin = _StubPlugin(MagicMock(), sim_core)
        assert plugin.sim_core is sim_core

    def test_config_defaults_to_empty_dict(self):
        plugin = _StubPlugin(MagicMock(), MagicMock())
        assert plugin.config == {}


class TestBridgePluginLifecycle:
    """Lifecycle methods are callable on concrete subclass."""

    def test_on_init_is_callable(self):
        plugin = _StubPlugin(MagicMock(), MagicMock())
        plugin.on_init()
        assert plugin.init_called

    def test_post_step_receives_sim_time(self):
        plugin = _StubPlugin(MagicMock(), MagicMock())
        plugin.post_step(1.5)
        plugin.post_step(3.0)
        assert plugin.post_step_calls == [1.5, 3.0]

    def test_on_reset_is_callable(self):
        plugin = _StubPlugin(MagicMock(), MagicMock())
        plugin.on_reset()
        assert plugin.reset_called

    def test_destroy_is_callable(self):
        plugin = _StubPlugin(MagicMock(), MagicMock())
        plugin.destroy()
        assert plugin.destroyed

    def test_base_hooks_are_noop(self):
        """BridgePlugin base hooks do nothing (no errors)."""
        plugin = BridgePlugin(MagicMock(), MagicMock())
        plugin.on_init()
        plugin.post_step(0.0)
        plugin.on_reset()
        plugin.destroy()


class TestBridgePluginRegistry:
    """Auto-registration and manual registration."""

    def test_manual_register_and_lookup(self):
        register_bridge_plugin("stub", _StubPlugin)
        assert list_bridge_plugins()["stub"] is _StubPlugin

    def test_auto_registration_via_registry_name(self):
        """Classes with _registry_name are auto-registered on definition."""

        class _AutoPlugin(BridgePlugin):
            _registry_name = "auto_test"

        assert list_bridge_plugins()["auto_test"] is _AutoPlugin


class TestBridgePluginFromConfig:
    """from_config introspection maps config keys to __init__ kwargs."""

    def test_from_config_passes_matching_keys(self):
        plugin = _ConfigurablePlugin.from_config(MagicMock(), MagicMock(), {"rate": 5.0, "enabled": False})
        assert plugin.rate == 5.0
        assert plugin.enabled is False

    def test_from_config_ignores_extra_keys(self):
        plugin = _ConfigurablePlugin.from_config(MagicMock(), MagicMock(), {"rate": 2.0, "unknown_key": "ignored"})
        assert plugin.rate == 2.0
        assert plugin.enabled is True  # default

    def test_from_config_uses_defaults(self):
        plugin = _ConfigurablePlugin.from_config(MagicMock(), MagicMock(), {})
        assert plugin.rate == 1.0
        assert plugin.enabled is True

    def test_from_config_stores_raw_config(self):
        cfg = {"rate": 3.0}
        plugin = _ConfigurablePlugin.from_config(MagicMock(), MagicMock(), cfg)
        assert plugin.config == {"rate": 3.0}

    def test_from_config_var_kwargs_passes_all(self):
        """StubPlugin uses **kwargs — all config keys pass through."""
        plugin = _StubPlugin.from_config(MagicMock(), MagicMock(), {"a": 1, "b": 2})
        assert plugin.init_kwargs == {"a": 1, "b": 2}
        assert plugin.config == {"a": 1, "b": 2}


class TestCreateBridgePluginFromEntry:
    """Factory function resolves type: and class: entries."""

    def test_type_entry_resolves_via_registry(self):
        register_bridge_plugin("stub", _StubPlugin)
        entry = {"type": "stub", "config": {"x": 10}}
        plugin = create_bridge_plugin_from_entry(entry, MagicMock(), MagicMock())
        assert isinstance(plugin, _StubPlugin)
        assert plugin.config == {"x": 10}

    def test_class_entry_resolves_via_dotted_path(self):
        entry = {
            "class": "test_bridge_plugin._ConfigurablePlugin",
            "config": {"rate": 7.0},
        }
        plugin = create_bridge_plugin_from_entry(entry, MagicMock(), MagicMock())
        assert isinstance(plugin, _ConfigurablePlugin)
        assert plugin.rate == 7.0

    def test_missing_type_and_class_raises(self):
        with pytest.raises(ValueError, match="type.*class"):
            create_bridge_plugin_from_entry({"config": {}}, MagicMock(), MagicMock())

    def test_unknown_type_raises_key_error(self):
        with pytest.raises(KeyError, match="no_such_plugin"):
            create_bridge_plugin_from_entry({"type": "no_such_plugin"}, MagicMock(), MagicMock())

    def test_create_bridge_plugin_convenience(self):
        register_bridge_plugin("stub", _StubPlugin)
        plugin = create_bridge_plugin("stub", MagicMock(), MagicMock(), {"y": 42})
        assert isinstance(plugin, _StubPlugin)
        assert plugin.config == {"y": 42}
