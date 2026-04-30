"""Tests for BridgePluginBase ABC — no ROS 2 dependency.

Validates:
- Abstract contract: post_step + destroy must be implemented
- Constructor protocol: (node, sim_core, config)
- Default no-ops where applicable
"""

from unittest.mock import MagicMock

import pytest

from pybullet_fleet_ros.bridge_plugin_base import BridgePluginBase


# ---- Concrete stub ----


class _StubPlugin(BridgePluginBase):
    """Minimal concrete implementation for testing."""

    def __init__(self, node, sim_core, config=None):
        super().__init__(node, sim_core, config)
        self.post_step_calls = []
        self.destroyed = False

    def post_step(self, sim_time: float) -> None:
        self.post_step_calls.append(sim_time)

    def destroy(self) -> None:
        self.destroyed = True


class _IncompletePlugin(BridgePluginBase):
    """Missing required methods — should fail to instantiate."""

    pass


# ---- Tests ----


class TestBridgePluginBaseContract:
    """BridgePluginBase enforces abstract protocol."""

    def test_cannot_instantiate_base_directly(self):
        """BridgePluginBase itself is abstract."""
        with pytest.raises(TypeError, match="abstract"):
            BridgePluginBase(MagicMock(), MagicMock(), {})

    def test_cannot_instantiate_incomplete_subclass(self):
        """Subclass without post_step + destroy raises TypeError."""
        with pytest.raises(TypeError, match="abstract"):
            _IncompletePlugin(MagicMock(), MagicMock(), {})

    def test_concrete_subclass_instantiates(self):
        """Complete subclass can be instantiated."""
        plugin = _StubPlugin(MagicMock(), MagicMock(), {"key": "val"})
        assert plugin is not None


class TestBridgePluginBaseInit:
    """Constructor stores node, sim_core, config."""

    def test_stores_node(self):
        node = MagicMock()
        plugin = _StubPlugin(node, MagicMock())
        assert plugin.node is node

    def test_stores_sim_core(self):
        sim_core = MagicMock()
        plugin = _StubPlugin(MagicMock(), sim_core)
        assert plugin.sim_core is sim_core

    def test_stores_config(self):
        plugin = _StubPlugin(MagicMock(), MagicMock(), {"a": 1})
        assert plugin.config == {"a": 1}

    def test_config_defaults_to_empty_dict(self):
        plugin = _StubPlugin(MagicMock(), MagicMock())
        assert plugin.config == {}


class TestBridgePluginBaseLifecycle:
    """Lifecycle methods are callable on concrete subclass."""

    def test_post_step_receives_sim_time(self):
        plugin = _StubPlugin(MagicMock(), MagicMock())
        plugin.post_step(1.5)
        plugin.post_step(3.0)
        assert plugin.post_step_calls == [1.5, 3.0]

    def test_destroy_is_callable(self):
        plugin = _StubPlugin(MagicMock(), MagicMock())
        plugin.destroy()
        assert plugin.destroyed
