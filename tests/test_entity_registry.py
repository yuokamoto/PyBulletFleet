"""Tests for entity_registry module — register_entity_class / ENTITY_REGISTRY."""

import pytest


class TestEntityRegistry:
    """Entity class registry: register_entity_class / ENTITY_REGISTRY."""

    def test_builtins_registered(self):
        """Built-in types 'agent' and 'sim_object' are auto-registered."""
        from pybullet_fleet.entity_registry import ENTITY_REGISTRY
        from pybullet_fleet.agent import Agent
        from pybullet_fleet.sim_object import SimObject

        assert "agent" in ENTITY_REGISTRY
        assert "sim_object" in ENTITY_REGISTRY
        assert ENTITY_REGISTRY["agent"] is Agent
        assert ENTITY_REGISTRY["sim_object"] is SimObject

    def test_register_and_lookup(self):
        """Custom class can be registered and looked up."""
        from pybullet_fleet.entity_registry import ENTITY_REGISTRY, register_entity_class
        from pybullet_fleet.sim_object import SimObject

        class CustomBot(SimObject):
            pass

        register_entity_class("custom_bot", CustomBot)
        try:
            assert "custom_bot" in ENTITY_REGISTRY
            assert ENTITY_REGISTRY["custom_bot"] is CustomBot
        finally:
            # Cleanup to avoid polluting other tests
            ENTITY_REGISTRY.pop("custom_bot", None)

    def test_unknown_type_not_in_registry(self):
        """Unregistered type is not in registry."""
        from pybullet_fleet.entity_registry import ENTITY_REGISTRY

        assert "nonexistent_type" not in ENTITY_REGISTRY

    def test_non_simobject_subclass_raises(self):
        """Registering a class not derived from SimObject raises TypeError."""
        from pybullet_fleet.entity_registry import register_entity_class

        class NotASimObject:
            pass

        with pytest.raises(TypeError, match="SimObject"):
            register_entity_class("bad", NotASimObject)
