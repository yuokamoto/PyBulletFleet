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


class TestRegisterFromConfig:
    """register_entity_classes_from_config: dynamic import from dotted paths."""

    def test_register_builtin_class_via_dotted_path(self):
        """Can register a built-in class via dotted path string."""
        from pybullet_fleet.entity_registry import ENTITY_REGISTRY, register_entity_classes_from_config

        register_entity_classes_from_config(
            {
                "agent_alias": "pybullet_fleet.agent.Agent",
            }
        )
        try:
            from pybullet_fleet.agent import Agent

            assert "agent_alias" in ENTITY_REGISTRY
            assert ENTITY_REGISTRY["agent_alias"] is Agent
        finally:
            ENTITY_REGISTRY.pop("agent_alias", None)

    def test_register_simobject_via_dotted_path(self):
        """Can register SimObject itself via dotted path."""
        from pybullet_fleet.entity_registry import ENTITY_REGISTRY, register_entity_classes_from_config

        register_entity_classes_from_config(
            {
                "obj_alias": "pybullet_fleet.sim_object.SimObject",
            }
        )
        try:
            from pybullet_fleet.sim_object import SimObject

            assert ENTITY_REGISTRY["obj_alias"] is SimObject
        finally:
            ENTITY_REGISTRY.pop("obj_alias", None)

    def test_register_multiple_classes(self):
        """Can register multiple classes in one call."""
        from pybullet_fleet.entity_registry import ENTITY_REGISTRY, register_entity_classes_from_config

        register_entity_classes_from_config(
            {
                "alias_a": "pybullet_fleet.agent.Agent",
                "alias_b": "pybullet_fleet.sim_object.SimObject",
            }
        )
        try:
            assert "alias_a" in ENTITY_REGISTRY
            assert "alias_b" in ENTITY_REGISTRY
        finally:
            ENTITY_REGISTRY.pop("alias_a", None)
            ENTITY_REGISTRY.pop("alias_b", None)

    def test_bad_module_raises_import_error(self):
        """Non-existent module raises ImportError."""
        from pybullet_fleet.entity_registry import register_entity_classes_from_config

        with pytest.raises(ImportError):
            register_entity_classes_from_config(
                {
                    "bad": "no_such_module.NoClass",
                }
            )

    def test_bad_class_raises_attribute_error(self):
        """Non-existent class in valid module raises AttributeError."""
        from pybullet_fleet.entity_registry import register_entity_classes_from_config

        with pytest.raises(AttributeError):
            register_entity_classes_from_config(
                {
                    "bad": "pybullet_fleet.agent.NoSuchClass",
                }
            )

    def test_non_simobject_raises_type_error(self):
        """Dotted path to a non-SimObject class raises TypeError."""
        from pybullet_fleet.entity_registry import register_entity_classes_from_config

        # logging.Logger is not a SimObject subclass
        with pytest.raises(TypeError, match="SimObject"):
            register_entity_classes_from_config(
                {
                    "bad": "logging.Logger",
                }
            )

    def test_invalid_dotted_path_raises_value_error(self):
        """Path without a dot (no module separator) raises ValueError."""
        from pybullet_fleet.entity_registry import register_entity_classes_from_config

        with pytest.raises(ValueError, match="dotted Python path"):
            register_entity_classes_from_config(
                {
                    "bad": "JustAClassName",
                }
            )
