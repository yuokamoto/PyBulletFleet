"""Entity class registry for config-driven spawning.

Mirrors the controller registry pattern (``register_controller`` /
``create_controller``): a lightweight name → class mapping that enables
YAML config to specify ``type: agent``, ``type: sim_object``, or any
user-registered custom entity class.

Built-in entity types are auto-registered at import time:

- ``"agent"`` → :class:`~pybullet_fleet.agent.Agent`
- ``"sim_object"`` → :class:`~pybullet_fleet.sim_object.SimObject`

Usage::

    from pybullet_fleet.entity_registry import register_entity_class, ENTITY_REGISTRY

    # Register a custom entity class
    register_entity_class("forklift", ForkliftAgent)

    # It will now be available in spawn_from_config:
    # entities:
    #   - type: forklift
    #     name: forklift0
    #     urdf_path: robots/forklift.urdf
"""

from typing import Dict, Type

from pybullet_fleet.sim_object import SimObject


ENTITY_REGISTRY: Dict[str, Type[SimObject]] = {}


def register_entity_class(name: str, cls: Type[SimObject]) -> None:
    """Register an entity class under *name*.

    Args:
        name: Registry key (used as ``type`` value in YAML config).
        cls: A :class:`SimObject` subclass with ``from_params(spawn_params, sim_core)``.

    Raises:
        TypeError: If *cls* is not a :class:`SimObject` subclass.
    """
    if not (isinstance(cls, type) and issubclass(cls, SimObject)):
        raise TypeError(f"Entity class must be a SimObject subclass, got {cls!r}")
    ENTITY_REGISTRY[name] = cls


def list_entity_classes() -> Dict[str, Type[SimObject]]:
    """Return a copy of the entity registry for inspection.

    Example::

        from pybullet_fleet.entity_registry import list_entity_classes
        for name, cls in list_entity_classes().items():
            print(f"{name}: {cls.__name__}")
    """
    return dict(ENTITY_REGISTRY)


def _register_builtins() -> None:
    """Auto-register built-in entity types (deferred to avoid circular imports)."""
    from pybullet_fleet.agent import Agent

    register_entity_class("agent", Agent)
    register_entity_class("sim_object", SimObject)


_register_builtins()
