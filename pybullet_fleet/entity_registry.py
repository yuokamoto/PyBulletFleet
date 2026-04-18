"""Entity class registry for config-driven spawning.

Mirrors the controller registry pattern (``register_controller`` /
``create_controller``): a lightweight name → class mapping that enables
YAML config to specify ``type: agent``, ``type: sim_object``, or any
user-registered custom entity class.

Built-in entity types are auto-registered at import time:

- ``"agent"`` → :class:`~pybullet_fleet.agent.Agent`
- ``"sim_object"`` → :class:`~pybullet_fleet.sim_object.SimObject`

**Programmatic registration** (Python)::

    from pybullet_fleet.entity_registry import register_entity_class

    register_entity_class("forklift", ForkliftAgent)

**Config-driven registration** (YAML) — no Python code needed::

    entity_classes:
      forklift: "my_ros_pkg.entities.ForkliftAgent"
      conveyor: "warehouse_sim.conveyor.ConveyorObject"

    robots:
      - type: forklift
        urdf_path: robots/forklift.urdf
        pose: [0, 0, 0.05]

The ``entity_classes`` section is processed by
:func:`register_entity_classes_from_config` before robots are spawned,
so custom types are available for ``type:`` dispatch in the ``robots``
section.
"""

import importlib
import logging
from typing import Any, Dict, Type

from pybullet_fleet.sim_object import SimObject

logger = logging.getLogger(__name__)

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


def register_entity_classes_from_config(entity_classes: Dict[str, Any]) -> None:
    """Register entity classes from a config dictionary via dynamic import.

    Each key is the registry name (used as ``type`` in the ``robots``
    section) and each value is a dotted Python path
    ``"module.path.ClassName"``.

    Called automatically by :meth:`~pybullet_fleet.core_simulation.MultiRobotSimulationCore.from_yaml`
    and :meth:`~pybullet_fleet.core_simulation.MultiRobotSimulationCore.from_dict`
    when the config contains an ``entity_classes`` section.

    Args:
        entity_classes: Mapping of ``{name: "dotted.module.ClassName", ...}``.

    Raises:
        ImportError: If the module cannot be imported.
        AttributeError: If the class does not exist in the module.
        TypeError: If the resolved class is not a :class:`SimObject` subclass.

    Example::

        register_entity_classes_from_config({
            "forklift": "my_pkg.entities.ForkliftAgent",
            "conveyor": "warehouse_sim.conveyor.ConveyorObject",
        })
    """
    for name, dotted_path in entity_classes.items():
        if not isinstance(dotted_path, str) or "." not in dotted_path:
            raise ValueError(
                f"entity_classes[{name!r}] must be a dotted Python path " f"'module.ClassName', got: {dotted_path!r}"
            )
        module_path, class_name = dotted_path.rsplit(".", 1)
        try:
            module = importlib.import_module(module_path)
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError(
                f"entity_classes[{name!r}]: could not import module '{module_path}' "
                f"(from '{dotted_path}'). Did you `pip install` the package?"
            ) from exc
        try:
            cls = getattr(module, class_name)
        except AttributeError:
            available = [n for n in dir(module) if not n.startswith("_")]
            raise AttributeError(
                f"entity_classes[{name!r}]: class '{class_name}' not found in "
                f"module '{module_path}'. Available names: {available}"
            ) from None
        register_entity_class(name, cls)
        logger.info("Registered entity class %r → %s", name, dotted_path)


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
