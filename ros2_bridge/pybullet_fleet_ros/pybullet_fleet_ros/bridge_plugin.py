"""BridgePlugin — base class for singleton bridge plugins.

Bridge plugins are loaded once by :class:`BridgeNode` and provide
global ROS functionality (as opposed to per-robot
:class:`RobotHandlerBase` instances).

Uses the same registry + factory patterns as ``SimPlugin``:

- **Auto-registration**: set ``_registry_name`` on a concrete subclass
  to register it (e.g. ``_registry_name = "workcell"``).
- **from_config()**: maps YAML config keys to ``__init__`` kwargs
  via ``from_config_introspect``.
- **YAML shorthand**: ``type: workcell`` resolves via registry;
  ``class: my_pkg.MyPlugin`` resolves via dotted path.

Lifecycle::

    __init__(node, sim_core, ...)      # construction (typed kwargs)
    on_init()                          # after all plugins are loaded
    post_step(sim_time)                # called each sim step (after robot handlers)
    on_reset()                         # when simulation is reset
    destroy()                          # cleanup before shutdown

Example::

    class MyBridgePlugin(BridgePlugin):
        _registry_name = "my_plugin"

        def __init__(self, node, sim_core, some_param: int = 42):
            super().__init__(node, sim_core)
            self._pub = node.create_publisher(String, "/my_topic", 10)

        def post_step(self, sim_time: float) -> None:
            self._pub.publish(String(data=f"t={sim_time:.2f}"))

        def destroy(self) -> None:
            self._node.destroy_publisher(self._pub)

YAML config::

    bridge_plugins:
      - type: my_plugin           # registry shorthand
        config:
          some_param: 42
      - class: my_pkg.OtherPlugin # dotted path (no registry needed)
        config: {}
"""

from typing import TYPE_CHECKING, Any, Dict, Optional, Type

from pybullet_fleet.plugin_utils import PluginRegistry, from_config_introspect

if TYPE_CHECKING:
    from pybullet_fleet.core_simulation import MultiRobotSimulationCore
    from rclpy.node import Node


# ------------------------------------------------------------------
# Registry
# ------------------------------------------------------------------

_registry: PluginRegistry["BridgePlugin"] = PluginRegistry("bridge_plugin")


def register_bridge_plugin(name: str, cls: Type["BridgePlugin"]) -> None:
    """Manually register a bridge plugin class under *name*."""
    _registry.register(name, cls)


def create_bridge_plugin(
    name: str,
    node: "Node",
    sim_core: "MultiRobotSimulationCore",
    config: Optional[Dict[str, Any]] = None,
) -> "BridgePlugin":
    """Create a bridge plugin by registered *name*.

    Convenience wrapper around :func:`create_bridge_plugin_from_entry`.
    """
    entry: Dict[str, Any] = {"type": name, "config": config or {}}
    return create_bridge_plugin_from_entry(entry, node, sim_core)


def create_bridge_plugin_from_entry(
    entry: Dict[str, Any],
    node: "Node",
    sim_core: "MultiRobotSimulationCore",
) -> "BridgePlugin":
    """Create a bridge plugin from a YAML-style config entry.

    Supports two formats:

    - **Registry shorthand**: ``{"type": "workcell", "config": {...}}``
    - **Dotted path**: ``{"class": "my_pkg.MyPlugin", "config": {...}}``

    Args:
        entry: Plugin definition dict with ``type`` or ``class`` key.
        node: The ROS 2 BridgeNode instance.
        sim_core: The simulation core instance.

    Returns:
        Instantiated :class:`BridgePlugin`.
    """
    cls = _registry.resolve_from_entry(entry, BridgePlugin)
    config = entry.get("config", {})
    return cls.from_config(node, sim_core, config)


def list_bridge_plugins() -> Dict[str, Type["BridgePlugin"]]:
    """Return a copy of the bridge plugin registry for inspection."""
    return _registry.items()


# ------------------------------------------------------------------
# Base class
# ------------------------------------------------------------------


class BridgePlugin:
    """Base class for singleton bridge plugins.

    All lifecycle hooks are **optional no-ops** by default — override
    only what your plugin needs.

    Subclasses with a ``_registry_name`` class attribute are
    auto-registered (e.g. ``_registry_name = "workcell"``).

    Attributes:
        node: The ROS 2 BridgeNode instance.
        sim_core: The PyBulletFleet simulation core.
        config: Plugin-specific configuration dict.
    """

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        _registry.auto_register_subclass(cls)

    def __init__(
        self,
        node: "Node",
        sim_core: "MultiRobotSimulationCore",
    ) -> None:
        self.node = node
        self.sim_core = sim_core
        self.config: Dict[str, Any] = {}

    @classmethod
    def from_config(
        cls,
        node: "Node",
        sim_core: "MultiRobotSimulationCore",
        config: Optional[Dict[str, Any]] = None,
    ) -> "BridgePlugin":
        """Create from config dict, matching keys to ``__init__`` parameters.

        Inspects the subclass ``__init__`` signature and passes matching
        config keys as kwargs (same pattern as ``SimPlugin.from_config``).

        Args:
            node: The ROS 2 BridgeNode instance.
            sim_core: The simulation core instance.
            config: Configuration dict whose keys are matched to ``__init__``
                parameters (unmatched keys are silently ignored).
        """
        return from_config_introspect(cls, (node, sim_core), config or {})

    def on_init(self) -> None:
        """Called after all bridge plugins are loaded.

        Override to perform one-time setup that depends on sim_core
        or other plugins being ready.  No-op by default.
        """

    def post_step(self, sim_time: float) -> None:
        """Called after each simulation step.

        Publish ROS messages, update state, etc.  No-op by default.

        Args:
            sim_time: Current simulation time in seconds.
        """

    def on_reset(self) -> None:
        """Called when the simulation is reset.

        Override to reinitialise plugin state without destroying
        ROS resources.  No-op by default.
        """

    def destroy(self) -> None:
        """Clean up all ROS resources (publishers, subscribers, timers).

        Called when BridgeNode shuts down.  No-op by default.
        """


# Backward-compatible alias
BridgePluginBase = BridgePlugin
