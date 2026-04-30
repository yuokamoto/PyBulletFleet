"""SimPlugin — lifecycle-managed simulation plugins.

Gazebo World Plugin equivalent for PyBulletFleet.  Plugins receive
lifecycle callbacks and can access the full simulation API via
``self.sim_core``.

All lifecycle methods (``on_init``, ``on_step``, ``on_reset``,
``on_shutdown``) are optional no-ops by default.  Override only the
methods your plugin needs.

**Step-based plugin** (called every sim step / at a fixed frequency)::

    class TrafficMonitor(SimPlugin):
        def on_init(self) -> None:
            self.counts = {}

        def on_step(self, dt: float) -> None:
            for a in self.sim_core.agents:
                self.counts[a.name] = self.counts.get(a.name, 0) + 1

    sim.register_plugin(TrafficMonitor, config={}, frequency=2.0)

**Event-driven plugin** (reacts to EventBus events, no per-step work)::

    class CollisionLogger(SimPlugin):
        def on_init(self) -> None:
            self.sim_core.events.on(
                SimEvents.COLLISION_STARTED, self._on_collision
            )

        def _on_collision(self, obj_a, obj_b):
            print(f"Collision: {obj_a.name} <-> {obj_b.name}")

    sim.register_plugin(CollisionLogger)

**YAML-driven usage**::

    plugins:
      - class: "my_pkg.plugins.TrafficMonitor"
        frequency: 2.0
        config:
          log_interval: 5.0
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Dict, List, Optional

from pybullet_fleet.config_utils import resolve_class

if TYPE_CHECKING:
    from pybullet_fleet.core_simulation import MultiRobotSimulationCore

logger = logging.getLogger(__name__)


class SimPlugin:
    """Base class for simulation plugins.

    Plugins receive lifecycle callbacks and can access the full
    simulation API via ``self.sim_core``.

    All lifecycle hooks are **optional no-ops** by default — override
    only what your plugin needs.  Step-based plugins override
    :meth:`on_step`; event-driven plugins subscribe to
    ``self.sim_core.events`` in :meth:`on_init` and leave
    ``on_step`` alone.

    Lifecycle::

        __init__(sim_core, config)   # construction (before world/robots)
        on_init()                    # after world/robots are spawned
        on_step(dt)                  # each simulation step (frequency-controlled)
        on_reset()                   # when simulation is reset
        on_shutdown()                # before PyBullet disconnects
    """

    def __init__(self, sim_core: "MultiRobotSimulationCore", config: Dict[str, Any]) -> None:
        self.sim_core = sim_core
        self.config = config
        self.frequency: Optional[float] = None  # Step frequency in Hz (None = every step)
        self._accumulator: float = 0.0  # Time accumulator for frequency control

    def on_init(self) -> None:
        """Called after world and robots are spawned.

        Override to query ``self.sim_core.agents``, subscribe to
        events, or perform one-time setup.  No-op by default.
        """

    def on_step(self, dt: float) -> None:
        """Called each simulation step (subject to frequency control).

        Override for step-based logic.  Event-driven plugins can leave
        this as a no-op and react via ``self.sim_core.events`` instead.

        Args:
            dt: Elapsed time since last call (seconds).  When frequency
                is set, this is the accumulated time since the last
                invocation, not the raw simulation timestep.
        """

    def on_reset(self) -> None:
        """Called when the simulation is reset.  Override to clear plugin state."""

    def on_shutdown(self) -> None:
        """Called before PyBullet disconnects.  Override to release resources."""


# ------------------------------------------------------------------
# YAML → plugin loading
# ------------------------------------------------------------------


def _load_plugins_from_config(
    plugin_configs: List[Dict[str, Any]],
    sim_core: "MultiRobotSimulationCore",
) -> List[SimPlugin]:
    """Load and instantiate plugins from YAML ``plugins:`` section.

    Each plugin's :attr:`frequency` is set from the YAML entry's
    ``frequency`` field (``None`` means every step).

    Args:
        plugin_configs: List of dicts, each with ``class`` (dotted path),
            optional ``frequency`` (Hz), and optional ``config`` (dict).
        sim_core: The simulation core instance passed to each plugin.

    Returns:
        List of instantiated :class:`SimPlugin` instances (with
        ``frequency`` already set).

    Raises:
        ValueError: If ``class`` is not a valid dotted path.
        ModuleNotFoundError: If the module cannot be imported.
        AttributeError: If the class is not found in the module.
        TypeError: If the class is not a :class:`SimPlugin` subclass.
    """
    plugins: List[SimPlugin] = []

    for entry in plugin_configs:
        dotted_path = entry["class"]
        freq = entry.get("frequency", None)
        config = entry.get("config", {})

        cls = resolve_class(dotted_path)

        if not (isinstance(cls, type) and issubclass(cls, SimPlugin)):
            raise TypeError(f"Plugin class must be a SimPlugin subclass, got {cls!r}. " f"Did you inherit from SimPlugin?")

        plugin = cls(sim_core, config)
        plugin.frequency = freq
        plugins.append(plugin)
        logger.info("Loaded plugin %r from %s", cls.__name__, dotted_path)

    return plugins
