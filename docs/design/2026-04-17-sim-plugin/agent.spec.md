# SimPlugin ABC — Agent Specification

## Requirements

### Functional

- `SimPlugin` ABC with `on_init()` and `on_step(dt: float)` as abstract methods
- `on_reset()` and `on_shutdown()` as optional hooks (default no-op)
- Constructor signature: `__init__(self, sim_core: MultiRobotSimulationCore, config: dict)`
- YAML `plugins:` section with `class`, `frequency`, `config` keys
- `importlib` dynamic loading (same pattern as `entity_classes`)
- Lifecycle: `__init__`(before spawn) → `on_init`(after spawn) → `on_step`(loop) → `on_reset`/`on_shutdown`
- Frequency control per plugin (Hz, None=every step) — same accumulator pattern as `register_callback`
- Programmatic API: `sim.register_plugin(instance)` and `sim.register_plugin(PluginClass, config={}, frequency=10.0)`
- `sim.plugins` read-only property returning list of active `SimPlugin` instances
- Error isolation: `on_step` exceptions caught and logged; `__init__`/`on_init` exceptions propagate

### Non-Functional

- Plugin `on_step` dispatch overhead < 5μs per plugin (dict lookup + accumulator comparison)
- Zero impact on existing tests (backward compatible, no API changes)
- No new required dependencies

## Constraints

- `SimPlugin` is NOT a `SimObject` subclass — separate inheritance tree
- Plugin accesses sim via `self.sim_core` (set in constructor)
- Must use `importlib.import_module` for dynamic loading (same as `entity_registry.py`)
- Tests must use `p.DIRECT` + `SimulationParams(gui=False, monitor=False)`
- No circular import concern: `sim_plugin.py` imports from `core_simulation` (forward ref with `TYPE_CHECKING`)

## Approach

Lightweight ABC with constructor dependency injection (`sim_core` + `config` dict).
YAML-driven dynamic import using the proven `entity_classes` pattern.
Frequency control using the same accumulator approach as `register_callback`.
Error isolation in `on_step` to prevent one bad plugin from crashing the simulation.

## Design

### Key Components

| Component | Responsibility | Location |
|-----------|---------------|----------|
| `SimPlugin` ABC | Plugin lifecycle interface | `pybullet_fleet/sim_plugin.py` (new) |
| `_load_plugins_from_config()` | YAML→plugin instantiation | `pybullet_fleet/sim_plugin.py` (new) |
| Plugin integration in core | `_step_plugins`, `_init_plugins`, reset/shutdown hooks | `pybullet_fleet/core_simulation.py` (modify) |
| Public API | `register_plugin()`, `plugins` property | `pybullet_fleet/core_simulation.py` (modify) |
| `__init__.py` | Export `SimPlugin` | `pybullet_fleet/__init__.py` (modify) |

### Architecture

```
YAML config
    │
    │  plugins:
    │    - class: "my_pkg.plugins.TrafficMonitor"
    │      frequency: 2.0
    │      config:
    │        log_interval: 5.0
    │
    ▼
_load_plugins_from_config()          # importlib.import_module + getattr
    │
    │  1. Validate: issubclass(cls, SimPlugin)
    │  2. Instantiate: plugin = cls(sim_core, config)    ← __init__ (before spawn)
    │  3. Store: self._plugins.append(plugin)
    │  4. Store: self._plugin_frequencies.append(freq)
    │  5. Store: self._plugin_accumulators.append(0.0)
    │
    ▼
from_yaml / from_dict
    │
    │  ┌─ entity_classes   (existing)
    │  ├─ plugins           ← NEW: _load_plugins_from_config()
    │  ├─ world loading
    │  ├─ robot spawning
    │  └─ _init_plugins()   ← NEW: call on_init() on all plugins
    │
    ▼
step_once()
    │
    │  ┌─ agent updates
    │  ├─ _step_plugins(dt)  ← NEW
    │  ├─ callbacks
    │  ├─ physics step
    │  ├─ collision detection
    │  └─ monitor update
    │
    ▼
reset()
    │  → plugin.on_reset() for each plugin  ← NEW (before re-spawn)
    │
    ▼
run_simulation() exit / explicit shutdown
    │  → plugin.on_shutdown() for each plugin  ← NEW (before p.disconnect)
```

### SimPlugin ABC

```python
# pybullet_fleet/sim_plugin.py

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Dict

if TYPE_CHECKING:
    from pybullet_fleet.core_simulation import MultiRobotSimulationCore


class SimPlugin(ABC):
    """Base class for simulation plugins.

    Plugins receive lifecycle callbacks and can access the full
    simulation API via ``self.sim_core``.

    Subclass and implement :meth:`on_init` and :meth:`on_step`.
    Override :meth:`on_reset` and :meth:`on_shutdown` as needed.
    """

    def __init__(self, sim_core: "MultiRobotSimulationCore", config: Dict[str, Any]) -> None:
        self.sim_core = sim_core
        self.config = config

    @abstractmethod
    def on_init(self) -> None:
        """Called after world and robots are spawned.

        Use this to query ``self.sim_core.agents`` or subscribe to events.
        """

    @abstractmethod
    def on_step(self, dt: float) -> None:
        """Called each simulation step (subject to frequency control).

        Args:
            dt: Elapsed time since last call (seconds). When frequency is
                set, this is the accumulated time since the last invocation,
                not the raw simulation timestep.
        """

    def on_reset(self) -> None:
        """Called when the simulation is reset. Override to clear plugin state."""

    def on_shutdown(self) -> None:
        """Called before PyBullet disconnects. Override to release resources."""
```

### Plugin loader

```python
# In pybullet_fleet/sim_plugin.py

import importlib
import logging
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


def _load_plugins_from_config(
    plugin_configs: List[Dict[str, Any]],
    sim_core: "MultiRobotSimulationCore",
) -> tuple:
    """Load and instantiate plugins from YAML config.

    Returns:
        (plugins, frequencies, accumulators) — parallel lists.
    """
    plugins: List[SimPlugin] = []
    frequencies: List[Optional[float]] = []
    accumulators: List[float] = []

    for entry in plugin_configs:
        dotted_path = entry["class"]
        freq = entry.get("frequency", None)
        config = entry.get("config", {})

        if not isinstance(dotted_path, str) or "." not in dotted_path:
            raise ValueError(
                f"plugins[].class must be a dotted Python path "
                f"'module.ClassName', got: {dotted_path!r}"
            )

        module_path, class_name = dotted_path.rsplit(".", 1)
        module = importlib.import_module(module_path)
        cls = getattr(module, class_name)

        if not (isinstance(cls, type) and issubclass(cls, SimPlugin)):
            raise TypeError(
                f"Plugin class must be a SimPlugin subclass, got {cls!r}"
            )

        plugin = cls(sim_core, config)
        plugins.append(plugin)
        frequencies.append(freq)
        accumulators.append(0.0)
        logger.info("Loaded plugin %r from %s", class_name, dotted_path)

    return plugins, frequencies, accumulators
```

### Core simulation integration

```python
# In MultiRobotSimulationCore.__init__():
self._plugins: List[SimPlugin] = []
self._plugin_frequencies: List[Optional[float]] = []
self._plugin_accumulators: List[float] = []

# In from_yaml / from_dict (after entity_classes, before world/robots):
plugin_configs = config.get("plugins", [])
if plugin_configs:
    from pybullet_fleet.sim_plugin import _load_plugins_from_config
    plugins, freqs, accums = _load_plugins_from_config(plugin_configs, sim)
    sim._plugins.extend(plugins)
    sim._plugin_frequencies.extend(freqs)
    sim._plugin_accumulators.extend(accums)

# After world + robots are loaded:
sim._init_plugins()

# New methods on MultiRobotSimulationCore:

def _init_plugins(self) -> None:
    """Call on_init() on all loaded plugins (after spawn)."""
    for plugin in self._plugins:
        plugin.on_init()

def _step_plugins(self, dt: float) -> None:
    """Dispatch on_step() to plugins respecting frequency control."""
    for i, plugin in enumerate(self._plugins):
        freq = self._plugin_frequencies[i]
        if freq is None:
            try:
                plugin.on_step(dt)
            except Exception:
                logger.exception("Plugin %s.on_step() failed", type(plugin).__name__)
        else:
            self._plugin_accumulators[i] += dt
            interval = 1.0 / freq
            if self._plugin_accumulators[i] >= interval:
                try:
                    plugin.on_step(self._plugin_accumulators[i])
                except Exception:
                    logger.exception("Plugin %s.on_step() failed", type(plugin).__name__)
                self._plugin_accumulators[i] = 0.0

def register_plugin(
    self,
    plugin_or_cls,
    config: Optional[Dict[str, Any]] = None,
    frequency: Optional[float] = None,
) -> "SimPlugin":
    """Register a plugin programmatically.

    Args:
        plugin_or_cls: Either a SimPlugin instance or a SimPlugin subclass.
        config: Config dict (only used when plugin_or_cls is a class).
        frequency: Step frequency in Hz (None = every step).

    Returns:
        The registered plugin instance.
    """
    if isinstance(plugin_or_cls, SimPlugin):
        plugin = plugin_or_cls
    elif isinstance(plugin_or_cls, type) and issubclass(plugin_or_cls, SimPlugin):
        plugin = plugin_or_cls(self, config or {})
    else:
        raise TypeError(f"Expected SimPlugin instance or subclass, got {plugin_or_cls!r}")

    self._plugins.append(plugin)
    self._plugin_frequencies.append(frequency)
    self._plugin_accumulators.append(0.0)
    return plugin

@property
def plugins(self) -> List["SimPlugin"]:
    """Read-only list of active plugins."""
    return list(self._plugins)

# In step_once(), after agent updates and before _check_callbacks:
self._step_plugins(dt)

# In reset(), after clearing state:
for plugin in self._plugins:
    plugin.on_reset()

# In run_simulation() exit, before p.disconnect():
for plugin in self._plugins:
    try:
        plugin.on_shutdown()
    except Exception:
        logger.exception("Plugin %s.on_shutdown() failed", type(plugin).__name__)
```

### YAML config format

```yaml
# config.yaml

plugins:
  - class: "my_pkg.plugins.TrafficMonitor"
    frequency: 2.0           # Hz, optional (None = every step)
    config:                   # optional dict passed to __init__
      log_interval: 5.0
      output_dir: "/tmp/logs"

  - class: "my_pkg.plugins.CollisionLogger"
    # no frequency → called every step
    config:
      log_file: "collisions.csv"

entity_classes:
  forklift: "my_pkg.entities.ForkliftAgent"

simulation:
  timestep: 0.01
  physics: false

entities:
  - type: forklift
    urdf_path: robots/forklift.urdf
    count: 10
```

### Code Patterns to Follow

**Dynamic import pattern** (from `entity_registry.py`):
```python
module_path, class_name = dotted_path.rsplit(".", 1)
module = importlib.import_module(module_path)
cls = getattr(module, class_name)
```

**Frequency accumulator pattern** (from `_check_callbacks` in `core_simulation.py`):
```python
cb["last_exec"] += dt
interval = 1.0 / cb["frequency"]
if cb["last_exec"] >= interval:
    cb["func"](self, cb["last_exec"])
    cb["last_exec"] = 0.0
```

**Lazy import in from_yaml/from_dict** (from entity_classes integration):
```python
entity_classes = config.get("entity_classes")
if entity_classes:
    from pybullet_fleet.entity_registry import register_entity_classes_from_config
    register_entity_classes_from_config(entity_classes)
```

## File References

Files the plan agent MUST read before planning:

- `pybullet_fleet/entity_registry.py` — importlib dynamic load pattern, validation, logging
- `pybullet_fleet/controller.py:99-200` — Controller ABC pattern (ABC with abstract methods)
- `pybullet_fleet/events.py:98-150` — EventBus class (plugins can subscribe via `self.sim_core.events`)
- `pybullet_fleet/core_simulation.py:503-525` — `register_callback` and frequency accumulator
- `pybullet_fleet/core_simulation.py:2423-2490` — `reset()` method (plugin on_reset hook point)
- `pybullet_fleet/core_simulation.py:2830-2860` — `run_simulation` exit (plugin on_shutdown hook point)
- `pybullet_fleet/core_simulation.py:2912-2960` — `step_once()` method (plugin on_step hook point)
- `tests/conftest.py` — MockSimCore, autouse fixtures, test patterns
- `tests/test_entity_registry.py` — dynamic import test patterns (TestRegisterFromConfig)
- `pybullet_fleet/__init__.py` — public API exports

## Testing Strategy

| Test | What it verifies |
|------|------------------|
| `test_plugin_lifecycle` | `on_init` → `on_step` × N → `on_shutdown` called in order |
| `test_plugin_frequency` | Plugin with `frequency: 2.0` fires ~2x/sec, not every step |
| `test_plugin_config_passthrough` | `config` dict from YAML arrives in `plugin.config` |
| `test_plugin_access_sim_core` | `plugin.sim_core.agents` is populated in `on_init` |
| `test_plugin_on_reset` | `on_reset` called during `sim.reset()` |
| `test_plugin_step_error_isolated` | Exception in `on_step` is logged, sim continues |
| `test_plugin_load_from_yaml` | Full YAML with `plugins:` section → plugins instantiated |
| `test_plugin_bad_class_path` | Invalid dotted path raises `ImportError` / `ValueError` |
| `test_plugin_not_subclass` | Class that isn't `SimPlugin` subclass raises `TypeError` |
| `test_register_plugin_api` | `sim.register_plugin()` programmatic API works |

All tests: `SimulationParams(gui=False, monitor=False)` + `p.DIRECT`.

## Success Criteria

- [ ] `SimPlugin` ABC defined in `pybullet_fleet/sim_plugin.py` with `on_init`, `on_step` abstract
- [ ] `_load_plugins_from_config()` loads plugins from YAML `plugins:` section
- [ ] `_step_plugins(dt)` dispatches with frequency control
- [ ] `_init_plugins()` calls `on_init()` after spawn
- [ ] `on_reset()` called in `reset()`
- [ ] `on_shutdown()` called before `p.disconnect()`
- [ ] `on_step` exceptions caught and logged (sim continues)
- [ ] `register_plugin()` programmatic API works
- [ ] `sim.plugins` property returns read-only list
- [ ] `SimPlugin` exported from `pybullet_fleet.__init__`
- [ ] All existing tests pass (backward compatible)
- [ ] 10+ new tests covering lifecycle, frequency, errors, YAML loading
- [ ] `make verify` passes
