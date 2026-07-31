# Logging

PyBulletFleet uses Python's standard `logging` package. This page distinguishes
the log-level configuration used by simulation users from lazy message
construction used by developers writing hot-path code.

## Configure Simulation Logs

Set the level in the `simulation` section of a YAML configuration, or through
`SimulationParams`. The default is `warn` (equivalent to Python's `WARNING`).

```yaml
simulation:
  log_level: info
```

```python
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams

sim = MultiRobotSimulationCore(
    SimulationParams(
        log_level="info",
    )
)
```

Use the usual Python logging names: `debug`, `info`, `warning` (or `warn`),
`error`, and `critical`. For example, time and memory profiling summaries are
logged at `INFO`, so use `log_level: info` when those summaries are required.

### Application integration

PyBulletFleet configures the Python root logger when
`MultiRobotSimulationCore` is created. Consequently, `SimulationParams.log_level`
affects other libraries that use the root logger in the same process. Configure
handlers, output destinations, and application-wide formatting in the host
application using the standard `logging` API.

`PYBULLET_LOG_LEVEL` is read when `pybullet_fleet.core_simulation` is imported,
but creating a simulation subsequently applies its `SimulationParams.log_level`.
Use the configuration field or constructor parameter as the authoritative
per-simulation setup rather than relying on that environment variable.

## Write Efficient Logs in Custom Code

Python logging already defers interpolation when arguments are passed
separately. Prefer this for simple values:

```python
logger.debug("agent=%s goal=%s", agent.name, goal)
```

Avoid eager f-strings for a disabled level:

```python
# Formats the array before logging decides whether DEBUG is enabled.
logger.debug(f"pose={pose_array}")
```

Use `LazyLogger` when constructing the message requires an expensive
calculation, array conversion, or another operation that should not run when
the level is filtered out.

```python
from pybullet_fleet.logging_utils import get_lazy_logger

logger = get_lazy_logger(__name__)

logger.debug(lambda: f"mean={samples.mean():.3f}, first={samples[:10]}")
```

For several related messages, an explicit standard logging guard is equally
clear:

```python
import logging

if logger.isEnabledFor(logging.DEBUG):
    logger.debug("direction=%s", direction)
    logger.debug("alignment=%.3f", alignment)
```

## Lazy Logger APIs

### Module-level logger

Use `get_lazy_logger(__name__)` for ordinary module-level logging.

```python
from pybullet_fleet.logging_utils import get_lazy_logger

logger = get_lazy_logger(__name__)
logger.info("World loaded")
logger.debug(lambda: f"objects={len(objects)}, aabbs={aabbs}")
```

`debug`, `info`, `warning`, `error`, and `critical` accept either a plain
string or a zero-argument callable. A plain string is already constructed
before the call; use a callable only when deferring work matters.

`LazyLogger.isEnabledFor(level)` and `LazyLogger.logger` expose the equivalent
standard logging check and underlying `logging.Logger` when needed. Existing
loggers can be wrapped with `wrap_existing_logger(logging.getLogger(__name__))`.

### Instance-level logger with a prefix

Use `get_named_lazy_logger()` when every record from an object should include a
stable identifier. `set_prefix()` can update that identifier after construction.

```python
from pybullet_fleet.logging_utils import get_named_lazy_logger

self._log = get_named_lazy_logger(__name__, prefix=f"[Agent:{self.object_id}] ")
self._log.info("Path complete")
self._log.debug(lambda: f"pose={self.get_pose()}")
```

The prefix and callable are applied only when that level is enabled.

## Choosing an Approach

| Situation | Preferred form |
|---|---|
| Constant or already-available scalar values | `logger.debug("count=%s", count)` |
| Expensive computation only needed for a message | `lazy_logger.debug(lambda: f"stats={compute_stats()}")` |
| Several related debug records | `if logger.isEnabledFor(logging.DEBUG): ...` |
| Per-object records that need an identifier | `get_named_lazy_logger()` |

Lazy logging avoids unnecessary work; its performance benefit depends on the
message construction, logging configuration, and workload. Measure the real
hot path before making broad changes.

## Verify Changes

Run the focused tests after changing the logging helpers:

```bash
pytest tests/test_logging_utils.py
```

For repository performance investigations, use the maintained profiling tools
under `benchmark/profiling/`, such as
`benchmark/profiling/agent_manager_set_goal.py`. See the
[Profiling Guide](../benchmarking/profiling-guide) for the broader workflow.

## See Also

- [Time Profiling](time-profiling) — Per-step timing and programmatic results
- [Custom Profiling](custom-profiling) — Custom timing fields in callbacks and plugins
