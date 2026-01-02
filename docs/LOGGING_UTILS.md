# Logging Utilities - Usage Guide

## Overview

`logging_utils.py` provides lazy evaluation logging to prevent expensive string formatting (e.g., NumPy array conversion) when the log level is disabled.

## Performance Impact

**Without lazy evaluation:**
```python
# ❌ BAD: Array is converted to string even if DEBUG is disabled
logger.debug(f"Array data: {numpy_array}")  # 687ms for 10k calls
```

**With lazy evaluation:**
```python
# ✅ GOOD: Array conversion only happens if DEBUG is enabled
lazy_logger.debug(lambda: f"Array data: {numpy_array}")  # 5ms for 10k calls
```

**Performance: ~145x faster** when log level is disabled!

## When to Use Lazy Logging

| Use Case | Method | Reason |
|----------|--------|--------|
| **NumPy arrays** | LazyLogger or isEnabledFor | Array to string conversion is expensive |
| **Large objects** | LazyLogger or isEnabledFor | repr/str conversion is expensive |
| **Expensive computations** | LazyLogger | Skip computation entirely |
| **Simple strings/numbers** | Standard logger | No overhead benefit |
| **INFO or higher levels** | Standard logger | Usually executed anyway |

## Usage Examples

### Method 1: LazyLogger (Recommended for New Code)

```python
from pybullet_fleet.logging_utils import get_lazy_logger

# Create lazy logger
lazy_logger = get_lazy_logger(__name__)

# Use with lambda for expensive operations
import numpy as np
arr = np.random.rand(1000)

lazy_logger.debug(lambda: f"Array stats: mean={arr.mean()}, std={arr.std()}, data={arr[:10]}")
lazy_logger.info(lambda: f"Large object: {expensive_object}")

# Simple strings work too (lambda is optional but consistent)
lazy_logger.debug(lambda: "Simple message")
lazy_logger.debug("Simple message also works")  # Backward compatible
```

### Method 2: isEnabledFor Check (Good for Existing Code)

```python
import logging

logger = logging.getLogger(__name__)

# Protect expensive logging with isEnabledFor
if logger.isEnabledFor(logging.DEBUG):
    logger.debug(f"Array data: {numpy_array}")
    logger.debug(f"Position: {position[:2]}")
    logger.debug(f"Orientation: {quaternion}")
```

### Method 3: Migration Helper (Best for Gradual Migration)

```python
from pybullet_fleet.logging_utils import migrate_logger

# Get both standard and lazy loggers
logger, lazy_logger = migrate_logger(__name__)

# Existing code continues to work
logger.info("Simple message")
logger.warning(f"Count: {count}")

# New expensive logging uses lazy logger
lazy_logger.debug(lambda: f"Expensive: {numpy_array}")
```

## Real-World Example: agent.py

**Before optimization:**
```python
def _init_differential_rotation_trajectory(self, goal):
    # These 4 lines caused 316ms overhead for 1000 calls!
    logger.debug(f"Agent {self.body_id} checking orientation alignment:")
    logger.debug(f"  Movement direction: {x_axis_target}")  # NumPy array
    logger.debug(f"  Goal's X-axis: {x_axis_goal}")         # NumPy array
    logger.debug(f"  Alignment: {alignment:.3f}")
```

**After optimization (76% faster):**
```python
def _init_differential_rotation_trajectory(self, goal):
    if logger.isEnabledFor(logging.DEBUG):
        logger.debug(f"Agent {self.body_id} checking orientation alignment:")
        logger.debug(f"  Movement direction: {x_axis_target}")
        logger.debug(f"  Goal's X-axis: {x_axis_goal}")
        logger.debug(f"  Alignment: {alignment:.3f}")
```

## API Reference

### `get_lazy_logger(name: str) -> LazyLogger`

Create a new lazy logger.

```python
logger = get_lazy_logger(__name__)
logger.debug(lambda: f"Data: {array}")
```

### `wrap_existing_logger(logger: logging.Logger) -> LazyLogger`

Wrap an existing logger with lazy evaluation.

```python
std_logger = logging.getLogger(__name__)
lazy_logger = wrap_existing_logger(std_logger)
lazy_logger.debug(lambda: f"Data: {array}")
```

### `migrate_logger(logger_name: str) -> Tuple[logging.Logger, LazyLogger]`

Get both standard and lazy loggers for gradual migration.

```python
logger, lazy_logger = migrate_logger(__name__)
logger.info("Standard")
lazy_logger.debug(lambda: f"Lazy: {array}")
```

### `LazyLogger` Methods

All standard logging methods are supported:
- `debug(msg_func, *args, **kwargs)`
- `info(msg_func, *args, **kwargs)`
- `warning(msg_func, *args, **kwargs)`
- `error(msg_func, *args, **kwargs)`
- `critical(msg_func, *args, **kwargs)`

**msg_func** can be:
- A lambda/callable: `lambda: f"Data: {array}"` (lazy evaluation)
- A plain string: `"Simple message"` (backward compatible)

## Testing

Run the test suite to verify functionality and see performance benchmarks:

```bash
python PyBulletFleet/tests/test_logging_utils.py
```

Expected output:
```
Performance Benchmark (10,000 iterations)
Standard logger:           686.90ms (baseline)
Standard with check:       1.08ms (99.8% faster)
Lazy logger:               4.75ms (99.3% faster)

Conclusion: Lazy logger is ~145x faster for disabled log levels
```

## Best Practices

### ✅ DO

```python
# Use lazy logging for arrays and expensive operations
lazy_logger.debug(lambda: f"Array: {np_array}")
lazy_logger.debug(lambda: f"Mean: {data.mean()}")

# Or use isEnabledFor for multiple related logs
if logger.isEnabledFor(logging.DEBUG):
    logger.debug(f"Array 1: {arr1}")
    logger.debug(f"Array 2: {arr2}")
```

### ❌ DON'T

```python
# Don't use lazy logging for simple strings (unnecessary overhead)
lazy_logger.debug(lambda: "Simple message")  # Just use: logger.debug("Simple message")

# Don't forget the lambda
lazy_logger.debug(f"Array: {arr}")  # ❌ Still evaluates f-string!
lazy_logger.debug(lambda: f"Array: {arr}")  # ✅ Correct
```

## Performance Tips

1. **Group expensive logs**: Use `isEnabledFor` to wrap multiple related DEBUG logs
2. **Profile first**: Use `profile_agent_manager_set_goal.py` to identify bottlenecks
3. **Focus on hot paths**: Optimize logs in frequently-called functions first
4. **Keep it simple**: Don't over-optimize INFO/WARNING/ERROR logs

## Migration Checklist

- [ ] Identify hot paths (frequently called functions)
- [ ] Search for DEBUG logs with arrays/objects: `logger.debug(f".*\{.*\[`
- [ ] Add `isEnabledFor` checks or use `LazyLogger`
- [ ] Run tests to verify functionality
- [ ] Profile to measure improvement
- [ ] Update documentation

## Related Files

- `pybullet_fleet/logging_utils.py` - Implementation
- `tests/test_logging_utils.py` - Tests and benchmarks
- `tests/profile_agent_manager_set_goal.py` - Real-world profiling example
- `pybullet_fleet/agent.py` - Example of optimized logging in production code
