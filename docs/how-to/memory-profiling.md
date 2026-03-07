# Memory Profiling User Guide

## Overview

Memory profiling is an optional feature in PyBulletFleet that tracks memory usage during simulation. This is particularly useful for:

- **Detecting memory leaks** in long-running simulations
- **Monitoring memory consumption** in multi-robot scenarios
- **Optimizing memory usage** by identifying memory-intensive operations
- **Continuous Integration (CI)** testing to prevent memory regressions

Memory profiling uses Python's built-in `tracemalloc` module, so no external dependencies are required.

---

## Quick Start

### 1. Enable Memory Profiling in Configuration

Add `enable_memory_profiling: true` to your YAML configuration file:

```yaml
# config.yaml
enable_profiling: true          # Time profiling (optional, independent)
enable_memory_profiling: true   # Memory profiling
profiling_interval: 100         # Report every 100 steps
```

### 2. Run Your Simulation

```python
from pybullet_fleet.core_simulation import MultiRobotSimulationCore

# Load configuration with memory profiling enabled
sim = MultiRobotSimulationCore.from_config("config.yaml")

# Run simulation
sim.run_simulation(duration=60.0)  # 60 seconds simulation time
```

### 3. Check Memory Usage Output

Memory statistics will be printed every `profiling_interval` steps:

```
[MEMORY] Last 100 steps: current=245.32MB (min=243.18, max=246.57), peak=248.45MB (max=250.12), growth=+3.25MB
```

---

## API Reference

### Enable/Disable Memory Profiling

**Configuration File:**
```yaml
enable_memory_profiling: true   # Enable memory profiling
profiling_interval: 100         # Report interval (steps)
```

**Python API:**
```python
sim = MultiRobotSimulationCore(
    enable_memory_profiling=True,
    profiling_interval=100
)
```

### Get Current Memory Usage

```python
# Get current memory usage (returns None if profiling not enabled)
mem_usage = sim.get_memory_usage()

if mem_usage:
    print(f"Current: {mem_usage['current_mb']:.2f} MB")
    print(f"Peak: {mem_usage['peak_mb']:.2f} MB")
```

**Return Value:**
- `None` if memory profiling is not enabled
- Dictionary with keys:
  - `current_mb`: Current memory usage in MB
  - `peak_mb`: Peak memory usage since profiling started

---

## Understanding Memory Statistics

### Memory Report Fields

```
[MEMORY] Last 100 steps: current=245.32MB (min=243.18, max=246.57), peak=248.45MB (max=250.12), growth=+3.25MB
```

- **current**: Average current memory usage over last N steps
  - **min**: Minimum current memory in the interval
  - **max**: Maximum current memory in the interval
- **peak**: Average peak memory (highest ever reached) over last N steps
  - **max**: Maximum peak memory in the interval
- **growth**: Memory increase from first to last sample in interval
  - Positive value indicates memory growth (potential leak)
  - Negative value indicates memory release
  - Zero or near-zero indicates stable memory usage

### Interpreting Growth Values

| Growth Value | Interpretation |
|--------------|----------------|
| `+0.5MB` to `+2MB` | Normal for initialization or object spawning |
| `+2MB` to `+10MB` | Monitor closely, may indicate inefficiency |
| `+10MB` or more | Likely memory leak, investigate immediately |
| `-1MB` to `+1MB` (steady) | Healthy, stable memory usage |
| Consistently positive | Potential memory leak |

---

## Use Cases

### 1. Detecting Memory Leaks in Long-Running Simulations

```python
# Enable memory profiling for long simulation
config = {
    "enable_memory_profiling": True,
    "profiling_interval": 500,  # Report every 500 steps
    "duration": 3600.0,  # 1 hour simulation
}

sim = MultiRobotSimulationCore.from_config(config)
sim.run_simulation()

# Check final memory usage
final_mem = sim.get_memory_usage()
print(f"Final memory: {final_mem['current_mb']:.2f} MB")
```

**Expected Output:**
```
[MEMORY] Last 500 steps: current=250.45MB (...), growth=+0.12MB
[MEMORY] Last 500 steps: current=250.58MB (...), growth=+0.15MB
[MEMORY] Last 500 steps: current=251.02MB (...), growth=+0.45MB  # Potential leak!
```

### 2. Comparing Memory Usage Between Configurations

```python
import pytest

def test_memory_usage_within_limits():
    """Ensure simulation memory stays below threshold."""
    config = {
        "enable_memory_profiling": True,
        "duration": 60.0,
    }

    sim = MultiRobotSimulationCore.from_config(config)
    sim.run_simulation()

    mem_usage = sim.get_memory_usage()
    assert mem_usage["current_mb"] < 500.0, "Memory usage exceeds 500MB limit"
```

### 3. Profiling Multi-Robot Scenarios

```python
# Test memory scaling with robot count
for num_robots in [10, 50, 100, 200]:
    config = {
        "enable_memory_profiling": True,
        "profiling_interval": 100,
    }

    sim = MultiRobotSimulationCore.from_config(config)

    # Spawn robots
    for i in range(num_robots):
        sim.spawn_robot(f"robot_{i}", position=[i, 0, 0])

    # Run for a while
    sim.run_simulation(duration=10.0)

    mem = sim.get_memory_usage()
    print(f"{num_robots} robots: {mem['current_mb']:.2f} MB")

    sim.close()
```

**Expected Output:**
```
10 robots: 125.34 MB
50 robots: 156.78 MB
100 robots: 245.12 MB
200 robots: 478.56 MB
```

### 4. Combining Time and Memory Profiling

```python
# Enable both profiling features
config = {
    "enable_profiling": True,        # Time profiling
    "enable_memory_profiling": True, # Memory profiling
    "profiling_interval": 100,
}

sim = MultiRobotSimulationCore.from_config(config)
sim.run_simulation(duration=30.0)
```

**Output:**
```
[PROFILING] Last 100 steps average: agent_update=0.45ms (22.5%), callbacks=0.12ms (6.0%), ...
[MEMORY] Last 100 steps: current=245.32MB (min=243.18, max=246.57), peak=248.45MB, growth=+0.15MB
```

---

## Integration with CI/CD

### pytest Test Case

```python
# tests/test_performance.py
import pytest
from pybullet_fleet.core_simulation import MultiRobotSimulationCore

def test_no_memory_leak_after_1000_steps():
    """Verify memory is stable over 1000 steps."""
    config = {
        "enable_memory_profiling": True,
        "gui": False,
    }

    sim = MultiRobotSimulationCore.from_config(config)
    sim.initialize_simulation()

    # Get baseline memory
    for _ in range(100):
        sim.step_once()
    baseline = sim.get_memory_usage()["current_mb"]

    # Run 1000 more steps
    for _ in range(1000):
        sim.step_once()
    final = sim.get_memory_usage()["current_mb"]

    # Memory growth should be less than 10MB
    growth = final - baseline
    assert growth < 10.0, f"Memory leak detected: {growth:.2f}MB growth"

    sim.close()
```

### GitHub Actions Workflow

```yaml
# .github/workflows/performance-test.yml
name: Performance Tests

on: [push, pull_request]

jobs:
  memory-test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.10'
      - name: Install dependencies
        run: pip install -r requirements.txt
      - name: Run memory profiling tests
        run: pytest tests/test_memory_profiling.py -v
      - name: Run long-running memory test
        run: |
          python -c "
          from pybullet_fleet.core_simulation import MultiRobotSimulationCore
          config = {'enable_memory_profiling': True, 'gui': False, 'duration': 60.0}
          sim = MultiRobotSimulationCore.from_config(config)
          sim.run_simulation()
          mem = sim.get_memory_usage()
          assert mem['current_mb'] < 500.0, f'Memory exceeded: {mem["current_mb"]:.2f}MB'
          "
```

---

## Best Practices

### ✅ DO

1. **Enable memory profiling for long-running simulations** (>1 minute)
2. **Monitor growth values** to detect potential leaks
3. **Set profiling_interval based on simulation length:**
   - Short simulations (< 1 min): `profiling_interval: 50`
   - Medium simulations (1-10 min): `profiling_interval: 100`
   - Long simulations (> 10 min): `profiling_interval: 500`
4. **Add memory assertions to tests:**
   ```python
   assert mem["current_mb"] < MAX_MEMORY_MB
   ```
5. **Profile before and after code changes** to detect regressions

### ❌ DON'T

1. **Don't enable memory profiling in production** (slight performance overhead)
2. **Don't panic over small growth values** (< 2MB per 100 steps is normal)
3. **Don't rely only on memory profiling** - use time profiling together
4. **Don't forget to close simulation** after profiling

---

## Performance Impact

Memory profiling uses Python's `tracemalloc` module, which has minimal overhead:

- **CPU overhead**: < 1% in most cases
- **Memory overhead**: ~5-10% additional memory for tracking
- **Recommended for**: Development, testing, debugging
- **Not recommended for**: Production deployments requiring maximum performance

---

## Troubleshooting

### Memory profiling returns None

**Problem:**
```python
mem = sim.get_memory_usage()
print(mem)  # None
```

**Solution:**
Ensure `enable_memory_profiling: true` in configuration:
```yaml
enable_memory_profiling: true
```

### No memory statistics printed

**Problem:**
Memory profiling enabled but no `[MEMORY]` logs appear.

**Solution:**
- Check `profiling_interval` setting (default: 100 steps)
- Run enough steps to trigger report: `sim.step_once()` × profiling_interval
- Ensure logging level allows INFO messages

### Memory keeps growing

**Problem:**
```
[MEMORY] Last 100 steps: growth=+5.2MB
[MEMORY] Last 100 steps: growth=+4.8MB
[MEMORY] Last 100 steps: growth=+5.1MB  # Continuously positive!
```

**Solution:**
1. Check for memory leak candidates:
   ```python
   # Bad: List comprehension creates new list every frame
   self._data = [x for x in self._data if condition(x)]

   # Good: In-place removal
   while self._data and not condition(self._data[0]):
       self._data.pop(0)
   ```

2. Use external profilers for detailed analysis:
   ```bash
   pip install memory_profiler
   mprof run python examples/your_demo.py
   mprof plot
   ```

---

## Comparison: Memory Profiling vs Time Profiling

| Feature | Memory Profiling | Time Profiling |
|---------|------------------|----------------|
| **Measures** | Memory usage (MB) | Execution time (ms) |
| **Detects** | Memory leaks, high memory usage | Performance bottlenecks |
| **Tool** | `tracemalloc` (built-in) | `time.perf_counter()` |
| **Overhead** | ~5-10% memory | < 0.1% CPU |
| **Use case** | Long-running stability | Optimization |
| **Config** | `enable_memory_profiling` | `enable_profiling` |

**Recommendation:** Use both together for comprehensive performance analysis.

---

## Example Output

### Normal Memory Usage (Healthy)

```
[MEMORY] Last 100 steps: current=245.32MB (min=243.18, max=246.57), peak=248.45MB (max=250.12), growth=+0.15MB
[MEMORY] Last 100 steps: current=245.48MB (min=244.32, max=246.89), peak=248.45MB (max=250.12), growth=+0.12MB
[MEMORY] Last 100 steps: current=245.55MB (min=244.89, max=246.34), peak=248.45MB (max=250.12), growth=-0.05MB
```

**Interpretation:** Stable memory usage (~245MB), minimal growth, healthy simulation.

### Memory Leak Detected (Unhealthy)

```
[MEMORY] Last 100 steps: current=245.32MB (min=243.18, max=246.57), peak=248.45MB (max=250.12), growth=+5.23MB
[MEMORY] Last 100 steps: current=250.55MB (min=248.92, max=252.14), peak=253.67MB (max=255.34), growth=+4.87MB
[MEMORY] Last 100 steps: current=255.42MB (min=253.78, max=257.09), peak=258.54MB (max=260.21), growth=+5.12MB
```

**Interpretation:** Continuous memory growth (+5MB per interval), potential memory leak, requires investigation.

---

## References

- [Python tracemalloc Documentation](https://docs.python.org/3/library/tracemalloc.html)
- [Real-time Synchronization Design](../architecture/realtime-sync) - Real-time synchronization design
- [Collision Detection Design](../architecture/collision-detection) - Collision detection system

---

## Summary

Memory profiling is a powerful tool for:
- ✅ Detecting memory leaks early in development
- ✅ Monitoring long-running simulations
- ✅ Preventing performance regressions in CI/CD
- ✅ Optimizing memory usage in multi-robot scenarios

Enable it during development and testing, disable it in production for maximum performance.
