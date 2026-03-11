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
enable_time_profiling: true          # Time profiling (optional, independent)
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

`get_memory_usage()` can be called **at any time** while `tracemalloc` is active —
including inside a callback during `run_simulation()`:

```python
# Inside a callback: monitor memory every N steps
def memory_monitor(sim_core, dt):
    mem = sim_core.get_memory_usage()
    if mem:
        print(f"Heap: {mem['current_mb']:.2f} MB, Peak: {mem['peak_mb']:.2f} MB")

sim.register_callback(memory_monitor, frequency=10)
sim.run_simulation()
```

You can also call it after the simulation ends to get the final snapshot:

```python
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

## Use Case: Detecting Memory Leaks in Long-Running Simulations

```python
config = {
    "enable_memory_profiling": True,
    "profiling_interval": 500,  # Report every 500 steps
}

sim = MultiRobotSimulationCore.from_config(config)
# ... spawn agents ...
sim.run_simulation()  # Ctrl+C or GUI close returns normally

# Check final memory usage
final_mem = sim.get_memory_usage()
if final_mem:
    print(f"Final memory: {final_mem['current_mb']:.2f} MB")
```

During the run, watch the `growth` field in the periodic reports:

```
[MEMORY] Last 500 steps: current=250.45MB (...), growth=+0.12MB   # stable ✓
[MEMORY] Last 500 steps: current=250.58MB (...), growth=+0.15MB   # stable ✓
[MEMORY] Last 500 steps: current=251.02MB (...), growth=+5.45MB   # investigate!
```

For CI, assert a ceiling in a test:

```python
def test_memory_usage_within_limits():
    """Ensure simulation memory stays below threshold."""
    sim = MultiRobotSimulationCore(
        enable_memory_profiling=True,
    )
    # ... spawn agents, run ...
    mem = sim.get_memory_usage()
    assert mem and mem["current_mb"] < 500.0, "Memory usage exceeds 500 MB limit"
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
- [Collision Detection Overview](../architecture/collision-overview) - Collision detection system
- [Profiling Guide](../benchmarking/profiling-guide) — Benchmark scripts for CPU time analysis (`simulation_profiler.py`, `wrapper_overhead.py`, etc.)
