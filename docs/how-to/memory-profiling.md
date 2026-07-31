# Memory Profiling User Guide

## Overview

Memory profiling is an optional feature in PyBulletFleet that tracks memory usage during simulation. This is particularly useful for:

- **Detecting memory leaks** in long-running simulations
- **Monitoring memory consumption** in multi-robot scenarios
- **Optimizing memory usage** by identifying memory-intensive operations
- **Continuous Integration (CI)** testing to prevent memory regressions

Memory profiling uses Python's built-in `tracemalloc` module, so no external
dependencies are required. It measures Python allocations tracked by
`tracemalloc`; it is not a measurement of process RSS or all memory allocated
by native libraries such as PyBullet. Use an OS-level process monitor when RSS
is the quantity you need to bound.

---

## Quick Start

### 1. Enable Memory Profiling in Configuration

Add `enable_memory_profiling: true` to your YAML configuration file:

```yaml
# config.yaml
simulation:
  enable_time_profiling: true      # Optional and independent
  enable_memory_profiling: true
  profiling_interval: 100         # Report every 100 steps
```

### 2. Run Your Simulation

```python
from pybullet_fleet.core_simulation import MultiRobotSimulationCore

# Load configuration with memory profiling enabled
sim = MultiRobotSimulationCore.from_yaml("config.yaml")

# Run simulation
sim.run_simulation(duration=60.0)  # 60 seconds simulation time
```

### 3. Check Memory Usage Output

Memory statistics will be printed every `profiling_interval` steps:

```
[MEMORY] Last 100 steps: current=...MB (min=..., max=...), peak=...MB (max=...), growth=+...MB
```

---

## API Reference

### Enable/Disable Memory Profiling

**Configuration File:**
```yaml
simulation:
  enable_memory_profiling: true   # Enable memory profiling
  profiling_interval: 100         # Report interval (steps)
```

**Python API:**
```python
from pybullet_fleet.core_simulation import SimulationParams

sim = MultiRobotSimulationCore(
    SimulationParams(
        enable_memory_profiling=True,
        profiling_interval=100,
    )
)
```

### Get Current Memory Usage

`get_memory_usage()` can be called while `tracemalloc` is active, including
inside a callback during `run_simulation()`. PyBulletFleet starts tracing during
`initialize_simulation()`, so a newly constructed simulation has no value yet.

```python
# Inside a callback: monitor memory every N steps
def memory_monitor(sim_core, dt):
    mem = sim_core.get_memory_usage()
    if mem:
        print(f"Heap: {mem['current_mb']:.2f} MB, Peak: {mem['peak_mb']:.2f} MB")

sim.register_callback(memory_monitor, frequency=10)
sim.run_simulation()
```

You can also call it after `run_simulation()` returns to get the final snapshot:

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

`tracemalloc` is process-global. If another library has already started it,
PyBulletFleet reuses that session. The interval report resets the traced peak
when Python provides `tracemalloc.reset_peak()`, so do not share the same
tracing session when another component needs an independent cumulative peak.

---

## Understanding Memory Statistics

### Memory Report Fields

```
[MEMORY] Last 100 steps: current=...MB (min=..., max=...), peak=...MB (max=...), growth=+...MB
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

Treat growth as a trend, not a universal threshold. Object creation, Python's
allocator, caches, and the selected workload can all change the value. Compare
multiple equivalent intervals after warm-up; persistent growth under the same
workload is a reason to investigate, while a one-off increase is not by itself
evidence of a leak.

---

## Use Case: Detecting Memory Leaks in Long-Running Simulations

```python
config = {
    "simulation": {
        "enable_memory_profiling": True,
        "profiling_interval": 500,  # Report every 500 steps
    },
}

sim = MultiRobotSimulationCore.from_dict(config)
# ... spawn agents ...
sim.run_simulation()  # Ctrl+C or GUI close returns normally

# Check final memory usage
final_mem = sim.get_memory_usage()
if final_mem:
    print(f"Final memory: {final_mem['current_mb']:.2f} MB")
```

During the run, watch the `growth` field in the periodic reports:

```
[MEMORY] Last 500 steps: current=...MB (...), growth=+0.12MB
[MEMORY] Last 500 steps: current=...MB (...), growth=+0.15MB
[MEMORY] Last 500 steps: current=...MB (...), growth=+5.45MB
```

The third value warrants comparison against repeated, warmed-up intervals; it
is not automatically a leak solely because it is larger than the first two.

For CI, assert a ceiling in a test:

```python
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams


def test_memory_usage_within_limits():
    """Ensure simulation memory stays below threshold."""
    sim = MultiRobotSimulationCore(
        SimulationParams(
            enable_memory_profiling=True,
        )
    )
    # ... spawn agents, run ...
    mem = sim.get_memory_usage()
    assert mem and mem["current_mb"] < 500.0, "Memory usage exceeds 500 MB limit"
```

---

## Best Practices

### ✅ DO

1. **Enable memory profiling for leak investigations and development runs**
2. **Monitor growth values** to detect potential leaks
3. **Set profiling_interval based on simulation length:**
   - Use a short interval to inspect short-lived spikes
   - Use a longer interval to reduce log volume in long runs
4. **Add workload-specific memory assertions to tests:**
   ```python
   assert mem["current_mb"] < MAX_MEMORY_MB
   ```
5. **Profile before and after code changes** to detect regressions

### ❌ DON'T

1. **Don't enable memory profiling in production or timing benchmarks**
2. **Don't infer a leak from one interval alone**
3. **Don't rely only on memory profiling** - use time profiling together
4. **Don't treat traced Python heap as process RSS**

---

## Performance Impact

Memory profiling uses Python's `tracemalloc` module. It is useful for leak
investigation, but it can materially slow allocation-heavy hot loops:

- **CPU overhead**: workload-dependent; can be large in step-time / RTF benchmarks
- **Memory overhead**: workload-dependent additional memory for allocation tracking
- **Recommended for**: Development, testing, debugging
- **Not recommended for**: Production deployments or benchmark runs measuring
  maximum RTF / step time

The benchmark workers keep `tracemalloc` out of the timed simulation section for
this reason. Use RSS-based process memory for release benchmark tables, and use
`tracemalloc` separately when the goal is memory leak diagnosis.

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
simulation:
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
1. Check for memory leak candidates. Prefer bounded buffers or aggregate values
   rather than retaining one new value per step:
   ```python
   from collections import deque

   # Initialise once, then retain only a bounded history.
   self._recent_samples = deque(maxlen=1_000)
   # Each update:
   self._recent_samples.append(sample)
   ```

2. Use external profilers for detailed analysis:
   ```bash
   pip install memory_profiler
   mprof run python examples/your_demo.py
   mprof plot
   ```

---

## Example Output

### Illustrative Stable Trend

```
[MEMORY] Last 100 steps: current=...MB (min=..., max=...), peak=...MB (max=...), growth=+0.15MB
[MEMORY] Last 100 steps: current=...MB (min=..., max=...), peak=...MB (max=...), growth=+0.12MB
[MEMORY] Last 100 steps: current=...MB (min=..., max=...), peak=...MB (max=...), growth=-0.05MB
```

**Interpretation:** Repeated intervals remain in a similar range after warm-up.

### Illustrative Growth Requiring Investigation

```
[MEMORY] Last 100 steps: current=...MB (min=..., max=...), peak=...MB (max=...), growth=+5.23MB
[MEMORY] Last 100 steps: current=...MB (min=..., max=...), peak=...MB (max=...), growth=+4.87MB
[MEMORY] Last 100 steps: current=...MB (min=..., max=...), peak=...MB (max=...), growth=+5.12MB
```

**Interpretation:** Repeated positive growth under an unchanged workload merits
allocation-level investigation, but should be reproduced before concluding that
there is a leak.

---

## References

- [Python tracemalloc Documentation](https://docs.python.org/3/library/tracemalloc.html)
- [Real-time Synchronization Design](../architecture/realtime-sync) - Real-time synchronization design
- [Collision Detection Overview](../architecture/collision-overview) - Collision detection system
- [Profiling Guide](../benchmarking/profiling-guide) — Benchmark scripts for CPU time analysis (`simulation_profiler.py`, `wrapper_overhead.py`, etc.)
