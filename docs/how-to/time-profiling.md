# Time Profiling User Guide

## Overview

Time profiling measures **per-step execution time** of each simulation component inside `step_once()`.
It helps identify performance bottlenecks by breaking down where time is spent every simulation step.

| Component | What it measures |
|-----------|-----------------|
| `agent_update` | All `Agent.update(dt)` calls (kinematics, path following, actions) |
| `callbacks` | User-registered callbacks via `register_callback()` |
| `step_simulation` | `p.stepSimulation()` (physics mode) or AABB/spatial grid updates |
| `collision_check` | Spatial-hash broad phase + PyBullet narrow phase |
| `monitor_update` | `DataMonitor` overlay refresh |
| `total` | End-to-end step time |

**Overhead**: < 0.1% CPU — safe to leave enabled during development.

---

## Quick Start

### 1. Enable Time Profiling in Configuration

```yaml
# config.yaml
enable_time_profiling: true   # Enable step-timing reports
profiling_interval: 100       # Print average every 100 steps
log_level: "info"             # Required: reports use logger.info()
```

### 2. Run Your Simulation

```python
from pybullet_fleet.core_simulation import SimulationParams, MultiRobotSimulationCore

params = SimulationParams.from_config("config/config.yaml")
# Or directly:
params = SimulationParams(
    enable_time_profiling=True,
    profiling_interval=100,
)
sim = MultiRobotSimulationCore(params)
```

### 3. Read the Output

Every `profiling_interval` steps, a summary line is logged:

```
[PROFILING] Last 100 steps average: agent_update=0.45ms (35.2%), callbacks=0.02ms (1.6%),
step_simulation=0.00ms (0.0%), collision_check=0.75ms (58.6%), monitor_update=0.06ms (4.7%),
total=1.28ms (100.0%)
```

Each component shows **average time (ms)** and **percentage of total**.

---

## Configuration Reference

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_time_profiling` | bool | `false` | Enable per-step timing measurement |
| `profiling_interval` | int | `100` | Steps between summary reports |

Both parameters can be set via YAML config or `SimulationParams` constructor.

`profiling_interval` is shared with memory profiling — it controls the reporting
interval for both systems.

---

## Interpreting Results

### Healthy Profile (kinematics mode, 100 robots)

```
[PROFILING] Last 100 steps average: agent_update=0.45ms (35.2%), collision_check=0.75ms (58.6%), total=1.28ms
```

- `collision_check` dominates — normal for large scenes
- `step_simulation` ≈ 0 — expected in physics-off mode
- Total < 2ms per step — good performance

### Bottleneck: Collision Detection

```
[PROFILING] Last 100 steps average: agent_update=0.50ms (10.0%), collision_check=4.50ms (90.0%), total=5.00ms
```

- `collision_check` > 80% → optimize spatial hash cell size or reduce collision-enabled objects
- See [Spatial Hash Configuration](spatial-hash-config) for tuning

### Bottleneck: Agent Update

```
[PROFILING] Last 100 steps average: agent_update=3.20ms (80.0%), collision_check=0.75ms (18.8%), total=4.00ms
```

- `agent_update` > 70% → check action complexity, path length, or number of attached objects

### Bottleneck: Physics Step

```
[PROFILING] Last 100 steps average: step_simulation=8.00ms (72.7%), collision_check=2.50ms (22.7%), total=11.00ms
```

- `step_simulation` dominates → expected in physics-on mode
- Consider switching to kinematics mode if physics fidelity is not required

---

## Programmatic Access

Use `return_profiling=True` on `step_once()` to get timing data as a dictionary
instead of logging:

```python
for _ in range(1000):
    timings = sim.step_once(return_profiling=True)
    if timings:
        print(f"Collision: {timings['collision_check']:.2f}ms")
        print(f"Total: {timings['total']:.2f}ms")
```

Return value:

```python
{
    "agent_update": float,      # ms
    "callbacks": float,         # ms
    "step_simulation": float,   # ms
    "collision_check": float,   # ms
    "monitor_update": float,    # ms
    "total": float,             # ms
}
```

This is useful for custom analysis, plotting, or CI performance regression tests.

---

## Best Practices

1. **Always enable during development** — overhead is negligible (< 0.1%)
2. **Disable for production** — avoids log noise
3. **Use `profiling_interval=100`** for stable averages; lower values (10-50) for debugging spikes
4. **Combine with memory profiling** for comprehensive analysis:
   ```yaml
   enable_time_profiling: true
   enable_memory_profiling: true
   profiling_interval: 100
   ```
5. **Use `return_profiling=True`** for automated benchmarks instead of parsing logs

---

## Comparison: Time Profiling vs Memory Profiling

| Feature | Time Profiling | Memory Profiling |
|---------|----------------|------------------|
| **Measures** | Execution time (ms) | Memory usage (MB) |
| **Detects** | Performance bottlenecks | Memory leaks, high usage |
| **Tool** | `time.perf_counter()` | `tracemalloc` (built-in) |
| **Overhead** | < 0.1% CPU | ~5-10% memory |
| **Use case** | Optimization | Long-running stability |
| **Config** | `enable_time_profiling` | `enable_memory_profiling` |

For memory profiling details, see [Memory Profiling User Guide](memory-profiling).

---

## Troubleshooting

### No profiling output appears

**Cause:** `enable_time_profiling` is not enabled or log level is too high.

**Solution:**
```python
params = SimulationParams(
    enable_time_profiling=True,
    profiling_interval=100,
    log_level="info",  # Must be info or lower
)
```

Or in YAML:
```yaml
enable_time_profiling: true
profiling_interval: 100
log_level: "info"
```

### Output appears too frequently / infrequently

**Cause:** `profiling_interval` does not match your needs.

**Solution:** Adjust `profiling_interval`:
- `10-50`: Fine-grained debugging
- `100`: Normal development (default)
- `500-1000`: Long-running simulations

---

## References

- [Memory Profiling User Guide](memory-profiling) — Memory usage tracking
- [Spatial Hash Configuration](spatial-hash-config) — Collision cell size tuning
- [Profiling Tools Guide](../benchmarking/profiling) — External profiling scripts in `benchmark/profiling/`
- [Optimization Guide](../benchmarking/optimization-guide) — Performance optimization workflow
