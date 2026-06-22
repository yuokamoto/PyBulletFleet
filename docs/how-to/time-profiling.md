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
- See [Collision Configuration](collision-config) for tuning

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
        print(f"Agent update: {timings['phase1_update']:.2f}ms")
        print(f"Collision:    {timings['collision_check']:.2f}ms")
        print(f"Total:        {timings['total']:.2f}ms")
```

### Return value fields

All times are in milliseconds.

| Key | What it measures |
|-----|-----------------|
| `phase1_update` | Full Phase 1: agent.update + callbacks + plugin on_step (buffered writes) |
| `phase2_pose_flush` | Phase 2: flush buffered poses to PyBullet |
| `phase3_aabb_grid_flush` | Phase 3: kinematic AABB refresh + spatial-grid update |
| `agent_update` | Per-object update loop only (subset of `phase1_update`, kept for backward compat) |
| `callbacks` | User-registered callbacks via `register_callback()` |
| `step_simulation` | `p.stepSimulation()` — non-zero only when `physics=True` |
| `collision_check` | Broad phase (spatial hash) + narrow phase total |
| `collision_breakdown` | Nested dict — present only when collision ran (see below) |
| `events_pre_step` | EventBus `pre_step` dispatch |
| `events_post_step` | EventBus `post_step` dispatch |
| `events_collision` | EventBus collision-pair dispatch |
| `monitor_update` | DataMonitor overlay refresh |
| `total` | End-to-end step time |

`collision_breakdown` sub-fields:

| Key | What it measures |
|-----|-----------------|
| `get_aabbs` | Fetch AABB bounds from PyBullet |
| `spatial_hashing` | Rebuild spatial hash buckets |
| `aabb_filtering` | Broad-phase candidate pair filtering |
| `contact_points` | Narrow-phase contact query (`physics=True` only) |
| `total` | Sum of collision sub-phases |

### Example: per-step CSV logging

```python
import csv, sys

writer = csv.DictWriter(sys.stdout,
    fieldnames=["step", "phase1_update", "collision_check", "total"])
writer.writeheader()

for i in range(500):
    t = sim.step_once(return_profiling=True)
    if t:
        writer.writerow({"step": i,
                         "phase1_update": round(t["phase1_update"], 3),
                         "collision_check": round(t["collision_check"], 3),
                         "total": round(t["total"], 3)})
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

## Benchmark scripts

For repeatable, process-isolated measurement use the scripts under `benchmark/`:

| Script | Purpose |
|--------|---------|
| `benchmark/batch_perf.py` | Batch vs per-agent step-time comparison with phase breakdown. Runs a fixed number of steps (not wall-clock duration) for stable statistics. |
| `benchmark/run_benchmark.py` | Multi-run sweep with subprocess isolation and median/stdev. Supports `--compare` for scenario comparison. |
| `benchmark/mobile_benchmark.py` | Worker process called by `run_benchmark.py`. Outputs JSON. |

**Batch vs per-agent comparison** (requires ≥ 300 steps for stable results — use `--duration 30` at the default 0.1 s timestep):

```bash
# Quick comparison in one process (phase breakdown shown)
python3 benchmark/batch_perf.py --n 1000 --mode omni

# Rigorous comparison with process isolation across agent counts
python3 benchmark/run_benchmark.py --compare per_agent batch_omni \
    --sweep 100 500 1000 2000 --duration 30 --repetitions 3
```

Scenario names (`per_agent`, `batch_omni`, etc.) are defined under the `scenarios:`
key in `benchmark/configs/general.yaml`.  Use `--config` to point at a custom file.

> **Note:** With `timestep=0.1` and `duration=5`, only 50 steps are measured — GC
> pauses can inflate the mean by 10× or more. Use `--duration 30` (300 steps) or
> longer for reliable numbers.

## References

- [Memory Profiling User Guide](memory-profiling) — Memory usage tracking
- [Collision Configuration](collision-config) — Collision settings and cell size tuning
