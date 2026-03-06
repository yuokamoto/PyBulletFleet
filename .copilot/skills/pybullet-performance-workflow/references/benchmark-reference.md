# Benchmark Reference

Performance targets, known bottlenecks, and detailed benchmark configuration.

## Table of Contents

1. [Performance Targets](#performance-targets)
2. [Known Bottlenecks](#known-bottlenecks)
3. [Benchmark Output Metrics](#benchmark-output-metrics)
4. [Config Parameters Affecting Performance](#config-parameters-affecting-performance)

## Performance Targets

Known performance numbers from historical benchmarks (headless, physics=false):

| Scale | Target Step Time | Target FPS | Status |
|-------|-----------------|------------|--------|
| 100 agents | ≤4 ms/step | ~240 FPS | Achieved |
| 1000 agents | ≤40 ms/step | ~24 FPS | Achieved |
| 10,000 objects | ≤10 ms/step | — | PyBullet API bottleneck (54.69ms) |

**To get current numbers, always re-run benchmarks** — these are reference points, not guarantees.

## Known Bottlenecks

| Bottleneck | Where | Impact | Mitigation |
|------------|-------|--------|------------|
| `p.getClosestPoints()` per-pair | Narrow-phase collision | O(N²) naive, O(N) with spatial hash | Spatial hash broad-phase (default) |
| `p.resetBasePositionAndOrientation()` | Agent.set_pose | ~5.5μs/object | Batch only moved objects |
| `p.getBasePositionAndOrientation()` | SimObject.get_pose | Per-call overhead | Cached in get_pose() |
| `p.getAABB()` | Broad-phase AABB update | Called per moved object | Incremental update (only moved) |
| Shared shape cache miss | SimObject.create_shared_shapes | Redundant OpenGL shape creation | `_shared_shapes` dict (automatic) |
| Python GIL | Entire sim loop | Single-threaded | Headless mode, minimize Python overhead |
| `sim_object.py` wrapper overhead | Every PyBullet call | +7.6μs/object vs bare PyBullet | Accept or use raw IDs for hot paths |
| GUI rendering | PyBullet visualizer | 2-3x slower than headless | `gui: false` for benchmarks |
| matplotlib DataMonitor | `data_monitor.py` | GUI thread overhead | `monitor: false` for benchmarks |

## Benchmark Output Metrics

| Metric | Unit | What it measures |
|--------|------|-----------------|
| `spawn_time` | seconds | Time to create all agents |
| `simulation_wall` | seconds | Wall clock for simulation |
| `avg_step_time` | milliseconds | Average time per step_once() |
| `CPU%` | percent | CPU utilization |
| `real_time_factor` | ratio | sim_time / wall_time (>1 = faster than real-time) |
| `memory_rss` | MB | Resident set size at end |

## Config Parameters Affecting Performance

| Parameter | Default | Performance Impact |
|-----------|---------|-------------------|
| `gui` | true | **Major** — 2-3x slower with GUI |
| `physics` | false | Moderate — stepSimulation() cost |
| `monitor` | true | Minor — matplotlib overhead |
| `timestep` | 0.1 | Indirect — fewer steps = faster wall time |
| `speed` | 100 | 0 = max speed (no sleep) |
| `collision_check_frequency` | null (every step) | **Major** — 10 Hz vs every step |
| `spatial_hash_cell_size_mode` | auto_initial | Minor — constant is fastest |
| `collision_margin` | 0.02 | Negligible |
| `enable_profiling` | false | Negligible (<0.1%) |
| `enable_memory_profiling` | false | Minor — tracemalloc overhead |
| `log_level` | warn | Minor — debug logging is expensive |

**Optimal benchmark config:**
```yaml
gui: false
monitor: false
enable_monitor_gui: false
speed: 0
physics: false
log_level: warn
enable_profiling: true
```
