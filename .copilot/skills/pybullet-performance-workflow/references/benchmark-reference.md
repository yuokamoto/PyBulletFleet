# Benchmark Reference

Performance targets, known bottlenecks, and detailed benchmark configuration.

## Table of Contents

1. [Performance Targets](#performance-targets)
2. [Known Bottlenecks](#known-bottlenecks)
3. [Benchmark Output Metrics](#benchmark-output-metrics)
4. [Config Parameters Affecting Performance](#config-parameters-affecting-performance)

## Performance Targets

Current release-refresh numbers (2026-07-24, WSL2, headless, `simple_cube`
robots, physics=false, timestep=0.1, 50% moving, batch controller, fleet command
interface):

| Scale | Target Step Time | Target FPS | Status |
|-------|-----------------|------------|--------|
| 100 agents | ~0.70 ms/step | ~1430 FPS | Achieved |
| 250 agents | ~1.76 ms/step | ~568 FPS | Achieved |
| 500 agents | ~4.25 ms/step | ~235 FPS | Achieved |
| 1000 agents | ~9.88 ms/step | ~101 FPS | Achieved |
| 2000 agents | ~23.25 ms/step | ~43 FPS | Achieved |

**To get current numbers, always re-run benchmarks** — these are reference points, not guarantees.

## Known Bottlenecks

| Bottleneck | Where | Impact | Mitigation |
|------------|-------|--------|------------|
| `p.getClosestPoints()` per-pair | Narrow-phase collision | O(N²) naive, O(N) with spatial hash | Spatial hash broad-phase (default) |
| `p.resetBasePositionAndOrientation()` | Agent.set_pose | ~5.5μs/object | Batch only moved objects |
| `p.getBasePositionAndOrientation()` | SimObject.get_pose | Per-call overhead | Cached in get_pose() |
| `p.getAABB()` | Broad-phase AABB update | Called per moved object | Incremental update (only moved) |
| AABB / spatial-grid flush | Two-phase step Phase 3 | ~22% of release-like 1000-agent step | Reduce unnecessary flushes; use 2D ground path |
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
| `target_rtf` | 100 | 0 = max speed (no sleep) |
| `collision_check_frequency` | null (every step) | **Major** — 10 Hz vs every step |
| `spatial_hash_cell_size_mode` | auto_initial | Minor — constant is fastest |
| `collision_margin` | 0.02 | Negligible |
| `enable_time_profiling` | false | Negligible (<0.1%) |
| `enable_memory_profiling` | false | Major in timing loops — `tracemalloc` can distort RTF/step-time benchmarks |
| `log_level` | warn | Minor — debug logging is expensive |

**Optimal benchmark config:**
```yaml
gui: false
monitor: false
enable_monitor_gui: false
target_rtf: 0
physics: false
log_level: warn
enable_time_profiling: true
enable_memory_profiling: false
```
