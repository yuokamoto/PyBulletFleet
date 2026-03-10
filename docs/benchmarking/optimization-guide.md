# PyBullet Fleet - Performance Optimization Guide

This guide covers key parameters, configuration patterns, and troubleshooting for simulation performance.

---

## Key Parameters

### RTF Control (`target_rtf`)

| Value | Behavior | Use Case |
|-------|----------|----------|
| `0` | Maximum speed (no sleep) | Offline batch processing |
| `1.0` | Real-time synchronization | Interactive development |
| `2.0` | 2× real-time | Faster testing with GUI |

`target_rtf=0` runs as fast as the CPU allows.

---

### Timestep

| Mode | Recommended Value |
|------|-----------------|
| Kinematics (physics off) | `0.1` (default) |
| Physics enabled | `0.01` or smaller |
| GUI visualization | `0.01` for smooth motion |

---

### Collision Detection

**Frequency** (`collision_check_frequency` on `SimulationParams`):

Controls how many times per simulation-second collisions are checked. The effective check interval is:

```
interval (steps) = 1 / (frequency × timestep)
```

`null` means every step regardless of timestep.

| Value | Behavior | Overhead | Use Case |
|-------|----------|----------|----------|
| `null` | Every step | High | Safety-critical / dense environments |
| `N` Hz | Every `1/(N×timestep)` steps | Scales with N | General use |
| `0` (disabled) | Never | None | Baseline performance testing |

> **Example:** With `timestep=0.1`, `frequency=10.0` gives interval = 1 step (same as `null`). Use `1.0` Hz for an interval of 10 steps, or lower to meaningfully reduce overhead.

**Dimension** (`collision_mode` on `AgentSpawnParams` — per-agent):

| Mode | Neighbors | Speedup | Use Case |
|------|-----------|---------|----------|
| `CollisionMode.NORMAL_2D` | 9 (XY plane) | ~67% faster | Ground robots (AGVs) |
| `CollisionMode.NORMAL_3D` | 27 (3D cube) | baseline | Drones, lifts |
| `CollisionMode.DISABLED` | 0 | no overhead | Visualization-only objects |

```python
from pybullet_fleet.types import CollisionMode
from pybullet_fleet.agent import AgentSpawnParams

agent_params = AgentSpawnParams(
    urdf_path="robots/mobile_robot.urdf",
    collision_mode=CollisionMode.NORMAL_2D,  # 9 neighbors (XY only)
)
```

---

### Profiling

| Setting | Overhead | Use Case |
|---------|----------|----------|
| `enable_time_profiling=False` | None | Production |
| `enable_time_profiling=True` | ~5–10% | Development / optimization |

```python
sim_core = MultiRobotSimulationCore(params)
sim_core.set_profiling_log_frequency(10)  # Log every 10 steps
```

---

## Configuration Examples

### Offline / Production

```yaml
simulation:
  timestep: 0.1
  target_rtf: 0                    # Maximum speed
  physics: false
  gui: false
  collision_check_frequency: 1.0   # Every 10 steps with timestep=0.1 (use null for every step)
  ignore_static_collision: true
  monitor: true
  enable_monitor_gui: false
  enable_time_profiling: false
  enable_collision_shapes: false
  enable_shadows: false
  # collision_mode is per-agent: AgentSpawnParams(..., collision_mode=CollisionMode.NORMAL_2D)
```

**Expected:** ~6.6× RTF (500 agents), ~2.3× RTF (1000 agents)

---

### Development / Debugging

```yaml
simulation:
  timestep: 0.01
  target_rtf: 1.0
  physics: false
  gui: true
  collision_check_frequency: null  # Every step
  enable_collision_shapes: true
  enable_collision_color_change: true
  monitor: true
  enable_monitor_gui: true
  enable_time_profiling: true
  # collision_mode is per-agent: AgentSpawnParams(..., collision_mode=CollisionMode.NORMAL_3D)
```

---

## Performance Reference

### Benchmark Results

Based on latest measurement (2026-03-10, kinematics mode, 50% agents moving, headless):

| Agents | RTF  | Step Time    |
|--------|------|--------------|
| 100    | 41×  | 2.44 ± 0.15 ms |
| 250    | 15×  | 6.71 ± 0.25 ms |
| 500    | 6.6× | 15.12 ± 0.23 ms |
| 1000   | 2.3× | 42.65 ± 0.99 ms |
| 2000   | 1.1× | 88.44 ± 1.01 ms |

Scalability: O(n^1.3) — near-linear up to 500 agents, slightly super-linear above.

### Step Time Targets

| Application | Target |
|-------------|--------|
| Real-time control (100 Hz) | < 10 ms |
| Real-time visualization (60 FPS) | < 16.7 ms |
| Offline processing | No strict limit |

### Component Breakdown (1000 agents)

```
Agent Update     13.79 ms   88.2%
Collision Check   1.76 ms   11.2%
Monitor Update    0.08 ms    0.5%
Step Simulation   0.00 ms    0.0%
─────────────────────────────────
Total            15.63 ms
```

### Memory

~20 KB per agent above 500 agents (linear scaling). Below 500 agents, Python GC can show negative deltas.

---

## Troubleshooting

### Low RTF / High Step Time

Run profiling to identify the bottleneck:

```bash
python benchmark/profiling/simulation_profiler.py --agents 1000 --steps 100
```

| Cause | Solution |
|-------|----------|
| Collision check dominating | Reduce `collision_check_frequency` to 1 Hz (with `timestep=0.1`, 10 Hz = every step); use `NORMAL_2D` per-agent |
| GUI overhead | Set `gui: false`, `enable_monitor_gui: false` |
| Profiling enabled | Set `enable_time_profiling: false` for production |

### Memory Growth

Expected ~20 KB/agent above 500 agents. If super-linear:
- Check for memory leaks in custom callbacks
- Verify objects are cleaned up with `del` or weak references

---

## Performance Hierarchy

From fastest to slowest collision configuration (assuming `timestep=0.1`):

1. **Disabled** (`collision_check_frequency=0`) — ~6× speedup; baseline testing only
2. **`NORMAL_2D` at 1 Hz** — every 10 steps; ~3× speedup; sparse environments
3. **`NORMAL_2D` at 10 Hz / `null`** — every step; ~2× speedup; **recommended for production**
4. **`NORMAL_3D` at 10 Hz / `null`** — every step; baseline; 3D movement (drones, lifts)

> With `timestep=0.1`, `collision_check_frequency=10.0` and `null` are equivalent (both check every step).

---

## See Also

- [Benchmark Results](results) — Collected results with design rationale
- [Profiling Guide](profiling-guide) — Profiling tools
- [Benchmark Experiments](experiments) — Algorithm comparison experiments

---

**Last Updated:** 2026-03-08
