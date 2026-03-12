# Profiling Guide

Standalone scripts for identifying performance bottlenecks in PyBulletFleet simulation components. These scripts are run from the command line against a live simulation.

All scripts live in `benchmark/profiling/`. For overall benchmark results and quick start, see `benchmark/README.md`.

> **Looking to add profiling to your own code?** See the [Time Profiling User Guide](../how-to/time-profiling) (API-based) or [Custom Class Profiling](../how-to/custom-profiling) (subclass profiling).

---

## Tool Summary

| Tool | Purpose | What It Measures |
|------|---------|-----------------|
| `simulation_profiler.py` | Step-level component breakdown | Agent Update, Collision Check, PyBullet Step, etc. |
| `collision_check.py` | Detailed collision detection analysis | Get AABBs, Spatial Hashing, AABB Filtering, Contact Points |
| `agent_update.py` | Detailed `Agent.update()` analysis | 5 methods (cProfile, Manual, PyBullet API, Stationary, Motion Modes) |
| `agent_manager_set_goal.py` | Goal setting profiling | `set_goal_pose()` overhead and trajectory calculation |
| `wrapper_overhead.py` | Wrapper-layer overhead | Spawn time, update time, and memory: direct PyBullet vs SimObject vs Agent vs Manager |

## Measurement Methods by Script

Each profiling script uses one or more measurement techniques. The table below shows which `--test` options are available and what technique they use.

| Script | `--test` Option | Technique | Description |
|--------|----------------|-----------|-------------|
| `simulation_profiler.py` | `builtin` (default) | `time.perf_counter` | Per-component timing via `step_once(return_profiling=True)` |
| | `cprofile` | `cProfile` | Function-level call graph analysis |
| | `motion_modes` | `time.perf_counter` | OMNIDIRECTIONAL vs DIFFERENTIAL comparison |
| `collision_check.py` | `builtin` (default) | `time.perf_counter` | 4-stage pipeline breakdown via `check_collisions(return_profiling=True)` |
| | `cprofile` | `cProfile` | Function-level analysis of collision path |
| `agent_update.py` | `cprofile` (default) | `cProfile` | Function-level call graph of `agent.update()` |
| | `manual` | `time.perf_counter` | Manual timing of update sub-steps |
| | `pybullet` | `time.perf_counter` | PyBullet API call timing (resetBasePositionAndOrientation, etc.) |
| | `stationary` | `time.perf_counter` | Stationary vs moving agent cost comparison |
| | `motion_modes` | `time.perf_counter` | Per-motion-mode update cost |
| `agent_manager_set_goal.py` | _(no option)_ | Both | `time.perf_counter` for wall time + `cProfile` for call graph (always runs both) |

## Measurement Method Comparison

| Attribute | cProfile | CPU Time | Wall Time |
|-----------|----------|----------|-----------|
| **Goal** | Bottleneck identification | CPU usage measurement | Real-time measurement |
| **Granularity** | Function level | Process-wide | Process-wide |
| **Overhead** | Yes (high if many calls) | Almost none | Almost none |
| **Detail** | High (Python layer) | Low | Low |
| **Stability** | Medium | High (same environment) | Low (environment-sensitive) |
| **Use case** | Find optimization targets | Perf evaluation / regression | Perceived speed / RTF |

- **cProfile** — Function-level call counts and cumulative times. Adds 5-50% overhead. Cannot see inside PyBullet C++ internals.
- **CPU Time** (`psutil` / `time.process_time()`) — Actual CPU consumption. Near-zero overhead. Best for regression detection and before/after comparison.
- **Wall Time** (`time.perf_counter()`) — Real elapsed time including I/O waits. Best for perceived speed / RTF. Higher variance.
- **`step_once(return_profiling=True)`** — Built-in profiling in `MultiRobotSimulationCore` that returns per-component timing dict (agent_update, collision_check, etc.).

## When to Use Each Tool

1. **Identify overall bottleneck** → `simulation_profiler.py`
2. **Collision detection is slow** → `collision_check.py`
3. **Agent Update is slow** → `agent_update.py`
4. **Goal setting is slow** → `agent_manager_set_goal.py`
5. **Wrapper-layer overhead?** → `wrapper_overhead.py`

---

## Simulation Profiler (`simulation_profiler.py`)

Component-level time measurement and bottleneck identification within `step_once()`. Unlike `performance_benchmark.py` (overall JSON output), this tool provides per-component statistical breakdowns.

### Measured Components

| Component | Description | Typical Share |
|-----------|-------------|---------------|
| Agent Update | State updates for all agents (trajectory following, kinematics) | 80-90% (when ~50% agents move) |
| Collision Check | Collision detection (spatial hashing, AABB) | 10-15% |
| PyBullet Step | Physics simulation step | 0% (physics off) or 20-40% (physics on) |
| Monitor Update | Data monitor updates | <1% |

### Analysis Methods

| Method | Command | Purpose |
|--------|---------|---------|
| Built-in Profiling | `--test=builtin` (default) | Component time distribution from `step_once()` |
| cProfile | `--test=cprofile` | All-function bottleneck search |
| Motion Modes | `--test=motion_modes` | DIFFERENTIAL vs OMNIDIRECTIONAL comparison |

### CLI Usage

```bash
# Built-in profiling (default)
python benchmark/profiling/simulation_profiler.py --agents=1000 --steps=100

# Detailed analysis with cProfile
python benchmark/profiling/simulation_profiler.py --agents=1000 --test=cprofile

# Motion Mode comparison
python benchmark/profiling/simulation_profiler.py --agents=1000 --test=motion_modes

# Run all analyses
python benchmark/profiling/simulation_profiler.py --agents=1000 --test=all
```

### Output Example

```
Step Breakdown (OMNIDIRECTIONAL): 1000 agents (100 steps, kinematics mode)
======================================================================

Agent Update:
  Mean:      13.79ms ( 88.2%)
  Median:     0.21ms
  StdDev:    21.76ms

Collision Check:
  Mean:       1.76ms ( 11.2%)
  Median:     0.00ms

Monitor Update:
  Mean:       0.08ms (  0.5%)

Step Simulation:
  Mean:       0.00ms (  0.0%)  ← physics off

Total Step Time:
  Mean:      15.63ms (100.0%)
```

### Follow-Up Analysis

- Agent Update is slow → use `agent_update.py` for detailed analysis
- Collision Check is slow → use `collision_check.py` for detailed analysis

---

## Collision Check Profiler (`collision_check.py`)

Breaks the collision detection pipeline into 4 steps for bottleneck identification.

### 4-Step Breakdown

| Step | Description | Typical Share |
|------|-------------|---------------|
| Get AABBs | Fetch bounding boxes from PyBullet | ~10% |
| Spatial Hashing | Build spatial grid | ~6% |
| AABB Filtering | Candidate pair selection (27-neighbor search) | **~75%** |
| Contact Points | Actual collision check in PyBullet | ~9% |

### CLI Usage

```bash
# Built-in profiling (default, recommended)
python benchmark/profiling/collision_check.py --agents=1000 --iterations=100

# Detailed analysis with cProfile (function level)
python benchmark/profiling/collision_check.py --agents=1000 --test=cprofile

# Run both
python benchmark/profiling/collision_check.py --agents=1000 --test=all
```

### Output Example

```
Collision Check Breakdown for 1000 Agents (Built-in Profiling)
======================================================================

Get Aabbs:
  Mean:      0.523ms ( 10.2%)
  Median:    0.512ms

Spatial Hashing:
  Mean:      0.312ms (  6.1%)

Aabb Filtering:
  Mean:      3.845ms ( 75.2%)  ← largest bottleneck

Contact Points:
  Mean:      0.432ms (  8.5%)

Total:
  Mean:      5.112ms (100.0%)
```

Use `--test=cprofile` to drill into what is slow *inside* a step (e.g., AABB overlap checks, dict lookups in grid).

### Optimization Hints

- AABB Filtering at 75% → 2D mode can reduce it by ~67%
- Collision ratio of 0.3% → room for improving filtering precision

---

## Agent Update Profiler (`agent_update.py`)

`Agent.update()` runs every frame for every agent. This tool provides 5 analysis methods.

### Five Analysis Methods

| Method | Command | Purpose | Overhead |
|--------|---------|---------|----------|
| cProfile | `--test=cprofile` | All-function bottleneck search | Medium (5-50%) |
| Manual Timing | `--test=manual` | Precise measurement of specific methods | Minimal (<1%) |
| PyBullet API | `--test=pybullet` | C++ API cost measurement | Low (1-5%) |
| Stationary vs Moving | `--test=stationary` | Impact of movement/update processing | None |
| Motion Modes | `--test=motion_modes` | DIFFERENTIAL vs OMNIDIRECTIONAL | None |

### CLI Usage

```bash
# Run all analyses
python benchmark/profiling/agent_update.py --agents=1000 --updates=100

# Individual tests
python benchmark/profiling/agent_update.py --agents=1000 --test=cprofile
python benchmark/profiling/agent_update.py --agents=1000 --test=manual
python benchmark/profiling/agent_update.py --agents=100  --test=pybullet
python benchmark/profiling/agent_update.py --agents=1000 --test=stationary
python benchmark/profiling/agent_update.py --agents=1000 --test=motion_modes
```

### Output Summary

Each method produces a focused report:

- **Manual Timing** — per-component mean/median/max in microseconds (e.g., `update_differential`, `update_actions`)
- **PyBullet API** — call counts, total time, and per-call average for each PyBullet function
- **Stationary vs Moving** — total time and per-agent cost for idle vs. moving agents, plus overhead ratio
- **Motion Modes** — side-by-side DIFFERENTIAL vs OMNIDIRECTIONAL cost comparison

### When to Use Which Method

1. **Bottleneck identification** → `--test=cprofile` — find slow functions
2. **Optimization verification** → `--test=manual` — accurate before/after comparison
3. **PyBullet API optimization** → `--test=pybullet` — identify expensive API calls
4. **Stationary agent impact** → `--test=stationary` — quantify movement overhead
5. **Motion mode comparison** → `--test=motion_modes` — compare DIFFERENTIAL vs OMNIDIRECTIONAL

---

## Goal Setting Profiler (`agent_manager_set_goal.py`)

Uses cProfile to analyze `AgentManager.set_goal_pose()` and the trajectory calculation call chain.

### CLI Usage

```bash
python benchmark/profiling/agent_manager_set_goal.py --agents=1000
```

### Output Example

```
ncalls  tottime  cumtime  function
  1000    0.001    0.109  agent_manager.set_goal_pose()
  1000    0.000    0.108  agent.set_goal_pose()
  1000    0.007    0.107  agent.set_path()
  1000    0.022    0.092  _init_differential_rotation_trajectory()  ← 84.4%
  1000    0.003    0.032  _init_differential_forward_distance_trajectory()
```

`_init_differential_rotation_trajectory` typically accounts for ~84% of the time due to rotation matrix calculations (`scipy.spatial.transform`). Potential improvements: caching, pre-computation.

---

## Typical Bottlenecks and Fixes

### 1. Collision Check is slow (> 20% of step time)

**Fixes:**
- Set `collision_mode=CollisionMode.NORMAL_2D` on agent spawn params (~67% reduction)
- `collision_check_frequency=10.0` — reduce frequency (10 Hz)
- `ignore_static_collision=True` — ignore collisions with static structures

**Verification:**

```bash
python benchmark/profiling/collision_check.py --agents=1000
```

### 2. Agent Update is slow (> 40% of step time)

**Fixes:**
- Skip updates for stationary agents
- Reduce PyBullet API calls (caching)

**Verification:**

```bash
python benchmark/profiling/agent_update.py --agents=1000 --test=stationary
python benchmark/profiling/agent_update.py --agents=100  --test=pybullet
```

### 3. Goal setting is slow (set_goal_pose > 100ms)

**Fixes:**
- Cache trajectory calculations
- Use simplified trajectory generation

**Verification:**

```bash
python benchmark/profiling/agent_manager_set_goal.py --agents=1000
```

---

## Wrapper Overhead (`wrapper_overhead.py`)

Measures the overhead introduced by PyBulletFleet wrapper classes relative to bare PyBullet API calls. Useful for detecting regressions after refactoring SimObject, Agent, or Manager layers.

### What It Measures

| Test | Description |
|------|-------------|
| Direct PyBullet (baseline) | Bare `createMultiBody` / `resetBasePositionAndOrientation` |
| SimObject Wrapper | `SimObject.spawn()` + `get_pose()` + `set_pose()` |
| SimObjectManager (Bulk) | `SimObjectManager.spawn_grid()` + bulk pose operations |
| Agent Wrapper | `Agent.from_params()` + `get_pose()` + `set_pose()` |
| AgentManager (Bulk) | `AgentManager.spawn_agents_grid()` + `get_all_poses()` |

Each test runs in a **separate child process** (via `subprocess`) to eliminate cross-test contamination and measure clean-state memory.

### CLI Usage

```bash
# Default: 10000 objects, 5 repetitions
python benchmark/profiling/wrapper_overhead.py

# Custom object count and repetitions
python benchmark/profiling/wrapper_overhead.py --n=1000 --reps=3
```

### Metrics

- **Spawn time** — wall-clock and CPU (user+sys) time to create N objects
- **Update time** — wall-clock and CPU time for get_pose + set_pose per step
- **Memory overhead** — RSS delta between before/after spawn (MB, per-object KB)
- **CPU utilization** — `cpu_time / wall_time` ratio (stability indicator)
- **Extrapolation** — scaled to production `N_MAX_OBJECTS` with PASS/FAIL thresholds

---

## Troubleshooting

### Profiling logs are not displayed

**Cause:** `enable_time_profiling=False` or log level is not set to `DEBUG`.

**Fix:**

```python
params = SimulationParams(
    enable_time_profiling=True,
    log_level="debug"
)
```

Or in config:

```yaml
simulation:
  enable_time_profiling: true
  log_level: debug
```

### Segfault with cProfile

**Cause:** Compatibility issues between cProfile and PyBullet's C++ extension.

**Fix:**
- Use Manual Timing or Built-in Profiling instead
- Try `agent_update.py --test=manual`
- Use `agent_update.py --test=stationary` (does not use cProfile)

### High variance in measurement results

**Cause:** Background process interference, thermal throttling.

**Fix:**
- Run multiple iterations and compute statistics
- Increase iteration count with `--iterations`
- Check CPU utilization
