# Profiling Guide

## Overview

The profiling tools in `benchmark/profiling/` provide detailed CPU time and memory analysis of specific simulation components. This guide covers the available tools, measurement methodologies, and optimization workflows.

### Tool Summary

| Tool | Purpose | What It Measures |
|------|---------|-----------------|
| `simulation_profiler.py` | Step-level component breakdown | Agent Update, Collision Check, PyBullet Step, etc. |
| `collision_check.py` | Detailed collision detection analysis | Get AABBs, Spatial Hashing, AABB Filtering, Contact Points |
| `agent_update.py` | Detailed `Agent.update()` analysis | 5 methods (cProfile, Manual, PyBullet API, Stationary, Motion Modes) |
| `agent_manager_set_goal.py` | Goal setting profiling | `set_goal_pose()` overhead and trajectory calculation |

### When to Use Each Tool

1. **Identify overall bottleneck** → `simulation_profiler.py`
2. **Collision detection is slow** → `collision_check.py`
3. **Agent Update is slow** → `agent_update.py`
4. **Goal setting is slow** → `agent_manager_set_goal.py`

---

## Profiling Tools

### Simulation Profiler (simulation_profiler.py)

**Purpose:** Component-level time measurement and bottleneck identification within `step_once()`.

Breaks down each simulation step into multiple components for measurement.

**Difference from `performance_benchmark.py`:**

- `performance_benchmark.py`: Overall performance measurement (JSON output, memory, CPU utilization)
- `simulation_profiler.py`: Detailed per-component analysis (statistical output, profiling)

#### Measured Components

| Component | Description | Typical Share |
|-----------|-------------|---------------|
| Agent Update | State updates for all agents (trajectory following, kinematics) | 30–40% |
| Collision Check | Collision detection (spatial hashing, AABB) | 10–20% |
| PyBullet Step | Physics simulation step | 40–50% |
| Monitor Update | Data monitor updates | 1–5% |
| Other | Miscellaneous overhead | <5% |

#### Three Analysis Methods

| Method | Command | Purpose | Notes |
|--------|---------|---------|-------|
| Built-in Profiling | `--test=builtin` (default) | Component time distribution | Obtained directly from `step_once()` |
| cProfile | `--test=cprofile` | All-function bottleneck search | Function-level detailed analysis |
| Motion Modes | `--test=motion_modes` | DIFFERENTIAL vs OMNIDIRECTIONAL | Mode comparison |

#### CLI Usage

```bash
# Built-in profiling (default)
python benchmark/profiling/simulation_profiler.py --agents=1000 --steps=100

# Detailed analysis with cProfile
python benchmark/profiling/simulation_profiler.py --agents=1000 --test=cprofile

# Motion Mode comparison
python benchmark/profiling/simulation_profiler.py --agents=1000 --test=motion_modes

# Measure in DIFFERENTIAL mode
python benchmark/profiling/simulation_profiler.py --agents=1000 --mode=differential

# Run all analyses
python benchmark/profiling/simulation_profiler.py --agents=1000 --test=all
```

#### Output Example

```
Step Breakdown (OMNIDIRECTIONAL): 1000 agents (100 steps)
======================================================================

Agent Update:
  Mean:      12.45ms ( 35.2%)
  Median:    12.38ms
  StdDev:     0.87ms
  Range:  [ 11.23, 15.67]ms

Collision Check:
  Mean:       5.12ms ( 14.5%)
  Median:     5.08ms
  Range:  [  4.89,  6.23]ms

Pybullet Step:
  Mean:      15.34ms ( 43.4%)  ← physics computation cost
  Median:    15.21ms
  Range:  [ 14.78, 17.12]ms

Total Step Time:
  Mean:      35.37ms (100.0%)
  Real-Time Factor: 28.3x
```

#### Follow-Up Analysis

- If Agent Update is slow → use `agent_update.py` for detailed analysis
- If Collision Check is slow → use `collision_check.py` for detailed analysis

#### Related Source Files

- `pybullet_fleet/core_simulation.py` — `step_once()` implementation
- `benchmark/profiling/agent_update.py` — Agent Update detailed analysis
- `benchmark/profiling/collision_check.py` — Collision Check detailed analysis

---

### Collision Check Profiler (collision_check.py)

**Purpose:** Bottleneck identification and optimization verification for the collision detection pipeline.

Collision detection is a primary bottleneck in large-scale simulations. This tool breaks the pipeline into 4 steps.

#### 4-Step Breakdown

| Step | Description | Typical Share |
|------|-------------|---------------|
| Get AABBs | Fetch bounding boxes from PyBullet | ~10% |
| Spatial Hashing | Build spatial grid | ~6% |
| AABB Filtering | Candidate pair selection (27-neighbor search) | **~75%** ← largest bottleneck |
| Contact Points | Actual collision check in PyBullet | ~9% |

#### Two Analysis Methods

| Method | Command | Purpose | Notes |
|--------|---------|---------|-------|
| Built-in Profiling | `--test=builtin` (default) | Measure the 4-step time distribution | Obtained directly from the implementation (accurate) |
| cProfile | `--test=cprofile` | Function-level detailed analysis | Records internal function calls |

#### CLI Usage

```bash
# Built-in profiling (default, recommended)
python benchmark/profiling/collision_check.py --agents=1000 --iterations=100

# Detailed analysis with cProfile (function level)
python benchmark/profiling/collision_check.py --agents=1000 --test=cprofile

# Run both
python benchmark/profiling/collision_check.py --agents=1000 --test=all
```

#### Output Examples

**Built-in Profiling:**
```
Collision Check Breakdown for 1000 Agents (Built-in Profiling)
======================================================================

Get Aabbs:
  Mean:      0.523ms ( 10.2%)
  Median:    0.512ms
  Range:  [ 0.489,  0.678]ms

Spatial Hashing:
  Mean:      0.312ms (  6.1%)

Aabb Filtering:
  Mean:      3.845ms ( 75.2%)  ← largest bottleneck

Contact Points:
  Mean:      0.432ms (  8.5%)

Total:
  Mean:      5.112ms (100.0%)

Advantages of Built-in Profiling:
  ✅ No implementation duplication
  ✅ Always measures the latest implementation
  ✅ Accurate dynamic cell size, 2D/3D mode measurement
  ✅ Minimal overhead (return_profiling flag only)
```

**cProfile (step internals):**
```
ncalls  tottime  cumtime  function
384500    1.850    1.850  AABB overlap check
 27000    0.650    0.650  dict.get (grid lookup)
 27000    0.420    0.420  tuple addition (neighbor_cell)
```

#### When to Use Which Method

1. **Bottleneck identification** → `--test=builtin`
   - Immediately see which of the 4 steps is slow

2. **Step-internal detailed analysis** → `--test=cprofile`
   - Identify what's slow *inside* AABB Filtering

3. **Optimization verification** → `--test=builtin`
   - No overhead for accurate before/after comparison

#### Optimization Hints

- AABB Filtering at 75% → 2D mode can reduce it by ~67%
- Collision ratio of 0.3% → room for improving filtering precision

#### Related Source Files

- `pybullet_fleet/core_simulation.py` — `check_collisions()`, `filter_aabb_pairs()` implementation

---

### Agent Update Profiler (agent_update.py)

**Purpose:** Bottleneck identification and optimization verification for `Agent.update()`.

`Agent.update()` is the most frequently called operation — executed every frame for every agent. This tool provides 5 different analysis methods to pinpoint bottlenecks.

#### Five Analysis Methods

| Method | Command | Purpose | Overhead |
|--------|---------|---------|----------|
| cProfile | `--test=cprofile` | All-function bottleneck search | Medium (5–50%) |
| Manual Timing | `--test=manual` | Precise measurement of specific methods | Minimal (<1%) |
| PyBullet API | `--test=pybullet` | C++ API cost measurement | Low (1–5%) |
| Stationary vs Moving | `--test=stationary` | Impact of movement/update processing | None |
| Motion Modes | `--test=motion_modes` | DIFFERENTIAL vs OMNIDIRECTIONAL | None |

#### CLI Usage

```bash
# Run all analyses
python benchmark/profiling/agent_update.py --agents=1000 --updates=100

# cProfile only (bottleneck search)
python benchmark/profiling/agent_update.py --agents=1000 --test=cprofile

# Manual timing only (detailed measurement)
python benchmark/profiling/agent_update.py --agents=1000 --updates=100 --test=manual

# PyBullet API analysis only
python benchmark/profiling/agent_update.py --agents=100 --updates=10 --test=pybullet

# Stationary vs Moving only
python benchmark/profiling/agent_update.py --agents=1000 --test=stationary

# Motion Mode comparison only
python benchmark/profiling/agent_update.py --agents=1000 --test=motion_modes
```

#### Output Examples

**Manual Timing:**
```
Component                 Mean (μs)    Median (μs)  Max (μs)
----------------------------------------------------------------------
total                     120.50       118.30       250.10
update_differential       80.20        78.50        180.00
update_actions            35.10        34.00        90.00
```

**PyBullet API:**
```
Function                                  Calls      Total (ms)   Avg (μs)
---------------------------------------------------------------------------
resetBasePositionAndOrientation           1000       45.20        45.20
getBasePositionAndOrientation             2000       30.50        15.25
getBaseVelocity                           1000       12.30        12.30
```

**Stationary vs Moving:**
```
Stationary agents (1000):
  Total time: 15.20 ms
  Per agent: 15.20 μs

Moving agents (1000):
  Total time: 120.50 ms
  Per agent: 120.50 μs

Overhead ratio (moving/stationary): 7.93x
Potential savings if 50% stationary: 67.85 ms
```

**Motion Modes:**
```
Motion Mode Comparison (1000 agents)
======================================================================

DIFFERENTIAL mode:
  Total time: 120.50 ms
  Per agent: 120.50 μs

OMNIDIRECTIONAL mode:
  Total time: 95.30 ms
  Per agent: 95.30 μs

Overhead ratio (DIFFERENTIAL/OMNIDIRECTIONAL): 1.26x
Performance difference: OMNIDIRECTIONAL is 26% faster
```

#### When to Use Which Method

1. **Bottleneck identification** → `--test=cprofile`
   - List all functions and discover slow spots

2. **Optimization verification** → `--test=manual`
   - Accurate time comparison of before/after optimization

3. **PyBullet API optimization** → `--test=pybullet`
   - Check which APIs are called most frequently

4. **Measure the impact of stationary agents** → `--test=stationary`
   - Compares `update()` cost for agents without a goal (stationary) vs. with a goal (moving)
   - Shows the performance difference caused by movement and joint-update processing

5. **Motion Mode comparison** → `--test=motion_modes`
   - Compares `update()` cost of DIFFERENTIAL vs OMNIDIRECTIONAL

#### How Manual Timing Works (Monkey-Patching)

Each method is wrapped with `perf_counter()` calls via monkey-patching:

```python
# Monkey-patch Agent.update_differential()
original_update = Agent.update_differential

def patched_update(self, dt):
    t0 = time.perf_counter()
    result = original_update(self, dt)
    t1 = time.perf_counter()
    timings['update_differential'].append((t1 - t0) * 1000)  # ms
    return result

Agent.update_differential = patched_update
```

**Characteristics:**

- ✅ No cProfile — minimal overhead
- ✅ Only measures targeted methods — no noise
- ✅ Accurate before/after comparison

#### Related Source Files

- `pybullet_fleet/agent.py` — `Agent.update()` implementation

---

### Goal Setting Profiler (agent_manager_set_goal.py)

**Purpose:** Bottleneck identification for `AgentManager.set_goal_pose()`.

Uses cProfile to analyze the process of setting goals for multiple agents.

#### What It Measures

- `agent_manager.set_goal_pose()` overhead (typically <1%)
- `agent.set_goal_pose()` → `agent.set_path()` call chain
- `_init_differential_rotation_trajectory()` time (**largest bottleneck, typically 80–85%**)
- `_init_differential_forward_distance_trajectory()` time
- Rotation matrix computation and trajectory interpolation cost

#### CLI Usage

```bash
# Basic run (1000 agents)
python benchmark/profiling/agent_manager_set_goal.py --agents=1000

# Small-scale test
python benchmark/profiling/agent_manager_set_goal.py --agents=100
```

#### Output Example

```
ncalls  tottime  cumtime  function
  1000    0.001    0.109  agent_manager.set_goal_pose()
  1000    0.000    0.108  agent.set_goal_pose()
  1000    0.007    0.107  agent.set_path()
  1000    0.022    0.092  _init_differential_rotation_trajectory()  ← 84.4%
  1000    0.003    0.032  _init_differential_forward_distance_trajectory()
```

#### How to Read the Results

```
agent_manager.set_goal_pose(): cumtime=0.109s
  ├─ Own overhead: tottime=0.001s (0.9%)
  └─ agent.set_goal_pose(): cumtime=0.108s (99.1%)
      ├─ Own time: tottime=0.000s (0%)
      └─ agent.set_path(): cumtime=0.107s
          ├─ _init_differential_rotation_trajectory: 0.092s (84.4%) ← bottleneck
          └─ _init_differential_forward_distance_trajectory: 0.032s (29.4%)
```

#### Optimization Hints

- `_init_differential_rotation_trajectory` accounts for 84% of the time
- Rotation matrix calculations (`scipy.spatial.transform`) are expensive
- Trajectory interpolation (`two_point_interpolation`) also takes time
- Potential improvements: caching, pre-computation

#### Related Source Files

- `pybullet_fleet/agent_manager.py` — `AgentManager.set_goal_pose()` implementation
- `pybullet_fleet/agent.py` — `Agent.set_path()` implementation

---

## Measurement Methods

This section explains the three measurement methodologies used across the profiling tools, their trade-offs, and when to use each.

### cProfile (Function-Level Profiler)

**Goal:** Identify *which function* is slow.

**What it records:**

- Call count per function
- Cumulative execution time per function
- Per-call average time
- Call tree structure

**Usage example:**

```python
import cProfile
import pstats

profiler = cProfile.Profile()
profiler.enable()

# Code to measure
for agent in agents:
    agent.update(dt=0.01)

profiler.disable()

stats = pstats.Stats(profiler)
stats.sort_stats("cumulative")
stats.print_stats(30)  # Top 30 functions
```

**Output example:**

```
   ncalls  tottime  percall  cumtime  percall filename:lineno(function)
     1000    0.050    0.000    0.150    0.000 agent.py:123(update)
     1000    0.080    0.000    0.080    0.000 {method 'getBasePositionAndOrientation'}
     1000    0.020    0.000    0.020    0.000 geometry.py:45(__init__)
```

**Advantages:**

- ✅ Detailed analysis — identifies bottlenecks at function level
- ✅ Call graph — shows call relationships
- ✅ Standard library — no extra installation needed

**Disadvantages:**

- ❌ Overhead — measurement itself slows the code (especially with many function calls)
- ❌ Granularity limits — per-function/call breakdown is available, but C extension internals (e.g., inside PyBullet API) and line-level detail are not visible
- ❌ Hard to read — large lists of functions can be overwhelming

**Important notes:**

- ⚠️ **C extensions:** PyBullet runs in C++. cProfile records the *call* time for PyBullet APIs, but cannot see internal C++ details.
- ⚠️ **Overhead variability:** Strongly depends on call granularity. Many small function calls = worse overhead; heavy C extension or I/O-centric loops = less impact.

**When to use:**

- Investigation before optimization — "I don't know what's slow"
- Debugging — check if unexpected functions are being called
- Detailed analysis — when function-level breakdown is needed

### CPU Time Measurement (psutil)

**Goal:** Measure actual CPU time consumed.

**What it records:**

- User time: CPU time in user space
- System time: CPU time in kernel space
- Total CPU time = user + system

**Usage example:**

```python
import psutil
import time

proc = psutil.Process()

cpu_start = proc.cpu_times()
start_cpu = cpu_start.user + cpu_start.system
wall_start = time.perf_counter()

# Code to measure
run_simulation()

cpu_end = proc.cpu_times()
end_cpu = cpu_end.user + cpu_end.system
wall_time = time.perf_counter() - wall_start

cpu_time_used = end_cpu - start_cpu

# CPU utilization
cpu_utilization = (cpu_time_used / wall_time) * 100
# Note: 1 core = 100%. Multi-threaded can exceed 100% (e.g., 4 cores max = 400%)
```

**Advantages:**

- ✅ No overhead — almost zero measurement impact
- ✅ Stable — more stable than wall time (though scheduling changes from system load can cause wall time and CPU utilization to vary)
- ✅ Simple — a single number, easy to interpret
- ✅ Comparable — strong for same-environment comparisons and regression detection (cross-environment comparison requires matching conditions)

**Disadvantages:**

- ❌ No detail — doesn't tell you which function is slow
- ❌ Coarse granularity — process-level total only
- ❌ Excludes I/O wait — pure CPU time only (sleep, network wait not included)

**Important notes:**

- ⚠️ **Multi-core:** CPU utilization calculated as `cpu_time / wall_time * 100` is per-core (1 core = 100%). Multi-threaded/multi-process execution can exceed 100%.
- ⚠️ **Environment-dependent:** CPU time varies with machine specs (CPU speed, core count, frequency, turbo, power saving). Be cautious comparing absolute values across machines.

**When to use:**

- Performance benchmarks — quantify overall performance
- Optimization verification — compare before/after
- Environment comparison — compare machines or scenarios (with matched conditions)
- Statistical analysis — repeated runs for mean/stddev

### Wall Time (time.perf_counter())

**Goal:** Measure the *actual elapsed time* ("wall clock" time).

**What it records:**

- Actual elapsed time = CPU time + wait time (I/O, sleep, etc.)

**Usage example:**

```python
import time

start = time.perf_counter()
# Code to measure
elapsed = time.perf_counter() - start
```

**Characteristics:**

- **High precision:** nanosecond-level
- **Real time:** what the user actually perceives
- **Susceptible to noise:** affected by background processes, OS load, scheduling

**Important notes:**

- ⚠️ **Statistical approach:** Wall time can be used if you repeat measurements and take statistics (e.g., median), but noise is high — requires sufficient runs and a controlled environment.
- ⚠️ **Reproducibility:** Results can vary run-to-run even with the same code.

**When to use:**

- Perceived speed measurement — actual user wait time
- Real-Time Factor (RTF) calculation — simulation speed / real time
- Timeout control — "finish within X seconds"

### Comparison Table

| Attribute | cProfile | CPU Time | Wall Time |
|-----------|----------|----------|-----------|
| **Goal** | Bottleneck identification | CPU usage measurement | Real-time measurement |
| **Granularity** | Function level | Process-wide | Process-wide |
| **Overhead** | Yes (high if many calls) | Almost none | Almost none |
| **Detail** | High (Python layer) | Low | Low |
| **Stability** | Medium | High (same environment) | Low (environment-sensitive) |
| **Use case** | Find optimization targets | Perf evaluation / regression | Perceived speed / RTF |
| **Statistical analysis** | Unsuitable (heavy) | Well-suited | Possible (needs many runs) |

### Profiling Method Variants in the Tools

| Method | Overhead | Accuracy | Detail Level | Best For |
|--------|----------|----------|--------------|----------|
| **cProfile** | Medium (5–50%) | Medium | High (all functions) | Bottleneck search |
| **Manual Timing** (monkey-patching) | Minimal (<1%) | High | Medium (specific methods) | Optimization verification |
| **Built-in Profiling** (`return_profiling` flag) | Minimal (<0.1%) | High | Medium (specific steps) | Continuous monitoring |

**Built-in Profiling implementation example:**

```python
# core_simulation.py
def check_collisions(self, return_profiling: bool = False):
    timings = {}

    if return_profiling:
        t0 = time.perf_counter()
    # ... processing ...
    if return_profiling:
        timings['step1'] = (time.perf_counter() - t0) * 1000

    return result, timings
```

### Real-World Usage in the Benchmark Suite

**`performance_benchmark.py` uses CPU Time + Wall Time:**

```python
# Spawn timing example
cpu_spawn_start = cpu_time_s(proc)      # CPU time
wall_spawn_start = time.perf_counter()  # Wall time

agents = spawn_agents()

wall_spawn_time = time.perf_counter() - wall_spawn_start
cpu_spawn_time = cpu_time_s(proc) - cpu_spawn_start

cpu_percent = (cpu_spawn_time / wall_spawn_time * 100)
```

Why: no overhead, supports repeated runs for statistics (median, mean, stdev), gives CPU utilization, strong for same-environment comparisons.

**`profiling/agent_update.py` uses cProfile:**

```python
profiler = cProfile.Profile()
profiler.enable()

for agent in agents:
    agent.update(dt=0.01)

profiler.disable()
stats = pstats.Stats(profiler)
stats.print_stats(30)
```

Why: need to find *where inside* `Agent.update()` is slow; want function-level bottleneck identification; want to see PyBullet API call frequencies; or check for unexpected work being done.

**Limitations of cProfile with PyBullet:**

- PyBullet is implemented in C++. cProfile shows total call time for `p.getAABB()` etc., but cannot see internal C++ details (AABB calculation, matrix operations, etc.).
- Overhead makes absolute performance values unreliable — use cProfile for relative comparison and structural analysis.

### Additional Tool: `time.process_time()`

A lightweight standard-library alternative for CPU time:

```python
import time

start = time.process_time()
# processing
elapsed = time.process_time() - start
```

- ✅ Standard library (no psutil dependency)
- ✅ Returns process CPU time (user + system)
- ✅ Excludes sleep time

**vs `psutil.Process.cpu_times()`:**

- `process_time()` — simple and lightweight, suited for single-process use
- `psutil` — provides detailed info (user/system breakdown, memory, child processes, etc.)

Use `process_time()` for simple CPU time measurement; use `psutil` for detailed analysis or cross-platform needs.

### Important: Profiling Limits with PyBullet (C Extension)

PyBullet is implemented in C++:

```python
# cProfile only sees the overall call time for:
p.getBasePositionAndOrientation(body_id)  # ← total time for this call
# C++ internals (AABB calculation, matrix operations, etc.) are invisible
```

**Workarounds:**

- Measure at the "call unit" level for C extensions
- Compare relative costs of different PyBullet APIs
- For C++ internals, use system-level profilers (`perf`, `gprof`)

### Multi-Core CPU Utilization

```python
cpu_utilization = (cpu_time / wall_time) * 100
```

Interpretation:

- **100%** — 1 core fully utilized
- **200%** — 2 cores fully utilized (parallel processing)
- **400%** — 4 cores fully utilized
- **50%** — half a core (lots of wait time, or light processing)

PyBullet simulation is primarily single-threaded, so this value is typically around 100%.

### Cross-Environment Comparison

**Same-environment comparison** ✅:

```
Machine A: before 50ms → after 30ms (40% speedup)
```

**Cross-environment comparison** ⚠️:

```
Machine A (i7-1185G7): 30ms
Machine B (Ryzen 9):   25ms
→ Direct comparison is unreliable (different CPU speed, frequency, memory bandwidth, etc.)
```

**Recommendations:**

- Express improvement as a *percentage* (30ms → 20ms = 33% improvement)
- For cross-environment comparison, match conditions (same agent count, scenario, Python/PyBullet version)
- Record system info (CPU model, memory, etc.) — `performance_benchmark.py` does this automatically ✅

---

## Optimization Workflow

### Recommended Flow

1. **Initial analysis:** `simulation_profiler.py` — identify the overall bottleneck
2. **Detailed analysis:** cProfile — inspect the bottleneck's internals
3. **Optimization:** Implement code changes
4. **Verification:** Manual Timing or Built-in Profiling — measure improvement
5. **Ongoing monitoring:** Built-in Profiling — continue measuring in production

### Step-by-Step

**Step 1: Get the overall picture (CPU Time)**

```bash
python benchmark/run_benchmark.py --agents 1000 --repetitions 5
```

→ Result: RTF=5.34x, Step=18.72ms

**Step 2: Identify bottleneck (cProfile)**

```bash
python benchmark/profiling/simulation_profiler.py --agents 1000 --steps 100
```

→ Result: Collision Check accounts for 77.8% of step time

**Step 3: Detailed profiling**

```bash
python benchmark/profiling/collision_check.py --agents 1000
```

→ Result: `getAABB()` and spatial hashing are slow

**Step 4: Implement optimization**

- Implement 2D collision mode
- Implement frequency control

**Step 5: Verify improvement (CPU Time)**

```bash
python benchmark/run_benchmark.py --compare no_collision collision_2d_10hz collision_3d_full
```

→ Result: 3x speedup confirmed!

### Typical Bottlenecks and Fixes

#### 1. Collision Check is slow (Collision Check > 20%)

**Fixes:**

- `collision_check_2d=True` — enable 2D mode (~67% reduction)
- `collision_check_frequency=10.0` — reduce frequency (10 Hz)
- `ignore_static_collision=True` — ignore collisions with static structures

**Verification:**

```bash
# Before: 3D, every step
python benchmark/profiling/collision_check.py --agents=1000

# After: 2D, 10Hz (change settings in config.yaml)
python benchmark/profiling/collision_check.py --agents=1000
```

#### 2. Agent Update is slow (Agent Update > 40%)

**Fixes:**

- Skip updates for stationary agents (no computation needed if not moving)
- Reduce PyBullet API calls (caching)

**Verification:**

```bash
# Check stationary vs moving difference
python benchmark/profiling/agent_update.py --agents=1000 --test=stationary

# Check PyBullet API costs
python benchmark/profiling/agent_update.py --agents=100 --test=pybullet
```

#### 3. Goal setting is slow (set_goal_pose > 100ms)

**Fixes:**

- Cache trajectory calculations
- Use simplified trajectory generation

**Verification:**

```bash
python benchmark/profiling/agent_manager_set_goal.py --agents=1000
```

### Decision Summary

| Goal | Tool / Method | Reason |
|------|--------------|--------|
| Performance benchmark | CPU Time + Wall Time | No overhead, statistical analysis |
| Bottleneck identification | cProfile | Function-level detail (Python layer) |
| Perceived speed measurement | Wall Time | Actual elapsed time (for RTF) |
| Optimization verification | CPU Time | Strong for same-environment comparison |
| Function-internal analysis | cProfile + Manual Timing | Detailed breakdown (C extension internals not visible) |
| Cross-environment comparison | CPU Time + system info recording | Relative comparison with matched conditions |

**Core principle:**

1. Start with **CPU Time + Wall Time** for overall performance
2. Use **cProfile** to find what's slow (Python layer bottleneck)
3. After optimization, re-run **CPU Time** to verify improvement
4. **Record system information** for reproducibility

---

## Troubleshooting

### Profiling logs are not displayed

**Cause:** `enable_profiling=False` or log level is not set to `DEBUG`.

**Fix:**

```python
params = SimulationParams(
    enable_profiling=True,
    log_level="debug"
)
```

Or in config:

```yaml
# config.yaml
simulation:
  enable_profiling: true
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
