# PyBullet Fleet - Performance Benchmark Suite

This directory contains the benchmark scripts, profiling tools, experiment scripts, and configuration files for PyBulletFleet performance measurement and optimization. This file also renders as part of the project documentation on ReadTheDocs.

Last Updated: 2026-03-08

---

## Performance Optimization Workflow

```mermaid
flowchart TD
    Start([Start: Need Performance?]) --> Benchmark

    Benchmark[🎯 run_benchmark.py<br/>Measure overall performance<br/>RTF, Step Time, Memory]

    Benchmark -->|RTF OK?| Done([✅ Done])
    Benchmark -->|RTF Low?| Profile

    Profile[🔍 Profiling Tools<br/>simulation_profiler.py<br/>Identify bottlenecks]

    Profile --> Analyze{What's slow?}

    Analyze -->|Collision 10-15%| CollisionExp[🧪 collision_check.py<br/>Pipeline breakdown]
    Analyze -->|Agent Update| AgentExp[🧪 agent_update.py<br/>Analyze update methods]
    Analyze -->|Other| GenericExp[🧪 experiments/<br/>Test hypotheses]

    CollisionExp --> Implement[⚙️ Implement<br/>Best approach]
    AgentExp --> Implement
    GenericExp --> Implement

    Implement --> Profile2[🔍 Profile Again<br/>Verify improvement]
    Profile2 --> Benchmark2[🎯 Benchmark Again<br/>Measure impact]
    Benchmark2 -->|Improved?| Done
    Benchmark2 -->|Still slow?| Profile

    style Benchmark fill:#4A90E2,color:#fff
    style Profile fill:#F39C12,color:#fff
    style CollisionExp fill:#9B59B6,color:#fff
    style AgentExp fill:#9B59B6,color:#fff
    style GenericExp fill:#9B59B6,color:#fff
    style Implement fill:#27AE60,color:#fff
    style Done fill:#2ECC71,color:#fff
```

**Workflow Summary:**
1. **Benchmark** → Measure overall performance (RTF, step time)
2. **Profile** → Identify bottlenecks (which component is slow?)
3. **Experiment** → Test optimization ideas (A/B testing)
4. **Implement** → Apply best approach
5. **Verify** → Profile & benchmark again

**Tool Categories:**
- 🎯 **Benchmarking:** `run_benchmark.py`, `performance_benchmark.py`
- 🔍 **Profiling:** `profiling/simulation_profiler.py`, `collision_check.py`, `agent_update.py`
- 🧪 **Experiments:** `experiments/collision_methods_config_based.py`, `performance_analysis.py`

### Profiling vs Experiments

| | `profiling/` | `experiments/` |
|---|---|---|
| **Purpose** | Identify **what is slow** | Compare **which is faster** |
| **Scope** | Internal component breakdown of PyBulletFleet | Algorithm and API alternative comparisons |
| **Output** | Time breakdown by component (%) | Method A vs Method B comparison table |
| **Usage** | Bottleneck investigation → optimization verification | Design decisions → implementation strategy selection |

**Examples:**
- `profiling/` → "Agent Update accounts for 88%" (problem discovery)
- `experiments/` → "closest_points is 1.5× faster than contact_points" (solution selection)

---

## Directory Structure

```
benchmark/
├── README.md                          # This file
│
# Benchmark Configuration Files
├── configs/                           # Benchmark-specific configurations
│   ├── general.yaml                   # General benchmark config (default)
│   ├── collision_physics_off.yaml     # Collision: Physics OFF (recommended)
│   ├── collision_physics_on.yaml      # Collision: Physics ON
│   └── collision_hybrid.yaml          # Collision: Hybrid mode
│
# Benchmark Results
├── results/                           # Benchmark results (JSON files)
│   ├── benchmark_results_*.json       # Individual test results
│   └── benchmark_sweep_*.json         # Sweep test results
│
# Main Benchmarking Tools (Worker + Orchestrator Pattern)
├── performance_benchmark.py           # Worker: Single benchmark execution
├── run_benchmark.py                   # Orchestrator: Multi-run, sweep, comparison
│
# Profiling Tools — Identify "what is slow"
├── profiling/
│   ├── simulation_profiler.py         # step_once() component breakdown
│   ├── collision_check.py             # Collision detection pipeline 4-stage analysis
│   ├── collision_mode_comparison.py   # Physics ON/OFF profile comparison
│   ├── agent_update.py                # Agent.update() detailed analysis (5 methods)
│   ├── agent_manager_set_goal.py      # set_goal_pose() profiling
│   └── profiling_config.yaml          # Shared profiling configuration
│
# Experiments — Compare "which is faster"
├── experiments/
│   ├── collision_detection_methods_benchmark.py  # PyBullet API 3-method comparison
│   ├── collision_methods_config_based.py         # Production config 3-mode comparison (recommended)
│   ├── collision_method_comparison.py            # Spatial Hash vs Brute Force
│   ├── performance_analysis.py                   # Wrapper layer performance comparison
│   ├── list_filtering_benchmark.py               # List operation micro-benchmark
│   └── getaabb_performance.py                    # getAABB() bottleneck investigation
│
# Archive (deprecated/development tools)
└── archive/
    ├── collision_check_v1.py          # Legacy collision check script
    ├── parse_profile.py               # Legacy profiling output parser
    ├── simple_agent_profile.py        # Simple agent profiling script
    └── update_benchmark.py            # Legacy config update script
```

---

## Architecture: Worker + Orchestrator Pattern

The benchmark suite uses a clean separation of concerns:

### **Worker** (`performance_benchmark.py`)
- **Purpose**: Execute a single benchmark test
- **Responsibilities**:
  - Load configuration from YAML
  - Create and run simulation
  - Measure performance metrics (RTF, step time, memory)
  - Output JSON result to stdout
- **Interface**: Simple CLI with `--agents`, `--duration`, `--scenario`
- **Isolation**: Each test runs in a separate process for clean memory state

### **Orchestrator** (`run_benchmark.py`)
- **Purpose**: Manage multiple benchmark runs and analysis
- **Responsibilities**:
  - Spawn worker processes
  - Parse and aggregate JSON results
  - Compute statistics (median, mean, stdev)
  - Generate comparison tables and summaries
- **Modes**:
  1. **Single Test**: Run one benchmark with statistics
  2. **Sweep**: Test multiple agent counts
  3. **Compare**: Compare different scenarios

This design avoids self-recursion complexity and makes the codebase more maintainable.

---

## Quick Start

### Basic Performance Benchmark

Run a single benchmark with 1000 agents for 10 seconds:

```bash
python benchmark/run_benchmark.py --agents 1000 --duration 10
```

Runs 3 repetitions by default and reports RTF, step time, spawn time, and memory delta with statistical summary and pass/fail assessment.

### Multi-Agent Sweep Benchmark

Run benchmarks across multiple agent counts:

```bash
python benchmark/run_benchmark.py --sweep 100 500 1000 2000 5000
```

Runs each agent count and produces a comparative table showing how performance scales.

### Scenario Comparison

Compare different collision detection scenarios:

```bash
python benchmark/run_benchmark.py --compare no_collision collision_2d_10hz collision_3d_full --agents 1000
```

Runs each scenario at the specified agent count and produces a side-by-side comparison table.

### Custom Scenario

Run with a specific predefined scenario:

```bash
python benchmark/run_benchmark.py --agents 1000 --scenario no_collision
```

---

## Benchmark Tools

### `run_benchmark.py` (Orchestrator)

**Purpose:** Manage benchmark execution, statistics, and analysis

**Features:**
- Three modes: single test, sweep, compare
- Statistical analysis (median, mean, stdev)
- Formatted output with assessment
- JSON export for historical tracking
- Process isolation for clean measurements

**Usage:**

```bash
# Single test with default scenario
python benchmark/run_benchmark.py --agents 1000

# Single test with custom scenario
python benchmark/run_benchmark.py --agents 1000 --scenario no_collision --duration 10

# Multi-run for statistics
python benchmark/run_benchmark.py --agents 1000 --repetitions 5

# Sweep across agent counts
python benchmark/run_benchmark.py --sweep 100 500 1000 2000

# Sweep with custom scenario and duration
python benchmark/run_benchmark.py --sweep 100 1000 5000 --scenario collision_2d_10hz --duration 30

# Compare scenarios
python benchmark/run_benchmark.py --compare no_collision collision_2d_10hz collision_3d_full --agents 1000

# With GUI (slower, for visualization)
python benchmark/run_benchmark.py --agents 100 --gui
```

**Output Files:**
- `benchmark_results_<agents>agents_<duration>s_<scenario>.json` — Single test
- `benchmark_sweep_<duration>s.json` — Sweep results
- `benchmark_compare_<agents>agents_<duration>s.json` — Scenario comparison

### `performance_benchmark.py` (Worker)

**Purpose:** Execute a single benchmark test (usually called by `run_benchmark.py`)

**Features:**
- Load configuration from YAML
- Measure RTF, step time, memory usage
- Output JSON to stdout
- Simple, focused interface

**Direct Usage** (for scripting/automation):

```bash
# Basic test
python benchmark/performance_benchmark.py --agents 1000 --duration 10

# With scenario
python benchmark/performance_benchmark.py --agents 1000 --duration 10 --scenario no_collision

# With custom config
python benchmark/performance_benchmark.py --agents 1000 --config configs/general.yaml

# With GUI
python benchmark/performance_benchmark.py --agents 100 --gui
```

**Output Format** (JSON to stdout):

```json
{
  "num_agents": 1000,
  "duration_s": 10.0,
  "scenario": "no_collision",
  "real_time_factor": 3.12,
  "avg_step_time_ms": 32.04,
  "spawn_time_s": 0.254,
  "memory_delta_mb": 3.04,
  "timestamp": "2026-03-08T10:30:45"
}
```

---

## Benchmark Results

### TL;DR

| Agents | RTF (×) | Step Time (ms) | Collisions Scale |
|--------|---------|-----------------|------------------|
| 100    | 46.0    | 2.2             | Excellent        |
| 500    | 7.1     | 14.1            | Good             |
| 1000   | 3.1     | 32.0            | Good             |
| 2000   | 1.2     | 84.6            | Real-time limit  |

**Real-Time Factor (RTF):** How many seconds of simulation time can be executed in 1 second of wall-clock time (higher is better; >1.0 means faster than real-time).

- `RTF = 1.0`: Real-time (1s simulation = 1s wall time)
- `RTF > 1.0`: Faster than real-time (e.g., 2.0× = 2s simulation in 1s wall time)
- `RTF < 1.0`: Slower than real-time (e.g., 0.5× = 1s simulation takes 2s wall time)

**Assessment:**
- ✅ **Excellent:** RTF > 2.0
- ⚠️ **Good:** RTF 1.0 – 2.0
- ❌ **Poor:** RTF < 1.0

All runs use kinematics mode (physics OFF), headless (DIRECT), half of agents moving.

### Test Environment

- **CPU**: Intel Core i7-1185G7 (11th Gen, 4C/8T, 3.0 GHz / 4.8 GHz turbo)
- **Memory**: 32 GB RAM
- **OS**: Ubuntu 20.04.1 LTS (Linux 5.15.0-139-generic)
- **Python**: 3.8.10, PyBullet latest
- **Mode**: DIRECT (headless), kinematics
- **Methodology**: 3 repetitions, 10 s duration each, mean ± std reported

### Performance Summary

> **Script:** `run_benchmark.py --sweep 100 250 500 1000 2000 --duration 10 --repetitions 3`

Performance sweep across agent counts (3 reps, 10 s duration, kinematics mode, headless):

| Agents | RTF (×) | Step Time (ms) | Spawn Time (s) | Memory Delta (MB) |
|--------|---------|----------------|------------------|--------------------|
| 100    | 46.03±4.06 | 2.17±0.18  | 0.026±0.001      | −23.83±0.06        |
| 250    | 16.12±1.41 | 6.20±0.50  | 0.062±0.001      | −19.68±0.09        |
| 500    | 7.11±0.11  | 14.06±0.22 | 0.128±0.004      | −12.12±0.02        |
| 1000   | 3.12±0.09  | 32.04±0.87 | 0.254±0.003      | 3.04±0.20          |
| 2000   | 1.18±0.02  | 84.63±1.72 | 0.653±0.051      | 29.52±0.02         |

Negative memory delta means the process used less memory than the baseline measurement
taken before spawning agents (OS page-cache effects).

### Component Breakdown

> **Script:** `profiling/simulation_profiler.py` (1000 agents, 100 steps)

Profiled with 1000 agents, 100 steps:

| Component        | Time (ms) | Share (%) |
|------------------|-----------|-----------|
| Agent Update     | 13.79     | 88.2      |
| Collision Check  | 1.76      | 11.2      |
| Monitor Update   | 0.08      | 0.5       |
| Step Simulation  | 0.00      | 0.0       |
| **Total**        | **15.63** | **100.0** |

`Step Simulation` is 0 ms because `physics=false` (kinematics mode skips `stepSimulation()`).

Agent Update dominates because it includes path-following, velocity computation, and
`resetBasePositionAndOrientation()` calls for every agent each step.

### Collision Method Comparison

> **Script:** `experiments/collision_methods_config_based.py` (100 agents, 500 steps)

100 agents, 500 steps, headless:

| Config                      | Method          | Step (ms) | Collision (ms) | Collisions |
|-----------------------------|-----------------|-----------|----------------|------------|
| `collision_physics_off.yaml` | closest_points  | 0.252     | 0.004          | 39         |
| `collision_physics_on.yaml`  | contact_points  | 0.882     | 0.005          | 4          |
| `collision_hybrid.yaml`      | hybrid          | 0.801     | 0.006          | 37         |

Key observations:

- **Physics OFF is ~3.5× faster** than Physics ON overall.
- `closest_points` detects more collisions because it uses a safety margin (2 cm)
  that catches near-misses before actual contact.
- `contact_points` only reports actual penetrations, hence fewer detections.
- Hybrid combines both methods at a small extra cost.

**Recommendation**: Use Physics OFF with `closest_points` for production workloads.

### Scaling Analysis

#### Step Time

```text
Agents:     100  →   250  →   500  →  1000  →  2000
Step (ms):  2.17 →  6.20 → 14.06 → 32.04 → 84.63
Ratio:      1.0x →  2.9x →  6.5x → 14.8x → 39.0x
```

Scaling is approximately O(n^1.3). Below 500 agents the relationship is near-linear;
above that, increased collision-pair density adds super-linear overhead.

#### Spawn Time

Spawn time scales linearly: ~0.25 ms per agent.

#### Memory

Memory grows linearly above ~500 agents at roughly 20 KB per agent.
Below 500 agents the delta is dominated by OS page-cache noise.

*Data collected 2026-03-08 on the test environment described above.*

For collision detection architecture and design rationale, see the Collision Detection Architecture page in the project documentation.

---

## Benchmark Configs

All config files live in `benchmark/configs/`. They share common settings:

```yaml
target_rtf: 0              # Maximum speed (no sleep)
gui: false                  # Headless
enable_time_profiling: true # Profiling enabled
log_level: error            # Suppress logs
```

### `general.yaml` — Default Performance Benchmark

General-purpose sweep used by `performance_benchmark.py` and `run_benchmark.py`.

| Parameter   | Value  | Notes                  |
|-------------|--------|------------------------|
| `timestep`  | 0.1    | 10 Hz                  |
| `physics`   | false  | Kinematics only        |
| `num_agents`| 1000   | Default count          |
| `mass`      | 0.0    | Kinematic control      |
| `duration`  | 10.0   | Seconds per run        |

### `collision_physics_off.yaml` — Physics OFF + closest_points

Recommended collision benchmark. Fastest and deterministic.

| Parameter                    | Value            |
|------------------------------|------------------|
| `physics`                    | false            |
| `collision_detection_method` | `closest_points` |
| `collision_margin`           | 0.02 (2 cm)     |
| `timestep`                   | 0.00416666 (240 Hz) |

### `collision_physics_on.yaml` — Physics ON + contact_points

Physics verification benchmark with real contact manifold.

| Parameter                    | Value            |
|------------------------------|------------------|
| `physics`                    | true             |
| `collision_detection_method` | `contact_points` |
| `collision_margin`           | 0.0              |
| `timestep`                   | 0.00416666 (240 Hz) |

### `collision_hybrid.yaml` — Physics ON + hybrid

Mixed mode: `getContactPoints()` for physics pairs, `getClosestPoints()` for kinematic pairs.

| Parameter                    | Value            |
|------------------------------|------------------|
| `physics`                    | true             |
| `collision_detection_method` | `hybrid`         |
| `collision_margin`           | 0.02 (2 cm)     |
| `timestep`                   | 0.00416666 (240 Hz) |

### Running Benchmarks with Configs

```bash
# Performance sweep
cd benchmark
python performance_benchmark.py --config configs/general.yaml

# Collision method comparison
python experiments/collision_methods_config_based.py
```

---

## Profiling Guide

### Overview

The profiling tools in `benchmark/profiling/` provide detailed CPU time analysis of specific simulation components.

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

### Simulation Profiler (`simulation_profiler.py`)

Component-level time measurement and bottleneck identification within `step_once()`. Unlike `performance_benchmark.py` (overall JSON output), this tool provides per-component statistical breakdowns.

#### Measured Components

| Component | Description | Typical Share |
|-----------|-------------|---------------|
| Agent Update | State updates for all agents (trajectory following, kinematics) | 80-90% (when ~50% agents move) |
| Collision Check | Collision detection (spatial hashing, AABB) | 10-15% |
| PyBullet Step | Physics simulation step | 0% (physics off) or 20-40% (physics on) |
| Monitor Update | Data monitor updates | <1% |

#### Analysis Methods

| Method | Command | Purpose |
|--------|---------|---------|
| Built-in Profiling | `--test=builtin` (default) | Component time distribution from `step_once()` |
| cProfile | `--test=cprofile` | All-function bottleneck search |
| Motion Modes | `--test=motion_modes` | DIFFERENTIAL vs OMNIDIRECTIONAL comparison |

#### CLI Usage

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

#### Output Example

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

#### Follow-Up Analysis

- Agent Update is slow → use `agent_update.py` for detailed analysis
- Collision Check is slow → use `collision_check.py` for detailed analysis

### Collision Check Profiler (`collision_check.py`)

Breaks the collision detection pipeline into 4 steps for bottleneck identification.

#### 4-Step Breakdown

| Step | Description | Typical Share |
|------|-------------|---------------|
| Get AABBs | Fetch bounding boxes from PyBullet | ~10% |
| Spatial Hashing | Build spatial grid | ~6% |
| AABB Filtering | Candidate pair selection (27-neighbor search) | **~75%** |
| Contact Points | Actual collision check in PyBullet | ~9% |

#### CLI Usage

```bash
# Built-in profiling (default, recommended)
python benchmark/profiling/collision_check.py --agents=1000 --iterations=100

# Detailed analysis with cProfile (function level)
python benchmark/profiling/collision_check.py --agents=1000 --test=cprofile

# Run both
python benchmark/profiling/collision_check.py --agents=1000 --test=all
```

#### Output Example

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

#### Optimization Hints

- AABB Filtering at 75% → 2D mode can reduce it by ~67%
- Collision ratio of 0.3% → room for improving filtering precision

### Agent Update Profiler (`agent_update.py`)

`Agent.update()` runs every frame for every agent. This tool provides 5 analysis methods.

#### Five Analysis Methods

| Method | Command | Purpose | Overhead |
|--------|---------|---------|----------|
| cProfile | `--test=cprofile` | All-function bottleneck search | Medium (5-50%) |
| Manual Timing | `--test=manual` | Precise measurement of specific methods | Minimal (<1%) |
| PyBullet API | `--test=pybullet` | C++ API cost measurement | Low (1-5%) |
| Stationary vs Moving | `--test=stationary` | Impact of movement/update processing | None |
| Motion Modes | `--test=motion_modes` | DIFFERENTIAL vs OMNIDIRECTIONAL | None |

#### CLI Usage

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

#### Output Summary

Each method produces a focused report:

- **Manual Timing** — per-component mean/median/max in microseconds (e.g., `update_differential`, `update_actions`)
- **PyBullet API** — call counts, total time, and per-call average for each PyBullet function
- **Stationary vs Moving** — total time and per-agent cost for idle vs. moving agents, plus overhead ratio
- **Motion Modes** — side-by-side DIFFERENTIAL vs OMNIDIRECTIONAL cost comparison

#### When to Use Which Method

1. **Bottleneck identification** → `--test=cprofile` — find slow functions
2. **Optimization verification** → `--test=manual` — accurate before/after comparison
3. **PyBullet API optimization** → `--test=pybullet` — identify expensive API calls
4. **Stationary agent impact** → `--test=stationary` — quantify movement overhead
5. **Motion mode comparison** → `--test=motion_modes` — compare DIFFERENTIAL vs OMNIDIRECTIONAL

### Goal Setting Profiler (`agent_manager_set_goal.py`)

Uses cProfile to analyze `AgentManager.set_goal_pose()` and the trajectory calculation call chain.

#### CLI Usage

```bash
python benchmark/profiling/agent_manager_set_goal.py --agents=1000
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

`_init_differential_rotation_trajectory` typically accounts for ~84% of the time due to rotation matrix calculations (`scipy.spatial.transform`). Potential improvements: caching, pre-computation.

### Measurement Methods

The profiling tools use three measurement approaches. **cProfile** provides function-level call counts and cumulative times — use it to find *which* function is slow (note: adds 5-50% overhead and cannot see inside PyBullet C++ internals). **CPU Time** (`psutil` or `time.process_time()`) measures actual CPU consumption with near-zero overhead — use it for benchmarks, regression detection, and before/after comparison. **Wall Time** (`time.perf_counter()`) measures real elapsed time including I/O waits — use it for perceived speed and Real-Time Factor calculation, but expect higher variance.

| Attribute | cProfile | CPU Time | Wall Time |
|-----------|----------|----------|-----------|
| **Goal** | Bottleneck identification | CPU usage measurement | Real-time measurement |
| **Granularity** | Function level | Process-wide | Process-wide |
| **Overhead** | Yes (high if many calls) | Almost none | Almost none |
| **Detail** | High (Python layer) | Low | Low |
| **Stability** | Medium | High (same environment) | Low (environment-sensitive) |
| **Use case** | Find optimization targets | Perf evaluation / regression | Perceived speed / RTF |

### Profiling Optimization Workflow

#### Recommended Flow

1. **Initial analysis:** `simulation_profiler.py` — identify the overall bottleneck
2. **Detailed analysis:** cProfile — inspect the bottleneck's internals
3. **Optimization:** Implement code changes
4. **Verification:** Manual Timing or Built-in Profiling — measure improvement
5. **Ongoing monitoring:** Built-in Profiling — continue measuring in production

#### Typical Bottlenecks and Fixes

**1. Collision Check is slow (Collision Check > 20%)**

Fixes:
- Set `collision_mode=CollisionMode.NORMAL_2D` on agent spawn params — enable 2D mode (~67% reduction)
- `collision_check_frequency=10.0` — reduce frequency (10 Hz)
- `ignore_static_collision=True` — ignore collisions with static structures

Verification:

```bash
python benchmark/profiling/collision_check.py --agents=1000
```

**2. Agent Update is slow (Agent Update > 40%)**

Fixes:
- Skip updates for stationary agents
- Reduce PyBullet API calls (caching)

Verification:

```bash
python benchmark/profiling/agent_update.py --agents=1000 --test=stationary
python benchmark/profiling/agent_update.py --agents=100  --test=pybullet
```

**3. Goal setting is slow (set_goal_pose > 100ms)**

Fixes:
- Cache trajectory calculations
- Use simplified trajectory generation

Verification:

```bash
python benchmark/profiling/agent_manager_set_goal.py --agents=1000
```

### Profiling Troubleshooting

#### Profiling logs are not displayed

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

#### Segfault with cProfile

**Cause:** Compatibility issues between cProfile and PyBullet's C++ extension.

**Fix:**
- Use Manual Timing or Built-in Profiling instead
- Try `agent_update.py --test=manual`
- Use `agent_update.py --test=stationary` (does not use cProfile)

#### High variance in measurement results

**Cause:** Background process interference, thermal throttling.

**Fix:**
- Run multiple iterations and compute statistics
- Increase iteration count with `--iterations`
- Check CPU utilization

---

## Experiment Scripts

Experiments validate optimization hypotheses and compare different implementation approaches. Unlike profiling tools (which measure what is happening), these scripts answer specific questions about algorithm and API choices.

All scripts live in `benchmark/experiments/`.

### Collision Detection Experiments

#### `collision_detection_methods_benchmark.py`

**Purpose:** Compare PyBullet collision detection APIs in an isolated environment.

```bash
python benchmark/experiments/collision_detection_methods_benchmark.py
```

**What it tests:**
- `getContactPoints()` — physics contact manifold
- `getClosestPoints()` — distance-based detection
- Hybrid — closest for kinematic pairs, contact for physics pairs

Outputs a comparison table with average time per step and collision counts for each method.

See the Collision Detection Architecture page in the project documentation for design rationale.

#### `collision_methods_config_based.py` (Recommended)

**Purpose:** Compare collision detection using production config files with a full PyBulletFleet simulation (moving robots, physics integration).

```bash
python benchmark/experiments/collision_methods_config_based.py
```

**What it tests:**
- Physics OFF + `CLOSEST_POINTS` (kinematics mode)
- Physics ON + `CONTACT_POINTS` (physics mode)
- Physics ON + `HYBRID` (mixed mode)

Outputs per-config collision time and total step time, with a recommendation of the fastest mode.

#### `collision_method_comparison.py`

**Purpose:** Algorithm-level comparison — spatial hashing O(N) vs brute-force O(N²) vs raw PyBullet APIs.

```bash
python benchmark/experiments/collision_method_comparison.py
```

**What it tests:**
- Spatial Hashing (current implementation, AABB filtering + `getContactPoints`)
- Brute Force AABB (all-pairs overlap check)
- `getClosestPoints` (all pairs)
- `getContactPoints` batch (no args, all contacts at once)
- `getContactPoints` pairwise (per-pair calls)

Outputs a table comparing time and detected collisions for each method at configurable object counts.

See the Collision Detection Architecture page in the project documentation for design rationale.

### General Performance Experiments

#### `performance_analysis.py`

**Purpose:** Measure wrapper-layer overhead — direct PyBullet vs SimObject vs Manager APIs.

```bash
python benchmark/experiments/performance_analysis.py
```

**What it tests:**
- SimObject spawn/update performance
- SimObjectManager bulk operations
- Agent spawn/update performance
- AgentManager bulk operations

Each test runs in process isolation with CPU time (user+sys) and memory tracking. Outputs statistical summaries (median, mean, stdev) across repetitions.

#### `list_filtering_benchmark.py`

**Purpose:** Micro-benchmark for Python list filtering patterns used in collision detection.

```bash
python benchmark/experiments/list_filtering_benchmark.py
```

**What it tests:**
- List comprehension (current approach)
- Set difference
- Single-pass for-loop iteration
- Pre-computed set membership

Outputs speedup ratios relative to the current implementation.

#### `getaabb_performance.py`

**Purpose:** Determine whether `p.getAABB()` is a bottleneck.

```bash
python benchmark/experiments/getaabb_performance.py
```

**What it tests:**
- Per-call and batch `getAABB()` timing across many objects

Outputs per-call latency and batch throughput to confirm whether AABB retrieval is a limiting factor.

---

## Optimization Guide

### Quick Start Configurations

#### Maximum Performance (Offline Batch Processing)

```python
from pybullet_fleet.core_simulation import SimulationParams
from pybullet_fleet.types import CollisionMode

params = SimulationParams(
    target_rtf=0,                    # No sleep, maximum speed
    enable_monitor_gui=False,   # Headless data collection
    collision_check_frequency=10.0,  # 10 Hz collision check
    enable_time_profiling=False,     # Disable profiling overhead
    physics=False,              # Disable physics (if not needed)
    timestep=0.1                # Larger timestep for kinematics
)

# Collision mode is per-agent (set on AgentSpawnParams, not SimulationParams)
# AgentSpawnParams(..., collision_mode=CollisionMode.NORMAL_2D)  # 9 neighbors (XY only)
```

**Expected Performance:** 100 agents ~46× RTF, 500 agents ~7.1× RTF, 1000 agents ~3.1× RTF

#### Real-Time Visualization (Interactive Development)

```python
from pybullet_fleet.types import CollisionMode

params = SimulationParams(
    target_rtf=1.0,                  # Real-time synchronization
    gui=True,                   # Enable GUI
    enable_monitor_gui=True,    # Enable visualization
    collision_check_frequency=10.0,  # Reduced frequency for performance
    enable_time_profiling=True,      # Enable performance logging
    timestep=0.01               # Finer timestep for smooth visualization
)

# For 3D collision, set per-agent:
# AgentSpawnParams(..., collision_mode=CollisionMode.NORMAL_3D)  # 27 neighbors (full 3D)
```

**Recommendation:** Limit to **< 200 agents** for smooth 60 FPS rendering.

#### Production (Balanced Performance)

```yaml
simulation:
  target_rtf: 0                      # Maximum speed
  timestep: 0.1                 # Optimized for kinematics
  physics: false                # Disable unless needed
  gui: false                    # Headless

  # Collision detection
  collision_check_frequency: 10.0  # 10 Hz (balanced)
  ignore_static_collision: true # Skip structure-structure collisions
  # NOTE: collision_mode is per-agent (set on spawn params, not here)
  # Use collision_mode: normal_2d on AgentSpawnParams for 2D (9 neighbors)

  # Monitoring (headless)
  monitor: true
  enable_monitor_gui: false     # No GUI overhead

  # Performance
  enable_time_profiling: false       # Disable for production
  enable_collision_color_change: false  # Disable visual feedback
```

### Key Parameters

#### RTF Control

| Parameter | Value | Use Case |
|-----------|-------|----------|
| `target_rtf=0` | Maximum speed (no sleep) | Offline batch processing |
| `target_rtf=1.0` | Real-time | Interactive development, visualization |
| `target_rtf=2.0` | 2× real-time | Faster testing with GUI |

`target_rtf=0` runs as fast as possible, limited only by CPU performance.

#### Timestep

| Timestep | Use Case | Pros | Cons |
|----------|----------|------|------|
| `0.01` (10ms) | Physics simulation, smooth visualization | Accurate physics | Higher CPU load |
| `0.1` (100ms) | Kinematics-only, batch processing | Low overhead | Less precise physics |
| `0.001` (1ms) | High-precision physics | Very accurate | Very slow |

**Recommendation:** Kinematics mode → `0.1`; Physics enabled → `0.01` or smaller; Visualization → `0.01` for smooth motion.

#### Collision Detection

**Frequency Control** (`collision_check_frequency`):

| Frequency | Overhead | Use Case |
|-----------|----------|----------|
| `null` (every step) | High | Safety-critical, dense environments |
| `10.0` Hz | Low | Production (recommended) |
| `1.0` Hz | Very low | Sparse environments, low-risk |
| `0` (disabled) | None | Baseline performance testing |

**Dimension Control** (`collision_mode`, per-agent):

| Mode (per-agent) | Neighbors | Speedup | Use Case |
|-------------------|-----------|---------|----------|
| `collision_mode=CollisionMode.NORMAL_2D` | 9 (XY plane) | ~67% faster | Ground robots (AGVs), fixed Z |
| `collision_mode=CollisionMode.NORMAL_3D` | 27 (3D cube) | Baseline | Drones, lifts, 3D movement |
| `collision_mode=CollisionMode.DISABLED` | 0 | No overhead | Visualization-only objects |

```python
from pybullet_fleet.types import CollisionMode
from pybullet_fleet.agent import AgentSpawnParams

# 2D collision is set per-agent on spawn params (not on SimulationParams)
agent_params = AgentSpawnParams(
    urdf_path="robots/mobile_robot.urdf",
    collision_mode=CollisionMode.NORMAL_2D,  # 9 neighbors (XY only)
)
```

#### Profiling Toggle

| Mode | Overhead | Use Case |
|------|----------|----------|
| `enable_time_profiling=False` | None | Production |
| `enable_time_profiling=True` | ~5-10% | Development, optimization |

```python
params = SimulationParams(enable_time_profiling=True)
sim_core = MultiRobotSimulationCore(params)
sim_core.set_profiling_log_frequency(10)  # Log every 10 steps
```

### Configuration Examples

#### Warehouse (Ground Robots)

```yaml
simulation:
  timestep: 0.1
  target_rtf: 0
  physics: false
  gui: false
  collision_check_frequency: 10.0
  ignore_static_collision: true
  # NOTE: collision_mode is per-agent (set on AgentSpawnParams)
  # Example: AgentSpawnParams(..., collision_mode=CollisionMode.NORMAL_2D)
  monitor: true
  enable_monitor_gui: false
  enable_collision_shapes: false
  enable_structure_transparency: false
  enable_shadows: false
```

**Expected:** 500 agents ~7.1× RTF, 1000 agents ~3.1× RTF

#### Drone Swarm (3D Movement)

```yaml
simulation:
  timestep: 0.01
  target_rtf: 1.0
  physics: true
  gui: true
  collision_check_frequency: 30.0
  ignore_static_collision: false
  # NOTE: collision_mode is per-agent (set on AgentSpawnParams)
  # Example: AgentSpawnParams(..., collision_mode=CollisionMode.NORMAL_3D)
  monitor: true
  enable_monitor_gui: true
  enable_time_profiling: true
```

**Recommendation:** Limit to ~100 agents for real-time performance.

#### Development / Debugging

```yaml
simulation:
  timestep: 0.01
  target_rtf: 1.0
  physics: false
  gui: true
  collision_check_frequency: null  # Every step
  # NOTE: collision_mode is per-agent (set on AgentSpawnParams)
  # Use CollisionMode.NORMAL_3D for full 3D debugging
  enable_collision_shapes: true
  enable_structure_transparency: true
  enable_collision_color_change: true
  monitor: true
  enable_monitor_gui: true
  enable_time_profiling: true
```

### Performance Metrics

#### Step Time

**Definition:** Average time to execute one simulation step.

**Targets:**
- Real-time visualization (60 FPS): < 16.7 ms per step
- Real-time control (100 Hz): < 10 ms per step
- Offline processing: No strict requirement

**Typical Bottlenecks:**
1. **Agent Update (80-90%)** — Navigation, kinematics, and path following
2. **Collision Check (10-15%)** — Spatial hashing and AABB filtering
3. **Step Simulation (0-40%)** — 0% with physics off; up to 40% with physics on

#### Memory Usage

**Expected:** ~20 KB per agent (linear scaling above ~500 agents).

Memory grows linearly above ~500 agents. Below that, deltas are dominated by OS page-cache noise. See the Benchmark Results section above for detailed per-agent-count measurements.

### Use Case Recommendations

#### Real-Time Visualization (RTF > 1.0)

```python
params = SimulationParams(
    target_rtf=1.0,
    gui=True,
    timestep=0.01,
    collision_check_frequency=10.0
)
```

**Agent Limit:** < 200 agents. Tips: use `CollisionMode.NORMAL_2D` on agent spawn params if applicable, reduce frequency to 5-10 Hz, disable profiling for production.

#### Offline Batch Simulation

```python
params = SimulationParams(
    target_rtf=0,
    gui=False,
    enable_monitor_gui=False,
    timestep=0.1,
    collision_check_frequency=10.0
)
```

**Performance:** 1000 agents ~3.1× RTF, 500 agents ~7.1× RTF. Tips: disable all visualization, use 2D collision for ground robots, consider batching multiple simulations in parallel.

#### Large-Scale Testing (>2000 agents)

```python
params = SimulationParams(
    target_rtf=0,
    gui=False,
    timestep=0.1,
    collision_check_frequency=1.0,
)

# Use 2D collision per-agent for ground robots:
# AgentSpawnParams(..., collision_mode=CollisionMode.NORMAL_2D)
```

**Performance:** 2000 agents ~1.2× RTF (at real-time limit). Tips: accept slower-than-real-time for comprehensive testing, use profiling to identify bottlenecks, reduce collision frequency to 1 Hz.

#### Development / Debugging

```python
params = SimulationParams(
    target_rtf=1.0,
    gui=True,
    enable_time_profiling=True,
    enable_collision_color_change=True,
    collision_check_frequency=None  # Every step
)
```

Tips: use smaller agent counts (<100) for fast iteration, enable collision visualization, switch to `target_rtf=0` for stress testing.

### Troubleshooting

#### Low RTF / High Step Time

**Diagnosis:**

```bash
python benchmark/profiling/simulation_profiler.py --agents 1000 --steps 100
```

**Common fixes:**

1. **Collision check > 80% of step time** → use 2D collision per-agent (`collision_mode=CollisionMode.NORMAL_2D` on spawn params, `collision_check_frequency: 10.0`)
2. **GUI overhead** → disable GUI for batch processing (`gui: false`, `enable_monitor_gui: false`)
3. **Too many agents** → reduce agent count (real-time: < 200, offline: < 1000 recommended)
4. **Profiling overhead** → disable profiling for production (`enable_time_profiling: false`)

#### Memory Growth

**Expected:** ~20 KB per agent (linear scaling above ~500 agents).

**If super-linear:**
- Check for memory leaks in custom callbacks
- Verify objects are properly cleaned up
- Use `del` or weak references for temporary objects

```bash
python benchmark/performance_benchmark.py --agents 2000 --duration 60
```

#### Inconsistent Results

**Common causes:**
1. **Background processes** — close unnecessary applications, check `htop`
2. **Thermal throttling** — monitor CPU temperature, use shorter durations
3. **Swap usage** — check available RAM, reduce agent count
4. **PyBullet warmup** — first run may be slower; use multiple repetitions and report median

```bash
python benchmark/run_benchmark.py --agents 1000 --repetitions 10
```

### Advanced Optimization

#### Performance Hierarchy (Fastest to Slowest)

1. **No collision detection** (`collision_check_frequency=0`) — ~6× speedup, baseline testing only
2. **2D collision at 1 Hz** (`collision_mode=CollisionMode.NORMAL_2D` per-agent, `frequency=1.0`) — ~3× speedup, sparse environments
3. **2D collision at 10 Hz** (Recommended) — ~2× speedup, balanced for production
4. **3D collision at 10 Hz** — baseline, for 3D movement (drones, lifts)
5. **3D collision every step** (`frequency=null`) — slowest, maximum accuracy

#### Future Optimizations (Not Yet Implemented)

**Short-term:**
- Pre-computed neighbor lists for static structures
- Adaptive collision frequency based on agent density
- Multi-threading for collision detection
- GPU-accelerated collision detection

**Long-term:**
- Custom collision engine (bypass PyBullet overhead)
- Distributed simulation for >10,000 agents
- Incremental collision updates (only check moving agents)

### Summary

#### Quick Recommendations

| Use Case | Agents | Config | Expected RTF |
|----------|--------|--------|--------------|
| Real-time GUI | < 200 | `target_rtf=1.0`, `gui=true`, `2d=true` | > 7× |
| Offline batch | < 1000 | `target_rtf=0`, `gui=false`, `2d=true` | 3-7× |
| Large-scale test | < 2000 | `target_rtf=0`, `gui=false`, `freq=1Hz` | 1-3× |
| Development | < 100 | `target_rtf=1.0`, `gui=true`, `profiling=true` | > 46× |

#### Key Takeaways

1. **Use 2D collision** for ground robots → ~67% speedup
2. **Reduce collision frequency** to 10 Hz → minimal impact on safety
3. **Disable GUI** for batch processing → significant speedup
4. **Use `target_rtf=0`** for offline simulation → maximum throughput
5. **Limit agents** to < 200 for real-time, < 1000 for offline

---

**Last Updated:** 2026-03-08
**PyBullet Fleet Version:** Latest (post-optimization)
