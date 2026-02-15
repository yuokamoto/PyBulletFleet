# PyBullet Fleet - Performance Optimization Guide

This guide provides comprehensive recommendations for optimizing PyBullet Fleet simulation performance across different use cases.

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Key Parameters](#key-parameters)
3. [Configuration Examples](#configuration-examples)
4. [Performance Metrics](#performance-metrics)
5. [Use Case Recommendations](#use-case-recommendations)
6. [Troubleshooting](#troubleshooting)
7. [Advanced Optimization](#advanced-optimization)

---

## Quick Start

### For Maximum Performance (Offline Batch Processing)

```python
from pybullet_fleet.core_simulation import SimulationParams

params = SimulationParams(
    speed=0,                    # No sleep, maximum speed
    enable_monitor_gui=False,   # Headless data collection
    collision_check_2d=True,    # 2D optimization (9 neighbors vs 27)
    collision_check_frequency=10.0,  # 10 Hz collision check
    enable_profiling=False,     # Disable profiling overhead
    physics=False,              # Disable physics (if not needed)
    timestep=0.1                # Larger timestep for kinematics
)
```

**Expected Performance:**
- 1000 agents: ~0.45x RTF (slower than real-time, but acceptable for offline)
- 500 agents: ~0.93x RTF (near real-time)
- 100 agents: ~6.61x RTF (much faster than real-time)

---

### For Real-Time Visualization (Interactive Development)

```python
params = SimulationParams(
    speed=1.0,                  # Real-time synchronization
    gui=True,                   # Enable GUI
    enable_monitor_gui=True,    # Enable visualization
    collision_check_2d=False,   # Full 3D collision (if needed)
    collision_check_frequency=10.0,  # Reduced frequency for performance
    enable_profiling=True,      # Enable performance logging
    timestep=0.01               # Finer timestep for smooth visualization
)
```

**Recommendation:** Limit to **< 200 agents** for smooth 60 FPS rendering

---

### For Production (Balanced Performance)

```yaml
# config/production_config.yaml
simulation:
  speed: 0                      # Maximum speed
  timestep: 0.1                 # Optimized for kinematics
  physics: false                # Disable unless needed
  gui: false                    # Headless

  # Collision detection (2D optimized)
  collision_check_2d: true      # 2D mode (9 neighbors vs 27)
  collision_check_frequency: 10.0  # 10 Hz (balanced)
  ignore_static_collision: true # Skip structure-structure collisions

  # Monitoring (headless)
  monitor: true
  enable_monitor_gui: false     # No GUI overhead

  # Performance
  enable_profiling: false       # Disable for production
  enable_collision_color_change: false  # Disable visual feedback
```

---

## Key Parameters

### Speed Control

| Parameter | Value | Use Case |
|-----------|-------|----------|
| `speed=0` | Maximum speed (no sleep) | Offline batch processing |
| `speed=1.0` | Real-time | Interactive development, visualization |
| `speed=2.0` | 2x real-time | Faster testing with GUI |

**Note:** `speed=0` runs as fast as possible, limited only by CPU performance.

---

### Timestep

| Timestep | Use Case | Pros | Cons |
|----------|----------|------|------|
| `0.01` (10ms) | Physics simulation, smooth visualization | Accurate physics | Higher CPU load |
| `0.1` (100ms) | Kinematics-only, batch processing | Low overhead | Less precise physics |
| `0.001` (1ms) | High-precision physics | Very accurate | Very slow |

**Recommendation:**
- Kinematics mode: `0.1` (default for benchmarks)
- Physics enabled: `0.01` or smaller
- Visualization: `0.01` for smooth motion

---

### Collision Detection

#### Frequency Control

| Frequency | Overhead | Use Case |
|-----------|----------|----------|
| `null` (every step) | High | Safety-critical, dense environments |
| `10.0` Hz | Low | Production (recommended) |
| `1.0` Hz | Very low | Sparse environments, low-risk |
| `0` (disabled) | None | Baseline performance testing |

#### Dimension Control

| Mode | Neighbors | Speedup | Use Case |
|------|-----------|---------|----------|
| `collision_check_2d=true` | 9 (XY plane) | ~67% faster | Ground robots (AGVs), fixed Z |
| `collision_check_2d=false` | 27 (3D cube) | Baseline | Drones, lifts, 3D movement |

**Example:**
```python
# 2D collision (recommended for ground robots)
params = SimulationParams(
    collision_check_2d=True,
    collision_check_frequency=10.0
)
```

---

### Profiling

| Mode | Overhead | Use Case |
|------|----------|----------|
| `enable_profiling=False` | None | Production |
| `enable_profiling=True` | ~5-10% | Development, optimization |

**Usage:**
```python
params = SimulationParams(
    enable_profiling=True
)

# Set profiling log frequency
sim_core = MultiRobotSimulationCore(params)
sim_core.set_profiling_log_frequency(10)  # Log every 10 steps
```

---

## Configuration Examples

### Example 1: ASRS Warehouse (Ground Robots)

```yaml
# config/asrs_config.yaml
simulation:
  timestep: 0.1                 # Kinematics-optimized
  speed: 0                      # Maximum speed
  physics: false                # Kinematics only
  gui: false                    # Headless

  # 2D collision (ground robots)
  collision_check_2d: true
  collision_check_frequency: 10.0
  ignore_static_collision: true

  # Monitoring
  monitor: true
  enable_monitor_gui: false

  # Visualization (disabled for performance)
  enable_collision_shapes: false
  enable_structure_transparency: false
  enable_shadows: false
```

**Expected Performance:**
- 500 agents: ~0.93x RTF
- 1000 agents: ~0.45x RTF

---

### Example 2: Drone Swarm (3D Movement)

```yaml
# config/drone_config.yaml
simulation:
  timestep: 0.01                # Finer control
  speed: 1.0                    # Real-time
  physics: true                 # Enable physics
  gui: true                     # Visualization

  # 3D collision (drones)
  collision_check_2d: false
  collision_check_frequency: 30.0  # Higher frequency for safety
  ignore_static_collision: false

  # Monitoring
  monitor: true
  enable_monitor_gui: true

  # Development
  enable_profiling: true
```

**Recommendation:** Limit to ~100 agents for real-time performance

---

### Example 3: Development/Debugging

```yaml
# config/debug_config.yaml
simulation:
  timestep: 0.01
  speed: 1.0                    # Real-time
  physics: false
  gui: true                     # Enable GUI

  # Full collision (for debugging)
  collision_check_2d: false
  collision_check_frequency: null  # Every step

  # Visual debugging
  enable_collision_shapes: true
  enable_structure_transparency: true
  enable_collision_color_change: true

  # Monitoring
  monitor: true
  enable_monitor_gui: true
  enable_profiling: true
```

---

## Performance Metrics

### Real-Time Factor (RTF)

**Definition:** How many seconds of simulation time can be executed in 1 second of wall-clock time

**Interpretation:**
- `RTF = 1.0`: Real-time (1s simulation = 1s wall time)
- `RTF > 1.0`: Faster than real-time (e.g., 2.0x = 2s simulation in 1s wall time)
- `RTF < 1.0`: Slower than real-time (e.g., 0.5x = 1s simulation takes 2s wall time)

**Assessment:**
- ✅ **Excellent:** RTF > 2.0 (much faster than real-time)
- ⚠️ **Good:** RTF 1.0 - 2.0 (near real-time)
- ❌ **Poor:** RTF < 1.0 (slower than real-time)

**Example:**
- 100 agents: **6.61x RTF** → 10s simulation runs in ~1.5s
- 1000 agents: **0.45x RTF** → 10s simulation takes ~22s

---

### Step Time

**Definition:** Average time to execute one simulation step

**Targets:**
- **Real-time visualization (60 FPS):** < 16.7ms per step
- **Real-time control (100 Hz):** < 10ms per step
- **Offline processing:** No strict requirement

**Typical Bottlenecks:**
1. **Collision Check (70-80%)** - Spatial hashing optimization applied
2. **Step Simulation (10-15%)** - PyBullet physics engine
3. **Agent Update (5-10%)** - Navigation and control logic

**Example (1000 agents):**
```
Component               Time (ms)    Percentage
────────────────────────────────────────────────
Collision Check          3.61ms       77.8%
Step Simulation          0.57ms       12.3%
Agent Update             0.34ms        7.3%
────────────────────────────────────────────────
Total                    4.64ms      100.0%
```

---

### Memory Usage

**Expected:** ~10KB per agent (linear scaling)

| Agents | Memory Delta |
|--------|--------------|
| 100    | -24.2 MB     |
| 500    | -16.0 MB     |
| 1000   | -5.6 MB      |
| 2000   | +14.3 MB     |

**Note:** Negative values indicate memory freed by Python garbage collection during test

---

## Use Case Recommendations

### 1. Real-Time Visualization (RTF > 1.0)

**Requirements:**
- Smooth 60 FPS rendering
- Interactive GUI
- Visual feedback

**Configuration:**
```python
params = SimulationParams(
    speed=1.0,
    gui=True,
    timestep=0.01,
    collision_check_frequency=10.0
)
```

**Agent Limit:** **< 200 agents**

**Optimization Tips:**
- Use `collision_check_2d=True` if applicable
- Reduce `collision_check_frequency` to 5-10 Hz
- Disable `enable_profiling` for production
- Consider reducing physics timestep if motion is jerky

---

### 2. Offline Batch Simulation

**Requirements:**
- Maximum throughput
- Data collection for analysis
- No real-time constraints

**Configuration:**
```python
params = SimulationParams(
    speed=0,                    # Maximum speed
    gui=False,                  # Headless
    enable_monitor_gui=False,   # No GUI overhead
    timestep=0.1,               # Larger timestep
    collision_check_frequency=10.0
)
```

**Performance:**
- 1000 agents: **0.45x RTF** (acceptable)
- 500 agents: **0.93x RTF** (near real-time)

**Optimization Tips:**
- Use `speed=0` for maximum performance
- Disable all visualization features
- Use `collision_check_2d=True` if ground robots
- Consider batching multiple simulations in parallel

---

### 3. Large-Scale Testing (>2000 agents)

**Requirements:**
- Comprehensive system testing
- Stress testing
- Scalability analysis

**Configuration:**
```python
params = SimulationParams(
    speed=0,
    gui=False,
    timestep=0.1,
    collision_check_frequency=1.0,  # Lower frequency
    collision_check_2d=True
)
```

**Performance:**
- 2000 agents: **0.24x RTF** (slow but acceptable)
- 5000+ agents: Consider distributed simulation

**Optimization Tips:**
- Accept <0.3x RTF for comprehensive testing
- Use profiling to identify specific bottlenecks
- Consider distributed simulation architecture
- Or reduce collision check frequency to 1 Hz

---

### 4. Development/Debugging

**Requirements:**
- Visual feedback
- Detailed logging
- Performance analysis

**Configuration:**
```python
params = SimulationParams(
    speed=1.0,
    gui=True,
    enable_profiling=True,
    enable_collision_color_change=True,
    collision_check_frequency=null  # Every step
)
```

**Optimization Tips:**
- Use profiling tools to identify bottlenecks
- Enable collision visualization for debugging
- Use smaller agent counts (<100) for faster iteration
- Switch to `speed=0` for stress testing

---

## Troubleshooting

### Low RTF / High Step Time

**Symptoms:**
- RTF < 1.0
- Step time > 20ms
- Slow simulation progress

**Diagnosis:**
```bash
# Enable profiling
python your_script.py --enable-profiling

# Or use profiling tools
python benchmark/profiling/simulation_profiler.py --agents 1000 --steps 100
```

**Common Issues:**

1. **Collision check > 80% of step time**
   ```yaml
   # Solution: Enable 2D collision
   collision_check_2d: true
   collision_check_frequency: 10.0
   ```

2. **GUI overhead**
   ```yaml
   # Solution: Disable GUI for batch processing
   gui: false
   enable_monitor_gui: false
   ```

3. **Too many agents**
   ```yaml
   # Solution: Reduce agent count or use distributed simulation
   # For real-time: < 200 agents
   # For offline: < 1000 agents recommended
   ```

4. **Profiling overhead**
   ```yaml
   # Solution: Disable profiling for production
   enable_profiling: false
   ```

---

### Memory Growth

**Symptoms:**
- Super-linear memory growth
- Memory usage >> 10KB per agent
- System slowdown over time

**Diagnosis:**
```bash
# Monitor memory usage
python benchmark/performance_benchmark.py --agents 2000 --duration 60
```

**Expected:**
- ~10KB per agent (linear scaling)
- Slight negative growth due to Python GC

**If super-linear:**
- Check for memory leaks in custom callbacks
- Verify objects are properly cleaned up
- Use `del` or weak references for temporary objects

---

### Inconsistent Results

**Symptoms:**
- Large variance in RTF/step time
- Performance fluctuates between runs
- Results don't match expectations

**Diagnosis:**
```bash
# Use repetitions for statistical confidence
python benchmark/run_benchmark.py --agents 1000 --repetitions 10
```

**Common Causes:**

1. **Background processes**
   - Close unnecessary applications
   - Check `htop` for CPU usage
   - Disable system updates during testing

2. **Thermal throttling**
   - Monitor CPU temperature
   - Use shorter test durations
   - Ensure adequate cooling

3. **Swap usage**
   - Check available RAM
   - Close memory-hungry applications
   - Consider reducing agent count

4. **PyBullet warmup**
   - First run may be slower
   - Use multiple repetitions and report median

---

## Advanced Optimization

### Performance Hierarchy (Fastest to Slowest)

1. **No collision detection** (`collision_check_frequency=0`)
   - ~6x speedup
   - Only for baseline testing

2. **2D collision at 1 Hz** (`collision_check_2d=true`, `frequency=1.0`)
   - ~3x speedup
   - Good for sparse environments

3. **2D collision at 10 Hz** (Recommended)
   - ~2x speedup
   - Balanced for production

4. **3D collision at 10 Hz**
   - Baseline
   - For 3D movement (drones, lifts)

5. **3D collision every step** (`frequency=null`)
   - Slowest
   - Maximum accuracy

---

### Future Optimizations (Not Yet Implemented)

**Short-term (potential improvements):**
- [ ] Pre-computed neighbor lists for static structures
- [ ] Adaptive collision frequency based on agent density
- [ ] Multi-threading for collision detection
- [ ] GPU-accelerated collision detection

**Long-term (architectural changes):**
- [ ] Custom collision engine (bypass PyBullet overhead)
- [ ] Distributed simulation for >10,000 agents
- [ ] Incremental collision updates (only check moving agents)

---

## Benchmark Results Reference

Based on latest optimization (2026-01-04):

| Agents | RTF    | Step Time | Assessment    |
|--------|--------|-----------|---------------|
| 100    | 6.61x  | 1.51ms   | ✅ Excellent   |
| 250    | 2.34x  | 4.27ms   | ⚠️ Good       |
| 500    | 0.93x  | 10.80ms  | ❌ Acceptable |
| 1000   | 0.45x  | 22.27ms  | ❌ Poor       |
| 2000   | 0.24x  | 41.96ms  | ❌ Poor       |

**Scalability:** Near-linear up to 500 agents, slightly super-linear (O(n^1.2)) above

**See [`PERFORMANCE_REPORT.md`](PERFORMANCE_REPORT.md) for detailed analysis.**

---

## Summary

### Quick Recommendations

| Use Case | Agents | Config | Expected RTF |
|----------|--------|--------|--------------|
| Real-time GUI | < 200 | `speed=1.0`, `gui=true`, `2d=true` | > 1.0x |
| Offline batch | < 1000 | `speed=0`, `gui=false`, `2d=true` | 0.5-1.0x |
| Large-scale test | < 2000 | `speed=0`, `gui=false`, `freq=1Hz` | 0.2-0.5x |
| Development | < 100 | `speed=1.0`, `gui=true`, `profiling=true` | > 2.0x |

### Key Takeaways

1. **Use 2D collision** for ground robots → ~67% speedup
2. **Reduce collision frequency** to 10 Hz → minimal impact on safety
3. **Disable GUI** for batch processing → significant speedup
4. **Use `speed=0`** for offline simulation → maximum throughput
5. **Limit agents** to < 200 for real-time, < 1000 for offline

---

## See Also

- [`README.md`](README.md) - Benchmark suite overview
- [`PERFORMANCE_REPORT.md`](PERFORMANCE_REPORT.md) - Detailed performance analysis
- [`profiling/README.md`](profiling/README.md) - Profiling tools documentation
- [`experiments/README.md`](experiments/README.md) - Optimization experiments

---

**Last Updated:** 2026-01-12
**PyBullet Fleet Version:** Latest (post-optimization)
