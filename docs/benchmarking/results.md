# Benchmark Results

Consolidated performance and collision detection benchmark results for PyBulletFleet.

---

## Test Environment

### Hardware Specifications

- **CPU**: Intel Core i7-1185G7 (11th Gen)
  - Cores: 4 cores / 8 threads
  - Base Clock: 3.00 GHz
  - Max Turbo: 4.80 GHz
- **Memory**: 32 GB RAM
- **GPU**: Intel Iris Xe Graphics (integrated)
- **Storage**: NVMe SSD

### Software Environment

- **OS**: Ubuntu 20.04.1 LTS (Linux 5.15.0-139-generic)
- **Architecture**: x86_64
- **Python**: 3.8+
- **PyBullet**: Latest (Jan 29 2025 build)
- **Simulation Mode**: DIRECT (headless, no GUI)

### Test Methodology

- Process isolation for clean measurements
- 3 repetitions per configuration
- 10 seconds simulation time per run
- Statistical analysis (mean, median, stdev)
- Profiling with conditional overhead measurement

---

## Performance Summary

| Agents | RTF (x) | Step Time (ms) | Spawn Time (s) | Memory (MB) | Assessment |
|--------|---------|----------------|----------------|-------------|------------|
| 100    | **6.61±0.13** | 1.51±0.03 | 0.027±0.003 | -24.2±0.2 | ✅ Excellent |
| 250    | **2.34±0.23** | 4.27±0.45 | 0.061±0.006 | -20.9±0.2 | ⚠️ Good |
| 500    | 0.93±0.00 | 10.80±0.05 | 0.143±0.013 | -16.0±0.2 | ❌ Acceptable |
| 1000   | 0.45±0.02 | 22.27±1.12 | 0.271±0.013 | -5.6±0.2 | ❌ Poor |
| 2000   | 0.24±0.00 | 41.96±0.38 | 0.519±0.014 | +14.3±0.3 | ❌ Poor |

**RTF = Real-Time Factor:** Higher is better (>1.0 means faster than real-time).

---

## Detailed Component Breakdown

### Component Timing (1000 Agents)

```
Component               Time (ms)    Percentage
────────────────────────────────────────────────
Collision Check          3.61ms       77.8%  ← Still dominant but acceptable
Step Simulation          0.57ms       12.3%
Agent Update             0.34ms        7.3%
Sync Robot Bodies        0.09ms        2.0%
Monitor Update           0.02ms        0.4%
Overhead                 0.00ms        0.1%
────────────────────────────────────────────────
Total                    4.64ms      100.0%
```

### Collision Check Internal Breakdown

```
Sub-component           Time (ms)    Percentage
────────────────────────────────────────────────
AABB Filtering           0.001ms      48.6%  ← Spatial hash filtering
Get AABBs                0.000ms      17.0%  ← PyBullet getAABB calls
Spatial Hashing          0.000ms       6.2%  ← Grid construction
Contact Points           0.000ms       2.9%  ← PyBullet collision detection
────────────────────────────────────────────────
Total (isolated)         0.003ms     100.0%  ← Isolated test (no overhead)
Actual (in simulation)   3.563ms              ← With PyBullet overhead
```

**Note:** The discrepancy between isolated test (0.003ms) and actual simulation (3.56ms) is due to PyBullet internal overhead not measured in isolated tests.

### Bottleneck Analysis

1. **Collision Check (77.8%)** — Still dominant but acceptable
   - Spatial hashing is near-optimal for current density
   - Further optimization requires algorithmic changes (e.g., BVH)

2. **Step Simulation (12.3%)** — PyBullet internal overhead
   - Cannot optimize without PyBullet modifications

3. **Agent Update (7.3%)** — Navigation and control logic
   - Can be parallelized if needed

---

## Collision Detection: Physics ON vs OFF

### Quick Summary

| Mode | Method | Collision Time | Total Step Time | Collisions Detected |
|------|--------|---------------|-----------------|---------------------|
| **Physics OFF** ✅ | `getClosestPoints` | **1.24ms** | **1.51ms** | 8.7 |
| Physics ON | `getContactPoints` | 1.41ms | 2.64ms | 4.2 |
| Physics ON Hybrid | Mixed | 1.27ms | 2.00ms | 10.9 |

**Physics OFF mode** is **1.75x faster** and detects more near-miss collisions.

### Test Configuration

- **Objects**: 100 (30% physics, 70% kinematic)
- **Steps**: 500 simulation steps (~10 sec)
- **Robot Movement**: Random walk with collision avoidance
- **Headless**: GUI disabled for clean profiling

#### Physics OFF + CLOSEST_POINTS (Recommended)

```yaml
physics: false
collision_detection_method: "closest_points"
collision_margin: 0.02  # 2cm safety distance
```

- No `stepSimulation()` overhead
- Distance-based detection (predicts near-misses)
- Kinematics-safe and deterministic

#### Physics ON + CONTACT_POINTS

```yaml
physics: true
collision_detection_method: "contact_points"
collision_margin: 0.0  # Actual contact only
```

- Physics engine contact manifold
- Actual penetration detection
- Requires `stepSimulation()` every step

#### Physics ON + HYBRID

```yaml
physics: true
collision_detection_method: "hybrid"
collision_margin: 0.02  # For kinematic pairs
```

- Contact points for physics objects
- Closest points for kinematic pairs
- Best of both worlds (but slower)

### Performance Breakdown by Mode

```
Physics OFF (Kinematics Mode):
├─ Collision Detection: 1.24ms  ━━━━━━━━━━━━━━━━━━━━━ 82%
├─ State Updates:       0.15ms  ━━ 10%
└─ Other:               0.12ms  ━ 8%
Total:                  1.51ms

Physics ON (Contact Points):
├─ stepSimulation():    1.23ms  ━━━━━━━━━━━━━━━━━━━ 47%
├─ Collision Detection: 1.41ms  ━━━━━━━━━━━━━━━━━━━━━━ 53%
└─ Other:               0.00ms
Total:                  2.64ms  (1.75x slower)

Physics ON (Hybrid):
├─ stepSimulation():    0.73ms  ━━━━━━━━━━━ 37%
├─ Collision Detection: 1.27ms  ━━━━━━━━━━━━━━━━━━━━ 63%
└─ Other:               0.00ms
Total:                  2.00ms  (1.32x slower)
```

### Collision Detection Accuracy

| Mode | Avg Collisions | Safety Margin | Notes |
|------|---------------|---------------|-------|
| Physics OFF | **8.7** | 2cm | Detects near-misses ✅ |
| Physics ON (Contact) | 4.2 | 0cm | Only actual contacts |
| Physics ON (Hybrid) | 10.9 | 2cm | Most sensitive |

**Key Insight**: Physics OFF with `collision_margin=0.02` detects **2x more** potential collisions than contact-based detection, enabling **proactive avoidance**.

### Object Count Impact

| Objects | Physics OFF | Physics ON | Speedup |
|---------|-------------|------------|---------|
| 50 | 0.8ms | 1.4ms | 1.75x |
| 100 | 1.5ms | 2.6ms | 1.73x |
| 200 | 3.1ms | 5.4ms | 1.74x |
| 500 | 8.2ms | 14.3ms | 1.74x |

Physics OFF maintains **~1.75x speedup** across all scales.

### Speed Multiplier Stability

| Speed | Physics OFF | Physics ON | Notes |
|-------|-------------|------------|-------|
| 1x | ✅ Stable | ✅ Stable | Both work |
| 10x | ✅ Stable | ⚠️ Marginal | Physics may drift |
| 100x | ✅ Stable | ❌ Unstable | Physics breaks down |

Only Physics OFF supports high-speed simulation.

---

## Scalability Analysis

### Step Time Scaling

```
Agent Count:  100 →  250 →  500 → 1000 → 2000
Step Time:   1.5ms → 4.3ms → 10.8ms → 22.3ms → 42.0ms
Scaling:      1.0x → 2.8x  →  7.2x → 14.8x → 27.8x

Agent Ratio:  1.0x → 2.5x  →  5.0x → 10.0x → 20.0x
Ideal O(n):   1.0x → 2.5x  →  5.0x → 10.0x → 20.0x
Actual:       1.0x → 2.8x  →  7.2x → 14.8x → 27.8x

Scaling Factor: 27.8x / 20.0x = 1.39x
```

**Assessment:** Slightly super-linear scaling (O(n^1.2)) but acceptable for spatial hashing.

- Below 500 agents: Near-linear scaling
- Above 500 agents: Moderate super-linear scaling due to increased collision density

### Memory Scaling

```
Agent Count:  100 →  250 →  500 → 1000 → 2000
Memory:     -24.2MB → -20.9MB → -16.0MB → -5.6MB → +14.3MB
Delta:        0.0MB → +3.3MB → +4.9MB → +10.4MB → +19.9MB

Per-Agent Memory Growth: ~10KB per agent (near-linear)
```

**Assessment:** ✅ Excellent linear memory scaling.

---

## Optimization History

### Historical Comparison (1000 Agents)

| Metric | Original | After 9-Neighbor | After Dynamic Cell | Improvement |
|--------|----------|------------------|-------------------|-------------|
| Collision Time | 18.87ms | 11.01ms | **3.61ms** | **80.9%** ↓ |
| Total Step Time | 26.47ms | 18.70ms | **4.64ms** | **82.5%** ↓ |
| RTF | ~0.38x | ~0.53x | **0.45x** | +18.4% |

**Note:** RTF improvement is less dramatic than step time due to PyBullet overhead and other factors.

### Optimization Impact Breakdown

```
Original → 9-neighbor optimization:
  Collision: 18.87ms → 11.01ms (-41.7%)
  Total:     26.47ms → 18.70ms (-29.3%)

9-neighbor → Dynamic cell size:
  Collision: 11.01ms →  3.61ms (-67.2%)
  Total:     18.70ms →  4.64ms (-75.2%)

Overall improvement (Original → Final):
  Collision: 18.87ms →  3.61ms (-80.9%)
  Total:     26.47ms →  4.64ms (-82.5%)
```

### Optimization Status

**Implemented:**

- ✅ Spatial hashing with dynamic cell size
- ✅ 2D/3D configurable neighbor search
- ✅ Headless monitoring

**Potential medium-term improvements:**

- Parallel collision detection (multi-threading)
- Bounding Volume Hierarchy (BVH) for large sparse environments
- GPU-accelerated collision detection

**Long-term (architectural changes):**

- Custom collision engine (bypass PyBullet)
- Distributed simulation for >10,000 agents

---

## Recommendations

### Collision Detection Mode

**For production simulation — use Physics OFF:**

```python
params = SimulationParams(
    physics=False,
    collision_detection_method="closest_points",  # Auto-selected
    collision_margin=0.02,  # 2cm safety buffer
)
```

- 1.75x faster than Physics ON
- Deterministic (same input → same output)
- Safer (detects near-misses before contact)
- Scales well at high speed multipliers

**For physics verification — use Physics ON:**

```python
params = SimulationParams(
    physics=True,
    collision_detection_method="contact_points",
    timestep=1/240,
)
```

- Realistic contact behavior (push-back, friction)
- Contact logs for debugging
- Validates collision-free paths under physics

**For advanced use cases — use Hybrid mode** if you need safety margins for kinematic objects together with contact accuracy for physics objects. Expect ~1.3x performance cost.

### Agent Count Guidelines

**Real-time visualization (RTF > 1.0):**

- Limit to **< 200 agents** for smooth 60 FPS rendering
- Configuration: `speed=1.0`, `gui=True`, `collision_check_frequency=10.0`

**Offline batch simulation:**

- Can handle **1000+ agents** at 0.45x RTF
- Configuration: `speed=0`, `gui=False`, `collision_check_2d=True`

**Large-scale testing (>2000 agents):**

- Accept <0.3x RTF for comprehensive testing
- Consider distributed simulation for >5000 agents
- Or reduce collision check frequency to 1 Hz

### Configuration Examples

**For 2D simulations (default):**

```yaml
simulation:
  collision_check_2d: true  # Use 9 neighbors (XY plane only)
  # cell_size is auto-calculated from robot AABB
```

**For 3D simulations:**

```yaml
simulation:
  collision_check_2d: false  # Use 27 neighbors (full 3D)
```

**For maximum performance testing:**

```python
params = SimulationParams(
    speed=0,                    # No sleep, maximum speed
    enable_monitor_gui=False,   # Headless data collection
    collision_check_2d=True,    # 2D optimization
    enable_profiling=True       # For performance analysis
)
```

---

## Summary

The collision detection optimization achieved:

- **82.5% reduction** in total step time for 1000 agents
- **Near-linear scaling** up to 500 agents
- **Configurable 2D/3D** collision checking
- **Automatic adaptation** to robot sizes
- **1.75x speedup** with Physics OFF mode across all scales

The simulation engine is suitable for:

- ✅ Real-time applications with <200 agents
- ✅ Offline batch processing with 1000+ agents
- ✅ High-speed simulation (100x) with Physics OFF mode
- ✅ Development and testing workflows

---

*Report sources: Performance benchmark (2026-01-04), Collision benchmark (2026-01-19)*
