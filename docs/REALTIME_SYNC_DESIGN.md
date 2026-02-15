# Real-time Synchronization Design

## Overview

PyBulletFleet implements a real-time synchronization system to ensure smooth simulation playback at configurable speeds. This document explains the design, implementation, and best practices.

---

## Design Philosophy

### Absolute Time-based Control

The synchronization system uses **absolute time calculation** for stable, predictable behavior:

```python
elapsed_time = current_time - start_time
target_sim_time = elapsed_time * speed
time_diff = target_sim_time - current_sim_time
```

**Key Benefits:**
- **Stable control**: Fixed reference point (`start_time`) prevents oscillation
- **Simple logic**: No complex history management or filtering
- **Predictable**: Easy to reason about and debug
- **Efficient**: Minimal computational overhead

---

## Core Algorithm

### 1. Time Difference Calculation

```python
# Calculate how much simulation should have progressed
target_sim_time = elapsed_real_time * speed_multiplier

# Compare with actual simulation progress
time_diff = target_sim_time - current_sim_time
```

**Interpretation:**
- `time_diff > 0`: Simulation is **behind** → Execute multiple steps to catch up
- `time_diff < 0`: Simulation is **ahead** → Sleep until next frame
- `time_diff ≈ 0`: Simulation is **synchronized** → Continue at current pace

### 2. Adaptive Step Execution

When behind target:
```python
if time_diff > 0:
    steps_needed = int(time_diff / timestep)
    steps_needed = max(1, min(steps_needed, max_steps_per_frame))
    for _ in range(steps_needed):
        step_once()
```

**Saturation limit** (`max_steps_per_frame`):
- Default: 10 steps per frame
- Prevents sudden jumps from large `time_diff`
- Ensures GUI responsiveness

### 3. Sleep Strategy

When ahead of target:
```python
if time_diff < 0:
    sleep_time = abs(time_diff) - last_step_process_time

    if sleep_time > 0:
        time.sleep(sleep_time)  # Exact sleep
    elif sleep_time > -min_sleep:
        time.sleep(min_sleep)   # Minimum for GUI responsiveness
```

**GUI responsiveness:**
- `min_sleep = 1.0 / gui_min_fps` (default: 33ms for 30 FPS)
- Ensures smooth rendering even when simulation is ahead

---

## Pause/Resume Handling

### Reset `start_time` on Resume

```python
if last_pause_state and not self._simulation_paused:
    # Just resumed from pause
    start_time = current_time - current_sim_time / speed
    logger.info("Resumed from pause: start_time reset for smooth continuation")
```

**Timeline visualization:**
```
Real time:    100s -------- 105s [Pause 10s] 115s -------- 120s
Sim time:     0s ----------- 5s   (stopped)   5s ---------- 10s
start_time:   100s (initial) ──────────────→ 110s (reset) ────→

After reset:  target = (115 - 110) * 1.0 = 5s,  actual = 5s → smooth ✅
```

---

## Speed Multiplier Support

### Variable Speed Control

```python
speed = 0.5   # Half speed (slow motion)
speed = 1.0   # Real-time (1:1)
speed = 2.0   # Double speed (fast forward)
speed = 0     # Maximum speed (no sleep, as fast as possible)
```

**Example with speed=2.0:**
```python
# After 1 second of real time
elapsed_real = 1.0
target_sim_time = 1.0 * 2.0 = 2.0 seconds

# Simulation should have progressed 2 seconds
# If current_sim_time = 1.8, then time_diff = 0.2
# → Execute extra steps to catch up
```

---

## Best Practices

### 1. Spawn Objects Before `run_simulation()` ✅

**Recommended pattern:**
```python
# Setup phase (timing doesn't matter)
sim = MultiRobotSimulationCore.from_yaml("config.yaml")

# Spawn can take seconds - no impact on simulation time
for i in range(100):
    sim.spawn_large_structure()  # May take 10 seconds total

# Simulation starts HERE (start_time = time.time())
sim.run_simulation(duration=60.0)  # Smooth, no spawn delay impact
```

**Why it works:**
- `start_time` is initialized in `run_simulation()`
- Spawn time is completely excluded from synchronization
- No accumulated `time_diff` from spawn overhead

### 2. Use Pause/Resume for Dynamic Spawning of large/complex objects ✅

**Correct pattern:**
```python
# Method 1: Programmatic pause
sim._simulation_paused = True
#spawn objects here. Won't affect time_diff
sim._simulation_paused = False  # Auto-resets start_time
```
