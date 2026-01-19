# Collision Detection Methods Analysis

## Overview

This document analyzes the performance of different collision detection methods in PyBullet:
1. **getContactPoints** (current implementation)
2. **getClosestPoints** (alternative)
3. **Hybrid approach** (getClosestPoints for kinematic + getContactPoints for physics)

## Benchmark Results

### Test Configuration

- **Environment**: 50 objects (30% physics, 70% kinematic)
- **Movement**: Physics simulation with random velocities
- **Scenarios**: 3 object sizes (small/medium/large) creating varying collision densities
- **Measurements**: 100 iterations per method, averaged

### Results Summary

#### Test 1: Small objects (0.3m half-extent)
- Active contacts: ~14 collisions
- **getContactPoints**: 1.392ms ± 0.195ms ✅ **FASTEST**
- **getClosestPoints**: 2.051ms ± 0.335ms (1.47x slower)
- **Hybrid**: 2.085ms ± 0.149ms (1.50x slower)

#### Test 2: Medium objects (0.5m half-extent)
- Active contacts: ~6 collisions
- **getContactPoints**: 1.244ms ± 0.126ms ✅ **FASTEST**
- **getClosestPoints**: 1.876ms ± 0.198ms (1.51x slower)
- **Hybrid**: 1.787ms ± 0.168ms (1.44x slower)

#### Test 3: Large objects (0.8m half-extent)
- Active contacts: ~7 collisions
- **getContactPoints**: 1.296ms ± 0.108ms ✅ **FASTEST**
- **getClosestPoints**: 1.832ms ± 0.148ms (1.41x slower)
- **Hybrid**: 1.768ms ± 0.157ms (1.36x slower)

### Performance Comparison

| Method | Avg Time | vs getContactPoints | Complexity |
|--------|----------|-------------------|------------|
| **getContactPoints** | **1.31ms** | Baseline (1.00x) | Simple ✅ |
| getClosestPoints | 1.92ms | 1.46x slower ❌ | Simple |
| Hybrid | 1.88ms | 1.43x slower ❌ | Complex ❌ |

## Key Findings

### 1. getContactPoints is consistently faster

- **30-50% faster** than alternatives across all test scenarios
- Performance advantage holds regardless of:
  - Object size (0.3m - 0.8m)
  - Collision density (6-14 active contacts)
  - Object movement (physics simulation)

### 2. getClosestPoints is slower than expected

Initial hypothesis:
> "getClosestPoints should be faster for kinematic objects since it doesn't need physics contact resolution"

**Reality**: getClosestPoints performs **distance calculations** for all pairs, which adds overhead:
- Distance threshold check (even with distance=0.0)
- Closest point computation on mesh surfaces
- More expensive than simple contact point query

### 3. Hybrid approach adds complexity without benefit

The hybrid method:
```python
if is_physics_i and is_physics_j:
    result = p.getContactPoints(body_i, body_j)
else:
    result = p.getClosestPoints(body_i, body_j, distance=0.0)
```

**Problems**:
- Adds branching logic (O(N) overhead)
- getClosestPoints still slower even for kinematic pairs
- Code complexity increases
- Maintenance burden increases
- **No performance gain** (1.43x slower than current implementation)

### 4. Current implementation is already optimal

The current PyBulletFleet implementation uses:
- AABB filtering with spatial hashing (reduces candidate pairs)
- getContactPoints for final collision detection
- Incremental collision tracking (only check moved objects)

**This is the optimal approach** based on:
- ✅ Simplicity (single collision detection method)
- ✅ Performance (1.3-1.5x faster than alternatives)
- ✅ Maintainability (less code, fewer branches)
- ✅ Correctness (physics engine's native contact detection)

## Recommendations

### ✅ Keep current implementation

**Continue using `getContactPoints` for all collision detection**

Reasons:
1. **Fastest method** (1.3-1.5x faster than alternatives)
2. **Simplest code** (no branching logic)
3. **Most reliable** (PyBullet's native contact detection)
4. **Best maintainability** (single code path)

### ❌ Do NOT switch to alternatives

**Avoid getClosestPoints or Hybrid approach**

Reasons:
1. Slower performance (30-50% overhead)
2. Added complexity without benefit
3. Potential for bugs (distance threshold tuning, edge cases)

## Technical Details

### Why is getContactPoints faster?

1. **Native physics integration**
   - PyBullet's collision detection is optimized for getContactPoints
   - Direct access to internal contact manifolds
   - No additional distance calculations

2. **Early termination**
   - Returns immediately when contact detected
   - getClosestPoints computes distance even when not needed

3. **Optimized for contact queries**
   - Physics engine's narrow-phase collision is highly optimized
   - getContactPoints is the primary use case for PyBullet

### When might getClosestPoints be useful?

**Only for specific use cases** (not general collision detection):
- Proximity sensors (need distance < threshold, not just contact)
- Path planning (obstacle avoidance with safety margin)
- Grasp planning (distance to surface for manipulation)

**NOT for collision detection** in multi-robot simulation.

## Conclusion

The benchmark results **definitively show** that:

1. ✅ **Current implementation is optimal** (getContactPoints only)
2. ❌ **getClosestPoints is slower** (1.46x overhead)
3. ❌ **Hybrid approach is slower** (1.43x overhead + complexity)

**Recommendation**: **No changes needed** - keep using getContactPoints for all collision detection.

## Benchmark Scripts

- `benchmark/collision_detection_methods_benchmark.py` - Static objects benchmark
- `benchmark/collision_methods_with_movement.py` - Moving objects benchmark (realistic scenario)

To reproduce:
```bash
python benchmark/collision_methods_with_movement.py
```

---

**Last updated**: January 19, 2026
**Benchmark environment**: PyBullet 3.25, Python 3.8+
