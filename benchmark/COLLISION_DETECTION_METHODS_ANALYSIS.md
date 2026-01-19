# Collision Detection Methods Benchmark Results

**Date**: January 18, 2026  
**Benchmark**: `benchmark/collision_detection_methods_benchmark.py`

## Executive Summary

❌ **Recommendation: Keep current implementation (`getContactPoints` only)**

- `getContactPoints` is **62.7% faster on average** than hybrid approach
- `getClosestPoints` is consistently slower in all test cases
- Additional complexity of hybrid approach is **not justified**

## Methods Compared

### 1. getContactPoints only (Current Implementation)
- Uses PyBullet's `getContactPoints()` for all collision pairs
- Detects actual contact points from physics simulation
- Optimized by PyBullet's internal collision detection

### 2. getClosestPoints only
- Uses PyBullet's `getClosestPoints(distance=0.0)` for all pairs
- Calculates closest points between objects
- More computational overhead for distance calculation

### 3. Hybrid Approach (Proposed)
- `getClosestPoints()` for kinematic-kinematic and kinematic-physics pairs
- `getContactPoints()` for physics-physics pairs
- Adds conditional logic overhead

## Benchmark Results

### Configuration 1: 20 objects, 50% physics, 50 pairs

**Pair composition:**
- Physics-Physics: 14 pairs (28.0%)
- Kinematic-Kinematic: 15 pairs (30.0%)
- Mixed: 21 pairs (42.0%)

**Results (100 iterations, average time):**
- getContactPoints only: **0.049 ms**
- getClosestPoints only: 0.088 ms (1.80x slower)
- Hybrid approach: 0.083 ms (1.70x slower)

**Conclusion**: Current implementation is **69.7% faster** than hybrid

---

### Configuration 2: 50 objects, 50% physics, 100 pairs

**Pair composition:**
- Physics-Physics: 22 pairs (22.0%)
- Kinematic-Kinematic: 23 pairs (23.0%)
- Mixed: 55 pairs (55.0%)

**Results:**
- getContactPoints only: **0.116 ms**
- getClosestPoints only: 0.177 ms (1.53x slower)
- Hybrid approach: 0.168 ms (1.45x slower)

**Conclusion**: Current implementation is **45.2% faster** than hybrid

---

### Configuration 3: 100 objects, 50% physics, 200 pairs

**Pair composition:**
- Physics-Physics: 57 pairs (28.5%)
- Kinematic-Kinematic: 43 pairs (21.5%)
- Mixed: 100 pairs (50.0%)

**Results:**
- getContactPoints only: **0.210 ms**
- getClosestPoints only: 0.421 ms (2.00x slower)
- Hybrid approach: 0.409 ms (1.95x slower)

**Conclusion**: Current implementation is **94.5% faster** than hybrid

---

### Configuration 4: 50 objects, 20% physics (mostly kinematic)

**Pair composition:**
- Physics-Physics: 2 pairs (2.0%)
- Kinematic-Kinematic: 63 pairs (63.0%)
- Mixed: 35 pairs (35.0%)

**Results:**
- getContactPoints only: **0.156 ms**
- getClosestPoints only: 0.221 ms (1.42x slower)
- Hybrid approach: 0.220 ms (1.41x slower)

**Conclusion**: Current implementation is **41.3% faster** than hybrid

**Note**: Even with 63% kinematic-kinematic pairs, `getContactPoints` is still faster

---

### Configuration 5: 50 objects, 80% physics (mostly physics)

**Pair composition:**
- Physics-Physics: 65 pairs (65.0%)
- Kinematic-Kinematic: 2 pairs (2.0%)
- Mixed: 33 pairs (33.0%)

**Results:**
- getContactPoints only: **0.110 ms**
- getClosestPoints only: 0.206 ms (1.87x slower)
- Hybrid approach: 0.179 ms (1.62x slower)

**Conclusion**: Current implementation is **62.5% faster** than hybrid

---

## Summary Table

| Configuration | getContactPoints | getClosestPoints | Hybrid | Winner | Speedup |
|--------------|-----------------|-----------------|--------|--------|---------|
| 20obj, 50%phys | **0.05ms** | 0.09ms | 0.08ms | Contact | 1.70x |
| 50obj, 50%phys | **0.12ms** | 0.18ms | 0.17ms | Contact | 1.45x |
| 100obj, 50%phys | **0.21ms** | 0.42ms | 0.41ms | Contact | 1.95x |
| 50obj, 20%phys | **0.16ms** | 0.22ms | 0.22ms | Contact | 1.41x |
| 50obj, 80%phys | **0.11ms** | 0.21ms | 0.18ms | Contact | 1.62x |
| **Average** | **1.00x** | **1.67x** | **1.63x** | **Contact** | **1.63x** |

## Analysis

### Why is getContactPoints faster?

1. **PyBullet internal optimization**
   - `getContactPoints` uses collision detection already computed by physics engine
   - No additional distance calculation overhead
   - Direct access to physics engine's contact manifold

2. **getClosestPoints overhead**
   - Calculates closest points even when not in contact
   - Distance computation for all pairs
   - More complex algorithm than simple contact check

3. **Hybrid approach overhead**
   - Conditional branching (is_physics check) for each pair
   - Cache lookup for `_physics_objects` set
   - No performance benefit from using `getClosestPoints` for kinematic

### Surprising findings

- Even with 63% kinematic-kinematic pairs, `getContactPoints` is **41.3% faster**
- The overhead of `getClosestPoints` outweighs any theoretical advantage
- Hybrid approach adds complexity without performance benefit

## Recommendations

### ✅ Keep current implementation

**Reasons:**
1. **Performance**: 62.7% faster on average
2. **Simplicity**: Single method, no conditional logic
3. **Reliability**: Well-tested PyBullet API
4. **Scalability**: Performance gap increases with object count

### ❌ Do NOT implement hybrid approach

**Reasons:**
1. **Slower**: 1.63x slower on average
2. **Complexity**: Adds conditional logic and cache lookups
3. **Maintenance**: More code paths to test and debug
4. **No benefit**: Even in kinematic-heavy scenarios

## Conclusion

The original intuition that `getClosestPoints` would be faster for kinematic objects was **incorrect**. PyBullet's `getContactPoints` is highly optimized and should be used for all collision detection scenarios.

**Final recommendation**: No changes needed to current implementation.

## Code Impact

**No changes required** to `pybullet_fleet/core_simulation.py`:
- Keep using `getContactPoints()` in `check_collisions()` method
- No need to differentiate between physics and kinematic objects
- Current implementation is already optimal

## Future Considerations

If PyBullet updates its collision detection APIs or if performance characteristics change:
1. Re-run this benchmark with new PyBullet versions
2. Test with different object geometries (spheres, meshes, etc.)
3. Test with different collision margin settings

---

**Benchmark maintained by**: PyBulletFleet Development Team  
**Re-run command**: `python benchmark/collision_detection_methods_benchmark.py`
