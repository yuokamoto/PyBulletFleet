# Benchmark Experiments

This directory contains experimental scripts for performance analysis, optimization validation, and hypothesis testing.

## Purpose

Unlike profiling tools (which measure what is happening), experiments validate optimization hypotheses and compare different implementation approaches.

## Available Experiments

### `performance_analysis.py`

**Purpose:** Compare performance of different wrapper layers

**Features:**
- Tests direct PyBullet vs SimObject vs Manager APIs
- Process isolation for clean measurements
- CPU time and memory tracking
- Statistical analysis with median, mean, stdev

**Usage:**
```bash
# Run all performance tests
python benchmark/experiments/performance_analysis.py

# Test specific component
python benchmark/experiments/performance_analysis.py --test simobject
```

**Tests:**
- `test_simobject_wrapper()` - SimObject spawn/update performance
- `test_simobject_manager()` - SimObjectManager bulk operations
- `test_agent_wrapper()` - Agent spawn/update performance
- `test_agent_manager()` - AgentManager bulk operations

**Expected Output:**
```
======================================================================
Performance Analysis: SimObject Wrapper
======================================================================
Creating 1000 objects...
  Spawn Time: 1.23s
  Update Time: 0.45s (450μs per object)
  Memory Delta: +12.5MB
======================================================================
```

---

### `collision_optimization.py`

**Purpose:** Test and compare collision detection optimization approaches

**Features:**
- Brute-force O(n²) vs spatial hashing comparison
- Grid cell size optimization
- 2D vs 3D collision checking overhead (9 vs 27 neighbors)
- AABB filtering efficiency
- PyBullet API overhead measurement

**Usage:**
```bash
# Run all collision optimization tests
python benchmark/experiments/collision_optimization.py

# Test with specific agent count
python benchmark/experiments/collision_optimization.py --agents 1000

# Test specific method
python benchmark/experiments/collision_optimization.py --method spatial_hash
```

**Tests:**
- Brute-force pairwise collision detection
- Spatial hashing with different cell sizes
- 2D (9 neighbors) vs 3D (27 neighbors) comparison
- AABB filtering vs contact point detection
- PyBullet getAABB() and getContactPoints() overhead

**Expected Output:**
```
======================================================================
Collision Optimization Comparison (500 agents)
======================================================================
Method                    Time (ms)    Speedup    Collisions
----------------------  -----------  ---------  ------------
Brute Force O(n²)          125.3ms       1.00x          42
Spatial Hash (1.0m)         15.2ms       8.24x          42
Spatial Hash (2.0m)         12.8ms       9.79x          42
2D (9 neighbors)             8.3ms      15.10x          42
3D (27 neighbors)           12.8ms       9.79x          42
======================================================================
```

**Status:** ⚠️ Template created - implementation in progress

---

### `list_filtering_benchmark.py`

**Purpose:** Benchmark Python list filtering approaches (micro-optimization)

**Features:**
- List comprehension vs for loop
- Set membership check performance
- Set difference operations

**Usage:**
```bash
python benchmark/experiments/list_filtering_benchmark.py
```

**Tests:**
- Current list comprehension approach
- Set difference optimization
- Single-pass for loop iteration
- Pre-computed set optimization

**Expected Output:**
```
======================================================================
List Filtering Benchmark
======================================================================
Method                    Time (ms)    Speedup
----------------------  -----------  ---------
Current (list comp)        45.2ms       1.00x
Set difference             38.7ms       1.17x
Single-pass iteration      35.3ms       1.28x
Pre-computed set           32.1ms       1.41x
======================================================================
```

**Note:** This is a micro-benchmark for the list filtering pattern used in collision detection,
but does NOT test actual collision algorithms (spatial hashing, AABB, etc.).

---

### `getaabb_performance.py`

**Purpose:** Test if `p.getAABB()` is a performance bottleneck

**Features:**
- Isolated testing of PyBullet AABB retrieval
- Measure per-call and batch performance
- Compare with alternative approaches

**Usage:**
```bash
python benchmark/experiments/getaabb_performance.py
```

**Expected Output:**
```
======================================================================
getAABB Performance Test
======================================================================
Number of objects: 1000
Per-call time: 2.3μs
Batch time: 2.3ms (2.3μs per object)
Conclusion: getAABB is NOT a bottleneck
======================================================================
```

---

## See Also

- `../profiling/README.md` - Profiling tools documentation
- `../PERFORMANCE_REPORT.md` - Performance analysis results
- `../README.md` - Benchmark suite overview

---

**Last Updated:** 2026-01-12
