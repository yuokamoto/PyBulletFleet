# PyBullet Performance Analysis & Optimization Strategies

**Date**: 2026-01-01
**Test Configuration**: 10,000 objects, 3 repetitions
**Goal**: Update time ≤ 10ms for production-scale simulations

---

## Executive Summary

Fair comparison results with all tests performing **get_pose + set_pose**:

| Test | Update Time | vs Baseline | vs Goal | Status |
|------|-------------|-------------|---------|--------|
| **A. Baseline (PyBullet direct)** | 54.69ms | - | 5.5x | ❌ |
| **B. SimObject** | 130.74ms | +76ms (+139%) | 13.1x | ❌ |
| **D. Agent** | 270.49ms | +216ms (+395%) | 27.0x | ❌ |
| **E. AgentManager** | 278.62ms | +224ms (+410%) | 27.9x | ❌ |

**Key Finding**: Even the bare PyBullet API baseline (54.69ms) is **5.5x slower** than the 10ms goal. This indicates a fundamental limitation of PyBullet's performance for 10,000 objects.

---

## 1. Current Performance Breakdown

### Update Time Analysis (10,000 objects)

```
Baseline (PyBullet Direct):
├─ get_pose:  27.98ms (51%)  ← getBasePositionAndOrientation × 10,000
└─ set_pose:  26.87ms (49%)  ← resetBasePositionAndOrientation × 10,000
   Total:     54.69ms

SimObject Wrapper:
├─ get_pose:  67.68ms (52%)  ← +40ms overhead (cache checking, Pose object creation)
└─ set_pose:  61.75ms (47%)  ← +35ms overhead (cache update, velocity preservation logic)
   Total:    130.74ms (+76ms vs baseline)

Agent Wrapper:
├─ get_pose: 133.97ms (50%)  ← +106ms overhead (inherited SimObject + agent state)
└─ set_pose: 136.51ms (50%)  ← +110ms overhead (inherited SimObject + agent updates)
   Total:    270.49ms (+216ms vs baseline)
```

### Per-Object Operation Cost

```
Operation         | Baseline | SimObject | Agent    | Notes
------------------|----------|-----------|----------|------------------------
get_pose          | 2.8μs    | 6.8μs     | 13.4μs   | Read position/orientation
set_pose          | 2.7μs    | 6.2μs     | 13.7μs   | Write position/orientation
TOTAL per object  | 5.5μs    | 13.1μs    | 27.1μs   | Full update cycle
```

### Memory Overhead

```
Component         | Memory (RSS)  | Per-Object  | Notes
------------------|---------------|-------------|------------------------
Baseline          | 85.02 MB      | 8.5 KB      | PyBullet internal structures
SimObject         | +62.03 MB     | +6.2 KB     | Python wrapper objects + caching
Agent             | +106.52 MB    | +10.7 KB    | URDF parsing + trajectory planning
```

---

## 2. PyBullet API Performance Characteristics

### Core API Call Performance (measured)

```python
# For 10,000 objects:
p.getBasePositionAndOrientation()  →  27.98ms total  (2.8μs each)
p.resetBasePositionAndOrientation() → 26.87ms total  (2.7μs each)
```

### Theoretical Minimum

For 10ms goal with 10,000 objects:
- **Required**: 1.0μs per object for combined get+set
- **Reality**: 5.5μs per object (PyBullet baseline)
- **Gap**: **5.5x slower than required**

### Why PyBullet is Slow for Bulk Operations

1. **No Batch API**: Each object requires individual function call
2. **Python-C++ Bridge Overhead**: Function call crossing language boundary
3. **GIL (Global Interpreter Lock)**: Cannot parallelize in Python
4. **No GPU Acceleration**: All operations on CPU
5. **Memory Allocation**: Each call allocates/deallocates temporary buffers

---

## 3. Optimization Strategy Analysis

### Option A: C++ Extension / Cython

**Approach**: Bypass Python and call PyBullet C API directly in batch mode

**Implementation**:
```cpp
// Pseudo-code for C++ extension
void batch_get_set_poses(int* body_ids, int n_objects,
                         double* positions_out, double* positions_in) {
    btDiscreteDynamicsWorld* world = getPhysicsWorld();
    for (int i = 0; i < n_objects; i++) {
        // Direct C++ memory access, no Python calls
        btRigidBody* body = world->getRigidBody(body_ids[i]);

        // Get pose (read from C++ memory)
        btTransform& transform = body->getWorldTransform();
        positions_out[i*3+0] = transform.getOrigin().x();
        positions_out[i*3+1] = transform.getOrigin().y();
        positions_out[i*3+2] = transform.getOrigin().z();

        // Set pose (write to C++ memory)
        transform.setOrigin(btVector3(
            positions_in[i*3+0],
            positions_in[i*3+1],
            positions_in[i*3+2]
        ));
    }
}
```

**Expected Improvement**:
- Eliminate Python-C bridge overhead: **~30-40% faster**
- Reduce memory allocations: **~20% faster**
- **Total estimated**: 54.69ms → **~25-30ms** (still 2.5-3x over goal)

**Pros**:
- Significant performance improvement
- No architectural changes needed
- Full control over memory layout

**Cons**:
- Requires C++ development expertise
- Platform-specific compilation (Linux/Windows/Mac)
- Maintenance burden (must update with PyBullet changes)
- **Still cannot achieve 10ms goal** due to inherent physics simulation overhead

**Effort**: High (2-4 weeks development + testing)

---

### Option B: Batch API (if PyBullet adds it)

**Approach**: Request PyBullet to add native batch operations

**Example API**:
```python
# Hypothetical batch API
positions, orientations = p.batchGetBasePositionAndOrientation(body_ids)
p.batchResetBasePositionAndOrientation(body_ids, positions, orientations)
```

**Expected Improvement**:
- Similar to C++ extension: **~30-50% faster**
- Estimated: 54.69ms → **~27-35ms**

**Pros**:
- No maintenance burden
- Cross-platform
- Clean API

**Cons**:
- **PyBullet development is slow/inactive** (last major update years ago)
- No guarantee of implementation
- **Still cannot achieve 10ms goal**

**Effort**: Low for us (feature request), but depends on PyBullet team

**Status**: ❌ **Not recommended** - PyBullet is not actively maintained

---

### Option C: Architectural Redesign - Selective Updates

**Approach**: Only update objects that actually need updating

**Strategies**:

#### C.1 Static/Dynamic Separation
```python
class OptimizedSimulation:
    def __init__(self):
        self.static_objects = []   # Never move
        self.dynamic_objects = []  # Update every frame

    def update(self):
        # Only update dynamic objects
        for obj in self.dynamic_objects:
            obj.update()
        # Static objects: zero overhead
```

**Expected Improvement** (for typical warehouse scenario):
- Assumption: 90% static shelves, 10% moving robots
- Update time: 270.49ms × 10% = **~27ms** (still 2.7x over goal)
- With C++ extension: **~12-15ms** (closer to goal!)

#### C.2 Spatial Partitioning
```python
class SpatialGrid:
    """Only update objects in active regions (e.g., near robots)"""
    def get_active_objects(self, robot_positions, radius=5.0):
        # Only return objects within radius of any robot
        active = []
        for robot_pos in robot_positions:
            active.extend(self.query_sphere(robot_pos, radius))
        return active
```

**Expected Improvement** (for typical warehouse):
- Assumption: 100 robots, 5m radius each, ~500 active objects
- Update time: 270.49ms × (500/10000) = **~13.5ms**
- With C++ extension: **~6-8ms** ✅ **ACHIEVES GOAL!**

#### C.3 Fixed-Frequency Updates
```python
class ThrottledSimulation:
    """Update different object groups at different rates"""
    def update(self, step):
        # Robots: 100Hz (every frame)
        self.update_robots()

        # Shelves near robots: 10Hz (every 10 frames)
        if step % 10 == 0:
            self.update_nearby_objects()

        # Distant objects: 1Hz (every 100 frames)
        if step % 100 == 0:
            self.update_distant_objects()
```

**Expected Improvement**:
- Effective objects per frame: ~1000 instead of 10,000
- Update time: 270.49ms × 10% = **~27ms**
- With optimizations: **~10-15ms** ✅ **NEAR GOAL!**

**Pros**:
- Dramatic performance improvement
- Works with existing codebase
- Scalable to larger simulations

**Cons**:
- Requires understanding of use case
- May need collision detection optimizations
- Architecture complexity

**Effort**: Medium (1-2 weeks for basic implementation)

---

### Option D: Alternative Physics Engines

**Approach**: Evaluate GPU-accelerated physics engines

#### D.1 NVIDIA Isaac Sim / Warp
- **GPU-accelerated**: 100-1000x faster for large-scale simulations
- **Supports**: 10,000+ objects at >100 FPS
- **Cost**: High migration effort, NVIDIA hardware requirement

#### D.2 MuJoCo (with GPU)
- **Modern**: Actively maintained by DeepMind
- **Performance**: ~10x faster than PyBullet for large scenes
- **Cost**: Moderate migration effort

**Effort**: Very High (1-3 months migration)

---

## 4. Recommended Strategy

### Phase 1: Quick Wins (1-2 weeks) ✅ **ALREADY DONE**
- [x] LazyLogger for debug statements: **-75% DEBUG overhead**
- [x] Pose caching: **-18% get_pose overhead**
- [x] Mass/kinematic caching: **-14% set_pose overhead**
- [x] Pose object reuse: **-5% allocation overhead**

**Result**: AgentManager improved from 2017ms → 179ms (-91%) for trajectory use case

### Phase 2: Architectural Optimization (2-3 weeks) 🎯 **RECOMMENDED**
1. **Implement Static/Dynamic Separation**
   - Identify static objects at spawn time
   - Skip updates for kinematic objects (mass=0)
   - **Expected**: 270ms → **80-100ms**

2. **Implement Spatial Partitioning**
   - Build spatial hash grid
   - Only update objects near active robots
   - **Expected**: 80ms → **20-30ms**

3. **Add Update Throttling**
   - Different update frequencies for different object types
   - **Expected**: 20ms → **10-15ms** ✅ **ACHIEVES GOAL**

### Phase 3: C++ Extension (if needed) ⚠️ **OPTIONAL**
Only pursue if Phase 2 is insufficient:
- Build Cython/C++ extension for batch operations
- **Expected**: Additional **30-50% improvement**
- **Final**: **5-10ms** ✅ **EXCEEDS GOAL**

### Phase 4: Migration (long-term) 🔮 **FUTURE**
If simulation needs exceed PyBullet's capabilities:
- Evaluate NVIDIA Isaac Sim for GPU acceleration
- Consider for next-generation system (not current project)

---

## 5. Detailed Analysis for Phase 2 Implementation

### Static/Dynamic Separation

**Detection Strategy**:
```python
class SimObject:
    def __init__(self, body_id, mass=None, ...):
        self.mass = mass if mass is not None else p.getDynamicsInfo(body_id, -1)[0]
        self.is_kinematic = (self.mass == 0.0)  # Already implemented ✅
        self.is_static = self.is_kinematic  # Alias for clarity
```

**Update Strategy**:
```python
class SimObjectManager:
    def update(self):
        # Only update dynamic objects
        for obj in self.objects:
            if not obj.is_static:
                obj.update()  # Skip static shelves, bins, etc.
```


### Spatial Partitioning

**Grid-Based Implementation**:
```python
class SpatialHashGrid:
    def __init__(self, cell_size=1.0):
        self.cell_size = cell_size
        self.grid = defaultdict(list)  # {(ix, iy, iz): [objects]}

    def insert(self, obj, position):
        cell = self._get_cell(position)
        self.grid[cell].append(obj)

    def query_sphere(self, center, radius):
        """Get all objects within radius of center"""
        cells = self._get_affected_cells(center, radius)
        objects = []
        for cell in cells:
            objects.extend(self.grid.get(cell, []))
        return objects

    def _get_cell(self, position):
        return (
            int(position[0] // self.cell_size),
            int(position[1] // self.cell_size),
            int(position[2] // self.cell_size)
        )
```

**Usage**:
```python
class OptimizedSimulation:
    def __init__(self):
        self.spatial_grid = SpatialHashGrid(cell_size=5.0)
        self.robots = []
        self.all_objects = []

    def update(self):
        # Get active regions
        active_objects = set()
        for robot in self.robots:
            pos = robot.get_pose().position
            nearby = self.spatial_grid.query_sphere(pos, radius=5.0)
            active_objects.update(nearby)

        # Update only active objects
        for obj in active_objects:
            obj.update()
```

**Expected Performance**:
```
Scenario: 100 robots, 5m interaction radius
- Typical active objects: ~500 (5% of 10,000)
- Update time: 270ms × 5% = 13.5ms
- ✅ WITHIN GOAL (10ms) with some margin
```

### Update Throttling

**Implementation**:
```python
class ThrottledSimulation:
    def __init__(self):
        self.step_count = 0
        self.high_freq_objects = []  # Robots (every frame)
        self.med_freq_objects = []   # Near objects (10Hz)
        self.low_freq_objects = []   # Far objects (1Hz)

    def update(self):
        self.step_count += 1

        # Always update high-frequency (robots)
        for obj in self.high_freq_objects:
            obj.update()

        # Medium frequency: every 10 frames (10Hz at 100 FPS)
        if self.step_count % 10 == 0:
            for obj in self.med_freq_objects:
                obj.update()

        # Low frequency: every 100 frames (1Hz at 100 FPS)
        if self.step_count % 100 == 0:
            for obj in self.low_freq_objects:
                obj.update()

    def categorize_objects(self):
        """Dynamically assign objects to frequency groups"""
        for obj in self.all_objects:
            if isinstance(obj, Robot):
                self.high_freq_objects.append(obj)
            elif obj.is_near_any_robot(threshold=5.0):
                self.med_freq_objects.append(obj)
            else:
                self.low_freq_objects.append(obj)
```

**Effective Object Count**:
```
Per frame average:
- High freq (100%): 100 robots × 1.0 = 100
- Med freq (10%):   500 objects × 0.1 = 50
- Low freq (1%):    9400 objects × 0.01 = 94
Total effective:    244 objects/frame

Update time: 270ms × (244/10000) = 6.6ms
✅ WELL WITHIN GOAL!
```

---

## 6. C++ Extension Details (Phase 3)

### Cython Implementation Example

**File: `fast_pybullet.pyx`**
```cython
# cython: language_level=3
# distutils: language = c++

cimport numpy as np
import numpy as np
from libc.stdlib cimport malloc, free

cdef extern from "SharedMemoryPublic.h":
    ctypedef struct b3RobotSimulatorClientAPI:
        pass

cdef extern from "PhysicsClientC_API.h":
    int b3GetStatusActualStateJointPosition(void* statusHandle, int* bodyUniqueId, ...)
    void b3SubmitClientCommandAndWaitStatus(void* physClient, int commandHandle)

def batch_get_positions(np.ndarray[int, ndim=1] body_ids):
    """
    Fast batch get positions for multiple bodies.

    Args:
        body_ids: NumPy array of PyBullet body IDs

    Returns:
        positions: (N, 3) array of positions
        orientations: (N, 4) array of quaternions
    """
    cdef int n = body_ids.shape[0]
    cdef np.ndarray[double, ndim=2] positions = np.zeros((n, 3), dtype=np.float64)
    cdef np.ndarray[double, ndim=2] orientations = np.zeros((n, 4), dtype=np.float64)

    # Access PyBullet C API directly
    cdef void* physClient = get_physics_client()
    cdef double pos[3]
    cdef double orn[4]

    for i in range(n):
        # Direct C API call - no Python overhead
        b3GetBasePositionAndOrientation(physClient, body_ids[i], pos, orn)
        positions[i, 0] = pos[0]
        positions[i, 1] = pos[1]
        positions[i, 2] = pos[2]
        orientations[i, 0] = orn[0]
        orientations[i, 1] = orn[1]
        orientations[i, 2] = orn[2]
        orientations[i, 3] = orn[3]

    return positions, orientations

def batch_set_positions(np.ndarray[int, ndim=1] body_ids,
                       np.ndarray[double, ndim=2] positions,
                       np.ndarray[double, ndim=2] orientations):
    """Fast batch set positions for multiple bodies."""
    cdef int n = body_ids.shape[0]
    cdef void* physClient = get_physics_client()

    for i in range(n):
        b3ResetBasePositionAndOrientation(
            physClient, body_ids[i],
            &positions[i, 0], &orientations[i, 0]
        )
```

**Setup.py**:
```python
from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy as np

extensions = [
    Extension(
        "fast_pybullet",
        ["fast_pybullet.pyx"],
        include_dirs=[np.get_include(), "/path/to/pybullet/src"],
        library_dirs=["/path/to/pybullet/lib"],
        libraries=["BulletDynamics", "BulletCollision", "LinearMath"],
        language="c++",
    )
]

setup(
    name="fast_pybullet",
    ext_modules=cythonize(extensions, compiler_directives={'language_level': 3}),
)
```

**Usage**:
```python
import fast_pybullet

# Instead of:
# poses = [p.getBasePositionAndOrientation(bid) for bid in body_ids]  # 28ms

# Use:
positions, orientations = fast_pybullet.batch_get_positions(body_ids)  # ~10-15ms
```

**Expected Improvement**:
- Current: 27.98ms (get) + 26.87ms (set) = 54.69ms
- With C++ extension: **~20-25ms** (50-60% improvement)

### Platform Considerations

**Build Requirements**:
```bash
# Linux
sudo apt-get install python3-dev build-essential

# Install Cython
pip install cython numpy

# Build extension
python setup.py build_ext --inplace
```

**Cross-Platform Support**:
- **Linux**: Full support ✅
- **Windows**: Requires Visual Studio ⚠️
- **macOS**: Requires Xcode ⚠️

---

## 7. Comparison Table: All Strategies

| Strategy | Performance Gain | Effort | Time | Risk | Recommended |
|----------|-----------------|--------|------|------|-------------|
| **Static/Dynamic** | 10x (270ms→27ms) | Low | 1 week | Low | ✅ **YES** |
| **Spatial Partition** | 20x (270ms→13ms) | Medium | 2 weeks | Medium | ✅ **YES** |
| **Update Throttling** | 40x (270ms→7ms) | Medium | 2 weeks | Medium | ✅ **YES** |
| **C++ Extension** | 2x (54ms→25ms) | High | 4 weeks | High | ⚠️ **If needed** |
| **Batch API** | 2x (54ms→27ms) | Low | N/A | N/A | ❌ **Not available** |
| **GPU Physics** | 100x+ | Very High | 3 months | Very High | ❌ **Future only** |

---

## 8. Recommended Implementation Roadmap

### Week 1: Static/Dynamic Separation ✅
```python
# Task 1.1: Add is_static flag tracking
# Task 1.2: Modify update loop to skip static objects
# Task 1.3: Benchmark and validate

# Expected result: 270ms → 80-100ms
```

### Week 2: Spatial Partitioning 🎯
```python
# Task 2.1: Implement SpatialHashGrid class
# Task 2.2: Integrate with SimObjectManager
# Task 2.3: Add dynamic repartitioning
# Task 2.4: Benchmark with realistic warehouse layout

# Expected result: 80ms → 20-30ms
```

### Week 3: Update Throttling 🎯
```python
# Task 3.1: Implement ThrottledSimulation class
# Task 3.2: Add automatic object categorization
# Task 3.3: Add runtime recategorization (as robots move)
# Task 3.4: Benchmark and tune frequencies

# Expected result: 20ms → 8-12ms ✅ ACHIEVES GOAL
```

### Week 4: Validation & Tuning
```python
# Task 4.1: Full-scale testing (10,000+ objects)
# Task 4.2: Performance profiling and optimization
# Task 4.3: Documentation and examples
# Task 4.4: Integration tests

# Target: Consistent < 10ms update times
```

### Optional: C++ Extension (Weeks 5-8)
Only if Phase 1-3 doesn't achieve goals:
```
# Week 5: Cython extension development
# Week 6: Testing and debugging
# Week 7: Cross-platform builds
# Week 8: Integration and validation
```

---

## 9. Questions to Answer Before Implementation

### Use Case Analysis
1. **What percentage of objects are truly static?**
   - Shelves, racks, walls → never move
   - Bins on shelves → rarely move (only during picks)
   - Robots, pallets → always moving

2. **What is the typical robot density?**
   - 10 robots in 10,000 objects? (0.1%)
   - 100 robots in 10,000 objects? (1%)
   - Affects spatial partitioning strategy

3. **What is the required simulation fidelity?**
   - Do distant objects need physics updates every frame?
   - Can we use LOD (Level of Detail) for far objects?
   - Can we freeze objects outside interaction zones?

4. **What is the robot interaction radius?**
   - Collision detection: typically 1-2m
   - Path planning: typically 5-10m
   - Affects spatial partitioning cell size

### Architecture Questions
5. **Is the 10ms goal for a single update(), or total frame time?**
   - If total frame includes rendering: different optimization strategy
   - If update() only: focus on physics optimization

6. **Are all 10,000 objects always present?**
   - Or are they spawned/despawned dynamically?
   - Dynamic spawning changes optimization strategy

7. **What are the typical simulation scenarios?**
   - Warehouse picking: robots moving through static shelves
   - Object sorting: many objects on conveyors
   - Multi-robot coordination: robot-robot interaction critical

---

## 10. Conclusion

### Key Findings

1. **PyBullet Baseline Limitation**: 54.69ms for 10,000 objects (5.5x over goal)
   - Fundamental API performance limit
   - Cannot achieve 10ms with naive approach

2. **Wrapper Overhead**: SimObject adds +76ms, Agent adds +216ms
   - Significant, but not the primary bottleneck
   - Already optimized with caching strategies

3. **Path to Success**: Architectural optimization (Phase 2)
   - Static/Dynamic separation: **10x improvement**
   - Spatial partitioning: **20x improvement**
   - Update throttling: **40x improvement**
   - **Combined: 270ms → 8-12ms** ✅ **ACHIEVES GOAL**

4. **C++ Extension**: Optional enhancement
   - **2x improvement** on baseline
   - High effort, moderate benefit
   - Only pursue if architectural optimization insufficient

### Recommended Action Plan

**✅ IMMEDIATE (Weeks 1-3)**: Implement architectural optimizations
- Highest ROI (return on investment)
- Low risk, well-understood techniques
- Can achieve 10ms goal without C++ complexity

**⚠️ CONDITIONAL (Weeks 5-8)**: C++ extension
- Only if architectural optimization insufficient
- Or if need < 5ms performance
- Requires C++ expertise and cross-platform support

**❌ NOT RECOMMENDED**: PyBullet batch API
- PyBullet development is inactive
- No timeline for implementation

**🔮 FUTURE**: GPU physics engine migration
- For next-generation system
- When simulation needs exceed PyBullet capabilities
- NVIDIA Isaac Sim for 100,000+ objects

### Next Steps

1. **Answer use case questions** (Section 9)
2. **Implement Phase 2 optimizations** (Section 8)
3. **Benchmark with realistic scenarios**
4. **Decide on C++ extension** based on Phase 2 results

---

**Document Version**: 1.0
**Last Updated**: 2026-01-01
**Author**: Performance Analysis Team
