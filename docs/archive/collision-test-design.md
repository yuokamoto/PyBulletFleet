# Collision Detection Comprehensive Test Design

**Date**: 2026-01-21
**Purpose**: Comprehensive regression test for collision detection system

## Test Coverage Matrix

### 1. Dimensions to Test

#### A. CollisionMode (4 modes)
- `NORMAL_3D` - Full 3D collision (27 neighbors)
- `NORMAL_2D` - 2D only collision (9 neighbors, XY plane)
- `STATIC` - One-time AABB, never updated
- `DISABLED` - No collision detection

#### B. CollisionDetectionMethod (3 methods)
- `CLOSEST_POINTS` - Distance-based (kinematics-safe)
- `CONTACT_POINTS` - Contact manifold (physics)
- `HYBRID` - Mixed mode

#### C. Physics Mode (2 states)
- `physics=False` - Kinematics only
- `physics=True` - Physics simulation

#### D. Object Interaction Types (10 combinations)
1. NORMAL_3D ↔ NORMAL_3D
2. NORMAL_3D ↔ NORMAL_2D
3. NORMAL_3D ↔ STATIC
4. NORMAL_3D ↔ DISABLED
5. NORMAL_2D ↔ NORMAL_2D
6. NORMAL_2D ↔ STATIC
7. NORMAL_2D ↔ DISABLED
8. STATIC ↔ STATIC
9. STATIC ↔ DISABLED
10. DISABLED ↔ DISABLED

#### E. Collision States (4 states)
- **Clear separation** - No collision expected (distance > margin)
- **Near miss** - Within margin (should detect with CLOSEST_POINTS)
- **Touching** - Exactly at contact (distance ≈ 0)
- **Penetrating** - Overlapping (distance < 0)

#### F. Special Cases
- Multi-cell registration (large objects)
- Z-axis separation (2D vs 3D)
- `ignore_static_collision` flag
- Runtime mode changes
- Movement detection
- AABB cache invalidation

---

## 2. Test Structure

### Test Categories

#### Category 1: Basic Collision Detection
**Purpose**: Verify correct detection for each mode combination

**Test cases**:
- TC001: 3D-3D collision detection
- TC002: 2D-2D collision detection (ignores Z)
- TC003: 3D-STATIC collision detection
- TC004: 2D-STATIC collision detection
- TC005: DISABLED objects never collide

**For each test case**:
- ✅ Collision detected when overlapping
- ✅ Collision NOT detected when separated
- ✅ Near-miss detected with margin (CLOSEST_POINTS)
- ✅ No false positives

#### Category 2: CollisionDetectionMethod Verification
**Purpose**: Verify each detection method behaves correctly

**Test cases**:
- TC101: CLOSEST_POINTS detects near-miss
- TC102: CONTACT_POINTS requires stepSimulation()
- TC103: CONTACT_POINTS unstable for kinematics (known limitation)
- TC104: HYBRID mode switches correctly
- TC105: Auto-selection (physics=False → CLOSEST_POINTS)

#### Category 3: 2D vs 3D Neighbor Search
**Purpose**: Verify spatial grid neighbor offsets

**Test cases**:
- TC201: NORMAL_3D checks 27 neighbors
- TC202: NORMAL_2D checks 9 neighbors (XY plane only)
- TC203: Z-separated objects (2D should ignore, 3D should detect)
- TC204: Mixed 2D/3D collision correctly handled

#### Category 4: STATIC Object Optimization
**Purpose**: Verify static objects work correctly

**Test cases**:
- TC301: STATIC objects registered once (AABB not updated)
- TC302: Moving object ↔ STATIC collision detected
- TC303: STATIC ↔ STATIC with `ignore_static=True` (skip)
- TC304: STATIC ↔ STATIC with `ignore_static=False` (detect)

#### Category 5: DISABLED Mode
**Purpose**: Verify collision completely disabled

**Test cases**:
- TC401: DISABLED object not in spatial grid
- TC402: DISABLED ↔ NORMAL_3D no collision
- TC403: DISABLED ↔ STATIC no collision
- TC404: PyBullet collision filter disabled

#### Category 6: Multi-Cell Registration
**Purpose**: Verify large objects span multiple cells

**Test cases**:
- TC501: Large object (> threshold) uses multi-cell
- TC502: Small object (< threshold) uses single cell
- TC503: Large wall ↔ small robot collision detected
- TC504: Threshold configuration works

#### Category 7: Movement Detection
**Purpose**: Verify only moved objects trigger checks

**Test cases**:
- TC601: Moved object triggers collision check
- TC602: Stationary object doesn't trigger check
- TC603: STATIC objects never marked as moved
- TC604: `_moved_this_step` correctly tracked

#### Category 8: Runtime Mode Changes
**Purpose**: Verify collision mode can change at runtime

**Test cases**:
- TC701: NORMAL_3D → DISABLED (remove from grid)
- TC702: DISABLED → NORMAL_3D (add to grid)
- TC703: NORMAL_3D → STATIC (stop AABB updates)
- TC704: STATIC → NORMAL_3D (resume AABB updates)
- TC705: 2D → 3D neighbor count changes

#### Category 9: Edge Cases
**Purpose**: Test boundary conditions

**Test cases**:
- TC801: Zero-size objects
- TC802: Identical positions (perfect overlap)
- TC803: Negative collision margin
- TC804: Very large cell sizes
- TC805: Empty simulation (no objects)
- TC806: Single object (no pairs)
- TC807: 1000+ objects (scalability)

#### Category 10: Integration Tests
**Purpose**: Test complete workflows

**Test cases**:
- TC901: Full simulation loop with collisions
- TC902: Mixed physics/kinematics scenario
- TC903: Path planning with collision avoidance
- TC904: Batch spawn with various modes

---

## 3. Test Implementation Plan

### Phase 1: Foundation (Priority: HIGH)
**File**: `tests/test_collision_comprehensive.py`

**Implement**:
- Helper functions for creating test objects
- Assertion utilities for collision detection
- Parameterized test framework setup

**Key helpers**:
```python
def create_test_object(sim_core, position, collision_mode, size=0.5)
def assert_collision_detected(sim_core, obj1, obj2, should_collide=True)
def assert_collision_count(sim_core, expected_count)
def move_object(sim_core, obj, new_position)
```

### Phase 2: Core Tests (Priority: HIGH)
**Implement Categories 1, 2, 3**

Coverage:
- Basic collision detection (all mode combinations)
- Detection method verification
- 2D vs 3D behavior

### Phase 3: Advanced Tests (Priority: MEDIUM)
**Implement Categories 4, 5, 6**

Coverage:
- STATIC optimization
- DISABLED mode
- Multi-cell registration

### Phase 4: Behavioral Tests (Priority: MEDIUM)
**Implement Categories 7, 8**

Coverage:
- Movement detection
- Runtime mode changes

### Phase 5: Edge Cases & Integration (Priority: LOW)
**Implement Categories 9, 10**

Coverage:
- Boundary conditions
- Full workflows

---

## 4. Test Data Organization

### Fixture Setup
```python
@pytest.fixture
def sim_core_kinematics():
    """Simulation with physics=False, CLOSEST_POINTS"""
    params = SimulationParams(
        gui=False,
        physics=False,
        collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
        collision_margin=0.02,
    )
    return MultiRobotSimulationCore(params)

@pytest.fixture
def sim_core_physics():
    """Simulation with physics=True, CONTACT_POINTS"""
    params = SimulationParams(
        gui=False,
        physics=True,
        collision_detection_method=CollisionDetectionMethod.CONTACT_POINTS,
        collision_margin=0.02,
    )
    return MultiRobotSimulationCore(params)
```

### Test Object Templates
```python
TEST_OBJECTS = {
    "small_robot": {
        "size": 0.5,
        "collision_mode": CollisionMode.NORMAL_3D,
        "mass": 0.0,
    },
    "large_wall": {
        "size": 5.0,
        "collision_mode": CollisionMode.STATIC,
        "mass": 0.0,
    },
    "flying_drone": {
        "size": 0.3,
        "collision_mode": CollisionMode.NORMAL_3D,
        "mass": 0.0,
    },
    "ground_robot": {
        "size": 0.5,
        "collision_mode": CollisionMode.NORMAL_2D,
        "mass": 0.0,
    },
}
```

---

## 5. Expected Behavior Matrix

### Collision Detection Matrix

| Object 1 Mode | Object 2 Mode | Should Detect? | Notes |
|---------------|---------------|----------------|-------|
| NORMAL_3D | NORMAL_3D | ✅ Yes | Full 3D |
| NORMAL_3D | NORMAL_2D | ✅ Yes | Mixed detection |
| NORMAL_3D | STATIC | ✅ Yes (if !ignore_static) | |
| NORMAL_3D | DISABLED | ❌ No | DISABLED excludes |
| NORMAL_2D | NORMAL_2D | ✅ Yes | XY plane only |
| NORMAL_2D | STATIC | ✅ Yes (if !ignore_static) | |
| NORMAL_2D | DISABLED | ❌ No | DISABLED excludes |
| STATIC | STATIC | ⚠️ Depends | `ignore_static` flag |
| STATIC | DISABLED | ❌ No | DISABLED excludes |
| DISABLED | DISABLED | ❌ No | Neither checked |

### Z-Axis Separation (2D vs 3D)

| Object 1 | Object 2 | XY Overlap | Z Separation | Detect? |
|----------|----------|------------|--------------|---------|
| NORMAL_3D | NORMAL_3D | Yes | Yes | ❌ No (3D checks Z) |
| NORMAL_3D | NORMAL_3D | Yes | No | ✅ Yes |
| NORMAL_2D | NORMAL_2D | Yes | Yes | ✅ Yes (2D ignores Z) |
| NORMAL_2D | NORMAL_2D | Yes | No | ✅ Yes |
| NORMAL_3D | NORMAL_2D | Yes | Yes | ⚠️ Complex* |

*Mixed mode: Detection depends on which object initiates check

---

## 6. Success Criteria

### Functional Requirements
- ✅ All collision mode combinations work correctly
- ✅ DISABLED mode completely excludes objects
- ✅ STATIC objects don't update AABB after first registration
- ✅ 2D mode ignores Z-axis separation
- ✅ 3D mode checks full 3D space
- ✅ Multi-cell registration works for large objects

### Performance Requirements
- ✅ No false positives (collision when shouldn't)
- ✅ No false negatives (miss collision when should detect)
- ✅ Deterministic results (same input → same output)
- ✅ No memory leaks (objects properly cleaned up)

### Regression Protection
- ✅ All tests pass on current implementation
- ✅ Tests fail when expected behavior breaks
- ✅ Clear error messages for debugging

---

## 7. Known Limitations (Document, Don't Fix)

These are **expected behaviors** to document, not bugs:

1. **CONTACT_POINTS + Kinematics**
   - Kinematic-kinematic pairs may not detect with getContactPoints
   - Solution: Use CLOSEST_POINTS for kinematics

2. **Mixed 2D/3D Collision**
   - Detection behavior depends on initiator object's mode
   - Not fully symmetric

3. **STATIC-STATIC Detection**
   - Depends on `ignore_static_collision` flag
   - May not detect even if flag=False (both never move)

---

## 9. Collision Detection Pipeline — Test Coverage Map

The collision detection system is a multi-stage pipeline.
Each stage is tested in a different file/class, and the stages compose to
provide end-to-end coverage.

### Pipeline Stages

```
set_pose() / step_once()
        │
        ▼
┌─────────────────────────┐
│ 1. _moved_this_step     │  ← Which objects need re-evaluation?
│    (movement tracking)  │
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 2. filter_aabb_pairs()  │  ← AABB broadphase: generate candidate pairs
│    (broadphase filter)  │
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 3. check_collisions()   │  ← Narrowphase: PyBullet getClosestPoints /
│    (narrowphase +       │    getContactPoints per candidate pair.
│     pairs management)   │    Update _active_collision_pairs (add/remove).
└────────────┬────────────┘
             ▼
   _active_collision_pairs   ← Final result: set of colliding (i, j) tuples
```

### Where Each Stage Is Tested

| Stage | Test File | Test Class / Tests | What Is Verified |
|-------|-----------|-------------------|------------------|
| **1. Movement tracking** | `test_core_simulation.py` | `TestStepOnce` | Moving agent → added to `_moved_this_step`; stationary agent → not added; physics objects → always added; STATIC → added on creation only, not re-added by `step_once`; cleared after collision check |
| **2. Broadphase (filter_aabb_pairs)** | `test_core_simulation.py` | `TestFilterAabbPairs` | Overlapping AABBs → pair returned; distant objects → no pair; no moved objects → empty; DISABLED excluded; `ignore_static` flag; multi-cell objects; profiling timings |
| **3a. Narrowphase (detection)** | `test_collision_comprehensive.py` | `TestBasicCollisionDetection` | All 4×4 mode combinations × 2 distances = 32 parametrized cases; CLOSEST_POINTS near-miss (xfail: AABB limitation); CONTACT_POINTS with `step_once` |
| **3b. Pairs accuracy** | `test_collision_comprehensive.py` | `TestCollisionPairsAccuracy` | Exact set equality of `_active_collision_pairs`: 3-object chain, all-separated (empty), no-movement (unchanged), move adds/removes pairs, progressive accumulation |
| **3c. Movement → pairs** | `test_collision_comprehensive.py` | `TestMovementDetection` | `set_pose` into collision → detected; `set_pose` out of collision → resolved |
| **3d. Runtime mode changes** | `test_collision_comprehensive.py` | `TestRuntimeModeChanges` | Disable at runtime → pair removed; enable → pair added; same-mode no-op; `_remove_object_from_collision_system` clears pairs |
| **3e. Multi-cell** | `test_collision_comprehensive.py` | `TestMultiCellRegistration` | Single-cell vs multi-cell registration; `_get_overlapping_cells`; large wall ↔ small robot; threshold; `_coord_to_cell`; same-cell skip; `_discard_from_cells` |

### Cross-cutting concerns

| Concern | Where Tested |
|---------|-------------|
| `ignore_static_collision` flag | `TestFilterAabbPairs` (broadphase), `TestIgnoreStaticCollision` in `test_core_simulation.py` |
| `collision_check_frequency` | `TestCollisionCheckFrequency` in `test_core_simulation.py` |
| Object removal cleans pairs | `TestRemoveObject.test_remove_clears_active_collision_pairs` in `test_core_simulation.py` |
| Collision logging | `TestCollisionLogging` in `test_core_simulation.py` |

### Run All Tests
```bash
pytest tests/test_collision_comprehensive.py -v
```

### Run Specific Category
```bash
pytest tests/test_collision_comprehensive.py::TestBasicCollision -v
pytest tests/test_collision_comprehensive.py::TestMultiCell -v
```

### Run with Coverage
```bash
pytest tests/test_collision_comprehensive.py --cov=pybullet_fleet.core_simulation --cov-report=html
```

### Run Performance Benchmarks
```bash
pytest tests/test_collision_comprehensive.py --benchmark-only
```
