# Experiment Scripts

Experiments validate optimization hypotheses and compare different implementation approaches. Unlike profiling tools (which measure *what is happening*), these scripts answer specific questions about algorithm and API choices.

All scripts live in `benchmark/experiments/`. For profiling tools, see `benchmark/profiling/README.md`. For overall benchmark results, see `benchmark/README.md`.

---

## Collision Detection Experiments

### `collision_detection_methods_benchmark.py`

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

### `collision_methods_config_based.py` (Recommended)

**Purpose:** Compare collision detection using production config files with a full PyBulletFleet simulation (moving robots, physics integration).

```bash
python benchmark/experiments/collision_methods_config_based.py
```

**What it tests:**
- Physics OFF + `CLOSEST_POINTS` (kinematics mode)
- Physics ON + `CONTACT_POINTS` (physics mode)
- Physics ON + `HYBRID` (mixed mode)

Outputs per-config collision time and total step time, with a recommendation of the fastest mode.

### `collision_method_comparison.py`

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

### `collision_mode_comparison.py`

**Purpose:** Compare step-level performance across collision detection modes (NORMAL_3D vs NORMAL_2D vs DISABLED).

```bash
python benchmark/experiments/collision_mode_comparison.py --agents=1000 --iterations=100
```

**What it tests:**
- No Collision (`CollisionMode.DISABLED`) — baseline with collision detection off
- 2D Collision (`CollisionMode.NORMAL_2D`) — ignores Z-axis neighbors (9 neighbors)
- 3D Collision (`CollisionMode.NORMAL_3D`) — checks all directions (27 neighbors)

Outputs a comparison table with step time per mode and the relative cost of each collision mode.

---

## General Performance Experiments

### `performance_analysis.py`

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

### `list_filtering_benchmark.py`

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

### `getaabb_performance.py`

**Purpose:** Determine whether `p.getAABB()` is a bottleneck.

```bash
python benchmark/experiments/getaabb_performance.py
```

**What it tests:**
- Per-call and batch `getAABB()` timing across many objects

Outputs per-call latency and batch throughput to confirm whether AABB retrieval is a limiting factor.
