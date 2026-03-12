# Experiment Scripts

Experiments validate optimization hypotheses and compare different implementation approaches. Unlike profiling tools (which measure *what is happening*), these scripts answer specific questions about algorithm and API choices.

All scripts live in `benchmark/experiments/`. For profiling tools, see [`profiling/README.md`](../profiling/README.md). For overall benchmark results, see the [Benchmark Suite README](../README.md).

---

## Collision Detection Experiments

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

**Purpose:** Compare `CollisionMode` settings — disabled, 2D (9 neighbors), and 3D (27 neighbors) — at scale.

```bash
python benchmark/experiments/collision_mode_comparison.py
```

**What it tests:**
- `DISABLED`: No collision check (baseline)
- `NORMAL_2D`: Spatial hashing with 9 neighbors (ignore Z-axis)
- `NORMAL_3D`: Spatial hashing with 27 neighbors (all directions)

Outputs per-mode step time and collision check breakdown to quantify the 2D vs 3D speedup.


> **Note:** `wrapper_overhead.py` (wrapper-layer overhead: PyBullet vs SimObject vs Agent)
> has moved to `benchmark/profiling/`. See the [Profiling Guide](profiling-guide.md) for details.
