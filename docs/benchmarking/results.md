# Benchmark Results Reference

Results are kept up-to-date with each release. For how to reproduce them,
see [Benchmark Suite](benchmark-suite) and [Profiling Guide](profiling-guide).

---

## Test Environment

| Item | Value |
|------|-------|
| OS | Ubuntu 20.04 (Linux 5.15.0) |
| CPU | Intel Core i7-1185G7 @ 3.00 GHz (4 cores / 8 logical) |
| RAM | 31 GB |
| Python | 3.8.10 |
| Conditions | Headless (`gui=false`), `physics=false`, `timestep=0.1`, 3 repetitions |

---

## Simulation Throughput

**Script:** `make bench-full` → `benchmark/run_benchmark.py --sweep 100 500 1000`
**Config:** `benchmark/configs/general.yaml` — `collision_check_frequency=null` (every step), 50% agents moving
**Last measured:** 2026-04-08

| Agents | Step Time (ms) | RTF   | Spawn Time | Memory Delta |
|--------|----------------|-------|------------|--------------|
| 100    | 2.17 ± 0.10    | 46.1× | 26 ms      | −24.8 MB     |
| 250    | 6.45 ± 0.30    | 16×   | 65 ms      | −19.7 MB †   |
| 500    | 13.21 ± 0.21   | 7.6×  | 137 ms     | −15.3 MB     |
| 1000   | 29.98 ± 0.39   | 3.3×  | 285 ms     | −3.2 MB      |
| 2000   | 94.82 ± 5.81   | 1.1×  | 731 ms     | +29.6 MB †   |

† Row from 2026-03-12; not included in `make bench-full` (100/500/1000 only).

**Source:** `benchmark/results/benchmark_sweep_10.0s.json`

**Memory note:** Negative delta below ~1000 agents is a Python GC artifact. Actual per-agent overhead is ~15 KB above 1000 agents (linear).

**Scalability:** O(n^1.3) — near-linear up to ~500 agents, slightly super-linear above.

---

## Step Time Component Breakdown

**Script:** `benchmark/profiling/simulation_profiler.py --test builtin --agents 500 --steps 500`

| Component | Share | Notes |
|-----------|-------|-------|
| Agent Update | ~81% | Dominant cost; much higher for moving than stationary |
| Collision Check | ~18% | Periodic (bursty); minimal cost on non-check steps |
| Monitor Update | ~1% | Near-zero if monitor disabled |
| Step Simulation | 0% | Physics off; up to ~40% with physics on |

> **Note on variance:** Profiler timestamps show high variance (stdev > mean) because collision checks
> fire in bursts and some steps include PyBullet warmup or GC pauses. The percentages above are
> representative of steady-state operation.

---

## Agent Update Cost

**Script:** `benchmark/profiling/agent_update.py --agents 500`

### Stationary vs Moving (500 agents)

| State | Total time | Per agent |
|-------|-----------|-----------|
| Stationary | 0.29 ms | 0.57 μs |
| Moving | 61.27 ms | 122.5 μs |
| Ratio | **214×** | |

**→ Design decision:** Benchmarks use 50% moving agents as a representative workload.
Moving agent cost dominates: kinematics, trajectory following, and PyBullet API calls
only execute for agents with an active goal.

### Motion Mode Comparison (500 agents, all moving)

| Mode | Total time | Per agent | Relative |
|------|-----------|-----------|---------|
| OMNIDIRECTIONAL | 9.26 ms | 18.5 μs | 1× (baseline) |
| DIFFERENTIAL | 26.66 ms | 53.3 μs | 2.9× slower |

**→ Optimisation note (2026-03-12):** DIFFERENTIAL was previously ~5× slower (90.6 μs/agent).
Replacing scipy Slerp with a precomputed quaternion slerp and removing per-tick numpy allocations
reduced the ROTATE phase from 37 μs to 5 μs. See [Differential Drive Optimization](#differential-drive-optimization) below.

**→ Design decision:** Benchmarks default to OMNIDIRECTIONAL. DIFFERENTIAL requires
heading alignment computation which adds ~2.9× update cost.

---

## Wrapper Layer Overhead

**Script:** `benchmark/profiling/wrapper_overhead.py --n 500 --reps 3`
**Conditions:** 500 objects, process-isolated per layer

| Layer | Spawn time | Update time (get+set pose) | Memory (RSS delta) |
|-------|-----------|---------------------------|-------------------|
| Direct PyBullet (baseline) | 63 ms | 2.21 ms | 5.2 MB |
| SimObject | 106 ms (+68%) | 6.80 ms (+3.1×) | 9.2 MB |
| SimObjectManager | 207 ms (+229%) | 10.75 ms (+4.9×) | 9.3 MB |
| Agent | 237 ms (+276%) | 10.60 ms (+4.8×) | 11.5 MB |
| AgentManager | 255 ms (+305%) | 10.20 ms (+4.6×) | 11.6 MB |

**Extrapolated to 10,000 objects (production feasibility, thresholds: spawn < 10 s, update < 150 ms/step, memory < +200 MB):**

| Layer | Spawn | Update/step | Memory | Pass? |
|-------|-------|-------------|--------|-------|
| SimObject | 2.1 s | 136 ms | +81 MB | ✅ all pass |
| Agent | 4.7 s | 212 ms | +127 MB | spawn/mem ✅, update ❌ |

**→ Design decision:** SimObject layer is production-viable at 10,000 scale.
Agent layer exceeds the 150 ms/step update threshold at 10,000 (extrapolated 212 ms),
but comfortably passes at 5,000 or below. For ≥10,000 agents, consider using SimObject
directly or batching update calls.

---

## Collision Mode Comparison (DISABLED / NORMAL_2D / NORMAL_3D)

**Script:** `benchmark/experiments/collision_mode_comparison.py --agents 500 --iterations 200`

| Mode | Step time (mean) | Collision time | vs Disabled |
|------|-----------------|---------------|-------------|
| DISABLED | 1.37 ms | — | baseline |
| NORMAL_2D (9 neighbors) | 2.43 ms | 0.21 ms | +77% |
| NORMAL_3D (27 neighbors) | 2.26 ms | 0.39 ms | +64% |

Collision check breakdown (both modes): AABB Filtering accounts for ~97–99% of collision check time.
Spatial Hashing and Contact Points together are <3%.

**→ Design decision:** NORMAL_2D yields only modest speedup (7%) over NORMAL_3D at this scale
because AABB filtering dominates in both. Use NORMAL_2D for ground robots where Z-axis neighbors
are irrelevant for correctness, not for dramatic performance gains.

---

## Collision Algorithm Comparison (Spatial Hashing vs Alternatives)

**Script:** `benchmark/experiments/collision_method_comparison.py --agents 100,500 --iterations 100`
**Conditions:** spacing=0.08m (dense), kinematic objects (mass=0)

| Method | 100 agents | 500 agents | Collisions detected | Valid? |
|--------|-----------|-----------|---------------------|--------|
| Spatial Hashing (current) | 8.0 ms | 92.1 ms | ✅ correct | ✅ |
| Brute Force AABB | 8.7 ms | 231.9 ms (2.5×) | ✅ correct | ✅ |
| `getClosestPoints` all-pairs | 23.0 ms | 370.6 ms (4.0×) | ✅ correct | ✅ |
| `getContactPoints()` no-args | 0.06 ms | 0.24 ms | ❌ 0 — invalid | ❌ |
| `getContactPoints(A,B)` pairwise | 12.1 ms | 190.3 ms | ❌ 0 — invalid | ❌ |

**Key finding:** `getContactPoints` variants return 0 collisions for kinematic objects (mass=0).
PyBullet's contact point solver only activates between physics-enabled bodies.
Spatial hashing with `getClosestPoints` is the only valid fast method for kinematic AGV-type robots.

**→ Design decision:** Spatial hashing (O(N) average) is used because:
1. It is the only algorithmically valid method for kinematic robots.
2. It is 2.5–4× faster than the valid alternatives at 500 agents.

---

## Differential Drive Optimization

**Script:** `benchmark/profiling/differential_breakdown.py`
**Conditions:** 1000 agents, DIFFERENTIAL mode (ROTATE phase)

The differential drive ROTATE phase was the single most expensive per-agent per-tick operation.
Profiling identified two root causes:

| Operation | Before | After | Speedup |
|-----------|--------|-------|---------|
| Quaternion slerp (scipy Slerp → `_quat_slerp`) | 26.8 μs | 2.3 μs | **11.8×** |
| Scalar clip (`np.clip` → `max/min`) | 6.8 μs | 0.15 μs | **44×** |
| Dead `np.array()` allocations (P0) | ~0.6 μs | 0 μs | eliminated |
| Redundant `np.dot` in slerp (P1) | ~0.3 μs | 0 μs | eliminated |
| Velocity array reallocation (P2) | ~0.3 μs | 0 μs | in-place |
| FORWARD direction recompute (P3) | ~0.5 μs | 0 μs | cached |
| **ROTATE phase total** | **36.1 μs** | **4.9 μs** | **7.3×** |

**End-to-end impact (1000 agents, differential):**

| Metric | Before | After | Improvement |
|--------|--------|-------|-------------|
| Per-agent update | 57 μs | 12.4 μs | 4.6× faster |
| DIFFERENTIAL / OMNIDIRECTIONAL ratio | 5.0× | 2.9× | Closer to parity |
| 500-agent ROTATE frame time | 18.0 ms | 2.5 ms | −15.6 ms/frame |

**Why was scipy Slerp slow?**

`scipy.spatial.transform.Slerp` is a generic array-oriented API designed for batch interpolation
(e.g., `slerp(np.linspace(0, 1, 10000))`). When called with a single scalar `t` per agent per tick,
the fixed overhead — input normalisation, dtype promotion, Rotation object wrapping — dominates
the actual trigonometry. The replacement `_quat_slerp` sidesteps this by precomputing
`dot/theta/sin(theta)` once per waypoint transition and using `math.sin`/`math.cos` (C-level scalar
functions) instead of numpy array dispatch.

Absolute timings vary by environment; the ratios above are the meaningful comparison metric.

---

## Collision Config-Based Comparison (Physics ON vs OFF)

**Script:** `benchmark/experiments/collision_methods_config_based.py`
**Conditions:** 100 objects, 500 steps

| Config | Physics | Method | Step time | Collision time |
|--------|---------|--------|-----------|---------------|
| `physics_off_closest.yaml` | OFF | `CLOSEST_POINTS` | 1.51 ms | 1.24 ms |
| `physics_on_contact.yaml` | ON | `CONTACT_POINTS` | 2.64 ms | 1.41 ms |
| `hybrid.yaml` | ON | `HYBRID` | 2.00 ms | 1.27 ms |

Physics ON adds ~75% step overhead due to the physics engine stepping each simulation step.

**→ Design decision:** Default is `physics=false` (kinematics mode) for maximum throughput.
Use `physics=true` only when rigid-body dynamics are required.

**Source:** `benchmark/results/collision_methods_config_based.txt`

---

## Arm Joint Control Performance

**Script:** `benchmark/profiling/arm_joint_update.py --test scaling`
**Conditions:** arm_robot.urdf (4 revolute joints), fixed-base, JointAction cycling, 100 steps per count

### Physics vs Kinematic Scaling

| Arms | Joints | Physics (ms/step) | Kinematic (ms/step) | Ratio |
|------|--------|--------------------|---------------------|-------|
| 1    | 4      | 0.029              | 0.010               | 2.8×  |
| 5    | 20     | 0.082              | 0.056               | 1.5×  |
| 10   | 40     | 0.152              | 0.090               | 1.7×  |
| 25   | 100    | 0.415              | 0.253               | 1.6×  |
| 50   | 200    | 0.886              | 0.552               | 1.6×  |

Kinematic mode is consistently faster than physics mode for joint control.
The gap comes from skipping `stepSimulation()` — kinematic mode uses `resetJointState()`
with per-step interpolation at URDF velocity limits.
Exact ratios are environment-dependent; the trend (kinematic faster) is consistent.

### Component Breakdown (10 arms)

**Script:** `benchmark/profiling/arm_joint_update.py --test builtin --arms 10`

| Component | Physics | Kinematic |
|-----------|---------|----------|
| agent_update | 27.0% (0.050 ms) | 96.7% (0.087 ms) |
| step_simulation | 69.6% (0.129 ms) | 0.2% (0.000 ms) |
| callbacks | 1.0% | 0.4% |
| **total** | **0.186 ms** | **0.090 ms** |

In physics mode, `stepSimulation()` dominates (70%). In kinematic mode, the physics engine
is bypassed entirely — agent_update (joint interpolation + `resetJointState`) is the sole cost.

### Kinematic Joint Cache Optimization

**Problem:** cProfile showed `p.getJointState()` consuming ~36% of kinematic update time
(called per-joint per-step to read current positions before interpolating).

**Solution:** `_kinematic_joint_positions` cache — joint positions stored in a Python dict,
initialized via batch `p.getJointStates()` at agent creation, updated after each `resetJointState()`.
`get_joint_state()` returns cached values for kinematic robots (zero PyBullet calls).

| Metric | Before cache | After cache | Improvement |
|--------|-------------|-------------|-------------|
| 50 arms step time | 0.826 ms | 0.552 ms | **1.5× faster** |
| `p.getJointState` calls/step | 200 (50 arms × 4 joints) | 0 | **eliminated** |

**→ Design decision:** Kinematic joint cache is always active for `mass=0.0` URDF robots.
The cache is invisible to callers — `get_joint_state()` API is unchanged.

*Data collected 2026-03-15. Absolute timings and ratios are environment-dependent
(CPU, OS, PyBullet version). The qualitative trends — kinematic faster than physics,
cache eliminating per-step PyBullet calls — are expected to hold across environments.
Re-run `benchmark/profiling/arm_joint_update.py` to obtain numbers for your setup.*

---

## See Also

- [Benchmark Suite](benchmark-suite) — How to run benchmarks and reproduce these results
- [Experiment Scripts](experiments) — Collision algorithm comparison scripts
- [Profiling Guide](profiling-guide) — Per-component profiling scripts
- [Optimization Guide](optimization-guide) — Parameter recommendations based on these results
