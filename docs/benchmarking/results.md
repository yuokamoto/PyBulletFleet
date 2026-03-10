# Benchmark Results Reference

All results below were measured in the same session (2026-03-10). For how to reproduce them,
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

**Script:** `benchmark/run_benchmark.py --sweep 100 250 500 1000 2000`
**Config:** `benchmark/configs/general.yaml` — `collision_check_frequency=null` (every step), 50% agents moving

| Agents | Step Time (ms) | RTF | Spawn Time | Memory Delta |
|--------|---------------|-----|------------|--------------|
| 100    | 2.44 ± 0.15   | 41× | 27 ms      | −23.8 MB     |
| 250    | 6.71 ± 0.25   | 15× | 65 ms      | −19.6 MB     |
| 500    | 15.12 ± 0.23  | 6.6×| 134 ms     | −12.2 MB     |
| 1000   | 42.65 ± 0.99  | 2.3×| 368 ms     | +3.0 MB      |
| 2000   | 88.44 ± 1.01  | 1.1×| 731 ms     | +29.5 MB     |

**Source:** `benchmark/results/benchmark_sweep_10.0s.json`

**Memory note:** Negative delta below ~1000 agents is a Python GC artifact. Actual per-agent overhead is ~15 KB above 1000 agents (linear).

**Scalability:** O(n^1.3) — near-linear up to ~500 agents, slightly super-linear above.

---

## Step Time Component Breakdown

**Script:** `benchmark/profiling/simulation_profiler.py --test builtin --agents 500 --steps 500`

| Component | Share | Notes |
|-----------|-------|-------|
| Agent Update | ~83% | Dominant cost; much higher for moving than stationary |
| Collision Check | ~15% | Periodic (bursty); minimal cost on non-check steps |
| Monitor Update | ~2% | Near-zero if monitor disabled |
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
| Stationary | 0.30 ms | 0.6 μs |
| Moving | 87.00 ms | 174 μs |
| Ratio | **288×** | |

**→ Design decision:** Benchmarks use 50% moving agents as a representative workload.
Moving agent cost dominates: kinematics, trajectory following, and PyBullet API calls
only execute for agents with an active goal.

### Motion Mode Comparison (500 agents, all moving)

| Mode | Total time | Per agent | Relative |
|------|-----------|-----------|---------|
| OMNIDIRECTIONAL | 9.12 ms | 18.3 μs | 1× (baseline) |
| DIFFERENTIAL | 45.31 ms | 90.6 μs | 5.0× slower |

**→ Design decision:** Benchmarks default to OMNIDIRECTIONAL. DIFFERENTIAL requires
heading alignment computation which adds ~4× update cost.

---

## Wrapper Layer Overhead

**Script:** `benchmark/profiling/wrapper_overhead.py --n 500 --reps 3`
**Conditions:** 500 objects, process-isolated per layer

| Layer | Spawn time | Update time (get+set pose) | Memory (RSS delta) |
|-------|-----------|---------------------------|-------------------|
| Direct PyBullet (baseline) | 71 ms | 2.07 ms | 5.1 MB |
| SimObject | 101 ms (+42%) | 5.83 ms (+2.8×) | 9.2 MB |
| SimObjectManager | 108 ms (+52%) | 5.88 ms (+2.8×) | 9.4 MB |
| Agent | 139 ms (+96%) | 7.37 ms (+3.6×) | 11.4 MB |
| AgentManager | 149 ms (+110%) | 7.60 ms (+3.7×) | 11.6 MB |

**Extrapolated to 10,000 objects (production feasibility, thresholds: spawn < 10 s, update < 150 ms/step, memory < +200 MB):**

| Layer | Spawn | Update/step | Memory | Pass? |
|-------|-------|-------------|--------|-------|
| SimObject | 2.0 s | 117 ms | +81 MB | ✅ all pass |
| Agent | 2.8 s | 147 ms | +126 MB | ✅ all pass |

**→ Design decision:** Both SimObject and Agent layers are production-viable at 10,000 scale.
The ~3–4× update overhead vs raw PyBullet covers the pose cache, type safety, and callback system.

---

## Collision Mode Comparison (DISABLED / NORMAL_2D / NORMAL_3D)

**Script:** `benchmark/experiments/collision_mode_comparison.py --agents 500 --iterations 200`

| Mode | Step time (mean) | Collision time | vs Disabled |
|------|-----------------|---------------|-------------|
| DISABLED | 0.96 ms | — | baseline |
| NORMAL_2D (9 neighbors) | 1.65 ms | 0.13 ms | +72% |
| NORMAL_3D (27 neighbors) | 1.77 ms | 0.29 ms | +84% |

**2D vs 3D speedup at 500 agents: 1.07× (7% faster)**

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
| Spatial Hashing (current) | 7.1 ms | 59.5 ms | ✅ correct | ✅ |
| Brute Force AABB | 7.4 ms | 177.6 ms (3.0×) | ✅ correct | ✅ |
| `getClosestPoints` all-pairs | 11.3 ms | 348.5 ms (5.9×) | ✅ correct | ✅ |
| `getContactPoints()` no-args | 0.03 ms | 0.22 ms | ❌ 0 — invalid | ❌ |
| `getContactPoints(A,B)` pairwise | 5.4 ms | 185.1 ms | ❌ 0 — invalid | ❌ |

**Key finding:** `getContactPoints` variants return 0 collisions for kinematic objects (mass=0).
PyBullet's contact point solver only activates between physics-enabled bodies.
Spatial hashing with `getClosestPoints` is the only valid fast method for kinematic AGV-type robots.

**→ Design decision:** Spatial hashing (O(N) average) is used because:
1. It is the only algorithmically valid method for kinematic robots.
2. It is 3–6× faster than the valid alternatives at 500 agents.

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

## See Also

- [Benchmark Suite](benchmark-suite) — How to run benchmarks and reproduce these results
- [Experiment Scripts](experiments) — Collision algorithm comparison scripts
- [Profiling Guide](profiling-guide) — Per-component profiling scripts
- [Optimization Guide](optimization-guide) — Parameter recommendations based on these results
