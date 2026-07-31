# Experiment Scripts

These scripts test a narrow implementation or API hypothesis. They are not the
release benchmark: they print observations for one synthetic workload to stdout
and do not produce versioned JSON results. Use `benchmark/run_benchmark.py` for
repeatable throughput measurements, and use these experiments to decide which
question to investigate next.

Keep the workload, PyBullet version, physics setting, collision configuration,
and host constant when comparing runs. A result from one experiment is not a
general production recommendation.

All scripts live in `benchmark/experiments/`. For component profiling, see the
Profiling Guide (`benchmark/profiling/README.md`). For versioned overall
benchmark results, see the Benchmark Suite (`benchmark/README.md`).

---

## Collision Detection Experiments

### `collision_methods_config_based.py`

**Purpose:** Inspect the three bundled collision configuration files against a
fixed synthetic scene of 100 box `SimObject` instances over 500 steps.

```bash
python benchmark/experiments/collision_methods_config_based.py
```

The scene contains 30% physics bodies and 70% manually moved kinematic bodies.
It loads these configurations:

- Physics OFF + `closest_points`
- Physics ON + `contact_points`
- Physics ON + `hybrid`

The script reports average step time, collision-query time, and active-pair
count for each configuration. Physics and collision method change together, so
the result is an integration observation, not an isolated method comparison.
The lowest measured collision-query time applies only to this fixed workload;
select a production configuration according to required dynamics and collision
semantics, then benchmark the target scene.

### `collision_method_comparison.py`

**Purpose:** Compare the spatial-hash pipeline with deliberately different
all-pairs baselines and raw PyBullet queries under dense, kinematic cubes.

```bash
# Default: 100, 500, and 1000 agents; all methods; 100 iterations
python benchmark/experiments/collision_method_comparison.py

# Constrain the workload or the compared methods
python benchmark/experiments/collision_method_comparison.py \
  --agents=100,500 --methods=spatial,brute,closest --iterations=100
```

The spatial-hash path uses the simulation's configured collision pipeline. In
the kinematic default this is closest-points based; `getContactPoints` is a
PyBullet API comparison target, not the default collision pipeline.

`getContactPoints` variants do not have the same kinematic collision semantics
as closest-points queries for mass-zero bodies. In addition, the no-arguments
variant includes a PyBullet physics step. Treat timing comparisons as valid
only between rows that detect equivalent collision sets under the chosen
workload. The script warns when counts differ.

### `collision_mode_comparison.py`

**Purpose:** Measure disabled, `NORMAL_2D`, and `NORMAL_3D` collision modes for
a moving-agent grid in one scene.

```bash
python benchmark/experiments/collision_mode_comparison.py \
  --agents=1000 --iterations=100
```

`NORMAL_2D` searches XY neighbours only, while `NORMAL_3D` also considers Z
neighbours. Use `NORMAL_2D` when that semantic restriction is correct for the
scene; do not choose it solely from a speed result. The reported difference is
specific to the grid density, cell size, object size, movement, and host used
for the run.

### Output Example

```text
Experiment: <name>
Workload: <agents_or_objects>, <iterations_or_steps>, <physics_and_collision_config>

Method or mode              Step (ms)    Collision (ms)    Active pairs
<candidate>                 <mean_ms>   <mean_ms>         <count>
<candidate>                 <mean_ms>   <mean_ms>         <count>

Interpretation: compare timings only when collision semantics and detected
collision sets are equivalent.
```

> **Note:** `wrapper_overhead.py` (wrapper-layer overhead: PyBullet vs
> SimObject vs Agent) lives in `benchmark/profiling/`. See the Profiling Guide
> for details.
