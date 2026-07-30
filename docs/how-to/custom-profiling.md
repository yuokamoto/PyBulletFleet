# Custom Class Profiling Guide

Profile custom `Agent`, `SimObject`, controller, callback, and plugin logic
without losing the framework's per-step timing breakdown.

## Start with built-in timing

Enable built-in profiling first. A custom `Agent.update()` runs inside the
normal step and is already included in both `agent_update` and the broader
`phase1_update` timing.

```yaml
simulation:
  enable_time_profiling: true
  profiling_interval: 100
  log_level: info
```

```python
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams

sim = MultiRobotSimulationCore(
    SimulationParams(
        gui=False,
        target_rtf=0,
        physics=False,
        enable_time_profiling=True,
        profiling_interval=100,
    )
)
```

Use [Time Profiling](time-profiling) for the complete built-in field list.
The fields most relevant to custom code are:

| Field | Includes |
|-------|----------|
| `agent_update` | Per-object `update(dt)` calls, including custom `Agent.update()` logic. |
| `callbacks` | Functions registered with `register_callback()`. |
| `phase1_update` | Pre-step events, batch advance, object updates, callbacks, and simulation plugin hooks. |
| `phase2_pose_flush` | Buffered base-pose writes committed to PyBullet. |
| `phase3_aabb_grid_flush` | Post-commit kinematic AABB and spatial-grid synchronization. |
| `total` | End-to-end framework step time. |

`agent_update` is a subset of `phase1_update`; do not add the two values
together. A custom timing field is also usually a subset of one of these
fields, rather than an independent portion of `total`.

## Add named custom fields

Use `sim.record_profiling(name, value_ms)` from code that runs inside a
simulation step. It auto-registers the field, accumulates contributions from
all callers in that step, and appears alongside built-in fields in both
periodic log output and `step_once(return_profiling=True)`.

### Custom `Agent`

```python
import time

from pybullet_fleet.agent import Agent


class ProfilingAgent(Agent):
    def update(self, dt: float) -> bool:
        t0 = time.perf_counter()
        self._do_custom_logic(dt)
        self.sim_core.record_profiling("custom_logic", (time.perf_counter() - t0) * 1000)

        return super().update(dt)

    def _do_custom_logic(self, dt: float) -> None:
        pass
```

```python
agent = ProfilingAgent.from_urdf(
    "robots/mobile_robot.urdf",
    sim_core=sim,
    controller={"type": "differential"},
)
```

If 100 agents report `"custom_logic"`, the value logged for a step is the
sum of their 100 measurements. This is useful for finding fleet-wide work;
use standalone timing below when you need per-agent distributions.

### Callback or plugin work

The same public API works from a registered callback. The callback's complete
cost remains included in `callbacks`, while the named field identifies the
portion you chose to measure.

```python
import time


def planner_callback(sim_core, dt):
    t0 = time.perf_counter()
    run_planner(sim_core)
    sim_core.record_profiling("planner", (time.perf_counter() - t0) * 1000)


sim.register_callback(planner_callback, frequency=10.0)
```

### Programmatic timing results

`return_profiling=True` enables measurement for that call even when periodic
profiling is disabled. Custom fields recorded during the step are returned in
the same dictionary.

```python
timings = sim.step_once(return_profiling=True)
print(timings["phase1_update"])
print(timings.get("custom_logic", 0.0))
```

When the simulation is driven by `run_simulation()`, a callback or `SimPlugin`
can instead read `sim.last_profiling`. It is a read-only snapshot of the
previous completed profiled step; custom fields are included alongside the
built-in fields. It is therefore suitable for in-simulation monitoring, but
not for retaining a history—copy values to your own collector when needed.

Do not write to `_profiling_stats` directly. It is an internal accumulator
whose lifecycle differs between periodic logging and returned per-step timing.

## Function-level profiling with `cProfile`

Use `cProfile` when a named custom field identifies a slow area and you need
the individual functions responsible.

```bash
python -m cProfile -s cumulative your_script.py
```

For a targeted section, warm up first and then profile only simulation steps
of interest:

```python
import cProfile
import pstats

for _ in range(5):
    sim.step_once()

profiler = cProfile.Profile()
profiler.enable()
for _ in range(100):
    sim.step_once()
profiler.disable()

pstats.Stats(profiler).sort_stats("cumulative").print_stats(30)
```

`cProfile` changes execution characteristics, so use it to locate expensive
functions rather than to publish absolute throughput numbers.

## Per-agent measurements

Use `time.perf_counter()` and retain samples yourself when the distribution
between agents matters more than fleet-wide totals.

```python
import statistics
import time

from pybullet_fleet.agent import Agent


class PerAgentProfilingAgent(Agent):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.custom_logic_ms = []

    def update(self, dt: float) -> bool:
        t0 = time.perf_counter()
        self._do_custom_logic(dt)
        self.custom_logic_ms.append((time.perf_counter() - t0) * 1000)
        return super().update(dt)

    def _do_custom_logic(self, dt: float) -> None:
        pass

    def print_timing_summary(self) -> None:
        if self.custom_logic_ms:
            print(f"mean={statistics.mean(self.custom_logic_ms):.3f} ms, " f"max={max(self.custom_logic_ms):.3f} ms")
```

This measurement is intentionally separate from framework profiling. Avoid
retaining unbounded samples in long-running simulations; aggregate or export
them periodically instead.

## Compare a custom agent with the base class

Use the same scene, controller, collision settings, warm-up, and step count
for both runs. The following minimal helper uses the supported spawning API:

```python
import time

from pybullet_fleet.geometry import Pose


def benchmark(agent_class, label, num_agents=100, num_steps=100):
    sim = MultiRobotSimulationCore(SimulationParams(gui=False, target_rtf=0, physics=False))
    try:
        for i in range(num_agents):
            agent = agent_class.from_urdf(
                "robots/mobile_robot.urdf",
                pose=Pose.from_xyz(i * 2.0, 0.0, 0.05),
                mass=0.0,
                sim_core=sim,
                controller={"type": "differential"},
            )
            agent.set_goal_pose(Pose.from_xyz(i * 2.0 + 1.0, 0.0, 0.05))

        for _ in range(5):
            sim.step_once()

        t0 = time.perf_counter()
        for _ in range(num_steps):
            sim.step_once()
        print(f"{label}: {(time.perf_counter() - t0) * 1000 / num_steps:.2f} ms/step")
    finally:
        sim.cleanup()
```

Run multiple trials and compare the resulting distributions. For repeatable
repository benchmarks, use the benchmark suite rather than this exploratory
helper.

## See also

- [Time Profiling User Guide](time-profiling) — Built-in fields and returned timing dictionaries
- [Profiling Guide](../benchmarking/profiling-guide) — Repository benchmark tools
- [Two-Phase Step](../architecture/two-phase-step) — Step lifecycle and buffered pose commits
