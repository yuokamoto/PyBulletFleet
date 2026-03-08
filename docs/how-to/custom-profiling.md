# Custom Class Profiling Guide

How to profile performance when you subclass `Agent` or `SimObject` with custom logic.

---

## Overview

When you extend PyBulletFleet with custom agent behavior (e.g., custom `update()`, custom actions, custom collision callbacks), the built-in profiling tools still work — but you may also want to measure your custom code specifically.

This guide covers three approaches:

| Approach | Best For | Overhead |
|----------|----------|----------|
| Built-in Profiling (`run_simulation`) | Component breakdown + custom field integration | < 0.1% |
| cProfile | Finding which custom functions are slow | 5-50% |
| Standalone `perf_counter` | Per-agent self-contained timing (non-integrated) | ~0% |

---

## Approach 1: Built-in Profiling (Recommended First Step)

Your custom `Agent.update()` is automatically measured as part of the `agent_update` component.
Just enable profiling and run — no code changes needed:

```yaml
# config.yaml
enable_time_profiling: true
profiling_interval: 100
log_level: info
```

```python
from pybullet_fleet.core_simulation import SimulationParams, MultiRobotSimulationCore

params = SimulationParams(
    gui=False, target_rtf=0, physics=False,
    enable_time_profiling=True,
    profiling_interval=100,
)
sim = MultiRobotSimulationCore(params)
# ... spawn agents, set goals ...
sim.run_simulation(duration=10.0)
# Profiling summaries are printed every 100 steps automatically:
#   [PROFILING] Last 100 steps average: agent_update=1.23ms (45.0%),
#     callbacks=0.01ms (0.4%), step_simulation=0.05ms (1.8%),
#     collision_check=0.42ms (15.4%), monitor_update=0.00ms (0.0%),
#     total=2.73ms (100.0%)
```

If `agent_update` is unexpectedly high but you need more detail, read on.

### 1-1. Adding Custom Fields — Agent Subclass (`record_profiling`)

Use `record_profiling()` to break down your custom logic into named fields
that appear **in the same profiling output** as built-in fields.
Fields are auto-registered on first call — no separate setup needed.

**Why `+=` (accumulation)?** Each step, 100 agents each call `record_profiling("planner", 0.1)`.
These are summed: the profiling output shows the **total** planner time across all agents for that step.
If `=` were used, only the last agent's value would survive.

```python
import time
from pybullet_fleet.agent import Agent


class ProfilingAgent(Agent):
    """Agent subclass that reports custom timing to built-in profiling."""

    def update(self, dt: float) -> bool:
        # Time your custom logic and report to sim_core
        t0 = time.perf_counter()
        self._do_custom_logic(dt)
        self.sim_core.record_profiling(
            "custom_logic", (time.perf_counter() - t0) * 1000
        )

        # Call parent update
        t1 = time.perf_counter()
        moved = super().update(dt)
        self.sim_core.record_profiling(
            "path_update", (time.perf_counter() - t1) * 1000
        )

        return moved

    def _do_custom_logic(self, dt: float):
        # Your custom behavior here
        pass
```

Setup and usage:

```python
from pybullet_fleet.core_simulation import SimulationParams, MultiRobotSimulationCore

params = SimulationParams(
    gui=False, target_rtf=0, physics=False,
    enable_time_profiling=True,  # Enable built-in profiling output
    profiling_interval=100,
)
sim = MultiRobotSimulationCore(params)

# No registration needed — record_profiling() auto-registers on first call

# Spawn your custom agents
for i in range(100):
    agent = ProfilingAgent.from_urdf("robots/mobile_robot.urdf", sim)
    # ... set goals ...

# Run — custom fields appear in the SAME profiling line as built-in fields!
# Output example (single line, all fields together):
#   [PROFILING] Last 100 steps average: agent_update=1.23ms (45.0%),
#     callbacks=0.01ms (0.4%), step_simulation=0.05ms (1.8%),
#     collision_check=0.42ms (15.4%), monitor_update=0.00ms (0.0%),
#     total=2.73ms (100.0%), custom_logic=0.50ms (18.3%), path_update=0.73ms (26.7%)
sim.run_simulation(duration=10.0)
```

### 1-2. Adding Custom Fields — SimulationCore Subclass (Direct Write)

If you subclass `MultiRobotSimulationCore` and override `step_once()`,
you can write directly to `_profiling_stats` using the same `[-1] = value`
pattern that built-in fields use. This is appropriate when **one** call
produces the measurement (no accumulation needed).

```python
import time
from pybullet_fleet.core_simulation import SimulationParams, MultiRobotSimulationCore


class CustomSimCore(MultiRobotSimulationCore):
    """SimulationCore subclass with a custom profiling field."""

    def step_once(self, return_profiling=False):
        # Let the parent do its normal step (which appends 0.0 to all fields)
        result = super().step_once(return_profiling=return_profiling)

        # Measure your custom post-step processing
        measure_timing = self._enable_time_profiling or return_profiling
        if measure_timing:
            t0 = time.perf_counter()

        self._custom_post_process()

        if measure_timing:
            elapsed_ms = (time.perf_counter() - t0) * 1000
            # Auto-register on first use, then direct assignment (single call per step)
            data = self._profiling_stats.get("post_process")
            if data is None:
                self._profiling_stats["post_process"] = [elapsed_ms]
            else:
                data[-1] = elapsed_ms  # Assignment, not +=

        return result

    def _custom_post_process(self):
        # Your custom logic here
        pass
```

```python
params = SimulationParams(
    gui=False, target_rtf=0, physics=False,
    enable_time_profiling=True,
    profiling_interval=100,
)
sim = CustomSimCore(params)

# Spawn agents...
sim.run_simulation(duration=10.0)
# Output:
#   [PROFILING] Last 100 steps average: agent_update=1.23ms (45.0%), ...,
#     total=2.73ms (100.0%), post_process=0.15ms (5.5%)
```

### How the Unified Design Works

All profiling data lives in a single `_profiling_stats: Dict[str, List[float]]` dict.
Each step follows the same lifecycle:

```
step start → append(0.0) to ALL existing keys
           → built-in fields:  _profiling_stats["agent_update"][-1] = value   (assignment)
           → custom via record_profiling:  _profiling_stats["planner"][-1] += value  (accumulation)
           → custom via direct write:      _profiling_stats["my_field"][-1] = value  (assignment)
step end   → _print_profiling_summary() reads ALL keys uniformly
```

There is no separate "custom fields" bucket — everything is a peer in the same dict.
Built-in fields are defined in `__init__`; custom fields are appended when first used.

**API reference:**

| Method | Description |
|--------|-------------|
| `sim.record_profiling(name, value_ms)` | Record timing in ms. Auto-registers on first call. `+=` accumulation (ideal for Agent subclasses where many instances report per step). |
| `self._profiling_stats[name][-1] = value` | Direct assignment (ideal for SimulationCore subclasses where one call produces the value). |

---

## Approach 2: cProfile (Find Slow Custom Functions)

Use Python's `cProfile` to get function-level call graphs that include your custom methods.

### External (No Code Changes)

The simplest way — just run your script with `python -m cProfile`:

```bash
# Sort by cumulative time, show top 30 functions
python -m cProfile -s cumulative your_script.py 2>&1 | head -40

# Filter to only your module
python -m cProfile -s cumulative your_script.py 2>&1 | grep -E "ncalls|your_module"
```

This profiles the entire script including all custom `update()`, action callbacks, and helper methods. No code changes needed.

### Programmatic (Targeted Profiling)

For more control (e.g., profiling only a specific section, excluding setup/teardown):

```python
import cProfile
import pstats
from pybullet_fleet.core_simulation import SimulationParams, MultiRobotSimulationCore

params = SimulationParams(gui=False, target_rtf=0, physics=False)
sim = MultiRobotSimulationCore(params)

# ... spawn your custom agents, set goals ...

# Warm up
for _ in range(5):
    sim.step_once()

# Profile
profiler = cProfile.Profile()
profiler.enable()

for _ in range(100):
    sim.step_once()

profiler.disable()

# Print results sorted by cumulative time
stats = pstats.Stats(profiler)
stats.sort_stats("cumulative")
stats.print_stats(30)  # Top 30 functions
```

**Tip:** Filter results to see only your custom module:

```python
stats.print_stats("your_module_name")  # Only functions matching this pattern
```

Your custom `update()`, action callbacks, and any helper methods will appear in the output with call counts and per-call timing. Look for functions with high `tottime` (time spent in the function itself, excluding callees).

---

## Approach 3: Standalone `perf_counter` (Per-Agent, Non-Integrated)

For per-agent timing data without integrating with the built-in profiling system.
Useful when you want to analyze individual agent performance rather than step-level totals.

```python
import time
from pybullet_fleet.agent import Agent


class ProfilingAgent(Agent):
    """Agent subclass with self-contained timing instrumentation."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._timing_data = {"custom_logic": [], "path_update": []}

    def update(self, dt: float) -> bool:
        t0 = time.perf_counter()
        self._do_custom_logic(dt)
        t1 = time.perf_counter()
        self._timing_data["custom_logic"].append((t1 - t0) * 1000)

        t2 = time.perf_counter()
        moved = super().update(dt)
        t3 = time.perf_counter()
        self._timing_data["path_update"].append((t3 - t2) * 1000)

        return moved

    def _do_custom_logic(self, dt: float):
        pass

    def print_timing_summary(self):
        import statistics
        for name, times in self._timing_data.items():
            if times:
                print(f"{name}: mean={statistics.mean(times):.3f}ms, "
                      f"max={max(times):.3f}ms, calls={len(times)}")
```

Usage:

```python
from pybullet_fleet.core_simulation import SimulationParams, MultiRobotSimulationCore
from pybullet_fleet.geometry import Pose

params = SimulationParams(gui=False, target_rtf=0, physics=False)
sim = MultiRobotSimulationCore(params)

# Spawn profiling agents
agents = []
for i in range(100):
    agent = ProfilingAgent.from_urdf(
        "robots/mobile_robot.urdf", sim,
        pose=Pose.from_xyz(i * 2.0, 0, 0),
    )
    agents.append(agent)

# Run simulation (standard workflow)
sim.run_simulation(duration=10.0)

# Print per-agent timing after simulation completes
for agent in agents[:5]:  # Sample
    agent.print_timing_summary()
```

> **Note:** `run_simulation()` is the standard way to run the simulation.
> Inspect results with `print_timing_summary()` after the run completes.
> You can also call `step_once()` directly if you need fine-grained loop control,
> but `run_simulation()` is sufficient for most use cases.

---

## Comparing Custom vs Base Performance

To measure the overhead of your custom logic relative to base `Agent`:

```python
import time
from pybullet_fleet.core_simulation import SimulationParams, MultiRobotSimulationCore
from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.geometry import Pose


def benchmark(agent_class, label, num_agents=500, num_steps=100):
    """Benchmark a specific agent class."""
    params = SimulationParams(gui=False, target_rtf=0, physics=False)
    sim = MultiRobotSimulationCore(params)

    for i in range(num_agents):
        agent = sim.spawn_agent(
            AgentSpawnParams(urdf_path="robots/mobile_robot.urdf"),
            Pose.from_xyz(i * 2.0, 0, 0),
        )
        agent.set_goal_pose(Pose.from_xyz(i * 2.0 + 10, 5, 0))

    # Warm up
    for _ in range(5):
        sim.step_once()

    # Measure
    t0 = time.perf_counter()
    for _ in range(num_steps):
        sim.step_once()
    elapsed = (time.perf_counter() - t0) * 1000  # ms

    avg_step = elapsed / num_steps
    print(f"{label}: {avg_step:.2f} ms/step ({num_agents} agents, {num_steps} steps)")

    sim.cleanup()
    return avg_step


# Compare
base_time = benchmark(Agent, "Base Agent")
# custom_time = benchmark(MyCustomAgent, "Custom Agent")
# overhead = (custom_time - base_time) / base_time * 100
# print(f"Custom overhead: {overhead:.1f}%")
```

---

## Tips

- **Start with Approach 1** (`enable_time_profiling=True` + `run_simulation`) to see if `agent_update` is actually the bottleneck. Don't optimize what isn't slow.
- **Use `target_rtf=0`** for benchmarks — eliminates sleep/sync noise.
- **Use `gui=False`** — rendering adds significant overhead.
- **Warm up** before measuring (3-5 steps) to avoid PyBullet initialization costs.
- **Run multiple iterations** and report mean/median — single measurements are noisy.
- **Profile with realistic conditions** — use the same agent count, collision settings, and goal patterns as production.
