# Benchmark Suite Refactoring - Agent Specification

## Requirements

### Functional
- Extract shared benchmark helpers to `benchmark/tools.py`
- Rename `performance_benchmark.py` → `mobile_benchmark.py`
- Convert `arm_benchmark.py` to worker pattern (single run → JSON stdout)
- Add `--type arm|mobile` to `run_benchmark.py`; default = mobile
- Add arm profiling script `profiling/arm_joint_update.py`

### Non-Functional
- No regression in `run_benchmark.py` default behavior
- Process isolation preserved (subprocess per worker)
- JSON output format compatible between arm and mobile

## Constraints
- `sys.path.insert(0, ...)` required in all entry-point scripts
- Workers must be callable as standalone scripts AND via subprocess
- `profiling_config.yaml` is flat-key format for `SimulationParams.from_config()`

## Approach

**Phase 1:** Extract `benchmark/tools.py` with shared code
**Phase 2:** Rename and update mobile worker
**Phase 3:** Refactor arm worker to subprocess-compatible form
**Phase 4:** Extend orchestrator with `--type` routing
**Phase 5:** Add arm profiling
**Phase 6:** Update documentation

## Design

### Architecture

```
run_benchmark.py  (orchestrator)
   --type mobile → calls mobile_benchmark.py via subprocess
   --type arm    → calls arm_benchmark.py via subprocess
                    (passes --arms, --duration, --mode, --timestep)

benchmark/tools.py  (shared helpers)
   get_system_info(), force_cleanup(), cpu_time_s(), get_memory_info()
```

### Key Components

| Component | Responsibility | Location |
|-----------|---------------|----------|
| `tools.py` | Shared helpers: system info, memory, cleanup, warmup | `benchmark/tools.py` |
| `mobile_benchmark.py` | Worker: mobile agent benchmark (renamed) | `benchmark/mobile_benchmark.py` |
| `arm_benchmark.py` | Worker: arm robot benchmark (refactored) | `benchmark/arm_benchmark.py` |
| `run_benchmark.py` | Orchestrator: --type routing, sweep, stats | `benchmark/run_benchmark.py` |
| `arm_joint_update.py` | Profiling: arm joint update breakdown | `benchmark/profiling/arm_joint_update.py` |

### tools.py API

```python
# benchmark/tools.py
"""Shared benchmark helpers."""

def get_system_info() -> Dict[str, Any]:
    """Collect system info (CPU, memory, OS) for reproducibility.
    Merges fields from both arm_benchmark and performance_benchmark."""

def get_memory_info() -> Dict[str, float]:
    """Get RSS + tracemalloc memory measurements."""

def force_cleanup():
    """Triple gc.collect + sleep for clean memory state."""

def cpu_time_s(process: psutil.Process) -> float:
    """Return user+sys CPU time seconds."""

def ensure_disconnected():
    """Disconnect PyBullet if connected."""

def warmup_steps(sim_core, n: int = 10):
    """Run n warmup steps."""
```

### arm_benchmark.py Worker Interface

Current self-contained arm_benchmark.py will be refactored to match mobile_benchmark.py pattern:

```bash
# Worker mode (called by run_benchmark.py):
python benchmark/arm_benchmark.py --arms 10 --duration 5 --mode physics --timestep 0.01
# Outputs: single JSON dict to stdout

# Direct mode (backwards compat - still works standalone):
python benchmark/arm_benchmark.py --arms 10 --duration 5 --mode both --sweep 1 10 50
# When --sweep or --mode=both: runs multiple configs, prints table + saves JSON
```

Worker JSON output format:
```json
{
  "num_arms": 10,
  "total_joints": 40,
  "mass": null,
  "mode": "physics",
  "physics_engine": true,
  "timestep": 0.01,
  "duration_s": 5.0,
  "spawn_time_s": 0.123,
  "simulation_wall_s": 2.456,
  "simulation_cpu_s": 2.100,
  "actual_steps": 500,
  "real_time_factor": 2.04,
  "avg_step_time_ms": 4.912,
  "mem_spawn_mb": {"rss_mb": 1.2, "py_traced_mb": 0.5},
  "mem_total_mb": {"rss_mb": 2.3, "py_traced_mb": 0.8},
  "system_info": { ... }
}
```

### run_benchmark.py --type Routing

```python
# New argument
parser.add_argument("--type", choices=["mobile", "arm"], default="mobile")

# Arm-specific arguments (only used when --type=arm)
parser.add_argument("--arms", type=int, default=None,
                    help="Number of arms (--type arm only; aliases --agents)")
parser.add_argument("--mode", choices=["physics", "kinematic", "both"],
                    default="both", help="Arm mode (--type arm only)")
parser.add_argument("--timestep", type=float, default=0.01,
                    help="Timestep (--type arm only)")
```

Worker selection in `run_worker()`:
```python
if benchmark_type == "arm":
    script = "arm_benchmark.py"
    cmd = [sys.executable, script_path, "--arms", str(num_agents), ...]
else:
    script = "mobile_benchmark.py"
    cmd = [sys.executable, script_path, "--agents", str(num_agents), ...]
```

### arm_joint_update.py Profiling

Mirrors `agent_update.py` structure but for arm robots:

```python
# Methods:
# 1. cProfile of agent.update() with JointAction running
# 2. Manual timing of _update_kinematic_joints() vs physics motor control
# 3. Scaling: measure per-joint cost as arm count increases

# CLI:
python benchmark/profiling/arm_joint_update.py --arms 10 --steps 100
python benchmark/profiling/arm_joint_update.py --arms 10 --test cprofile
python benchmark/profiling/arm_joint_update.py --arms 10 --test scaling
```

## File References

Files to read before implementation:
- `benchmark/arm_benchmark.py` — current arm benchmark (self-contained)
- `benchmark/performance_benchmark.py` — current mobile worker (subprocess pattern)
- `benchmark/run_benchmark.py` — orchestrator with sweep/compare/stats
- `benchmark/profiling/agent_update.py` — agent profiling pattern to follow
- `benchmark/profiling/simulation_profiler.py` — component breakdown pattern
- `benchmark/profiling/profiling_config.yaml` — shared profiling config
- `benchmark/README.md` — documentation to update

## Success Criteria

- [ ] `benchmark/tools.py` exists with get_system_info, get_memory_info, force_cleanup, cpu_time_s, ensure_disconnected, warmup_steps
- [ ] `benchmark/mobile_benchmark.py` exists and works (renamed from performance_benchmark.py)
- [ ] `benchmark/arm_benchmark.py` outputs single JSON to stdout in worker mode
- [ ] `run_benchmark.py --type arm --sweep 1 10 50 100` works
- [ ] `run_benchmark.py --sweep 100 500 1000` still works (mobile default)
- [ ] `benchmark/profiling/arm_joint_update.py` runs successfully
- [ ] `benchmark/README.md` reflects new structure
