# Benchmark Suite Refactoring - Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Unify benchmark architecture with shared tools, consistent naming, single orchestrator, and arm profiling.

**Architecture:** Worker + Orchestrator pattern. `run_benchmark.py` dispatches either `mobile_benchmark.py` or `arm_benchmark.py` via subprocess. Shared helpers in `tools.py`.

**Tech Stack:** Python 3.8+, psutil, tracemalloc, pybullet, yaml

---

### Task 1: Create `benchmark/tools.py` (SERIAL)

**Files:**
- Create: `benchmark/tools.py`

**Step 1: Create tools.py with shared helpers**

Extract from `performance_benchmark.py` and `arm_benchmark.py`:

```python
#!/usr/bin/env python3
"""Shared benchmark and profiling helpers."""
import gc
import os
import platform
import subprocess
import time
from typing import Any, Dict

import psutil
import tracemalloc
import pybullet as p


def get_system_info() -> Dict[str, Any]:
    """Collect system info for reproducibility."""
    info = {
        "platform": platform.system(),
        "platform_release": platform.release(),
        "platform_version": platform.version(),
        "architecture": platform.machine(),
        "processor": platform.processor(),
        "python_version": platform.python_version(),
        "cpu_count": psutil.cpu_count(logical=False),
        "cpu_count_logical": psutil.cpu_count(logical=True),
        "total_memory_gb": round(psutil.virtual_memory().total / (1024**3), 2),
    }
    if platform.system() == "Linux":
        try:
            result = subprocess.run(["lscpu"], capture_output=True, text=True, timeout=2)
            for line in result.stdout.split("\n"):
                if "Model name:" in line:
                    info["cpu_model"] = line.split(":", 1)[1].strip()
                elif "CPU max MHz:" in line:
                    try:
                        info["cpu_max_mhz"] = float(line.split(":", 1)[1].strip())
                    except (ValueError, IndexError):
                        pass
        except (FileNotFoundError, IOError):
            pass
    return info


def get_memory_info() -> Dict[str, float]:
    """Get RSS + tracemalloc memory measurements."""
    process = psutil.Process()
    mem = process.memory_info()
    py_traced_mb = 0.0
    if tracemalloc.is_tracing():
        current, _peak = tracemalloc.get_traced_memory()
        py_traced_mb = current / 1024 / 1024
    rss_mb = mem.rss / 1024 / 1024
    return {
        "rss_mb": rss_mb,
        "vms_mb": mem.vms / 1024 / 1024,
        "py_traced_mb": py_traced_mb,
        "rss_minus_tracemalloc_mb": rss_mb - py_traced_mb,
    }


def force_cleanup():
    """Best-effort cleanup inside a process."""
    gc.collect()
    gc.collect()
    gc.collect()
    time.sleep(0.05)


def cpu_time_s(process: psutil.Process) -> float:
    """Return user+sys CPU time seconds."""
    t = process.cpu_times()
    return float(t.user + t.system)


def ensure_disconnected():
    """Disconnect PyBullet if connected."""
    if p.isConnected():
        p.disconnect()


def warmup_steps(sim_core, n: int = 10):
    """Run n warmup steps to stabilize timing."""
    for _ in range(n):
        sim_core.step_once()
```

**Step 2: Verify import works**

Run: `cd /home/rapyuta/rr_sim_evaluation/PyBulletFleet && python -c "from benchmark.tools import get_system_info; print(get_system_info()['platform'])"`
Expected: `Linux`

---

### Task 2: Rename `performance_benchmark.py` → `mobile_benchmark.py` (SERIAL, depends on T1)

**Files:**
- Rename: `benchmark/performance_benchmark.py` → `benchmark/mobile_benchmark.py`
- Modify: `benchmark/mobile_benchmark.py` (use tools.py)

**Step 1: Rename file**

```bash
git mv benchmark/performance_benchmark.py benchmark/mobile_benchmark.py
```

**Step 2: Update mobile_benchmark.py to use tools.py**

Replace local `get_system_info`, `get_memory_info`, `force_cleanup`, `cpu_time_s` with imports from `benchmark.tools`. Keep `load_config` and `run_benchmark` function as-is (they are mobile-specific).

**Step 3: Verify worker still outputs valid JSON**

Run: `python benchmark/mobile_benchmark.py --agents 100 --duration 2`
Expected: Valid JSON to stdout

---

### Task 3: Refactor `arm_benchmark.py` to worker pattern (SERIAL, depends on T1)

**Files:**
- Modify: `benchmark/arm_benchmark.py`

**Step 1: Refactor to worker mode**

The worker function `run_single_bench()` stays, but:
- Import shared helpers from `benchmark.tools`
- Remove self-contained `get_system_info`
- Add `get_memory_info` + `cpu_time_s` + `tracemalloc` (matching mobile worker)
- When called with a single config (no `--sweep`, not `--mode both`), output JSON to stdout
- Keep `--sweep` and `--mode both` for direct CLI use (prints table + saves file)

Output JSON keys must include `system_info`, `mem_spawn_mb`, `mem_total_mb`, `simulation_cpu_s` to match mobile worker format.

**Step 2: Verify worker mode**

Run: `python benchmark/arm_benchmark.py --arms 10 --duration 2 --mode physics`
Expected: Single JSON dict to stdout

Run: `python benchmark/arm_benchmark.py --arms 10 --duration 2 --mode both`
Expected: Table output + JSON file (backwards compat)

---

### Task 4: Update `run_benchmark.py` with `--type arm|mobile` (SERIAL, depends on T2, T3)

**Files:**
- Modify: `benchmark/run_benchmark.py`

**Step 1: Add --type argument and arm-specific options**

```python
parser.add_argument("--type", choices=["mobile", "arm"], default="mobile",
                    help="Benchmark type: mobile agents or arm robots")
parser.add_argument("--arms", type=int, default=None,
                    help="Number of arms (--type arm; aliases --agents)")
parser.add_argument("--mode", choices=["physics", "kinematic", "both"],
                    default="both", help="Arm joint mode (--type arm only)")
parser.add_argument("--timestep", type=float, default=0.01,
                    help="Simulation timestep (--type arm only)")
```

**Step 2: Update run_worker() to select script**

```python
def run_worker(num_agents, duration, gui=False, config_path=None,
               scenario=None, benchmark_type="mobile", **arm_kwargs):
    if benchmark_type == "arm":
        script = "arm_benchmark.py"
        cmd = [sys.executable, os.path.join(os.path.dirname(__file__), script),
               "--arms", str(num_agents),
               "--duration", str(duration),
               "--mode", arm_kwargs.get("mode", "physics"),
               "--timestep", str(arm_kwargs.get("timestep", 0.01))]
    else:
        script = "mobile_benchmark.py"
        cmd = [sys.executable, os.path.join(os.path.dirname(__file__), script),
               "--agents", str(num_agents),
               "--duration", str(duration)]
        if gui: cmd.append("--gui")
        if config_path: cmd.extend(["--config", config_path])
        if scenario: cmd.extend(["--scenario", scenario])
    ...
```

**Step 3: Update main() to route --type**

When `--type arm`:
- If `--sweep`, iterate arm counts — for `--mode both`, run physics + kinematic for each count
- If no `--sweep`, run single config
- `--agents` is aliased by `--arms`

**Step 4: Verify**

Run: `python benchmark/run_benchmark.py --type arm --arms 10 --duration 2 --repetitions 1 --mode physics`
Expected: Formatted results output

Run: `python benchmark/run_benchmark.py --type arm --sweep 1 10 --duration 2 --repetitions 1`
Expected: Sweep summary table

Run: `python benchmark/run_benchmark.py --sweep 100 --duration 2 --repetitions 1`
Expected: Mobile sweep (backwards compat)

---

### Task 5: Add arm profiling `profiling/arm_joint_update.py` (PARALLEL with T4)

**Files:**
- Create: `benchmark/profiling/arm_joint_update.py`

**Step 1: Create arm joint profiling script**

Follow structure of `agent_update.py`. Three analysis methods:

1. **builtin** — Uses `step_once(return_profiling=True)` with arm robots running JointAction
2. **cprofile** — cProfile of `step_once()` with arm robots
3. **scaling** — Measure step time as arm count increases (1, 5, 10, 25, 50)

**Step 2: Verify**

Run: `python benchmark/profiling/arm_joint_update.py --arms 5 --steps 50`
Expected: Step breakdown output with agent_update, collision_check timings

Run: `python benchmark/profiling/arm_joint_update.py --arms 5 --steps 50 --test scaling`
Expected: Scaling table

---

### Task 6: Update README and verify (SERIAL, depends on T1–T5)

**Files:**
- Modify: `benchmark/README.md`

**Step 1: Update directory structure, file descriptions, CLI examples**

Key changes:
- `performance_benchmark.py` → `mobile_benchmark.py` in all references
- Add `tools.py` to directory tree
- Add `arm_joint_update.py` to profiling section
- Update CLI examples with `--type arm`
- Add arm benchmark results section (placeholder for future data)

**Step 2: Full verification**

```bash
# Mobile (default)
python benchmark/run_benchmark.py --sweep 100 --duration 2 --repetitions 1
# Arm
python benchmark/run_benchmark.py --type arm --sweep 1 10 --duration 2 --repetitions 1
# Arm profiling
python benchmark/profiling/arm_joint_update.py --arms 5 --steps 50
```
