# Benchmark Suite Refactoring Specification

**Date:** 2026-03-15
**Status:** Draft

## Context

The benchmark suite grew organically — `performance_benchmark.py` (mobile agents) and `arm_benchmark.py` were developed independently with duplicated code (system info, warmup, metrics, cleanup) and inconsistent architecture (arm is self-contained with its own sweep; mobile is a worker called by `run_benchmark.py`). Profiling tools only cover mobile agents.

## Decision

Unify the benchmark architecture under a single orchestrator pattern:

1. **Extract shared code** into `benchmark/tools.py`
2. **Rename** `performance_benchmark.py` → `mobile_benchmark.py` for consistency with `arm_benchmark.py`
3. **Convert** `arm_benchmark.py` to the worker pattern (single run → JSON stdout)
4. **Extend** `run_benchmark.py` with `--type arm|mobile` to orchestrate both worker types
5. **Add** arm-specific profiling to `benchmark/profiling/`

## Requirements

- `run_benchmark.py --type arm` drives arm benchmarks with same sweep/repetitions/stats as mobile
- `run_benchmark.py --type mobile` works exactly as `run_benchmark.py` does today (default)
- Both workers use shared helpers from `benchmark/tools.py`
- Arm profiling covers joint update and action pipeline analysis
- No breaking changes to existing CLI when `--type` is omitted (defaults to mobile)

## Constraints

- Workers are called via subprocess (process isolation for clean memory)
- `sys.path.insert` hack must remain (benchmark/ is not a package)
- profiling scripts load `profiling_config.yaml` via `SimulationParams.from_config()`

## Out of Scope

- Config YAML for arm benchmarks (arm params are simple enough to hardcode)
- Refactoring profiling tools to use `tools.py` (separate task)
- Adding `--type` support to profiling tools

## Success Criteria

- [ ] `python benchmark/run_benchmark.py --type arm --sweep 1 10 50 100` runs correctly
- [ ] `python benchmark/run_benchmark.py --sweep 100 500 1000` still works (mobile default)
- [ ] `python benchmark/mobile_benchmark.py --agents 100 --duration 5` outputs valid JSON
- [ ] `python benchmark/arm_benchmark.py --arms 10 --duration 5` outputs valid JSON
- [ ] `python benchmark/profiling/arm_joint_update.py --arms 10 --steps 100` runs correctly
- [ ] No duplicated `get_system_info()` across files
