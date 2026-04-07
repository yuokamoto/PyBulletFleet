# Centralized Default Values Specification

**Date:** 2026-04-05
**Status:** Ready for Implementation

## Context

PyBulletFleet has multiple initialization paths for core classes (`SimulationParams`, `AgentSpawnParams`, `SimObjectSpawnParams`). Default values are duplicated across dataclass field defaults, `from_dict()` fallbacks, and factory method signatures — up to 5× for some parameters. This has already caused a critical mismatch: `SimulationParams.timestep` defaults to `1/240` in the dataclass but `1/10` in `from_dict()`, a 24× difference.

Additionally, users have no lightweight mechanism to override defaults across an entire environment (e.g., `gui=false` for CI, or team-shared velocity limits) without editing YAML files or Python code.

## Decision

Introduce `pybullet_fleet/_defaults.py` as the **single source of truth** for all default parameter values. Dataclass fields and `from_dict()` methods reference this module instead of hardcoding values. Environment variables (`PBF_*`) override defaults at module load time. An optional `.env` file (via python-dotenv, no hard dependency) provides file-based overrides.

## Requirements

- All default values for `SimulationParams`, `AgentSpawnParams`, `SimObjectSpawnParams`, and `ShapeParams` are defined exactly once in `_defaults.py`
- Dataclass fields use direct reference: `timestep: float = _SIM_D["timestep"]`
- All `from_dict()` / `from_yaml()` methods use the same dictionary for `.get()` fallbacks
- Environment variables follow `PBF_{SECTION}_{KEY}` naming (e.g., `PBF_SIMULATION_TIMESTEP`)
- Type coercion handles bool, int, float, str automatically
- `.env.defaults` template committed to repo; `.env` in `.gitignore`
- `reload_defaults()` function exposed for test isolation

## Constraints

- No new hard dependencies (python-dotenv is optional)
- `timestep` default is unified to `0.1` (kinematics-first design)
- Backward compatible: existing YAML configs and explicit Python arguments are unaffected
- Priority order (low→high): `_defaults.py` → `.env` file → shell env vars → YAML config → Python code

## Out of Scope

- Runtime hot-reload of defaults (only applied at module import time)
- GUI for editing defaults
- Validation/schema enforcement in `.env` file
- Changing any existing YAML config file values

## Open Questions

- [ ] None remaining

## Success Criteria

- [ ] `SimulationParams()` and `SimulationParams.from_dict({})` produce identical values for all fields
- [ ] `PBF_SIMULATION_GUI=false` in environment produces `gui=False` in `SimulationParams()`
- [ ] No default value appears in more than one source file (except the reference to `_defaults.py`)
- [ ] All existing tests pass without modification (except timestep-sensitive ones that need explicit values)
- [ ] `make verify` passes

## References

- [core_simulation.py](../../../pybullet_fleet/core_simulation.py) — SimulationParams
- [agent.py](../../../pybullet_fleet/agent.py) — AgentSpawnParams, Agent factory methods
- [sim_object.py](../../../pybullet_fleet/sim_object.py) — SimObjectSpawnParams, ShapeParams
