# Centralized Default Values — Agent Specification

**Status:** Ready for Implementation

## Requirements

### Functional
- Create `pybullet_fleet/_defaults.py` containing all default values for `SimulationParams`, `AgentSpawnParams`, `SimObjectSpawnParams`, and `ShapeParams`
- Environment variables `PBF_{SECTION}_{KEY}` override defaults at module load time
- Optional python-dotenv support (try/except import, no hard dependency)
- `reload_defaults()` public function for test isolation
- Unify `timestep` default to `0.1` across all paths

### Non-Functional
- Zero new hard dependencies
- All existing tests continue to pass
- `make verify` passes
- Coverage ≥ 75%

## Constraints

- Priority order: `_defaults.py` < `.env` < shell env < YAML config < Python explicit
- Dataclass fields use direct reference style: `field = _SIM_D["key"]` (not `default_factory`)
- Mutable defaults (`dict`, `list`) remain as `None` with `__post_init__` initialization (no change)
- `PBF_` prefix is mandatory to avoid collisions with other tools

## Approach

### Phase 1: Create `_defaults.py`
### Phase 2: Refactor `SimulationParams` to reference `_defaults.py`
### Phase 3: Refactor `AgentSpawnParams` and `Agent` factory methods
### Phase 4: Refactor `SimObjectSpawnParams` and `ShapeParams`
### Phase 5: Add `.env.defaults` template and `.gitignore` entry
### Phase 6: Write tests
### Phase 7: Fix any broken tests from `timestep` unification

## Design

### Architecture

```
.env.defaults          (template, committed)
.env                   (user overrides, gitignored)
    ↓
pybullet_fleet/_defaults.py     (single source of truth)
    ↓ imported by
SimulationParams       (core_simulation.py)
AgentSpawnParams       (agent.py)
Agent.__init__         (agent.py)
Agent.from_mesh        (agent.py)
Agent.from_urdf        (agent.py)
SimObjectSpawnParams   (sim_object.py)
ShapeParams            (sim_object.py)
```

### Key Components

| Component | Responsibility | Location |
|-----------|---------------|----------|
| `_defaults.py` | Define + resolve all defaults | `pybullet_fleet/_defaults.py` |
| `SimulationParams` | Reference `_defaults.SIMULATION` | `pybullet_fleet/core_simulation.py` |
| `AgentSpawnParams` | Reference `_defaults.AGENT` | `pybullet_fleet/agent.py` |
| `SimObjectSpawnParams` | Reference `_defaults.SIM_OBJECT` | `pybullet_fleet/sim_object.py` |
| `ShapeParams` | Reference `_defaults.SHAPE` | `pybullet_fleet/sim_object.py` |
| `.env.defaults` | Template for user overrides | `.env.defaults` |
| `test_defaults.py` | Completeness + env override tests | `tests/test_defaults.py` |

### `_defaults.py` Structure

```python
"""Single source of truth for all default parameter values.

Env override: PBF_{SECTION}_{KEY} (e.g. PBF_SIMULATION_TIMESTEP=0.1)
.env file:    Loaded automatically if python-dotenv is installed (optional).
"""
import os
from typing import Any, Dict

try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass

_DEFAULTS: Dict[str, Dict[str, Any]] = {
    "simulation": {
        "target_rtf": 1.0,
        "timestep": 0.1,
        "duration": 0,
        "gui": True,
        "physics": False,
        "monitor": True,
        "enable_monitor_gui": True,
        "collision_check_frequency": None,  # None = not overridable via env
        "log_level": "warn",
        "max_steps_per_frame": 10,
        "gui_min_fps": 30,
        "enable_collision_shapes": False,
        "enable_structure_transparency": False,
        "enable_shadows": True,
        "enable_gui_panel": False,
        "ignore_static_collision": True,
        "enable_time_profiling": False,
        "profiling_interval": 100,
        "enable_memory_profiling": False,
        "enable_collision_color_change": False,
        "spatial_hash_cell_size_mode": "auto_initial",
        "spatial_hash_cell_size": None,
        "collision_detection_method": None,  # Auto-selected in __post_init__
        "collision_margin": 0.02,
        "multi_cell_threshold": 1.5,
        "enable_floor": True,
        "window_width": 0,
        "window_height": 0,
        "monitor_width": 200,
        "monitor_height": 290,
        "monitor_x": -1,
        "monitor_y": -1,
    },
    "agent": {
        "max_linear_vel": 2.0,
        "max_linear_accel": 5.0,
        "max_angular_vel": 3.0,
        "max_angular_accel": 10.0,
        "motion_mode": "differential",
        "use_fixed_base": False,
    },
    "sim_object": {
        "mass": 0.0,
        "pickable": True,
        "collision_mode": "normal_3d",
    },
    "shape": {
        "mesh_scale": [1.0, 1.0, 1.0],
        "half_extents": [0.5, 0.5, 0.5],
        "radius": 0.5,
        "height": 1.0,
        "rgba_color": [0.8, 0.8, 0.8, 1.0],
    },
}

_TYPE_MAP = {
    bool: lambda v: v.lower() in ("true", "1", "yes"),
    int: int,
    float: float,
    str: str,
}

def _apply_env_overrides() -> None:
    for section, params in _DEFAULTS.items():
        for key, default_val in params.items():
            if default_val is None:
                continue  # Skip None defaults (not env-overridable)
            env_key = f"PBF_{section.upper()}_{key.upper()}"
            env_val = os.environ.get(env_key)
            if env_val is not None:
                coerce = _TYPE_MAP.get(type(default_val), str)
                params[key] = coerce(env_val)

_apply_env_overrides()

def get(section: str, key: str) -> Any:
    return _DEFAULTS[section][key]

def reload_defaults() -> None:
    """Re-apply env overrides. Call after monkeypatching env vars in tests."""
    _apply_env_overrides()

SIMULATION = _DEFAULTS["simulation"]
AGENT = _DEFAULTS["agent"]
SIM_OBJECT = _DEFAULTS["sim_object"]
SHAPE = _DEFAULTS["shape"]
```

### Dataclass Reference Pattern

```python
# core_simulation.py
from pybullet_fleet._defaults import SIMULATION as _SIM_D

@dataclass
class SimulationParams:
    target_rtf: float = _SIM_D["target_rtf"]
    timestep: float = _SIM_D["timestep"]           # 0.1 (unified)
    gui: bool = _SIM_D["gui"]
    # ... all fields reference _SIM_D
    # Mutable defaults stay as None → __post_init__
    camera_config: Optional[Dict[str, Any]] = None
    model_paths: Optional[List[str]] = None
```

### `from_dict()` Reference Pattern

```python
@classmethod
def from_dict(cls, config: Dict[str, Any]) -> "SimulationParams":
    return cls(
        timestep=config.get("timestep", _SIM_D["timestep"]),
        gui=config.get("gui", _SIM_D["gui"]),
        # ...
    )
```

### Agent Factory Methods Pattern

```python
# agent.py
from pybullet_fleet._defaults import AGENT as _AGT_D

@dataclass
class AgentSpawnParams:
    max_linear_vel: float = _AGT_D["max_linear_vel"]
    # ...

class Agent:
    def __init__(self, ..., max_linear_vel=_AGT_D["max_linear_vel"], ...):
        ...

    @classmethod
    def from_mesh(cls, ..., max_linear_vel=_AGT_D["max_linear_vel"], ...):
        ...

    @classmethod
    def from_urdf(cls, ..., max_linear_vel=_AGT_D["max_linear_vel"], ...):
        ...
```

### `from_dict` collision_detection_method Fix

Currently `from_dict()` constructs the enum explicitly, bypassing `__post_init__` auto-selection. Change to:

```python
# Before (bypasses __post_init__):
collision_detection_method=CollisionDetectionMethod(config.get("collision_detection_method", ...))

# After (let __post_init__ auto-select when not in config):
collision_detection_method=(
    CollisionDetectionMethod(config["collision_detection_method"])
    if "collision_detection_method" in config
    else None  # __post_init__ will auto-select based on physics mode
),
```

### `.env.defaults` Template

```env
# PyBulletFleet Default Configuration
# Copy to .env and uncomment lines you want to override.
# See pybullet_fleet/_defaults.py for all available parameters.
#
# Naming: PBF_{SECTION}_{KEY} (case-insensitive for bool values)
# Install python-dotenv for .env file support: pip install python-dotenv

# --- Simulation ---
# PBF_SIMULATION_TIMESTEP=0.1
# PBF_SIMULATION_GUI=true
# PBF_SIMULATION_PHYSICS=false
# PBF_SIMULATION_MONITOR=true
# PBF_SIMULATION_LOG_LEVEL=warn
# PBF_SIMULATION_WINDOW_WIDTH=0
# PBF_SIMULATION_WINDOW_HEIGHT=0
# PBF_SIMULATION_MONITOR_WIDTH=200
# PBF_SIMULATION_MONITOR_HEIGHT=290
# PBF_SIMULATION_MONITOR_X=-1
# PBF_SIMULATION_MONITOR_Y=-1

# --- Agent ---
# PBF_AGENT_MAX_LINEAR_VEL=2.0
# PBF_AGENT_MAX_LINEAR_ACCEL=5.0
# PBF_AGENT_MAX_ANGULAR_VEL=3.0
# PBF_AGENT_MAX_ANGULAR_ACCEL=10.0

# --- SimObject ---
# PBF_SIM_OBJECT_MASS=0.0
# PBF_SIM_OBJECT_PICKABLE=true
```

### Test Structure

```python
# tests/test_defaults.py

class TestDefaultsCompleteness:
    """Verify _defaults.py has entries for every dataclass field."""

    def test_simulation_params_fields_covered(self):
        """Every non-mutable SimulationParams field has a _defaults entry."""
        from dataclasses import fields
        from pybullet_fleet._defaults import SIMULATION
        skip = {"camera_config", "model_paths"}  # Mutable → None
        for f in fields(SimulationParams):
            if f.name not in skip:
                assert f.name in SIMULATION, f"Missing default for SimulationParams.{f.name}"

    def test_agent_params_fields_covered(self):
        ...

    def test_sim_object_params_fields_covered(self):
        ...


class TestEnvOverride:
    """Test PBF_* environment variable overrides."""

    def test_float_override(self, monkeypatch):
        monkeypatch.setenv("PBF_SIMULATION_TIMESTEP", "0.05")
        from pybullet_fleet._defaults import reload_defaults, SIMULATION
        reload_defaults()
        assert SIMULATION["timestep"] == 0.05

    def test_bool_override(self, monkeypatch):
        monkeypatch.setenv("PBF_SIMULATION_GUI", "false")
        from pybullet_fleet._defaults import reload_defaults, SIMULATION
        reload_defaults()
        assert SIMULATION["gui"] is False

    def test_int_override(self, monkeypatch):
        monkeypatch.setenv("PBF_SIMULATION_MONITOR_WIDTH", "300")
        from pybullet_fleet._defaults import reload_defaults, SIMULATION
        reload_defaults()
        assert SIMULATION["monitor_width"] == 300


class TestConsistency:
    """Verify all initialization paths produce identical defaults."""

    def test_dataclass_matches_from_dict(self):
        """SimulationParams() and SimulationParams.from_dict({}) match."""
        direct = SimulationParams()
        from_dict = SimulationParams.from_dict({})
        for field in fields(SimulationParams):
            assert getattr(direct, field.name) == getattr(from_dict, field.name), \
                f"Mismatch on {field.name}"

    def test_yaml_overrides_env(self, monkeypatch):
        monkeypatch.setenv("PBF_SIMULATION_TIMESTEP", "0.05")
        from pybullet_fleet._defaults import reload_defaults
        reload_defaults()
        params = SimulationParams.from_dict({"timestep": 0.01})
        assert params.timestep == 0.01  # YAML wins
```

## File References

Files the plan agent MUST read before planning:

- `pybullet_fleet/core_simulation.py:47-170` — SimulationParams dataclass + from_dict
- `pybullet_fleet/agent.py:60-170` — AgentSpawnParams dataclass + from_dict
- `pybullet_fleet/agent.py:250-280` — Agent.__init__ defaults
- `pybullet_fleet/agent.py:650-670` — Agent.from_mesh defaults
- `pybullet_fleet/agent.py:750-770` — Agent.from_urdf defaults
- `pybullet_fleet/sim_object.py:60-110` — ShapeParams dataclass + from_dict
- `pybullet_fleet/sim_object.py:140-230` — SimObjectSpawnParams + from_dict
- `pybullet_fleet/sim_object.py:260-280` — SimObject.__init__ defaults
- `tests/conftest.py` — MockSimCore and fixtures
- `tests/test_core_simulation.py` — existing SimulationParams tests
- `.gitignore` — add `.env` entry

## Success Criteria

- [ ] `SimulationParams()` and `SimulationParams.from_dict({})` produce identical values for all fields
- [ ] `PBF_SIMULATION_GUI=false` in env → `SimulationParams().gui is False`
- [ ] No duplicate default values remain in dataclass fields or from_dict methods
- [ ] Agent's 5 initialization paths all reference `_defaults.AGENT`
- [ ] `test_defaults.py` validates completeness (every field has a _defaults entry)
- [ ] All existing tests pass (fix timestep-sensitive ones with explicit values)
- [ ] `make verify` passes (lint + tests + coverage ≥ 75%)
