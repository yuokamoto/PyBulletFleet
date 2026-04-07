# Centralized Defaults — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Eliminate default value duplication by centralizing all defaults in `_defaults.py`, with env-var override support.

**Architecture:** New `_defaults.py` module defines all defaults as nested dicts. Dataclass fields and `from_dict()` methods reference this module instead of hardcoding values. `PBF_*` env vars override at import time. Optional python-dotenv support.

**Tech Stack:** Python stdlib only (python-dotenv optional).

---

### Task 1: Create `_defaults.py` (SERIAL — foundation for all other tasks)

**Files:**
- Create: `pybullet_fleet/_defaults.py`

**Step 1: Create the defaults module**

```python
"""Single source of truth for all default parameter values.

Environment variables ``PBF_{SECTION}_{KEY}`` override defaults at
module load time (e.g. ``PBF_SIMULATION_TIMESTEP=0.05``).

If ``python-dotenv`` is installed, a ``.env`` file in the working
directory is loaded automatically.  This is optional — the module
works without it.
"""

import os
from typing import Any, Dict

# Optional: load .env if python-dotenv is installed (no hard dependency)
try:
    from dotenv import load_dotenv

    load_dotenv()
except ImportError:
    pass

# ---------------------------------------------------------------------------
# Raw defaults — the ONLY place default values are defined
# ---------------------------------------------------------------------------

_DEFAULTS: Dict[str, Dict[str, Any]] = {
    "simulation": {
        "target_rtf": 1.0,
        "timestep": 0.1,
        "duration": 0,
        "gui": True,
        "physics": False,
        "monitor": True,
        "enable_monitor_gui": True,
        "log_level": "warn",
        "max_steps_per_frame": 10,
        "gui_min_fps": 30,
        # Visualizer settings
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
        "collision_margin": 0.02,
        "multi_cell_threshold": 1.5,
        "enable_floor": True,
        # Window size
        "window_width": 0,
        "window_height": 0,
        # Monitor window
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
        "radius": 0.5,
        "height": 1.0,
    },
}

# ---------------------------------------------------------------------------
# Type coercion for env var string → Python type
# ---------------------------------------------------------------------------

_TYPE_MAP = {
    bool: lambda v: v.lower() in ("true", "1", "yes"),
    int: int,
    float: float,
    str: str,
}


def _apply_env_overrides() -> None:
    """Apply ``PBF_{SECTION}_{KEY}`` env-var overrides to ``_DEFAULTS``."""
    for section, params in _DEFAULTS.items():
        for key, default_val in params.items():
            if default_val is None:
                continue  # None-typed fields are not env-overridable
            env_key = f"PBF_{section.upper()}_{key.upper()}"
            env_val = os.environ.get(env_key)
            if env_val is not None:
                coerce = _TYPE_MAP.get(type(default_val), str)
                params[key] = coerce(env_val)


_apply_env_overrides()


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def get(section: str, key: str) -> Any:
    """Return the (possibly env-overridden) default for *section*.*key*."""
    return _DEFAULTS[section][key]


def reload_defaults() -> None:
    """Re-apply env overrides.  Useful after ``monkeypatch.setenv`` in tests."""
    _apply_env_overrides()


#: Convenience accessors — import these in consuming modules.
SIMULATION: Dict[str, Any] = _DEFAULTS["simulation"]
AGENT: Dict[str, Any] = _DEFAULTS["agent"]
SIM_OBJECT: Dict[str, Any] = _DEFAULTS["sim_object"]
SHAPE: Dict[str, Any] = _DEFAULTS["shape"]
```

**Note on ShapeParams:** `mesh_scale`, `half_extents`, `rgba_color` are mutable lists.
They stay as `field(default_factory=...)` in the dataclass (not moved to `_defaults.py`).
Only the immutable scalars (`radius`, `height`) go into `_defaults.py`.
The `from_dict()` hardcoded list defaults are fine as-is since they are always fresh copies.

**Step 2: Verify module loads**

Run: `cd /home/rapyuta/rr_sim_evaluation/PyBulletFleet && python -c "from pybullet_fleet._defaults import SIMULATION, AGENT; print(SIMULATION['timestep'], AGENT['max_linear_vel'])"`
Expected: `0.1 2.0`

**Step 3: Commit**

```bash
git add pybullet_fleet/_defaults.py
git commit -m "feat: add _defaults.py — single source of truth for default values"
```

---

### Task 2: Write tests for `_defaults.py` (SERIAL — before refactoring)

**Files:**
- Create: `tests/test_defaults.py`

**Step 1: Write test file**

```python
"""Tests for pybullet_fleet._defaults — centralized default values."""

import importlib
from dataclasses import fields

import pytest


class TestDefaultsCompleteness:
    """Verify _defaults has entries for every non-mutable dataclass field."""

    def test_simulation_params_fields_covered(self):
        from pybullet_fleet._defaults import SIMULATION
        from pybullet_fleet.core_simulation import SimulationParams

        # Fields that are mutable (None in dataclass, resolved in __post_init__)
        # or enum-typed (stored as string in _defaults, enum in dataclass)
        skip = {
            "camera_config",
            "model_paths",
            "collision_check_frequency",
            "spatial_hash_cell_size",
            "collision_detection_method",
            "spatial_hash_cell_size_mode",
        }
        for f in fields(SimulationParams):
            if f.name not in skip:
                assert f.name in SIMULATION, f"Missing default for SimulationParams.{f.name}"

    def test_agent_params_fields_covered(self):
        from pybullet_fleet._defaults import AGENT
        from pybullet_fleet.agent import AgentSpawnParams

        skip = {"urdf_path", "initial_pose", "ik_params", "controller_config", "collision_mode"}
        for f in fields(AgentSpawnParams):
            if f.name not in skip:
                assert f.name in AGENT, f"Missing default for AgentSpawnParams.{f.name}"

    def test_sim_object_params_fields_covered(self):
        from pybullet_fleet._defaults import SIM_OBJECT
        from pybullet_fleet.sim_object import SimObjectSpawnParams

        skip = {
            "visual_shape",
            "collision_shape",
            "initial_pose",
            "name",
            "visual_frame_pose",
            "collision_frame_pose",
            "user_data",
        }
        for f in fields(SimObjectSpawnParams):
            if f.name not in skip:
                assert f.name in SIM_OBJECT, f"Missing default for SimObjectSpawnParams.{f.name}"


class TestEnvOverride:
    """Test PBF_* environment variable overrides."""

    def _reload(self):
        """Force re-evaluation of env overrides."""
        import pybullet_fleet._defaults as mod

        mod.reload_defaults()
        return mod

    def test_float_override(self, monkeypatch):
        monkeypatch.setenv("PBF_SIMULATION_TIMESTEP", "0.05")
        mod = self._reload()
        assert mod.SIMULATION["timestep"] == 0.05

    def test_bool_override_false(self, monkeypatch):
        monkeypatch.setenv("PBF_SIMULATION_GUI", "false")
        mod = self._reload()
        assert mod.SIMULATION["gui"] is False

    def test_bool_override_true(self, monkeypatch):
        monkeypatch.setenv("PBF_SIMULATION_GUI", "true")
        mod = self._reload()
        assert mod.SIMULATION["gui"] is True

    def test_int_override(self, monkeypatch):
        monkeypatch.setenv("PBF_SIMULATION_MONITOR_WIDTH", "300")
        mod = self._reload()
        assert mod.SIMULATION["monitor_width"] == 300

    def test_string_override(self, monkeypatch):
        monkeypatch.setenv("PBF_SIMULATION_LOG_LEVEL", "debug")
        mod = self._reload()
        assert mod.SIMULATION["log_level"] == "debug"

    def test_none_fields_not_overridable(self, monkeypatch):
        """Fields with None default should not be affected by env vars."""
        monkeypatch.setenv("PBF_SIMULATION_COLLISION_CHECK_FREQUENCY", "10")
        mod = self._reload()
        # collision_check_frequency is not in _DEFAULTS (None), so unchanged
        assert "collision_check_frequency" not in mod.SIMULATION


class TestConsistency:
    """Verify all initialization paths produce identical defaults."""

    def test_dataclass_matches_from_dict(self):
        """SimulationParams() and SimulationParams.from_dict({}) must match."""
        from pybullet_fleet.core_simulation import SimulationParams

        direct = SimulationParams(gui=False, monitor=False)
        from_dict = SimulationParams.from_dict({"gui": False, "monitor": False})
        for f in fields(SimulationParams):
            val_direct = getattr(direct, f.name)
            val_from_dict = getattr(from_dict, f.name)
            assert val_direct == val_from_dict, (
                f"SimulationParams.{f.name}: direct={val_direct!r} vs from_dict={val_from_dict!r}"
            )

    def test_yaml_overrides_env(self, monkeypatch):
        """YAML explicit value must take priority over env var."""
        monkeypatch.setenv("PBF_SIMULATION_TIMESTEP", "0.05")
        import pybullet_fleet._defaults as mod

        mod.reload_defaults()
        from pybullet_fleet.core_simulation import SimulationParams

        params = SimulationParams.from_dict({"gui": False, "monitor": False, "timestep": 0.01})
        assert params.timestep == 0.01  # YAML wins

    def test_timestep_unified(self):
        """The critical mismatch is fixed: dataclass and from_dict agree."""
        from pybullet_fleet.core_simulation import SimulationParams

        direct = SimulationParams(gui=False, monitor=False)
        from_dict = SimulationParams.from_dict({"gui": False, "monitor": False})
        assert direct.timestep == pytest.approx(0.1)
        assert from_dict.timestep == pytest.approx(0.1)
```

**Step 2: Run tests — they should FAIL (from_dict still has old defaults)**

Run: `pytest tests/test_defaults.py -v --no-header 2>&1 | tail -30`
Expected: TestConsistency failures (timestep mismatch)

**Step 3: Commit test file**

```bash
git add tests/test_defaults.py
git commit -m "test: add _defaults completeness and consistency tests (RED)"
```

---

### Task 3: Refactor SimulationParams to use `_defaults.py` (SERIAL)

**Files:**
- Modify: `pybullet_fleet/core_simulation.py` (lines 47–167)

**Step 1: Add import at top of core_simulation.py**

After the existing imports (around line 34), add:
```python
from pybullet_fleet._defaults import SIMULATION as _SIM_D
```

**Step 2: Replace dataclass field defaults**

Change every hardcoded default in the `SimulationParams` dataclass to reference `_SIM_D["field_name"]`.

Before:
```python
    target_rtf: float = 1.0
    timestep: float = 1.0 / 240.0  # CHANGES to 0.1
    ...
```

After:
```python
    target_rtf: float = _SIM_D["target_rtf"]
    timestep: float = _SIM_D["timestep"]
    ...
```

Fields that stay unchanged (not in `_defaults.py`):
- `collision_check_frequency: Optional[float] = None`
- `spatial_hash_cell_size: Optional[float] = None`
- `collision_detection_method: Optional[CollisionDetectionMethod] = None`
- `spatial_hash_cell_size_mode: SpatialHashCellSizeMode = SpatialHashCellSizeMode.AUTO_INITIAL`
- `camera_config: Optional[Dict[str, Any]] = None`
- `model_paths: Optional[List[str]] = None`

**Step 3: Replace from_dict() defaults**

Change every `.get("key", hardcoded)` to `.get("key", _SIM_D["key"])`.

Before:
```python
    timestep=config.get("timestep", 1.0 / 10.0),
```

After:
```python
    timestep=config.get("timestep", _SIM_D["timestep"]),
```

Also fix `collision_detection_method` to let `__post_init__` auto-select:
```python
    # Before (bypasses __post_init__):
    collision_detection_method=CollisionDetectionMethod(
        config.get("collision_detection_method", ...)
    ),

    # After:
    collision_detection_method=(
        CollisionDetectionMethod(config["collision_detection_method"])
        if "collision_detection_method" in config
        else None
    ),
```

**Step 4: Run tests**

Run: `pytest tests/test_defaults.py tests/test_core_simulation.py -v --no-header 2>&1 | tail -30`
Expected: Most pass; `test_defaults` in TestSimulationParams may fail (old timestep assertion)

**Step 5: Fix `test_defaults` in TestSimulationParams**

In `tests/test_core_simulation.py` line 471, change:
```python
# Before:
assert params.timestep == pytest.approx(1.0 / 240.0)
# After:
assert params.timestep == pytest.approx(0.1)
```

**Step 6: Run full test suite**

Run: `pytest tests/ -x --no-header 2>&1 | tail -20`
Expected: All pass

**Step 7: Commit**

```bash
git add pybullet_fleet/core_simulation.py tests/test_core_simulation.py
git commit -m "refactor: SimulationParams uses _defaults.py for all default values"
```

---

### Task 4: Refactor AgentSpawnParams and Agent factories (SERIAL — after Task 3)

**Files:**
- Modify: `pybullet_fleet/agent.py` (5 locations)

**Step 1: Add import at top of agent.py**

```python
from pybullet_fleet._defaults import AGENT as _AGT_D
```

**Step 2: Replace AgentSpawnParams dataclass defaults (lines ~67–72)**

```python
# Before:
    max_linear_vel: float = 2.0
    max_linear_accel: float = 5.0
    max_angular_vel: float = 3.0
    max_angular_accel: float = 10.0
    use_fixed_base: bool = False

# After:
    max_linear_vel: float = _AGT_D["max_linear_vel"]
    max_linear_accel: float = _AGT_D["max_linear_accel"]
    max_angular_vel: float = _AGT_D["max_angular_vel"]
    max_angular_accel: float = _AGT_D["max_angular_accel"]
    use_fixed_base: bool = _AGT_D["use_fixed_base"]
```

Note: `motion_mode` stays as `MotionMode.DIFFERENTIAL` because `_defaults` stores the string `"differential"`, not the enum. The `from_dict()` already handles string→enum conversion. For the dataclass, keep the enum default.

**Step 3: Replace from_dict() defaults (lines ~157–162)**

```python
    max_linear_vel=config.get("max_linear_vel", _AGT_D["max_linear_vel"]),
    max_linear_accel=config.get("max_linear_accel", _AGT_D["max_linear_accel"]),
    ...
```

**Step 4: Replace Agent.__init__() defaults (lines ~259–264)**

```python
    def __init__(
        self,
        ...,
        max_linear_vel: float = _AGT_D["max_linear_vel"],
        max_linear_accel: float = _AGT_D["max_linear_accel"],
        max_angular_vel: float = _AGT_D["max_angular_vel"],
        max_angular_accel: float = _AGT_D["max_angular_accel"],
        use_fixed_base: bool = _AGT_D["use_fixed_base"],
        ...
    ):
```

**Step 5: Replace Agent.from_mesh() defaults (lines ~655–660)**

Same pattern as Step 4.

**Step 6: Replace Agent.from_urdf() defaults (lines ~751–756)**

Same pattern as Step 4.

**Step 7: Run Agent tests**

Run: `pytest tests/test_agent_core.py tests/test_defaults.py -v --no-header 2>&1 | tail -20`
Expected: All pass

**Step 8: Commit**

```bash
git add pybullet_fleet/agent.py
git commit -m "refactor: Agent/AgentSpawnParams uses _defaults.py for all default values"
```

---

### Task 5: Refactor SimObjectSpawnParams and ShapeParams (SERIAL — after Task 3)

**Files:**
- Modify: `pybullet_fleet/sim_object.py`

**Step 1: Add imports at top of sim_object.py**

```python
from pybullet_fleet._defaults import SIM_OBJECT as _OBJ_D, SHAPE as _SHP_D
```

**Step 2: Replace ShapeParams scalar defaults (lines ~76–77)**

```python
# Before:
    radius: float = 0.5
    height: float = 1.0

# After:
    radius: float = _SHP_D["radius"]
    height: float = _SHP_D["height"]
```

Mutable list fields (`mesh_scale`, `half_extents`, `rgba_color`) keep their `field(default_factory=...)`.

**Step 3: Replace ShapeParams.from_dict() scalar defaults (lines ~99–100)**

```python
    radius=data.get("radius", _SHP_D["radius"]),
    height=data.get("height", _SHP_D["height"]),
```

**Step 4: Replace SimObjectSpawnParams defaults (lines ~151–156)**

```python
# Before:
    mass: float = 0.0
    pickable: bool = True

# After:
    mass: float = _OBJ_D["mass"]
    pickable: bool = _OBJ_D["pickable"]
```

Note: `collision_mode` is an enum (`CollisionMode.NORMAL_3D`). Keep the enum default in the dataclass; reference `_OBJ_D["collision_mode"]` only in `from_dict()` for string→enum conversion.

**Step 5: Replace SimObjectSpawnParams.from_dict() defaults**

```python
    mass=data.get("mass", _OBJ_D["mass"]),
    pickable=data.get("pickable", _OBJ_D["pickable"]),
```

**Step 6: Run sim_object tests**

Run: `pytest tests/test_sim_object.py tests/test_defaults.py -v --no-header 2>&1 | tail -20`
Expected: All pass

**Step 7: Commit**

```bash
git add pybullet_fleet/sim_object.py
git commit -m "refactor: SimObject/ShapeParams uses _defaults.py for default values"
```

---

### Task 6: Add `.env.defaults` template and `.gitignore` entry (PARALLEL — independent)

**Files:**
- Create: `.env.defaults`
- Modify: `.gitignore`

**Step 1: Create `.env.defaults`**

```env
# PyBulletFleet Default Configuration
# Copy to .env and uncomment lines you want to override.
# Requires: pip install python-dotenv (optional)
#
# Naming: PBF_{SECTION}_{KEY}
# Bool values: true/false/1/0/yes/no (case-insensitive)

# --- Simulation ---
# PBF_SIMULATION_TARGET_RTF=1.0
# PBF_SIMULATION_TIMESTEP=0.1
# PBF_SIMULATION_GUI=true
# PBF_SIMULATION_PHYSICS=false
# PBF_SIMULATION_MONITOR=true
# PBF_SIMULATION_LOG_LEVEL=warn
# PBF_SIMULATION_ENABLE_SHADOWS=true
# PBF_SIMULATION_ENABLE_FLOOR=true
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

**Step 2: Add `.env` to `.gitignore`**

Append to `.gitignore`:
```
# User environment overrides (see .env.defaults for template)
.env
```

**Step 3: Commit**

```bash
git add .env.defaults .gitignore
git commit -m "feat: add .env.defaults template and gitignore .env"
```

---

### Task 7: Run full verification (SERIAL — must be last)

**Step 1: Run full test suite**

Run: `make verify 2>&1 | tail -20`
Expected: All tests pass, coverage ≥ 75%

**Step 2: Verify consistency test passes**

Run: `pytest tests/test_defaults.py::TestConsistency::test_dataclass_matches_from_dict -v`
Expected: PASS — the critical timestep mismatch is fixed

**Step 3: Verify env override works end-to-end**

Run: `PBF_SIMULATION_TIMESTEP=0.05 python -c "from pybullet_fleet.core_simulation import SimulationParams; p = SimulationParams(gui=False, monitor=False); print(p.timestep)"`
Expected: `0.05`

---

## Task Dependencies

```
Task 1 (_defaults.py)
  │
  ├── Task 2 (tests) ──── SERIAL
  │     │
  │     ├── Task 3 (SimulationParams) ──── SERIAL
  │     │
  │     ├── Task 4 (Agent) ──── SERIAL after Task 3
  │     │
  │     └── Task 5 (SimObject) ──── PARALLEL with Task 4
  │
  └── Task 6 (.env.defaults) ──── PARALLEL with Tasks 2-5
        │
        └── Task 7 (verify) ──── SERIAL (last)
```

- **PARALLEL tasks:** Task 4 and 5 (after Task 3); Task 6 (independent)
- **SERIAL tasks:** 1 → 2 → 3 → {4,5} → 7
