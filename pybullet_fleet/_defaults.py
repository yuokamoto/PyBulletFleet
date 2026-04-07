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
        "window_width": 1024,
        "window_height": 768,
        # Monitor window
        "monitor_width": 200,
        "monitor_height": 290,
        # monitor_x / monitor_y: -1 means "let the window manager decide"
        # (tkinter geometry string omits +x+y). Set to >= 0 for absolute
        # virtual-screen coordinates.
        "monitor_x": -1,
        "monitor_y": -1,
        # --- None sentinels (not env-overridable) ---
        # Listed here for documentation: these are valid SimulationParams
        # fields whose default is None (meaning "auto" or "disabled").
        # None = check every step (no throttling)
        "collision_check_frequency": None,
        # None = auto-select based on physics mode
        # (physics ON → CONTACT_POINTS, physics OFF → CLOSEST_POINTS)
        "collision_detection_method": None,
        # None = auto-calculate from largest entity AABB
        "spatial_hash_cell_size": None,
        # None = no camera configuration (use PyBullet defaults)
        "camera_config": None,
        # None = no additional model search paths
        "model_paths": None,
    },
    "agent": {
        "max_linear_vel": 2.0,
        "max_linear_accel": 5.0,
        "max_angular_vel": 3.0,
        "max_angular_accel": 10.0,
        "motion_mode": "differential",
        "use_fixed_base": False,
        # Not centralized (complex / per-instance):
        #   urdf_path          — str, per-robot URDF path
        #   ik_params          — IKParams dataclass (see "ik" section for scalars)
        #   controller_config  — dict, controller type + params
    },
    "sim_object": {
        "mass": 0.0,
        "pickable": True,
        "collision_mode": "normal_3d",
        # Not centralized (complex / per-instance):
        #   name                 — str, required per-object identity
        #   initial_pose         — Pose object
        #   visual_shape         — ShapeParams (mesh/box/sphere/cylinder)
        #   collision_shape      — ShapeParams
        #   visual_frame_pose    — Pose offset for visual shape
        #   collision_frame_pose — Pose offset for collision shape
        #   user_data            — dict, arbitrary per-object metadata
    },
    "shape": {
        "radius": 0.5,
        "height": 1.0,
        # Not centralized (mutable list defaults — use field(default_factory)):
        #   mesh_scale    — [1.0, 1.0, 1.0]
        #   half_extents  — [0.5, 0.5, 0.5]
        #   rgba_color    — [0.8, 0.8, 0.8, 1.0]
    },
    "ik": {
        "max_outer_iterations": 5,
        "convergence_threshold": 0.01,
        "max_inner_iterations": 200,
        "residual_threshold": 1e-4,
        "reachability_tolerance": 0.02,
        # Not centralized (complex / per-robot):
        #   seed_quartiles  — tuple of floats (default: 0.25, 0.5, 0.75)
        #   ik_joint_names  — optional tuple of joint name strings
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


def _snapshot() -> Dict[str, Dict[str, Any]]:
    """Return a deep copy of the current defaults (for test save/restore)."""
    import copy

    return copy.deepcopy(_DEFAULTS)


def _restore(snapshot: Dict[str, Dict[str, Any]]) -> None:
    """Restore defaults from a snapshot captured by :func:`_snapshot`."""
    for section, params in snapshot.items():
        _DEFAULTS[section].update(params)


#: Convenience accessors — import these in consuming modules.
SIMULATION: Dict[str, Any] = _DEFAULTS["simulation"]
AGENT: Dict[str, Any] = _DEFAULTS["agent"]
SIM_OBJECT: Dict[str, Any] = _DEFAULTS["sim_object"]
SHAPE: Dict[str, Any] = _DEFAULTS["shape"]
IK: Dict[str, Any] = _DEFAULTS["ik"]

#: Keys in ``simulation`` whose default is ``None`` (not env-overridable).
NONE_SENTINEL_KEYS = frozenset(k for k, v in _DEFAULTS["simulation"].items() if v is None)
