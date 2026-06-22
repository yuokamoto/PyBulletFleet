"""Shared parameter dataclass for kinematic controllers.

``ControllerParams`` carries all kinematic limits and behaviour flags that are
common to every :class:`KinematicController` subclass (omni, differential, …).
It is stored on the per-agent controller as ``controller.params`` and is the
single authoritative source consulted by both per-agent control loops and the
batched controllers (``BatchOmniController`` / ``BatchDifferentialController``).

Design rationale
----------------
* **Single source of truth** — eliminates duplication of ``max_linear_vel``,
  ``navigation_2d``, etc. across :class:`AgentSpawnParams`, :class:`Agent`, and
  the controllers themselves.
* **Per-axis kinematic limits** — ``max_*_vel/accel`` accept either a scalar
  (uniform limit, magnitude clamp semantics) or a 3-element sequence
  (per-axis limit, per-axis clamp / direction-projected TPI). Velocity-mode
  clamping and pose-mode TPI both branch on the runtime form.
* **Symmetry with batched controllers** — batch controllers read
  ``agent.controller.params`` during ``register_agent`` and pack the fields
  into ``(N,)`` arrays. Per-agent and batch share the exact same dataclass.
* **Pure data, no logic** — kept separate from ``controller.py`` (which holds
  the control-loop logic) to avoid bloating that module and to keep imports
  cycle-free between ``agent.py``, ``controller.py`` and ``controllers/*``.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any, Dict, List, Optional

import numpy as np

from pybullet_fleet._defaults import CONTROLLER as _CTRL_D
from pybullet_fleet.geometry import (
    ScalarOrAxes,
    as_axes,
    clamp_vec_to_limit,
    is_scalar,
    projected_axis_limit,
)
from pybullet_fleet.types import MovementDirection


@dataclass
class ControllerParams:
    """Kinematic limits and behaviour flags shared by all ``KinematicController`` subclasses.

    All numeric fields default to ``None`` ("not explicitly set").  Controllers
    resolve ``None`` to the framework default from
    :data:`pybullet_fleet._defaults.CONTROLLER` at the point of use, so a
    ``ControllerParams()`` with all-``None`` fields behaves identically to the
    old all-default instance.  The ``None`` sentinel enables fleet-level
    defaults in :class:`~pybullet_fleet.agent_manager.AgentManager`: only
    fields that are still ``None`` on an agent are filled in from the fleet
    config; explicitly set values are never overwritten.

    Attributes:
        max_linear_vel: Maximum linear velocity in m/s. Scalar = magnitude
            clamp (Euclidean norm). 3-element sequence ``[vx, vy, vz]`` =
            per-axis limit in body frame. ``None`` → framework default.
        max_angular_vel: Maximum angular velocity in rad/s. Same semantics
            as ``max_linear_vel``. ``None`` → framework default.
        max_linear_accel: Maximum linear acceleration in m/s² (used by TPI).
            ``None`` → framework default.
        max_angular_accel: Maximum angular acceleration in rad/s² (used by TPI).
            ``None`` → framework default.
        cmd_vel_timeout: Watchdog timeout in seconds; ``0.0`` or ``None``
            disables the watchdog. ``None`` → framework default (``0.0``).
        navigation_2d: When ``True``, pose-control trajectories preserve the
            current ``z`` coordinate (XY-plane navigation). ``None`` = not
            explicitly set (treated as ``False`` by all controllers).
        default_direction: Default movement direction for ``set_path``.
            Only meaningful for differential-drive controllers.
    """

    max_linear_vel: Optional[ScalarOrAxes] = None
    max_angular_vel: Optional[ScalarOrAxes] = None
    max_linear_accel: Optional[ScalarOrAxes] = None
    max_angular_accel: Optional[ScalarOrAxes] = None
    cmd_vel_timeout: Optional[float] = None
    navigation_2d: Optional[bool] = None
    default_direction: MovementDirection = MovementDirection.FORWARD

    @classmethod
    def from_dict(cls, config: Dict[str, Any]) -> "ControllerParams":
        """Create from a dict, ignoring unknown keys.

        Unknown keys are silently dropped so that subclass-specific extras
        (e.g. ``wheel_separation``) can coexist with shared params inside a
        single YAML block. Strings in ``default_direction`` are coerced to
        the :class:`MovementDirection` enum.
        """
        valid = {f.name for f in fields(cls)}
        kwargs = {k: v for k, v in config.items() if k in valid}
        if isinstance(kwargs.get("default_direction"), str):
            kwargs["default_direction"] = MovementDirection(kwargs["default_direction"])
        return cls(**kwargs)

    # ------------------------------------------------------------------
    # per-axis helpers
    # ------------------------------------------------------------------

    def _eff_linear_vel(self) -> ScalarOrAxes:
        return self.max_linear_vel if self.max_linear_vel is not None else _CTRL_D["max_linear_vel"]

    def _eff_angular_vel(self) -> ScalarOrAxes:
        return self.max_angular_vel if self.max_angular_vel is not None else _CTRL_D["max_angular_vel"]

    def _eff_linear_accel(self) -> ScalarOrAxes:
        return self.max_linear_accel if self.max_linear_accel is not None else _CTRL_D["max_linear_accel"]

    def _eff_angular_accel(self) -> ScalarOrAxes:
        return self.max_angular_accel if self.max_angular_accel is not None else _CTRL_D["max_angular_accel"]

    def _eff_cmd_vel_timeout(self) -> float:
        return self.cmd_vel_timeout if self.cmd_vel_timeout is not None else _CTRL_D["cmd_vel_timeout"]

    def linear_vel_along_direction(self, direction_unit: np.ndarray) -> float:
        """Maximum scalar speed along *direction_unit* respecting per-axis limits."""
        return projected_axis_limit(self._eff_linear_vel(), direction_unit)

    def linear_accel_along_direction(self, direction_unit: np.ndarray) -> float:
        """Maximum scalar acceleration along *direction_unit* respecting per-axis limits."""
        return projected_axis_limit(self._eff_linear_accel(), direction_unit)

    def clamp_linear_vec(self, vec: np.ndarray) -> np.ndarray:
        """Clamp a body-frame linear velocity vector ``(3,)``."""
        return clamp_vec_to_limit(vec, self._eff_linear_vel())

    def clamp_angular_vec(self, vec: np.ndarray) -> np.ndarray:
        """Clamp a body-frame angular velocity vector ``(3,)``."""
        return clamp_vec_to_limit(vec, self._eff_angular_vel())

    def scalar_max_linear_vel(self) -> float:
        """Forward-axis scalar ``max_linear_vel`` (for differential drive)."""
        v = self._eff_linear_vel()
        return float(v) if is_scalar(v) else float(as_axes(v)[0])  # type: ignore[arg-type]

    def scalar_max_linear_accel(self) -> float:
        """Forward-axis scalar ``max_linear_accel`` (for differential drive)."""
        a = self._eff_linear_accel()
        return float(a) if is_scalar(a) else float(as_axes(a)[0])  # type: ignore[arg-type]

    def scalar_max_angular_vel(self) -> float:
        """Yaw-axis scalar ``max_angular_vel`` (for differential drive)."""
        w = self._eff_angular_vel()
        return float(w) if is_scalar(w) else float(as_axes(w)[2])  # type: ignore[arg-type]

    def scalar_max_angular_accel(self) -> float:
        """Yaw-axis scalar ``max_angular_accel`` (for differential drive)."""
        a = self._eff_angular_accel()
        return float(a) if is_scalar(a) else float(as_axes(a)[2])  # type: ignore[arg-type]


__all__: List[str] = ["ControllerParams", "ScalarOrAxes"]
