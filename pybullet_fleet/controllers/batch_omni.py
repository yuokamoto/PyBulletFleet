"""Vectorized omnidirectional batch controller.

Manages many omni agents at once using NumPy arrays. For each registered
agent we store a single straight-line trapezoidal-velocity trajectory; one
``batch_advance(dt)`` evaluates all trajectories without per-agent Python
dispatch and writes the resulting poses via ``sim_core.set_poses``.

Scope (v1)
----------
- Pose mode only (path following). Velocity commands are not supported in
  the batched path; agents needing ``set_velocity`` should use the per-agent
  ``OmniController`` instead.
- Single waypoint per call to ``set_path`` — multi-waypoint paths follow the
  list as one waypoint per trajectory but without final-orientation
  alignment (orientation stays constant during translation, matching omni).

Numerical equivalence with ``OmniController`` for the supported scope is
within ~1e-6 per step (see ``tests/test_batch_controller.py``).
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, List

import numpy as np

from pybullet_fleet.controllers.batch_base import BatchKinematicController
from pybullet_fleet.geometry import Pose
from pybullet_fleet.logging_utils import get_lazy_logger

if TYPE_CHECKING:
    from pybullet_fleet.agent import Agent

logger = get_lazy_logger(__name__)


class BatchOmniController(BatchKinematicController):
    """Batched omnidirectional pose controller."""

    _registry_name = "batch_omni"

    def __init__(
        self,
        navigation_2d: bool = False,
    ) -> None:
        super().__init__(navigation_2d=navigation_2d)

        # Per-agent state arrays (sized (N,) or (N, 3)/(N, 4) — kept in sync
        # with ``self._agents`` via the lifecycle hooks below).
        self._t_start: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._p_start: np.ndarray = np.zeros((0, 3), dtype=np.float64)
        self._displacement: np.ndarray = np.zeros((0, 3), dtype=np.float64)
        self._total_distance: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._t_accel: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._t_const: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._t_total: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._accel_buf: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._target_orientation: np.ndarray = np.zeros((0, 4), dtype=np.float64)
        self._active: np.ndarray = np.zeros((0,), dtype=bool)
        # Multi-waypoint path state (Python lists are fine: waypoint switching
        # is rare relative to TPI evaluation).
        self._paths: List[List[Pose]] = []
        self._wp_index: List[int] = []

    # ------------------------------------------------------------------ #
    # Lifecycle hooks
    # ------------------------------------------------------------------ #

    def _resize_subclass(self, n: int) -> None:
        self._t_start = self._resize_rows(self._t_start, n)
        self._total_distance = self._resize_rows(self._total_distance, n)
        self._t_accel = self._resize_rows(self._t_accel, n)
        self._t_const = self._resize_rows(self._t_const, n)
        self._t_total = self._resize_rows(self._t_total, n)
        self._accel_buf = self._resize_rows(self._accel_buf, n)
        self._active = self._resize_rows(self._active, n)
        self._p_start = self._resize_rows(self._p_start, n)
        self._displacement = self._resize_rows(self._displacement, n)
        self._target_orientation = self._resize_rows(self._target_orientation, n)

    def _on_agent_registered(self, idx: int, agent: "Agent") -> None:
        self._resize_subclass(len(self._agents))
        self._active[idx] = False
        self._t_total[idx] = 0.0
        pose = agent.get_pose()
        self._target_orientation[idx] = pose.orientation
        # Lists
        while len(self._paths) <= idx:
            self._paths.append([])
            self._wp_index.append(0)
        self._paths[idx] = []
        self._wp_index[idx] = 0

    def _on_agent_unregistered(self, idx: int, agent: "Agent") -> None:
        # idx was already swapped to the end-row's value before this call,
        # so resize-to-n drops the last (now stale) row.
        n = len(self._agents)
        self._resize_subclass(n)
        if idx < len(self._paths):
            # The swap in _swap_rows took care of moving the last entries
            # into idx; drop the trailing slots.
            while len(self._paths) > n:
                self._paths.pop()
                self._wp_index.pop()

    def _swap_rows(self, i: int, j: int) -> None:
        super()._swap_rows(i, j)
        self._t_start[[i, j]] = self._t_start[[j, i]]
        self._p_start[[i, j]] = self._p_start[[j, i]]
        self._displacement[[i, j]] = self._displacement[[j, i]]
        self._total_distance[[i, j]] = self._total_distance[[j, i]]
        self._t_accel[[i, j]] = self._t_accel[[j, i]]
        self._t_const[[i, j]] = self._t_const[[j, i]]
        self._t_total[[i, j]] = self._t_total[[j, i]]
        self._accel_buf[[i, j]] = self._accel_buf[[j, i]]
        self._active[[i, j]] = self._active[[j, i]]
        self._target_orientation[[i, j]] = self._target_orientation[[j, i]]
        self._paths[i], self._paths[j] = self._paths[j], self._paths[i]
        self._wp_index[i], self._wp_index[j] = self._wp_index[j], self._wp_index[i]

    # ------------------------------------------------------------------ #
    # Public API — set a path for an already-registered agent
    # ------------------------------------------------------------------ #

    def set_path(self, agent: "Agent", path: List[Pose]) -> None:
        """Set a waypoint path. Triggers the first trajectory immediately."""
        if self._sim_core is None:
            raise RuntimeError("BatchOmniController must be attached to a sim_core before set_path.")
        if not path:
            raise ValueError("path must be a non-empty list of Poses.")
        idx = self._agent_index.get(id(agent))
        if idx is None:
            raise KeyError(f"Agent {agent} is not registered with this batch controller.")

        self._paths[idx] = list(path)
        self._wp_index[idx] = 0
        self._begin_waypoint(idx, agent, path[0], float(self._sim_core.sim_time))

    def _begin_waypoint(self, idx: int, agent: "Agent", goal: Pose, sim_time: float) -> None:
        """Initialise trajectory state at row ``idx`` for a single goal pose."""
        current = agent.get_pose()
        start = np.asarray(current.position, dtype=np.float64)
        goal_pos = np.asarray(goal.position, dtype=np.float64)
        if self._navigation_2d:
            goal_pos = goal_pos.copy()
            goal_pos[2] = start[2]

        disp = goal_pos - start
        total = float(np.linalg.norm(disp))

        # Use agent's own kinematic limits (per-axis arrays — average like
        # OmniController._init_pose_trajectory does).
        avg_vel = float(np.mean(agent.max_linear_vel))
        avg_accel = float(np.mean(agent.max_linear_accel))

        t_accel, t_const, t_total = _trapezoid_params(total, avg_vel, avg_accel)

        self._t_start[idx] = sim_time
        self._p_start[idx] = start
        self._displacement[idx] = disp
        self._total_distance[idx] = total
        self._t_accel[idx] = t_accel
        self._t_const[idx] = t_const
        self._t_total[idx] = t_total
        self._accel_buf[idx] = avg_accel
        # Omni does not rotate during translation: hold current orientation.
        self._target_orientation[idx] = current.orientation
        self._active[idx] = True

    # ------------------------------------------------------------------ #
    # Vectorized step
    # ------------------------------------------------------------------ #

    def batch_advance(self, dt: float) -> np.ndarray:
        n = len(self._agents)
        self._moved_mask.fill(False)
        if n == 0:
            return self._moved_mask

        sim_core = self._sim_core
        if sim_core is None:
            return self._moved_mask

        active = self._active
        if not bool(active.any()):
            return self._moved_mask

        # Match OmniController._apply_pose semantics: it evaluates the TPI at
        # ``agent.sim_core.sim_time`` (the value set at the top of step_once),
        # not at ``sim_time + dt``. Keeping this identical is essential for
        # numerical equivalence with the per-agent controller.
        now = float(sim_core.sim_time)
        tau = np.maximum(now - self._t_start, 0.0)
        # distance traveled along straight-line displacement (vectorised TPI)
        distance = _trapezoid_distance(
            tau,
            self._t_accel,
            self._t_const,
            self._t_total,
            self._accel_buf,
            self._total_distance,
        )

        # ratio in [0, 1]; guard against zero-distance trajectories
        total_dist_safe = np.where(self._total_distance > 1e-9, self._total_distance, 1.0)
        ratio = np.clip(distance / total_dist_safe, 0.0, 1.0)
        new_pos = self._p_start + ratio[:, None] * self._displacement

        # Detect agents whose trajectory finished this step.
        completed = active & (tau >= self._t_total)
        # Snap completed agents to exact goal (start + displacement).
        if bool(completed.any()):
            new_pos[completed] = self._p_start[completed] + self._displacement[completed]

        # Write into buffers (only active rows are meaningful for moved_mask).
        self._pos_buf[:] = new_pos
        self._orn_buf[:] = self._target_orientation
        # Mark all currently-active agents as moved this step (so set_poses
        # writes their fresh row; SimObject's own moved-detection collapses
        # any rows that didn't actually change).
        self._moved_mask[:] = active

        # Apply phase 1 — single batched set_poses call.
        self._apply_phase1()

        # Advance waypoints / clear active flags for completed trajectories.
        if bool(completed.any()):
            for i in np.flatnonzero(completed):
                self._advance_waypoint(int(i), now)

        return self._moved_mask

    def _advance_waypoint(self, idx: int, sim_time: float) -> None:
        path = self._paths[idx]
        if not path:
            self._active[idx] = False
            return
        self._wp_index[idx] += 1
        if self._wp_index[idx] < len(path):
            agent = self._agents[idx]
            self._begin_waypoint(idx, agent, path[self._wp_index[idx]], sim_time)
        else:
            self._active[idx] = False
            self._paths[idx] = []
            self._wp_index[idx] = 0


# ---------------------------------------------------------------------------
# Vectorised trapezoidal-velocity TPI primitives
# ---------------------------------------------------------------------------


def _trapezoid_params(distance: float, vmax: float, accel: float):
    """Solve a symmetric trapezoidal profile (v0 = ve = 0).

    Returns (t_accel, t_const, t_total) for a single trajectory. ``distance``
    must be non-negative.
    """
    if distance <= 1e-12 or vmax <= 0.0 or accel <= 0.0 or not math.isfinite(vmax) or not math.isfinite(accel):
        return 0.0, 0.0, 0.0
    d_to_vmax = vmax * vmax / (2.0 * accel)
    if 2.0 * d_to_vmax <= distance:
        t_accel = vmax / accel
        t_const = (distance - 2.0 * d_to_vmax) / vmax
        t_total = 2.0 * t_accel + t_const
    else:
        v_peak = math.sqrt(accel * distance)
        t_accel = v_peak / accel
        t_const = 0.0
        t_total = 2.0 * t_accel
    return t_accel, t_const, t_total


def _trapezoid_distance(
    tau: np.ndarray,
    t_accel: np.ndarray,
    t_const: np.ndarray,
    t_total: np.ndarray,
    accel: np.ndarray,
    total_distance: np.ndarray,
) -> np.ndarray:
    """Vectorised distance-traveled lookup for trapezoidal profiles.

    All inputs are (N,) arrays; returns (N,).
    """
    out = np.zeros_like(tau)
    # phase masks
    in_const = (tau > t_accel) & (tau <= t_accel + t_const)
    in_decel = (tau > t_accel + t_const) & (tau < t_total)
    done = tau >= t_total
    in_accel = (tau > 0.0) & ~in_const & ~in_decel & ~done

    # accel phase: 0.5 * a * t^2
    if in_accel.any():
        out[in_accel] = 0.5 * accel[in_accel] * tau[in_accel] * tau[in_accel]
    # const phase: 0.5 * a * t_accel^2 + v_peak * (t - t_accel),
    # where v_peak = a * t_accel
    if in_const.any():
        ta = t_accel[in_const]
        a = accel[in_const]
        out[in_const] = 0.5 * a * ta * ta + (a * ta) * (tau[in_const] - ta)
    # decel phase: total_distance - 0.5 * a * (t_total - t)^2
    if in_decel.any():
        a = accel[in_decel]
        t_left = t_total[in_decel] - tau[in_decel]
        out[in_decel] = total_distance[in_decel] - 0.5 * a * t_left * t_left
    if done.any():
        out[done] = total_distance[done]
    return out
