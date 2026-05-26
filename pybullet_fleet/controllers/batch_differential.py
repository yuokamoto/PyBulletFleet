"""Vectorized differential-drive batch controller.

Manages many differential-drive agents at once using NumPy arrays. Each
registered agent runs the standard ROTATE → FORWARD lifecycle per waypoint:

1. ROTATE: trapezoidal-velocity TPI on rotation angle + quaternion slerp.
2. FORWARD: trapezoidal-velocity TPI on straight-line distance.

A single ``batch_advance(dt)`` call evaluates both phases for all agents
without per-agent Python dispatch and writes the resulting poses via
``sim_core.set_poses``.

Scope (v1)
----------
- Pose mode only (path following). No ``set_velocity``.
- Forward direction only (``MovementDirection.FORWARD``). No AUTO/BACKWARD.
- No final-orientation alignment.
- 2D navigation friendly (``navigation_2d=True`` flattens goal z).

Agents needing the unsupported features should keep using the per-agent
``DifferentialController``.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, List

import numpy as np
from scipy.spatial.transform import Rotation as R

from pybullet_fleet.controllers.batch_base import BatchKinematicController
from pybullet_fleet.controllers._tpi import trapezoid_distance, trapezoid_params
from pybullet_fleet.geometry import Pose
from pybullet_fleet.logging_utils import get_lazy_logger

if TYPE_CHECKING:
    from pybullet_fleet.agent import Agent

logger = get_lazy_logger(__name__)


# Phase tags (kept as int8 in arrays for cheap masking).
_PHASE_IDLE = 0
_PHASE_ROTATE = 1
_PHASE_FORWARD = 2


class BatchDifferentialController(BatchKinematicController):
    """Batched differential-drive pose controller."""

    _registry_name = "batch_differential"

    def __init__(
        self,
        navigation_2d: bool = False,
    ) -> None:
        super().__init__(navigation_2d=navigation_2d)

        # Phase per agent
        self._phase: np.ndarray = np.zeros((0,), dtype=np.int8)

        # ROTATE-phase state
        self._rot_t_start: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._rot_start_quat: np.ndarray = np.zeros((0, 4), dtype=np.float64)
        self._rot_target_quat: np.ndarray = np.zeros((0, 4), dtype=np.float64)
        # Slerp precompute (per agent): cached dot (always >= 0) and theta_0/sin_theta_0
        self._rot_dot: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._rot_theta0: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._rot_sin_theta0: np.ndarray = np.zeros((0,), dtype=np.float64)
        # Rotation angle TPI params
        self._rot_total_angle: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._rot_t_accel: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._rot_t_const: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._rot_t_total: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._rot_accel: np.ndarray = np.zeros((0,), dtype=np.float64)

        # FORWARD-phase state
        self._fwd_t_start: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._fwd_p_start: np.ndarray = np.zeros((0, 3), dtype=np.float64)
        self._fwd_displacement: np.ndarray = np.zeros((0, 3), dtype=np.float64)
        self._fwd_total_distance: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._fwd_t_accel: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._fwd_t_const: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._fwd_t_total: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._fwd_accel: np.ndarray = np.zeros((0,), dtype=np.float64)
        self._fwd_target_quat: np.ndarray = np.zeros((0, 4), dtype=np.float64)

        # Multi-waypoint path state (Python lists — waypoint switching is rare).
        self._paths: List[List[Pose]] = []
        self._wp_index: List[int] = []

    # ------------------------------------------------------------------ #
    # Lifecycle hooks
    # ------------------------------------------------------------------ #

    def _resize_subclass(self, n: int) -> None:
        self._phase = self._resize_rows(self._phase, n)
        # ROTATE
        self._rot_t_start = self._resize_rows(self._rot_t_start, n)
        self._rot_start_quat = self._resize_rows(self._rot_start_quat, n)
        self._rot_target_quat = self._resize_rows(self._rot_target_quat, n)
        self._rot_dot = self._resize_rows(self._rot_dot, n)
        self._rot_theta0 = self._resize_rows(self._rot_theta0, n)
        self._rot_sin_theta0 = self._resize_rows(self._rot_sin_theta0, n)
        self._rot_total_angle = self._resize_rows(self._rot_total_angle, n)
        self._rot_t_accel = self._resize_rows(self._rot_t_accel, n)
        self._rot_t_const = self._resize_rows(self._rot_t_const, n)
        self._rot_t_total = self._resize_rows(self._rot_t_total, n)
        self._rot_accel = self._resize_rows(self._rot_accel, n)
        # FORWARD
        self._fwd_t_start = self._resize_rows(self._fwd_t_start, n)
        self._fwd_p_start = self._resize_rows(self._fwd_p_start, n)
        self._fwd_displacement = self._resize_rows(self._fwd_displacement, n)
        self._fwd_total_distance = self._resize_rows(self._fwd_total_distance, n)
        self._fwd_t_accel = self._resize_rows(self._fwd_t_accel, n)
        self._fwd_t_const = self._resize_rows(self._fwd_t_const, n)
        self._fwd_t_total = self._resize_rows(self._fwd_t_total, n)
        self._fwd_accel = self._resize_rows(self._fwd_accel, n)
        self._fwd_target_quat = self._resize_rows(self._fwd_target_quat, n)

    def _on_agent_registered(self, idx: int, agent: "Agent") -> None:
        self._resize_subclass(len(self._agents))
        self._phase[idx] = _PHASE_IDLE
        pose = agent.get_pose()
        # Default identity targets so an idle agent's row is well-defined.
        self._rot_start_quat[idx] = pose.orientation
        self._rot_target_quat[idx] = pose.orientation
        self._fwd_target_quat[idx] = pose.orientation
        while len(self._paths) <= idx:
            self._paths.append([])
            self._wp_index.append(0)
        self._paths[idx] = []
        self._wp_index[idx] = 0

    def _on_agent_unregistered(self, idx: int, agent: "Agent") -> None:
        n = len(self._agents)
        self._resize_subclass(n)
        while len(self._paths) > n:
            self._paths.pop()
            self._wp_index.pop()

    def _swap_rows(self, i: int, j: int) -> None:
        super()._swap_rows(i, j)
        for arr in (
            self._phase,
            self._rot_t_start,
            self._rot_dot,
            self._rot_theta0,
            self._rot_sin_theta0,
            self._rot_total_angle,
            self._rot_t_accel,
            self._rot_t_const,
            self._rot_t_total,
            self._rot_accel,
            self._fwd_t_start,
            self._fwd_total_distance,
            self._fwd_t_accel,
            self._fwd_t_const,
            self._fwd_t_total,
            self._fwd_accel,
        ):
            arr[[i, j]] = arr[[j, i]]
        for arr2d in (
            self._rot_start_quat,
            self._rot_target_quat,
            self._fwd_p_start,
            self._fwd_displacement,
            self._fwd_target_quat,
        ):
            arr2d[[i, j]] = arr2d[[j, i]]
        self._paths[i], self._paths[j] = self._paths[j], self._paths[i]
        self._wp_index[i], self._wp_index[j] = self._wp_index[j], self._wp_index[i]

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def set_path(self, agent: "Agent", path: List[Pose]) -> None:
        """Set a waypoint path. Triggers the first ROTATE trajectory immediately."""
        if self._sim_core is None:
            raise RuntimeError("BatchDifferentialController must be attached to a sim_core before set_path.")
        if not path:
            raise ValueError("path must be a non-empty list of Poses.")
        idx = self._agent_index.get(id(agent))
        if idx is None:
            raise KeyError(f"Agent {agent} is not registered with this batch controller.")
        self._paths[idx] = list(path)
        self._wp_index[idx] = 0
        self._begin_waypoint(idx, agent, path[0], float(self._sim_core.sim_time))

    # ------------------------------------------------------------------ #
    # Waypoint lifecycle (per-agent, called rarely)
    # ------------------------------------------------------------------ #

    def _begin_waypoint(self, idx: int, agent: "Agent", goal: Pose, sim_time: float) -> None:
        """Initialise the ROTATE phase for a waypoint.

        Mirrors ``DifferentialController._init_pose_trajectory`` for
        ``direction=FORWARD``, ``navigation_2d`` honoured, no final-orientation.
        """
        current = agent.get_pose()
        start_pos = np.asarray(current.position, dtype=np.float64)
        goal_pos = np.asarray(goal.position, dtype=np.float64)
        if self._navigation_2d:
            goal_pos = goal_pos.copy()
            goal_pos[2] = start_pos[2]

        direction_vec = goal_pos - start_pos
        total_distance = float(np.linalg.norm(direction_vec))

        # Cache forward state up front (will be used after ROTATE completes).
        self._fwd_p_start[idx] = start_pos
        self._fwd_displacement[idx] = direction_vec
        self._fwd_total_distance[idx] = total_distance

        # Target orientation: align x-axis to direction of motion.
        start_quat = np.asarray(current.orientation, dtype=np.float64)
        if total_distance > 1e-6:
            direction_unit = direction_vec / total_distance
            goal_rot = R.from_quat(goal.orientation)
            goal_mat = goal_rot.as_matrix()
            x_axis_goal = goal_mat[:, 0]
            z_axis_goal = goal_mat[:, 2]
            alignment = float(np.dot(x_axis_goal, direction_unit))
            if alignment > 0.95:
                target_quat = np.asarray(goal.orientation, dtype=np.float64)
            else:
                y_axis = np.cross(z_axis_goal, direction_unit)
                y_norm = float(np.linalg.norm(y_axis))
                if y_norm < 1e-6:
                    fallback_up = np.array([0.0, 0.0, 1.0]) if abs(direction_unit[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
                    y_axis = np.cross(fallback_up, direction_unit)
                    y_axis = y_axis / np.linalg.norm(y_axis)
                else:
                    y_axis = y_axis / y_norm
                z_axis_final = np.cross(direction_unit, y_axis)
                rot_mat = np.column_stack([direction_unit, y_axis, z_axis_final])
                target_quat = np.asarray(R.from_matrix(rot_mat).as_quat(), dtype=np.float64)
        else:
            target_quat = np.asarray(goal.orientation, dtype=np.float64)

        self._fwd_target_quat[idx] = target_quat

        # Compute rotation angle and TPI; if negligible, snap heading and skip
        # to FORWARD immediately (matching DifferentialController behaviour).
        angle = _quat_angle_between_np(start_quat, target_quat)
        if angle <= 1e-6:
            self._begin_forward(idx, agent, target_quat, sim_time)
            return

        # Precompute slerp constants (with shortest-path correction).
        dot = float(np.dot(start_quat, target_quat))
        if dot < 0.0:
            target_for_slerp = -target_quat
            dot = -dot
        else:
            target_for_slerp = target_quat
        dot = min(1.0, max(0.0, dot))
        theta0 = math.acos(dot)
        sin_theta0 = math.sin(theta0)

        ang_vel = float(agent.max_angular_vel[0])
        ang_accel = float(agent.max_angular_accel[0])
        t_acc, t_cst, t_tot = trapezoid_params(angle, ang_vel, ang_accel)

        self._phase[idx] = _PHASE_ROTATE
        self._rot_t_start[idx] = sim_time
        self._rot_start_quat[idx] = start_quat
        self._rot_target_quat[idx] = target_for_slerp
        self._rot_dot[idx] = dot
        self._rot_theta0[idx] = theta0
        self._rot_sin_theta0[idx] = sin_theta0
        self._rot_total_angle[idx] = angle
        self._rot_t_accel[idx] = t_acc
        self._rot_t_const[idx] = t_cst
        self._rot_t_total[idx] = t_tot
        self._rot_accel[idx] = ang_accel

    def _begin_forward(self, idx: int, agent: "Agent", target_quat: np.ndarray, sim_time: float) -> None:
        """Switch to FORWARD phase: snap heading, init distance TPI."""
        # Snap orientation buffer + flush (so the per-agent agent.set_pose_raw
        # parity with the per-agent controller's behaviour during phase switch
        # is preserved).
        self._pos_buf[idx] = self._fwd_p_start[idx]
        self._orn_buf[idx] = target_quat
        # Caller will set _moved_mask; no immediate flush here.

        distance = float(self._fwd_total_distance[idx])
        avg_vel = float(np.mean(agent.max_linear_vel))
        avg_accel = float(np.mean(agent.max_linear_accel))
        t_acc, t_cst, t_tot = trapezoid_params(distance, avg_vel, avg_accel)

        self._phase[idx] = _PHASE_FORWARD
        self._fwd_t_start[idx] = sim_time
        self._fwd_t_accel[idx] = t_acc
        self._fwd_t_const[idx] = t_cst
        self._fwd_t_total[idx] = t_tot
        self._fwd_accel[idx] = avg_accel

    def _advance_waypoint(self, idx: int, sim_time: float) -> None:
        path = self._paths[idx]
        if not path:
            self._phase[idx] = _PHASE_IDLE
            return
        self._wp_index[idx] += 1
        if self._wp_index[idx] < len(path):
            self._begin_waypoint(idx, self._agents[idx], path[self._wp_index[idx]], sim_time)
        else:
            self._phase[idx] = _PHASE_IDLE
            self._paths[idx] = []
            self._wp_index[idx] = 0

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

        phase = self._phase
        rotate_mask = phase == _PHASE_ROTATE
        forward_mask = phase == _PHASE_FORWARD
        if not (rotate_mask.any() or forward_mask.any()):
            return self._moved_mask

        now = float(sim_core.sim_time)

        # ----- ROTATE: compute slerp orientations for rotate_mask rows -----
        completed_rotate = np.zeros(n, dtype=bool)
        if rotate_mask.any():
            rm = rotate_mask
            tau = np.maximum(now - self._rot_t_start[rm], 0.0)
            angle_traveled = trapezoid_distance(
                tau,
                self._rot_t_accel[rm],
                self._rot_t_const[rm],
                self._rot_t_total[rm],
                self._rot_accel[rm],
                self._rot_total_angle[rm],
            )
            total_angle_safe = np.where(self._rot_total_angle[rm] > 1e-9, self._rot_total_angle[rm], 1.0)
            t_frac = np.clip(angle_traveled / total_angle_safe, 0.0, 1.0)
            new_quat = _batch_slerp(
                self._rot_start_quat[rm],
                self._rot_target_quat[rm],
                t_frac,
                self._rot_dot[rm],
                self._rot_theta0[rm],
                self._rot_sin_theta0[rm],
            )
            self._orn_buf[rm] = new_quat
            # Hold position during rotation.
            self._pos_buf[rm] = self._fwd_p_start[rm]
            self._moved_mask[rm] = True
            # Completed rotations: tau >= t_total (or total_angle <= 1e-9).
            done = tau >= self._rot_t_total[rm]
            if done.any():
                rotate_idx = np.flatnonzero(rm)
                completed_rotate[rotate_idx[done]] = True
                # Snap to exact target for those rows.
                self._orn_buf[rotate_idx[done]] = self._rot_target_quat[rotate_idx[done]]

        # ----- FORWARD: trapezoid distance + position interpolation -----
        completed_forward = np.zeros(n, dtype=bool)
        if forward_mask.any():
            fm = forward_mask
            tau = np.maximum(now - self._fwd_t_start[fm], 0.0)
            distance = trapezoid_distance(
                tau,
                self._fwd_t_accel[fm],
                self._fwd_t_const[fm],
                self._fwd_t_total[fm],
                self._fwd_accel[fm],
                self._fwd_total_distance[fm],
            )
            total_dist_safe = np.where(self._fwd_total_distance[fm] > 1e-9, self._fwd_total_distance[fm], 1.0)
            ratio = np.clip(distance / total_dist_safe, 0.0, 1.0)
            new_pos = self._fwd_p_start[fm] + ratio[:, None] * self._fwd_displacement[fm]
            self._pos_buf[fm] = new_pos
            self._orn_buf[fm] = self._fwd_target_quat[fm]
            self._moved_mask[fm] = True
            done = tau >= self._fwd_t_total[fm]
            if done.any():
                fwd_idx = np.flatnonzero(fm)
                done_rows = fwd_idx[done]
                completed_forward[done_rows] = True
                # Snap to exact goal (start + displacement) for those rows.
                self._pos_buf[done_rows] = self._fwd_p_start[done_rows] + self._fwd_displacement[done_rows]

        # Write rows to sim now.
        self._apply_phase1()

        # Handle phase transitions for completed rotations (ROTATE -> FORWARD).
        if completed_rotate.any():
            for i in np.flatnonzero(completed_rotate):
                self._begin_forward(int(i), self._agents[int(i)], self._rot_target_quat[int(i)], now)
        # Handle waypoint advance for completed forwards.
        if completed_forward.any():
            for i in np.flatnonzero(completed_forward):
                self._advance_waypoint(int(i), now)

        return self._moved_mask


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _quat_angle_between_np(q0: np.ndarray, q1: np.ndarray) -> float:
    """Shortest-path rotation angle (radians) between two unit quaternions."""
    dot = abs(float(np.dot(q0, q1)))
    dot = min(dot, 1.0)
    return 2.0 * math.acos(dot)


def _batch_slerp(
    q0: np.ndarray,
    q1: np.ndarray,
    t: np.ndarray,
    dot: np.ndarray,
    theta0: np.ndarray,
    sin_theta0: np.ndarray,
) -> np.ndarray:
    """Vectorised slerp for N quaternion pairs.

    Args:
        q0: (N, 4) start quaternions.
        q1: (N, 4) end quaternions (already shortest-path corrected).
        t: (N,) interpolation parameters in [0, 1].
        dot, theta0, sin_theta0: (N,) precomputed constants.

    Returns:
        (N, 4) interpolated unit quaternions.
    """
    # Avoid div-by-zero in main path
    sin_theta0_safe = np.where(sin_theta0 > 1e-8, sin_theta0, 1.0)
    theta = theta0 * t
    sin_theta = np.sin(theta)
    s0 = np.cos(theta) - dot * sin_theta / sin_theta0_safe
    s1 = sin_theta / sin_theta0_safe
    result_main = s0[:, None] * q0 + s1[:, None] * q1

    # Near-identical fallback: normalized lerp
    use_lerp = sin_theta0 <= 1e-8
    if use_lerp.any():
        lerp = q0 + t[:, None] * (q1 - q0)
        norms = np.linalg.norm(lerp, axis=1, keepdims=True)
        norms_safe = np.where(norms > 1e-12, norms, 1.0)
        lerp = lerp / norms_safe
        result_main[use_lerp] = lerp[use_lerp]

    return result_main
