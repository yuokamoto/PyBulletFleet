"""Unified controller hierarchy + built-in controllers + registry.

Each kinematic model (omni, differential) gets one controller class that handles
both velocity commands (``set_velocity``) and pose commands (``set_goal_pose`` /
``set_path``) via an internal mode state machine.

Hierarchy
---------
::

    Controller (ABC)            — pure compute + hooks
    └── KinematicController     — mode state machine + velocity & pose dispatch
        ├── OmniController      — 6-DoF velocity + omnidirectional TPI pose
        └── DifferentialController — unicycle velocity + rotate-then-forward TPI pose

Mode state machine (B3 — input-based + interruption)
-----------------------------------------------------
::

    IDLE ──set_velocity()──→ VELOCITY
    IDLE ──set_goal_pose()──→ POSE
    VELOCITY ──set_goal_pose()──→ POSE  (cancel velocity)
    VELOCITY ──watchdog timeout──→ IDLE
    POSE ──set_velocity()──→ VELOCITY  (cancel pose/TPI)
    POSE ──trajectory complete──→ IDLE

Registry
--------
::

    from pybullet_fleet.controller import create_controller

    ctrl = create_controller("omni")
    ctrl = create_controller("differential", {"max_linear_vel": 1.5})
"""

import inspect
import math
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Dict, List, Optional, Type

import numpy as np
from scipy.spatial.transform import Rotation as R  # used only in DifferentialController init

from pybullet_fleet.geometry import (
    Pose,
    quat_angle_between,
    quat_from_rotvec,
    quat_multiply,
    quat_slerp,
    quat_slerp_precompute,
    rotate_vector,
)
from pybullet_fleet.logging_utils import get_lazy_logger
from pybullet_fleet.types import ControllerMode, MovementDirection, PosePhase

from two_point_interpolation import TwoPointInterpolation

if TYPE_CHECKING:
    from pybullet_fleet.agent import Agent

logger = get_lazy_logger(__name__)


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

CONTROLLER_REGISTRY: Dict[str, Type["Controller"]] = {}


def register_controller(name: str, cls: Type["Controller"]) -> None:
    """Register a controller class under *name*."""
    CONTROLLER_REGISTRY[name] = cls


def create_controller(name: str, config: Optional[dict] = None) -> "Controller":
    """Create a controller by registered *name*, optionally passing *config*.

    Raises:
        KeyError: If *name* is not registered.
    """
    if name not in CONTROLLER_REGISTRY:
        raise KeyError(f"Unknown controller: {name!r}. Available: {list(CONTROLLER_REGISTRY)}")
    cls = CONTROLLER_REGISTRY[name]
    return cls.from_config(config or {})


def list_controllers() -> Dict[str, Type["Controller"]]:
    """Return a copy of the controller registry for inspection."""
    return dict(CONTROLLER_REGISTRY)


# ---------------------------------------------------------------------------
# ABC
# ---------------------------------------------------------------------------


class Controller(ABC):
    """Agent movement control interface (Strategy pattern).

    Pure base class: only ``compute()`` + lifecycle hooks.
    All built-in controllers are **kinematic**: they update agent pose directly
    via ``agent.set_pose()``.  No physics forces or torques are applied.
    """

    @abstractmethod
    def compute(self, agent: "Agent", dt: float) -> bool:
        """Compute one step of control.

        Returns:
            ``True`` if the agent moved, ``False`` otherwise.
        """
        ...

    @classmethod
    def from_config(cls, config: dict) -> "Controller":
        """Create from config dict, matching keys to ``__init__`` parameters."""
        sig = inspect.signature(cls.__init__)
        valid = {p for p in sig.parameters if p != "self"}
        kwargs = {k: v for k, v in config.items() if k in valid}
        return cls(**kwargs)

    def on_stop(self, agent: "Agent") -> None:
        """Called when ``stop()`` is invoked on the agent."""

    def reset(self) -> None:
        """Reset internal controller state."""


# ---------------------------------------------------------------------------
# KinematicController — unified ABC for velocity + pose control
# ---------------------------------------------------------------------------


class KinematicController(Controller):
    """Unified base for kinematic controllers handling both velocity and pose.

    Internal mode state machine:
        IDLE     — no active command
        VELOCITY — processing velocity input (body-frame)
        POSE     — processing goal pose via TPI trajectory

    Input-based mode switching with interruption (B3):
        - ``set_velocity()`` → switch to VELOCITY (cancels POSE if active)
        - ``set_goal_pose()`` / ``set_path()`` → switch to POSE (cancels VELOCITY)
        - Watchdog timeout → IDLE
        - Trajectory complete → IDLE
    """

    def __init__(
        self,
        max_linear_vel: float = math.inf,
        max_angular_vel: float = math.inf,
        cmd_vel_timeout: float = 0.0,
    ) -> None:
        self._max_linear_vel: float = max_linear_vel
        self._max_angular_vel: float = max_angular_vel
        self._cmd_vel_timeout: float = cmd_vel_timeout
        self._time_since_last_set_velocity: float = 0.0
        self._velocity_ever_set: bool = False
        self._linear_velocity = np.zeros(3)  # body-frame [vx, vy, vz]
        self._angular_velocity = np.zeros(3)  # body-frame [wx, wy, wz]
        self._mode: ControllerMode = ControllerMode.IDLE

        # Path-following state (owned by controller, not agent)
        self._path: List[Pose] = []
        self._current_waypoint_index: int = 0
        self._goal_pose: Optional[Pose] = None
        self._movement_direction: MovementDirection = MovementDirection.FORWARD
        self._align_final_orientation: bool = False
        self._final_target_orientation: Optional[np.ndarray] = None
        self._is_final_orientation_aligning: bool = False

        # Rotation slerp state (shared by Omni and Differential controllers)
        self._tpi_rotation_angle: Optional[TwoPointInterpolation] = None
        self._slerp_precomp = None
        self._rotation_start_quat: Optional[np.ndarray] = None
        self._rotation_target_quat: Optional[np.ndarray] = None
        self._rotation_total_angle: float = 0.0

        # Forward (straight-line) movement state (shared by Omni and Differential)
        self._tpi_forward: Optional[TwoPointInterpolation] = None
        self._forward_start_pos: Optional[np.ndarray] = None
        self._forward_direction_3d: Optional[np.ndarray] = None
        self._forward_direction_unit: Optional[np.ndarray] = None
        self._forward_total_distance_3d: float = 0.0

        # Pose phase state machine: ROTATE → FORWARD
        self._pose_phase: PosePhase = PosePhase.FORWARD

    @property
    def mode(self) -> ControllerMode:
        """Current controller mode."""
        return self._mode

    @property
    def goal_pose(self) -> Optional[Pose]:
        """Current goal pose (active waypoint)."""
        return self._goal_pose

    @property
    def current_waypoint_index(self) -> int:
        """Current waypoint index in the path (0-based)."""
        return self._current_waypoint_index

    @property
    def path(self) -> List[Pose]:
        """Current path (list of waypoints). Empty when idle."""
        return self._path

    # -- Template method -----------------------------------------------

    def compute(self, agent: "Agent", dt: float) -> bool:
        """Template: dispatch to velocity or pose handler based on mode."""
        if self._mode == ControllerMode.VELOCITY:
            is_idle = self._check_velocity_timeout(dt) or (
                np.allclose(self._linear_velocity, 0.0) and np.allclose(self._angular_velocity, 0.0)
            )
            if is_idle:
                self._zero_velocity()
                self._mode = ControllerMode.IDLE
                agent._current_velocity[:] = 0.0
                agent._current_angular_velocity = 0.0
                agent._reset_pybullet_velocity()
                return False
            return self._apply_velocity(agent, dt)
        elif self._mode == ControllerMode.POSE:
            return self._apply_pose(agent, dt)
        # IDLE
        return False

    def on_stop(self, agent: "Agent") -> None:
        """Zero all velocity commands, cancel pose, and clear path state."""
        self._zero_velocity()
        self._mode = ControllerMode.IDLE
        self._cancel_pose_internal()
        self._reset_path_state()

    def reset(self) -> None:
        """Reset all internal state."""
        self._zero_velocity()
        self._mode = ControllerMode.IDLE
        self._time_since_last_set_velocity = 0.0
        self._velocity_ever_set = False
        self._cancel_pose_internal()
        self._reset_path_state()

    # -- Velocity interface --------------------------------------------

    @abstractmethod
    def _apply_velocity(self, agent: "Agent", dt: float) -> bool:
        """Apply stored velocity to *agent* for one time step."""
        ...

    def set_velocity(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        wx: float = 0.0,
        wy: float = 0.0,
        wz: float = 0.0,
    ) -> None:
        """Set velocity command (body frame, 6-DoF). Override in subclasses for clamping."""
        if self._mode == ControllerMode.POSE:
            self._cancel_pose_internal()
        self._mode = ControllerMode.VELOCITY
        self._linear_velocity[:] = [vx, vy, vz]
        self._angular_velocity[:] = [wx, wy, wz]
        self._time_since_last_set_velocity = 0.0
        self._velocity_ever_set = True

    def _zero_velocity(self) -> None:
        """Zero all velocity state."""
        self._linear_velocity[:] = 0.0
        self._angular_velocity[:] = 0.0

    def _check_velocity_timeout(self, dt: float) -> bool:
        """Check if velocity command has timed out."""
        if self._cmd_vel_timeout <= 0.0 or not self._velocity_ever_set:
            return False
        self._time_since_last_set_velocity += dt
        return self._time_since_last_set_velocity > self._cmd_vel_timeout

    # -- Pose interface ------------------------------------------------

    @abstractmethod
    def _init_pose_trajectory(self, agent: "Agent", goal: Pose, **kwargs) -> None:
        """Initialize TPI trajectory for a single waypoint goal."""
        ...

    def cancel_pose(self) -> None:
        """Cancel active pose navigation and switch to IDLE."""
        self._cancel_pose_internal()
        self._mode = ControllerMode.IDLE

    def _cancel_pose_internal(self) -> None:
        """Reset shared pose-mode TPI state (rotation + forward + phase).

        Subclasses should call ``super()._cancel_pose_internal()`` and
        reset their own additional state.
        """
        self._reset_rotation_state()
        self._reset_forward_state()
        self._pose_phase = PosePhase.FORWARD

    def _reset_rotation_state(self) -> None:
        """Zero all rotation/slerp TPI state."""
        self._tpi_rotation_angle = None
        self._slerp_precomp = None
        self._rotation_start_quat = None
        self._rotation_target_quat = None
        self._rotation_total_angle = 0.0

    def _reset_forward_state(self) -> None:
        """Zero all forward (straight-line) TPI state."""
        self._tpi_forward = None
        self._forward_start_pos = None
        self._forward_direction_3d = None
        self._forward_direction_unit = None
        self._forward_total_distance_3d = 0.0

    def set_path(
        self,
        agent: "Agent",
        path: List[Pose],
        final_orientation_align: bool = True,
        direction: MovementDirection = MovementDirection.FORWARD,
    ) -> None:
        """Set a waypoint path for the agent to follow.

        The controller owns all path-navigation state: waypoint index, movement
        direction, final-orientation alignment.  ``Agent.set_path()`` handles
        validation, auto-approach, and visualisation, then delegates here.

        Args:
            agent: Agent to navigate.
            path: Non-empty list of waypoint Poses.
            final_orientation_align: Rotate to match last waypoint orientation
                after reaching its position.
            direction: ``FORWARD`` or ``BACKWARD`` (differential only).
        """
        self._path = list(path)
        self._current_waypoint_index = 0
        self._goal_pose = path[0]
        self._movement_direction = direction
        self._is_final_orientation_aligning = False

        if final_orientation_align and path:
            self._final_target_orientation = np.array(path[-1].orientation)
            self._align_final_orientation = True
        else:
            self._final_target_orientation = None
            self._align_final_orientation = False

        # Start trajectory for first waypoint
        if self._mode == ControllerMode.VELOCITY:
            self._zero_velocity()
        self._mode = ControllerMode.POSE
        self._init_pose_trajectory(agent, path[0], direction=direction)

    def _init_forward_trajectory(self, agent: "Agent", distance: float) -> None:
        """Initialize a single distance TPI for straight-line forward motion.

        Uses averaged max velocity / acceleration across all axes.
        After calling this, ``_apply_forward()`` can interpolate position
        along the cached direction vector.
        """
        t0 = agent.sim_core.sim_time if agent.sim_core is not None else 0.0

        avg_vel = float(np.mean(agent.max_linear_vel))
        avg_accel = float(np.mean(agent.max_linear_accel))

        try:
            self._tpi_forward = TwoPointInterpolation()
            self._tpi_forward.init(
                p0=0.0,
                pe=distance,
                acc_max=avg_accel,
                vmax=avg_vel,
                t0=t0,
                v0=0.0,
                ve=0.0,
                dec_max=avg_accel,
            )
            self._tpi_forward.calc_trajectory()
        except ValueError:
            # Fallback: zero-distance trajectory (already at goal)
            self._tpi_forward = TwoPointInterpolation()
            self._tpi_forward.init(
                p0=0.0,
                pe=0.0,
                acc_max=avg_accel,
                vmax=avg_vel,
                t0=t0,
                v0=0.0,
                ve=0.0,
                dec_max=avg_accel,
            )
            self._tpi_forward.calc_trajectory()

    def _init_rotation_tpi(
        self,
        agent: "Agent",
        start_quat: np.ndarray,
        target_quat: np.ndarray,
        t0: float,
    ) -> bool:
        """Set up rotation state + TPI for slerp between two quaternions.

        Computes the rotation angle, creates a TPI trajectory, and
        pre-computes the slerp data.  On success the five rotation state
        variables are populated; on failure they are reset.

        Returns:
            True if a rotation trajectory was created.
            False if the angle was negligible (< 1e-6 rad) or TPI creation
            failed — rotation state is reset via ``_reset_rotation_state()``.
        """
        rotation_angle = quat_angle_between(tuple(start_quat), tuple(target_quat))

        if rotation_angle <= 1e-6:
            self._reset_rotation_state()
            return False

        self._rotation_start_quat = np.asarray(start_quat)
        self._rotation_target_quat = np.asarray(target_quat)
        self._rotation_total_angle = float(rotation_angle)

        angular_vel = agent.max_angular_vel[0]
        angular_accel = agent.max_angular_accel[0]

        try:
            self._tpi_rotation_angle = TwoPointInterpolation()
            self._tpi_rotation_angle.init(
                p0=0.0,
                pe=rotation_angle,
                acc_max=angular_accel,
                vmax=angular_vel,
                t0=t0,
                v0=0.0,
                ve=0.0,
                dec_max=angular_accel,
            )
            self._tpi_rotation_angle.calc_trajectory()
            self._slerp_precomp = quat_slerp_precompute(self._rotation_start_quat, self._rotation_target_quat)
            return True
        except ValueError:
            self._reset_rotation_state()
            return False

    def _apply_pose(self, agent: "Agent", dt: float) -> bool:
        """Unified TPI pose control: ROTATE phase → FORWARD phase.

        **Path-following lifecycle** (driven by ``_handle_waypoint_reached``)::

            For each waypoint:
                [ROTATE] → [FORWARD] → waypoint reached
            After last waypoint (if final-orientation enabled):
                [ROTATE] → (forward distance = 0, completes immediately)

        Omni skips the initial ROTATE (starts at FORWARD); Differential
        always starts at ROTATE to align heading toward the next waypoint.

        The ``_init_pose_trajectory`` sets the initial ``_pose_phase``:

        - Omni (translation): ``FORWARD`` (no rotation)
        - Omni (final alignment): ``ROTATE``, forward distance = 0
        - Differential: ``ROTATE``, then ``FORWARD``
        """
        current_pose = agent.get_pose()
        current_time = agent.sim_core.sim_time if agent.sim_core is not None else 0.0

        if self._pose_phase == PosePhase.ROTATE:
            if self._apply_rotation(agent, current_time, current_pose):
                self._pose_phase = PosePhase.FORWARD
                self._init_forward_trajectory(agent, self._forward_total_distance_3d)
            return True

        if self._pose_phase == PosePhase.FORWARD:
            return self._apply_forward(agent, current_time, current_pose)

        return False

    def _apply_forward(self, agent: "Agent", current_time: float, current_pose: Pose) -> bool:
        """Interpolate position along the straight-line direction using the distance TPI.

        Returns True when trajectory is complete (caller should handle waypoint arrival).
        Returns False if no forward TPI is active.
        """
        if self._tpi_forward is None:
            return False

        distance_traveled, forward_vel, _ = self._tpi_forward.get_point(current_time)
        trajectory_complete = current_time >= self._tpi_forward.get_end_time()

        if trajectory_complete:
            self._handle_waypoint_reached(agent, current_pose)
            return True

        if self._forward_start_pos is None or self._forward_direction_3d is None or self._forward_direction_unit is None:
            return False

        total_distance = self._forward_total_distance_3d
        ratio = distance_traveled / total_distance if total_distance > 1e-6 else 0.0
        new_pos = self._forward_start_pos + self._forward_direction_3d * ratio

        # Orientation: use rotation target if set, else preserve current
        if self._rotation_target_quat is not None:
            orientation = self._rotation_target_quat.tolist()
        else:
            orientation = current_pose.orientation

        np.multiply(self._forward_direction_unit, forward_vel, out=agent._current_velocity)
        agent.set_pose_raw(new_pos.tolist(), orientation)
        agent._current_angular_velocity = 0.0
        return True

    def _apply_rotation(self, agent: "Agent", current_time: float, current_pose: Pose) -> bool:
        """Rotate in place via slerp TPI.

        Returns True when rotation is complete (or no rotation TPI is active).
        On completion, snaps orientation to the target and zeros velocity.
        Returns False while still rotating (intermediate orientation applied).
        """
        slerp_result = self._compute_slerp(current_time)
        if slerp_result is None:
            return True

        orientation, rot_complete = slerp_result
        if rot_complete:
            if self._rotation_target_quat is not None:
                agent.set_pose_raw(current_pose.position, self._rotation_target_quat.tolist())
            agent._current_velocity[:] = 0.0
            agent._current_angular_velocity = 0.0
            return True

        agent.set_pose_raw(current_pose.position, orientation)
        agent._current_velocity[:] = 0.0
        if self._tpi_rotation_angle is not None:
            _, angular_vel, _ = self._tpi_rotation_angle.get_point(current_time)
            agent._current_angular_velocity = angular_vel
        else:
            agent._current_angular_velocity = 0.0
        return False

    def _compute_slerp(self, current_time: float):
        """Compute slerp orientation from rotation TPI.

        Returns:
            ``(orientation_list, rot_complete)`` if rotation TPI is active,
            ``None`` if no rotation TPI is set.
        """
        if self._tpi_rotation_angle is None or self._slerp_precomp is None:
            return None

        rot_angle_val, _, _ = self._tpi_rotation_angle.get_point(current_time)
        rot_complete = current_time >= self._tpi_rotation_angle.get_end_time()

        if self._rotation_total_angle > 1e-9:
            t_fraction = max(0.0, min(rot_angle_val / self._rotation_total_angle, 1.0))
        else:
            t_fraction = 1.0

        new_quat = quat_slerp(
            self._rotation_start_quat,
            self._rotation_target_quat,
            t_fraction,
            self._slerp_precomp,
        )
        return list(new_quat), rot_complete

    def _handle_waypoint_reached(self, agent: "Agent", current_pose: Pose) -> None:
        """Handle waypoint arrival: snap, advance, or complete path.

        Called from ``_apply_pose()`` when the TPI trajectory for the current
        waypoint completes.  Manages the full waypoint-advance → final-orientation
        → path-complete state machine internally.
        """
        # Snap to exact goal position
        agent.set_pose_raw(self._goal_pose.position, current_pose.orientation, preserve_velocity=False)

        logger.debug("Reached waypoint at position %s", self._goal_pose.position[:2])

        # Zero velocity on agent
        agent._current_velocity[:] = 0.0
        agent._current_angular_velocity = 0.0
        agent._is_moving = False

        if self._path:
            self._current_waypoint_index += 1
            if self._current_waypoint_index < len(self._path):
                logger.debug("Moving to next waypoint %d", self._current_waypoint_index)
                self._goal_pose = self._path[self._current_waypoint_index]
                agent._is_moving = True
                self._init_pose_trajectory(agent, self._goal_pose, direction=self._movement_direction)
            else:
                # All waypoints visited
                if self._align_final_orientation and self._final_target_orientation is not None:
                    logger.info("Path complete, aligning to final orientation...")
                    self._start_final_orientation(agent)
                else:
                    self._complete_path(agent)
        else:
            # Path empty → final orientation alignment just completed
            self._is_final_orientation_aligning = False
            self._goal_pose = None
            self._mode = ControllerMode.IDLE
            agent._reset_pybullet_velocity()
            logger.info("Final orientation alignment complete")

    def _start_final_orientation(self, agent: "Agent") -> None:
        """Begin rotation-in-place after reaching the last waypoint."""
        current_pose = agent.get_pose()
        final_goal = Pose(
            position=current_pose.position,
            orientation=self._final_target_orientation.tolist(),
        )
        self._goal_pose = final_goal
        agent._is_moving = True

        # Polymorphic — each controller knows its rotation strategy
        self._init_final_orientation_trajectory(agent, final_goal)

        self._path = []
        self._align_final_orientation = False
        self._is_final_orientation_aligning = True

    def _init_final_orientation_trajectory(self, agent: "Agent", goal: Pose) -> None:
        """Initialize trajectory for final orientation alignment (rotation in place).

        Called by ``_start_final_orientation()`` when all waypoints have been
        reached and the agent needs to rotate to match the goal orientation.

        Default implementation delegates to ``_init_pose_trajectory``.
        Subclasses may override to use a rotation-only strategy.
        """
        self._init_pose_trajectory(agent, goal)

    def _complete_path(self, agent: "Agent") -> None:
        """Clean up after all waypoints (and optional final orientation) are done."""
        self._path = []
        self._goal_pose = None
        self._is_final_orientation_aligning = False
        self._mode = ControllerMode.IDLE
        agent._reset_pybullet_velocity()
        logger.info("Path complete")

    def _reset_path_state(self) -> None:
        """Reset path-following state to defaults."""
        self._path = []
        self._current_waypoint_index = 0
        self._goal_pose = None
        self._movement_direction = MovementDirection.FORWARD
        self._align_final_orientation = False
        self._final_target_orientation = None
        self._is_final_orientation_aligning = False


# ---------------------------------------------------------------------------
# OmniController (6-DoF velocity + omnidirectional TPI pose)
# ---------------------------------------------------------------------------


class OmniController(KinematicController):
    """Unified omnidirectional controller: velocity commands + TPI pose navigation.

    **Velocity mode (VELOCITY):**
        6-DoF body-frame velocity (vx, vy, vz, wx, wy, wz) → quaternion-based
        world-frame integration. Linear and angular magnitudes are clamped.

    **Pose mode (POSE):**
        Per-axis TPI with distance-ratio velocity scaling for straight-line
        motion from start to goal.
    """

    def __init__(
        self,
        max_linear_vel: float = math.inf,
        max_angular_vel: float = math.inf,
        cmd_vel_timeout: float = 0.0,
    ) -> None:
        super().__init__(
            max_linear_vel=max_linear_vel,
            max_angular_vel=max_angular_vel,
            cmd_vel_timeout=cmd_vel_timeout,
        )

    # -- Velocity kinematics -------------------------------------------

    def set_velocity(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        wx: float = 0.0,
        wy: float = 0.0,
        wz: float = 0.0,
    ) -> None:
        """Set velocity in **body frame** (6-DoF) with magnitude clamping."""
        lin = np.array([vx, vy, vz])
        ang = np.array([wx, wy, wz])

        # Magnitude clamping — preserves direction
        lin_mag = float(np.linalg.norm(lin))
        if lin_mag > self._max_linear_vel:
            lin = lin * (self._max_linear_vel / lin_mag)

        ang_mag = float(np.linalg.norm(ang))
        if ang_mag > self._max_angular_vel:
            ang = ang * (self._max_angular_vel / ang_mag)

        super().set_velocity(vx=lin[0], vy=lin[1], vz=lin[2], wx=ang[0], wy=ang[1], wz=ang[2])

    def _apply_velocity(self, agent: "Agent", dt: float) -> bool:
        pose = agent.get_pose()

        qx, qy, qz, qw = pose.orientation

        # Body → world rotation for linear velocity (SciPy-free hot path)
        world_vel = rotate_vector(tuple(self._linear_velocity), (qx, qy, qz, qw))

        new_pos = [
            pose.x + world_vel[0] * dt,
            pose.y + world_vel[1] * dt,
            pose.z + world_vel[2] * dt,
        ]

        # Angular velocity integration via quaternion multiplication (SciPy-free)
        ang_zero = np.allclose(self._angular_velocity, 0.0)
        if ang_zero:
            new_quat = [qx, qy, qz, qw]
        else:
            omega = self._angular_velocity
            rotvec = (omega[0] * dt, omega[1] * dt, omega[2] * dt)
            delta_q = quat_from_rotvec(rotvec)
            new_q = quat_multiply((qx, qy, qz, qw), delta_q)  # body-frame
            new_quat = list(new_q)

        new_pose = Pose(position=new_pos, orientation=new_quat)
        agent.set_pose(new_pose)

        agent._current_velocity[:] = world_vel
        agent._current_angular_velocity = self._angular_velocity[2]
        return True

    # -- Pose (TPI) kinematics ----------------------------------------

    def _init_pose_trajectory(self, agent: "Agent", goal: Pose, **kwargs) -> None:
        """Initialize single-distance TPI for omnidirectional straight-line motion.

        Orientation is NOT handled here — omni agents keep orientation constant
        during translation.  Sets ``_pose_phase = FORWARD`` to skip rotation.
        For rotation-only (final alignment), use
        ``_init_rotation_only_trajectory`` instead.
        """
        current_pose = agent.get_pose()
        current_pos = np.array(current_pose.position)
        goal_pos = np.array(goal.position)

        # Reset rotation state (position-only trajectory)
        self._reset_rotation_state()

        # Skip rotation phase — omni moves in any direction
        self._pose_phase = PosePhase.FORWARD

        # Cache direction for straight-line interpolation
        displacement = goal_pos - current_pos
        total_dist = float(np.linalg.norm(displacement))

        self._forward_start_pos = current_pos.copy()
        self._forward_total_distance_3d = total_dist
        if total_dist > 1e-9:
            self._forward_direction_3d = displacement.copy()
            self._forward_direction_unit = displacement / total_dist
        else:
            self._forward_direction_3d = np.zeros(3)
            self._forward_direction_unit = np.zeros(3)

        self._init_forward_trajectory(agent, total_dist)

    def _init_final_orientation_trajectory(self, agent: "Agent", goal: Pose) -> None:
        """Rotation-only final alignment (no position movement)."""
        self._init_rotation_only_trajectory(agent, goal)

    def _init_rotation_only_trajectory(self, agent: "Agent", goal: Pose) -> None:
        """Initialize slerp+TPI for rotation-only (final alignment).

        Sets ``_pose_phase = ROTATE`` and resets forward state
        so the unified ``_apply_pose`` will rotate then immediately complete
        the zero-distance forward phase.
        """
        current_pose = agent.get_pose()
        t0 = agent.sim_core.sim_time if agent.sim_core is not None else 0.0

        self._pose_phase = PosePhase.ROTATE
        self._reset_forward_state()
        self._init_rotation_tpi(
            agent,
            np.array(current_pose.orientation),
            np.array(goal.orientation),
            t0,
        )


# ---------------------------------------------------------------------------
# DifferentialController (unicycle velocity + rotate-then-forward TPI pose)
# ---------------------------------------------------------------------------


class DifferentialController(KinematicController):
    """Unified differential-drive controller: velocity commands + TPI pose.

    **Velocity mode (VELOCITY):**
        Unicycle model — only forward (``vx``) and yaw rate (``wz``).
        Lateral velocity and other DoF are ignored.

    **Pose mode (POSE):**
        Two-phase TPI: ROTATE to face target, then FORWARD to reach it.
        Uses TPI + quaternion slerp for smooth rotation.
    """

    def __init__(
        self,
        max_linear_vel: float = 2.0,
        max_angular_vel: float = 1.5,
        wheel_separation: float = 0.0,
        cmd_vel_timeout: float = 0.0,
    ) -> None:
        super().__init__(
            max_linear_vel=max_linear_vel,
            max_angular_vel=max_angular_vel,
            cmd_vel_timeout=cmd_vel_timeout,
        )
        self._wheel_separation = wheel_separation

    def set_velocity(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        wx: float = 0.0,
        wy: float = 0.0,
        wz: float = 0.0,
    ) -> None:
        """Set velocity (body frame). Only ``vx`` and ``wz`` are used."""
        clamped_vx = max(-self._max_linear_vel, min(vx, self._max_linear_vel))
        clamped_wz = max(-self._max_angular_vel, min(wz, self._max_angular_vel))
        super().set_velocity(vx=clamped_vx, wz=clamped_wz)

    # -- Velocity kinematics (unicycle) --------------------------------

    def _apply_velocity(self, agent: "Agent", dt: float) -> bool:
        v = self._linear_velocity[0]
        wz = self._angular_velocity[2]
        pose = agent.get_pose()
        yaw = pose.yaw

        world_vx = v * math.cos(yaw)
        world_vy = v * math.sin(yaw)

        new_pose = Pose.from_yaw(
            pose.x + world_vx * dt,
            pose.y + world_vy * dt,
            pose.z,
            yaw + wz * dt,
        )
        agent.set_pose(new_pose)

        agent._current_velocity[:] = [world_vx, world_vy, 0.0]
        agent._current_angular_velocity = wz
        return True

    # -- Pose (TPI) kinematics ----------------------------------------

    def _init_pose_trajectory(self, agent: "Agent", goal: Pose, **kwargs) -> None:
        """Initialize rotation trajectory for differential drive (ROTATE phase)."""
        self._movement_direction = kwargs.get("direction", MovementDirection.FORWARD)
        if isinstance(self._movement_direction, str):
            self._movement_direction = MovementDirection(self._movement_direction)

        current_pose = agent.get_pose()
        current_pos = np.array(current_pose.position)
        goal_pos = np.array(goal.position)

        direction_vec = goal_pos - current_pos

        # Cache forward direction for straight-line phase
        self._forward_start_pos = current_pos.copy()
        self._forward_total_distance_3d = float(np.linalg.norm(direction_vec))

        if self._forward_total_distance_3d > 1e-6:
            self._forward_direction_3d = goal_pos - current_pos
            self._forward_direction_unit = self._forward_direction_3d / self._forward_total_distance_3d
        else:
            self._forward_direction_3d = np.zeros(3)
            self._forward_direction_unit = np.zeros(3)

        t0 = agent.sim_core.sim_time if agent.sim_core is not None else 0.0

        self._pose_phase = PosePhase.ROTATE

        start_quat = np.array(current_pose.orientation)

        # Calculate target orientation
        if self._forward_total_distance_3d > 1e-6:
            movement_direction_vec = direction_vec / self._forward_total_distance_3d

            if self._movement_direction == MovementDirection.BACKWARD:
                x_axis_target = -movement_direction_vec
            else:
                x_axis_target = movement_direction_vec

            goal_rotation = R.from_quat(goal.orientation)
            goal_rot_matrix = goal_rotation.as_matrix()
            x_axis_goal = goal_rot_matrix[:, 0]
            z_axis_goal = goal_rot_matrix[:, 2]

            alignment = np.dot(x_axis_goal, x_axis_target)

            if alignment > 0.95:
                target_quat = np.array(goal.orientation)
            else:
                y_axis = np.cross(z_axis_goal, x_axis_target)
                y_norm_magnitude = np.linalg.norm(y_axis)

                if y_norm_magnitude < 1e-6:
                    if abs(x_axis_target[2]) < 0.9:
                        y_axis = np.cross(np.array([0, 0, 1]), x_axis_target)
                    else:
                        y_axis = np.cross(np.array([0, 1, 0]), x_axis_target)
                    y_axis = y_axis / np.linalg.norm(y_axis)
                else:
                    y_axis = y_axis / y_norm_magnitude

                z_axis_final = np.cross(x_axis_target, y_axis)
                rotation_matrix = np.column_stack([x_axis_target, y_axis, z_axis_final])
                r = R.from_matrix(rotation_matrix)
                target_quat = np.array(r.as_quat())
        else:
            target_quat = np.array(goal.orientation)

        if not self._init_rotation_tpi(agent, start_quat, target_quat, t0):
            # Negligible or failed rotation — snap heading and skip to forward
            agent.set_pose_raw(current_pos.tolist(), target_quat.tolist(), preserve_velocity=False)
            self._pose_phase = PosePhase.FORWARD
            self._init_forward_trajectory(agent, self._forward_total_distance_3d)


# ---------------------------------------------------------------------------
# Auto-register built-in controllers
# ---------------------------------------------------------------------------
register_controller("omni", OmniController)
register_controller("differential", DifferentialController)
