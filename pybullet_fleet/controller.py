"""Controller ABC hierarchy + built-in controllers + registry (Plugin Architecture Phase 2).

Controller is a Strategy pattern for Agent movement.  Agent "has-a" Controller.
When a controller is set, ``Agent.update()`` delegates to ``controller.compute()``.
When no controller is set (``None``), legacy TPI logic runs — fully backward compatible.

All built-in controllers are **kinematic**: they teleport the agent to new poses
via ``agent.set_pose()`` each step.  No physics forces or torques are applied.

Hierarchy
---------
::

    Controller (ABC)            — pure compute + hooks
    └── VelocityController      — set_velocity + cmd_vel timeout
        ├── OmniVelocityController         — 6-DoF, quaternion-based
        └── DifferentialVelocityController — unicycle model

*TPIController* – (future) extract existing TPI trajectory logic from Agent.

Registry
--------
Controllers are registered by name for config-driven instantiation::

    from pybullet_fleet.controller import create_controller

    ctrl = create_controller("omni_velocity")
    ctrl = create_controller("differential_velocity", {"max_linear_vel": 1.5})

Usage::

    from pybullet_fleet.controller import OmniVelocityController

    ctrl = OmniVelocityController()
    agent.set_controller(ctrl)
    ctrl.set_velocity(vx=1.0, vy=0.0, wz=0.2)  # body-frame

    agent.set_controller(None)  # revert to legacy goal-based mode
"""

import inspect
import math
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Dict, Optional, Type

import numpy as np
from scipy.spatial.transform import Rotation

from pybullet_fleet.geometry import Pose
from pybullet_fleet.logging_utils import get_lazy_logger

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
    """Return a copy of the controller registry for inspection.

    Example::

        from pybullet_fleet.controller import list_controllers
        for name, cls in list_controllers().items():
            print(f"{name}: {cls.__name__}")
    """
    return dict(CONTROLLER_REGISTRY)


# ---------------------------------------------------------------------------
# ABC
# ---------------------------------------------------------------------------


class Controller(ABC):
    """Agent movement control interface (Strategy pattern).

    Pure base class: only ``compute()`` + lifecycle hooks.
    All built-in controllers are **kinematic**: they update agent pose directly
    via ``agent.set_pose()``.  No physics forces or torques are applied.

    Subclasses must implement :meth:`compute`.  The optional hook methods
    (``on_goal_set``, ``on_path_set``, ``on_stop``, ``reset``) have default
    no-op implementations.

    For velocity-based controllers, see :class:`VelocityController`.
    """

    @abstractmethod
    def compute(self, agent: "Agent", dt: float) -> bool:
        """Compute one step of control.

        Args:
            agent: The agent to control.  Read state via ``agent.get_pose()``,
                   write via ``agent.set_pose()``.
            dt: Time step in seconds.

        Returns:
            ``True`` if the agent moved, ``False`` otherwise.
        """
        ...

    @classmethod
    def from_config(cls, config: dict) -> "Controller":
        """Create from config dict, matching keys to ``__init__`` parameters.

        Extracts only keys that match ``cls.__init__`` parameter names
        (excluding ``self``).  Unrecognised keys (e.g. ``type``) are
        silently ignored.  Missing keys use ``__init__`` defaults.
        """
        sig = inspect.signature(cls.__init__)
        valid = {p for p in sig.parameters if p != "self"}
        kwargs = {k: v for k, v in config.items() if k in valid}
        return cls(**kwargs)

    def on_goal_set(self, agent: "Agent", goal: "Pose") -> None:
        """Called when ``set_goal_pose()`` is invoked on the agent."""

    def on_path_set(self, agent: "Agent", path: list) -> None:
        """Called when ``set_path()`` is invoked on the agent."""

    def on_stop(self, agent: "Agent") -> None:
        """Called when ``stop()`` is invoked on the agent."""

    def reset(self) -> None:
        """Reset internal controller state."""


# ---------------------------------------------------------------------------
# VelocityController — intermediate ABC for velocity-based controllers
# ---------------------------------------------------------------------------


class VelocityController(Controller):
    """Base for velocity-command controllers.

    Adds :meth:`set_velocity` (body-frame 6-DoF), velocity limits, and an
    optional cmd_vel watchdog timeout.

    :meth:`compute` implements a **template method pattern**:

    1. Check watchdog timeout → zero velocity if expired
    2. Early return ``False`` if velocity is zero
    3. Delegate to :meth:`_apply_velocity` for kinematics

    Subclasses must implement :meth:`_apply_velocity`.

    Shared state:
        ``_linear_velocity``  — body-frame linear velocity [vx, vy, vz] (np.ndarray)
        ``_angular_velocity`` — body-frame angular velocity [wx, wy, wz] (np.ndarray)

    Velocity limits:
        ``max_linear_vel`` / ``max_angular_vel`` are stored here for
        subclasses to use in their :meth:`set_velocity` overrides.
        Set to ``math.inf`` (default) to disable.

    Watchdog timeout:
        If ``cmd_vel_timeout > 0``, the controller automatically zeros velocity
        when :meth:`set_velocity` has not been called for longer than the timeout
        (measured in **sim-time** via ``dt`` accumulation).
        Set to ``0.0`` (default) to disable.
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

    # -- Template method -----------------------------------------------

    def compute(self, agent: "Agent", dt: float) -> bool:
        """Template method: timeout → zero-check → kinematics."""
        if self._check_velocity_timeout(dt):
            self._linear_velocity[:] = 0.0
            self._angular_velocity[:] = 0.0
        if np.allclose(self._linear_velocity, 0.0) and np.allclose(self._angular_velocity, 0.0):
            agent._current_velocity[:] = 0.0
            agent._current_angular_velocity = 0.0
            return False
        return self._apply_velocity(agent, dt)

    def on_stop(self, agent: "Agent") -> None:
        """Zero all velocity commands when the agent is stopped."""
        self._linear_velocity[:] = 0.0
        self._angular_velocity[:] = 0.0

    # -- Subclass hook -------------------------------------------------

    @abstractmethod
    def _apply_velocity(self, agent: "Agent", dt: float) -> bool:
        """Apply stored velocity to *agent* for one time step.

        Called only when velocity is non-zero.

        Returns:
            ``True`` if the agent moved.
        """
        ...

    # -- Velocity interface --------------------------------------------

    def set_velocity(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        wx: float = 0.0,
        wy: float = 0.0,
        wz: float = 0.0,
    ) -> None:
        """Set velocity command (body frame, 6-DoF).  Override in subclasses."""
        self._linear_velocity[:] = [vx, vy, vz]
        self._angular_velocity[:] = [wx, wy, wz]
        self._time_since_last_set_velocity = 0.0
        self._velocity_ever_set = True

    def _check_velocity_timeout(self, dt: float) -> bool:
        """Check if velocity command has timed out.

        Must be called at the start of ``compute()`` with the current ``dt``.
        Accumulates elapsed sim-time since the last :meth:`set_velocity` call.

        Returns:
            ``True`` if timed out (caller should zero velocity), ``False`` otherwise.
        """
        if self._cmd_vel_timeout <= 0.0 or not self._velocity_ever_set:
            return False
        self._time_since_last_set_velocity += dt
        return self._time_since_last_set_velocity > self._cmd_vel_timeout


# ---------------------------------------------------------------------------
# OmniVelocityController (6-DoF, body-frame input, quaternion rotation)
# ---------------------------------------------------------------------------


class OmniVelocityController(VelocityController):
    """Kinematic omnidirectional 6-DoF velocity controller.

    Accepts body-frame velocity via :meth:`set_velocity` with full 6 DoF:
    linear (vx, vy, vz) and angular (wx, wy, wz).

    In :meth:`_apply_velocity`, rotates body-frame linear velocities to world
    frame using the agent's full quaternion orientation, then performs Euler
    integration.  Angular velocity is integrated via quaternion multiplication
    (no gimbal lock).

    Velocity magnitudes are clamped to ``max_linear_vel`` / ``max_angular_vel``
    (direction preserved).  Default ``math.inf`` means no limit.

    This is a **kinematic** controller — it teleports the agent to the new pose
    via ``agent.set_pose()`` each step.  No physics forces are applied.

    Usage::

        ctrl = OmniVelocityController()
        agent.set_controller(ctrl)
        ctrl.set_velocity(vx=1.0, vy=0.0, wz=0.2)  # body-frame
    """

    def set_velocity(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        wx: float = 0.0,
        wy: float = 0.0,
        wz: float = 0.0,
    ) -> None:
        """Set velocity command in **body frame** (6-DoF).

        Linear and angular velocity magnitudes are clamped to
        ``max_linear_vel`` / ``max_angular_vel`` respectively,
        preserving direction.

        Args:
            vx: Forward velocity (body X, m/s).
            vy: Lateral velocity (body Y, m/s).
            vz: Vertical velocity (body Z, m/s).
            wx: Angular velocity around body X (rad/s, roll rate).
            wy: Angular velocity around body Y (rad/s, pitch rate).
            wz: Angular velocity around body Z (rad/s, yaw rate).
        """
        lin = np.array([vx, vy, vz])
        ang = np.array([wx, wy, wz])

        # Magnitude clamping — preserves direction
        lin_mag = float(np.linalg.norm(lin))
        if lin_mag > self._max_linear_vel:
            lin = lin * (self._max_linear_vel / lin_mag)

        ang_mag = float(np.linalg.norm(ang))
        if ang_mag > self._max_angular_vel:
            ang = ang * (self._max_angular_vel / ang_mag)

        self._linear_velocity[:] = lin
        self._angular_velocity[:] = ang
        self._time_since_last_set_velocity = 0.0
        self._velocity_ever_set = True

    # -- Template hook -------------------------------------------------

    def _apply_velocity(self, agent: "Agent", dt: float) -> bool:  # noqa: D401
        pose = agent.get_pose()

        # Current orientation as scipy Rotation (quaternion xyzw → scalar-last)
        qx, qy, qz, qw = pose.orientation
        current_rot = Rotation.from_quat([qx, qy, qz, qw])

        # Body → world rotation for linear velocity
        world_vel = current_rot.apply(self._linear_velocity)

        # Euler integration — position
        new_pos = [
            pose.x + world_vel[0] * dt,
            pose.y + world_vel[1] * dt,
            pose.z + world_vel[2] * dt,
        ]

        # Angular velocity integration via quaternion multiplication
        ang_zero = np.allclose(self._angular_velocity, 0.0)
        if ang_zero:
            new_quat = [qx, qy, qz, qw]
        else:
            omega = self._angular_velocity
            angle = np.linalg.norm(omega) * dt
            axis = omega / np.linalg.norm(omega)
            delta_rot = Rotation.from_rotvec(axis * angle)
            new_rot = current_rot * delta_rot  # body-frame rotation
            new_quat_xyzw = new_rot.as_quat()  # [x, y, z, w]
            new_quat = list(new_quat_xyzw)

        new_pose = Pose(position=new_pos, orientation=new_quat)
        agent.set_pose(new_pose)

        agent._current_velocity[:] = world_vel
        agent._current_angular_velocity = self._angular_velocity[2]  # wz for compat
        return True


# ---------------------------------------------------------------------------
# DifferentialVelocityController (unicycle model)
# ---------------------------------------------------------------------------


class DifferentialVelocityController(VelocityController):
    """Kinematic differential-drive (unicycle) velocity controller.

    Non-holonomic: only forward/backward (``vx``) and rotation (``wz``).
    Lateral velocity (``vy``) and all other DoF are ignored.

    This is a **kinematic** controller — it teleports the agent to the new pose
    via ``agent.set_pose()`` each step.  No physics forces are applied.

    Kinematics::

        x  += v * cos(yaw) * dt
        y  += v * sin(yaw) * dt
        yaw += wz * dt

    Args:
        max_linear_vel: Maximum forward/backward speed (m/s).
        max_angular_vel: Maximum angular speed (rad/s).
        wheel_separation: Distance between wheels (m).  Informational
            only (not used in unicycle model).
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
        """Set velocity command (body frame).

        Only ``vx`` and ``wz`` are used; all other DoF are silently ignored
        (non-holonomic constraint).

        Args:
            vx: Forward velocity (m/s). Clamped to ``[-max_linear_vel, max_linear_vel]``.
            vy: Ignored.
            vz: Ignored.
            wx: Ignored.
            wy: Ignored.
            wz: Angular velocity (rad/s). Clamped to ``[-max_angular_vel, max_angular_vel]``.
        """
        self._linear_velocity[:] = [max(-self._max_linear_vel, min(vx, self._max_linear_vel)), 0.0, 0.0]
        self._angular_velocity[:] = [0.0, 0.0, max(-self._max_angular_vel, min(wz, self._max_angular_vel))]
        self._time_since_last_set_velocity = 0.0
        self._velocity_ever_set = True

    # -- Template hook -------------------------------------------------

    def _apply_velocity(self, agent: "Agent", dt: float) -> bool:  # noqa: D401
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


# ---------------------------------------------------------------------------
# Auto-register built-in controllers
# ---------------------------------------------------------------------------
register_controller("omni_velocity", OmniVelocityController)
register_controller("differential_velocity", DifferentialVelocityController)
