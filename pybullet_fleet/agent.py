"""
core/agent.py
Agent class for goal-based position control with max velocity and acceleration constraints.
Supports both mobile (use_fixed_base=False) and fixed (use_fixed_base=True) robots.
Supports both Mesh and URDF loading.
"""

import logging
import math
from dataclasses import dataclass, fields
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np
import pybullet as p
from .geometry import Pose
from .sim_object import SimObject, SimObjectSpawnParams, ShapeParams
from .action import Action
from .types import MotionMode, MovementDirection, ActionStatus, CollisionMode
from .tools import normalize_vector_param
from pybullet_fleet.tools import resolve_joint_index, resolve_link_index
from .logging_utils import get_lazy_logger
from .robot_models import resolve_model
from pybullet_fleet.events import SimEvents
from pybullet_fleet._defaults import AGENT as _AGT_D, IK as _IK_D
from pybullet_fleet.config_utils import forward_spawn_params as _forward_spawn_params

# Create logger for this module
logger = logging.getLogger(__name__)
lazy_logger = get_lazy_logger(__name__)


@dataclass
class AgentSpawnParams(SimObjectSpawnParams):
    """
    Agent spawn parameters extending SimObjectSpawnParams.

    These parameters define the physical properties, appearance, and initial state of an agent.
    They are used by Agent.from_params() and AgentManager.spawn_agents_grid().

    Supports three types of robots:
    1. Mesh robots: Specify visual_shape (and optionally collision_shape)
    2. URDF robots: Specify urdf_path (for robots with joints)
    3. Virtual agents: Specify neither (invisible, no collision, useful for tracking/planning)

    Attributes (in addition to SimObjectSpawnParams):
        urdf_path: Path to robot URDF file (for URDF-based robots with joints)
        max_linear_vel: Maximum velocity in m/s - float or [vx, vy, vz] (ignored if use_fixed_base=True)
        max_linear_accel: Maximum acceleration in m/s² - float or [ax, ay, az] (ignored if use_fixed_base=True)
        max_angular_vel: Maximum angular velocity in rad/s (for differential drive)
        max_angular_accel: Maximum angular acceleration in rad/s² (for differential drive, default: 10.0)
        motion_mode: MotionMode.OMNIDIRECTIONAL (move in any direction) or MotionMode.DIFFERENTIAL (rotate then move forward)
        use_fixed_base: If True, robot base is fixed and doesn't move (default: False)

    Inherited from SimObjectSpawnParams:
        name: Optional string name for human-readable identification.
              Duplicates allowed - use for debugging/filtering, not unique lookup.
              For unique identification, use object_id after spawning.

    Note:
        AgentManager.spawn_agents_grid() calculates positions automatically and may override initial_pose.
    """

    urdf_path: Optional[str] = None
    max_linear_vel: Union[float, List[float]] = _AGT_D["max_linear_vel"]
    max_linear_accel: Union[float, List[float]] = _AGT_D["max_linear_accel"]
    max_angular_vel: Union[float, List[float]] = _AGT_D["max_angular_vel"]
    max_angular_accel: Union[float, List[float]] = _AGT_D["max_angular_accel"]
    motion_mode: Union[MotionMode, str] = MotionMode(_AGT_D["motion_mode"])
    use_fixed_base: bool = _AGT_D["use_fixed_base"]
    ik_params: Optional["IKParams"] = None
    navigation_2d: bool = False
    controller_config: Optional[Dict[str, Any]] = None

    def __post_init__(self):
        """Validate agent spawn parameters."""
        has_visual = self.visual_shape is not None
        has_urdf = self.urdf_path is not None

        if not has_visual and not has_urdf:
            logger.warning(
                "AgentSpawnParams: Neither visual_shape nor urdf_path specified. "
                "This will create a virtual agent (invisible, no collision)."
            )
        if has_visual and has_urdf:
            logger.warning(
                "AgentSpawnParams: Both visual_shape and urdf_path specified. "
                "visual_shape will be ignored - URDF will be used."
            )

    @classmethod
    def from_dict(cls, config: Dict[str, Any]):
        """
        Create AgentSpawnParams from configuration dictionary.

        Args:
            config: Dictionary with robot parameters including:
                    - visual_shape/collision_shape or urdf_path
                    - initial_pose: Pose object (optional)
                    - max_linear_vel, max_linear_accel, use_fixed_base, etc. (optional)

        Returns:
            AgentSpawnParams instance

        Example::

            config = {
                'visual_shape': ShapeParams(
                    shape_type='mesh',
                    mesh_path='robot.obj',
                    mesh_scale=[1.0, 1.0, 1.0],
                    rgba_color=[0.2, 0.2, 0.2, 1.0]
                ),
                'collision_shape': ShapeParams(
                    shape_type='box',
                    half_extents=[0.2, 0.1, 0.2]
                ),
                'initial_pose': Pose.from_xyz(1.0, 2.0, 0.0),
                'max_linear_vel': 2.0,
                'max_linear_accel': 5.0,
                'use_fixed_base': False
            }
            params = AgentSpawnParams.from_dict(config)

            # Or for URDF:
            config = {
                'urdf_path': 'robots/mobile_robot.urdf',
                'initial_pose': Pose.from_xyz(1.0, 2.0, 0.0),
                'use_fixed_base': False
            }
            params = AgentSpawnParams.from_dict(config)
        """
        # Get base fields from parent
        base = SimObjectSpawnParams.from_dict(config)
        base_kwargs = {f.name: getattr(base, f.name) for f in fields(base)}

        # Get motion_mode and convert string to enum if needed
        motion_mode_value = config.get("motion_mode", _AGT_D["motion_mode"])
        if isinstance(motion_mode_value, str):
            motion_mode_value = MotionMode(motion_mode_value)

        # Parse ik_params from nested dict if provided
        ik_params_value = config.get("ik_params")
        if isinstance(ik_params_value, dict):
            ik_params_value = IKParams(**ik_params_value)

        # Parse controller_config: string shortcut → {"type": string}
        controller_config_value = config.get("controller_config")
        if isinstance(controller_config_value, str):
            controller_config_value = {"type": controller_config_value}

        return cls(
            **base_kwargs,
            urdf_path=config.get("urdf_path"),
            max_linear_vel=config.get("max_linear_vel", _AGT_D["max_linear_vel"]),
            max_linear_accel=config.get("max_linear_accel", _AGT_D["max_linear_accel"]),
            max_angular_vel=config.get("max_angular_vel", _AGT_D["max_angular_vel"]),
            max_angular_accel=config.get("max_angular_accel", _AGT_D["max_angular_accel"]),
            motion_mode=motion_mode_value,
            use_fixed_base=config.get("use_fixed_base", _AGT_D["use_fixed_base"]),
            ik_params=ik_params_value,
            navigation_2d=config.get("navigation_2d", _AGT_D["navigation_2d"]),
            controller_config=controller_config_value,
        )


@dataclass(frozen=True)
class IKParams:
    """Inverse kinematics solver configuration.

    Controls the multi-seed iterative IK solver used by
    :meth:`Agent._solve_ik` and :meth:`Agent.move_end_effector`.

    Attributes:
        max_outer_iterations: Maximum refinement iterations per seed.
        convergence_threshold: EE-to-target distance (m) below which
            the IK solution is accepted early.
        max_inner_iterations: ``maxNumIterations`` passed to
            ``pybullet.calculateInverseKinematics``.
        residual_threshold: ``residualThreshold`` passed to
            ``pybullet.calculateInverseKinematics``.
        reachability_tolerance: Maximum EE distance (m) for
            :meth:`Agent.move_end_effector` to report *reachable*.
        seed_quartiles: Tuple of joint-range fractions (0–1) used to
            build diversified rest-pose seeds.  Each value ``q`` produces
            a seed at ``lower + q * range`` for every joint.
        ik_joint_names: Explicit tuple of joint **names** that the IK
            solver is allowed to move.  All other movable joints are
            locked at their current position.  When ``None`` (default),
            the solver falls back to the automatic heuristic (continuous
            joints are locked, everything else is free).

    Example::

        cfg = IKParams(max_outer_iterations=10, seed_quartiles=(0.1, 0.5, 0.9))
        arm = Agent.from_urdf("arm.urdf", ik_params=cfg)

        # Mobile manipulator — solve only for arm joints
        cfg = IKParams(ik_joint_names=(
            "mount_to_shoulder", "shoulder_to_elbow",
            "elbow_to_wrist", "wrist_to_end",
        ))
        robot = Agent.from_urdf("mobile_manipulator.urdf", ik_params=cfg)
    """

    max_outer_iterations: int = _IK_D["max_outer_iterations"]
    convergence_threshold: float = _IK_D["convergence_threshold"]
    max_inner_iterations: int = _IK_D["max_inner_iterations"]
    residual_threshold: float = _IK_D["residual_threshold"]
    reachability_tolerance: float = _IK_D["reachability_tolerance"]
    seed_quartiles: Tuple[float, ...] = (0.25, 0.5, 0.75)
    ik_joint_names: Optional[Tuple[str, ...]] = None


class Agent(SimObject):
    """
    Agent class with goal-based position control.

    Inherits from SimObject to get attach/detach and callback functionality.
    Supports both mobile robots (use_fixed_base=False) and fixed robots (use_fixed_base=True).
    Supports both Mesh and URDF loading.

    Class Attributes:
        _spawn_params_cls: The SpawnParams dataclass used by config-driven
            grid spawning (``AgentSpawnParams``).

    Class Constants:
        _KINEMATIC_JOINT_FALLBACK_VELOCITY: Default max joint velocity for
            revolute joints (rad/s) used when the URDF
            ``<limit velocity="...">`` is 0 or missing.
        _KINEMATIC_PRISMATIC_FALLBACK_VELOCITY: Default max joint velocity
            for prismatic joints (m/s) used when the URDF limit is 0 or
            missing.
        _DEFAULT_JOINT_TOLERANCE: Default tolerance (rad / m) for joint
            target comparison methods.

    Features:
    - Accepts Pose goals (compatible with ROS2 geometry_msgs/Pose)
    - Max velocity and acceleration constraints (for mobile robots)
    - Current pose query API
    - Shared shape optimization for fast spawning
    - Fixed robot support (non-moving objects)
    - URDF support with joint control
    - Attach/detach objects (inherited from SimObject)

    Object Identification (inherited from SimObject):
    - object_id: Unique integer ID (assigned by sim_core, use for lookups)
    - name: Optional string name (duplicates allowed, use for debugging/filtering)

    This class is designed to be eventually controlled by ROS2 nodes,
    so the API follows ROS message conventions.
    """

    _needs_update = True  # Agents always participate in the step_once() update loop

    _entity_type_name = "agent"  # Auto-registers in ENTITY_REGISTRY via SimObject.__init_subclass__

    _KINEMATIC_JOINT_FALLBACK_VELOCITY: float = 2.0  # rad/s (revolute)
    _KINEMATIC_PRISMATIC_FALLBACK_VELOCITY: float = 0.5  # m/s (prismatic)
    _DEFAULT_JOINT_TOLERANCE: float = 0.01  # rad (or m for prismatic)

    # Override parent: use AgentSpawnParams for config-driven grid spawning
    _spawn_params_cls = AgentSpawnParams
    _kinematic_joints_physics_off_logged: bool = False  # Log physics=False fallback only once

    def __init__(
        self,
        body_id: int,
        urdf_path: Optional[str] = None,
        max_linear_vel: Union[float, List[float]] = _AGT_D["max_linear_vel"],
        max_linear_accel: Union[float, List[float]] = _AGT_D["max_linear_accel"],
        max_angular_vel: Union[float, List[float]] = _AGT_D["max_angular_vel"],
        max_angular_accel: Union[float, List[float]] = _AGT_D["max_angular_accel"],
        motion_mode: Union[MotionMode, str] = MotionMode(_AGT_D["motion_mode"]),
        use_fixed_base: bool = _AGT_D["use_fixed_base"],
        collision_mode: CollisionMode = CollisionMode.NORMAL_3D,
        sim_core=None,
        name: Optional[str] = None,
        user_data: Optional[Dict[str, Any]] = None,
        mass: Optional[float] = None,
        ik_params: Optional[IKParams] = None,
        joint_tolerance: Optional[Union[float, list, dict]] = None,
    ):
        """
        Initialize Agent.

        Args:
            body_id: PyBullet body ID
            urdf_path: Path to robot URDF file (if URDF-based)
            max_linear_vel: Maximum velocity (m/s) - float or [vx, vy, vz] for per-axis limits
            max_linear_accel: Maximum acceleration (m/s²) - float or [ax, ay, az] for per-axis limits
            max_angular_vel: Maximum angular velocity (rad/s) - float or [yaw, pitch, roll] for per-axis limits
            max_angular_accel: Maximum angular acceleration (rad/s²) - float or [yaw, pitch, roll] for per-axis limits
            motion_mode: MotionMode.OMNIDIRECTIONAL or MotionMode.DIFFERENTIAL drive
            use_fixed_base: If True, robot base is fixed and doesn't move
            collision_mode: Collision detection mode (default: NORMAL_3D)
            sim_core: Reference to simulation core (optional)
            ik_params: IK solver configuration (default: ``IKParams()``).
            joint_tolerance: Default joint tolerance for convergence checks.
                Accepts ``float`` (all joints), ``list`` (per-joint), or
                ``dict`` (``{joint_name: tol}``).
                If ``None``, uses the class default (0.01).
                For mixed prismatic / revolute chains, a dict is
                recommended so prismatic joints (metres) can use a
                tighter tolerance than revolute joints (radians).

        Note:
            For spawning robots from AgentSpawnParams, use Agent.from_params() instead.
            For mesh robots, use Agent.from_mesh() factory method.
            For URDF robots, use Agent.from_urdf() factory method.
        """
        # Initialize SimObject base class (collision_mode is forwarded so
        # add_object receives the correct mode directly – no post-hoc transition)
        super().__init__(body_id, sim_core=sim_core, collision_mode=collision_mode, name=name, user_data=user_data, mass=mass)

        self.urdf_path = urdf_path

        # Normalize max_linear_vel and max_linear_accel to numpy arrays [vx, vy, vz]
        self.max_linear_vel = normalize_vector_param(max_linear_vel, "max_linear_vel", 3)
        self.max_linear_accel = normalize_vector_param(max_linear_accel, "max_linear_accel", 3)

        # Normalize max_angular_vel and max_angular_accel to numpy arrays [yaw, pitch, roll]
        self.max_angular_vel = normalize_vector_param(max_angular_vel, "max_angular_vel", 3)
        self.max_angular_accel = normalize_vector_param(max_angular_accel, "max_angular_accel", 3)

        self.use_fixed_base = use_fixed_base

        # IK solver configuration
        self._ik_params: IKParams = ik_params if ik_params is not None else IKParams()

        # Instance-level joint tolerance (None → use class default)
        self._joint_tolerance: Optional[Union[float, list, dict]] = joint_tolerance

        # URDF-specific: joint information
        self.joint_info = []
        if urdf_path is not None:
            num_joints = p.getNumJoints(body_id, physicsClientId=self._pid)
            self.joint_info = [p.getJointInfo(body_id, j, physicsClientId=self._pid) for j in range(num_joints)]

        # Cached movable (non-fixed) joint indices — avoids recomputing in
        # _solve_ik on every call.  Built once at construction time since
        # joint topology does not change after loading.
        self._cached_movable_indices: list = [i for i, info in enumerate(self.joint_info) if info[2] != p.JOINT_FIXED]

        # Goal tracking (only used for non-static robots) - Private internal state
        self._current_velocity = np.array([0.0, 0.0, 0.0])
        self._current_angular_velocity = 0.0  # rad/s (for differential drive)
        self._is_moving = False  # Private: use property for read-only access

        # Controller chain — low-level controller at index 0, higher-level
        # controllers appended via add_controller().  update() iterates in
        # reversed order (high → low) so that patrol/random-walk controllers
        # can set goals before the base navigation controller executes.
        self._controllers: List[Any] = []

        # Action queue system for high-level task execution
        self._action_queue: List[Action] = []
        self._current_action: Optional[Action] = None

        # Last commanded joint targets — persists after arrival.
        # Written by set_joint_target() in both kinematic and physics modes.
        # Used by are_joints_at_targets(None) and _update_kinematic_joints().
        self._last_joint_targets: Dict[int, float] = {}

        # Cached flag: True when joints must use kinematic interpolation
        # (mass=0 OR sim_core has physics disabled).  Computed once to avoid
        # per-step property overhead.
        self._use_kinematic_joints: bool = self._compute_use_kinematic_joints()

        # Kinematic joint position cache — avoids per-step p.getJointState() calls.
        # Only seeded when kinematic joints are actually used (skip for physics mode).
        if self._use_kinematic_joints and self.joint_info:
            indices = list(range(len(self.joint_info)))
            states = p.getJointStates(body_id, indices, physicsClientId=self._pid)
            self._kinematic_joint_positions: Dict[int, float] = {i: states[i][0] for i in indices}
        else:
            self._kinematic_joint_positions: Dict[int, float] = {}

        # Override pickable default for Agent (robots are not pickable by default)
        self.pickable = False

        # Path visualization settings
        self.path_visualize: bool = False  # Enable/disable path visualization
        self.path_visualize_color: Optional[List[float]] = None  # RGB color (None = default [0, 1, 1])
        self.path_visualize_width: float = 2.0  # Line width
        self._path_debug_ids: List[int] = []  # Store debug line IDs for cleanup

        # Initialize motion mode and TPI instances (must be after all attributes are set)
        # Note: This will set _motion_mode and create TPI instances
        self._motion_mode = None  # Temporary: will be set by set_motion_mode()
        self.set_motion_mode(motion_mode)  # This normalizes mode and creates TPI instances

        # Update log prefix to include Agent class name (SimObject sets it initially)
        self._update_log_prefix()

    @property
    def joint_tolerance(self) -> Union[float, list, dict]:
        """Default joint tolerance used by convergence checks.

        Returns the instance-level tolerance if set, otherwise the class
        default (``_DEFAULT_JOINT_TOLERANCE = 0.01``).

        For robots with mixed prismatic (metres) and revolute (radians)
        joints, set this to a ``dict`` so each joint type can have an
        appropriate tolerance::

            agent = Agent.from_urdf(
                ...,
                joint_tolerance={"rail_joint": 0.005, "elbow_to_wrist": 0.05},
            )
        """
        if self._joint_tolerance is not None:
            return self._joint_tolerance
        return self._DEFAULT_JOINT_TOLERANCE

    @joint_tolerance.setter
    def joint_tolerance(self, value: Optional[Union[float, list, dict]]) -> None:
        self._joint_tolerance = value

    def _resolve_joint_tolerance(
        self,
        tolerance: Optional[Union[float, list, dict]],
        joint_index: int,
    ) -> float:
        """Resolve a possibly-compound tolerance to a scalar for one joint.

        Handles the full fallback chain:

        1. If *tolerance* is ``None``, fall back to ``self.joint_tolerance``.
        2. If the result is a ``dict``, look up by joint name
           (falling back to ``_DEFAULT_JOINT_TOLERANCE`` if the name is absent).
        3. If the result is a ``list``/``tuple``, index by *joint_index*
           (falling back to ``_DEFAULT_JOINT_TOLERANCE`` when out of range).
        4. Otherwise return the scalar as-is.

        Args:
            tolerance: ``float``, ``list``, ``dict``, or ``None``.
            joint_index: 0-based joint index.

        Returns:
            Scalar tolerance (``float``).
        """
        tol = tolerance if tolerance is not None else self.joint_tolerance
        if isinstance(tol, dict):
            name = self.joint_info[joint_index][1].decode("utf-8")
            return tol.get(name, self._DEFAULT_JOINT_TOLERANCE)
        if isinstance(tol, (list, tuple)):
            return tol[joint_index] if joint_index < len(tol) else self._DEFAULT_JOINT_TOLERANCE
        return float(tol)

    @property
    def is_moving(self) -> bool:
        """
        Read-only property: Check if robot is currently moving.

        To change movement state, use set_goal_pose(), set_path(), or stop().

        Returns:
            True if robot is moving, False otherwise
        """
        return self._is_moving

    @property
    def motion_mode(self) -> MotionMode:
        """
        Read-only property: Get current motion mode.

        To change motion mode, use set_motion_mode() method.

        Returns:
            Current MotionMode (OMNIDIRECTIONAL or DIFFERENTIAL)
        """
        return self._motion_mode

    @property
    def velocity(self) -> np.ndarray:
        """
        Read-only property: Get current velocity vector.

        Returns:
            Velocity [vx, vy, vz] in m/s (copy of internal state)
        """
        return self._current_velocity.copy()

    @property
    def angular_velocity(self) -> float:
        """
        Read-only property: Get current angular velocity.

        Returns:
            Angular velocity in rad/s
        """
        return self._current_angular_velocity

    @property
    def goal_pose(self) -> Optional[Pose]:
        """
        Read-only property: Get current goal pose.

        To set a new goal, use set_goal_pose() or set_path().

        Returns:
            Current goal Pose, or None if no goal is set
        """
        if self._controllers:
            return self._controllers[0].goal_pose
        return None

    @property
    def current_waypoint_index(self) -> int:
        """
        Read-only property: Get current waypoint index in the path.

        Returns:
            Current waypoint index (0-based), or 0 if no path is set
        """
        if self._controllers:
            return self._controllers[0].current_waypoint_index
        return 0

    @property
    def path(self) -> List[Pose]:
        """Read-only property: Current waypoint path. Empty when idle."""
        if self._controllers:
            return self._controllers[0].path
        return []

    @classmethod
    def from_params(cls, spawn_params: AgentSpawnParams, sim_core=None) -> "Agent":
        """
        Create an Agent from AgentSpawnParams.

        This is the recommended way to spawn robots when using AgentSpawnParams.
        Automatically detects whether to use mesh, URDF, or virtual agent based on spawn_params.

        Args:
            spawn_params: AgentSpawnParams instance with all robot parameters
            sim_core: Reference to simulation core (optional)

        Returns:
            Agent instance

        Example (Mesh)::

            params = AgentSpawnParams(
                visual_shape=ShapeParams(
                    shape_type="mesh",
                    mesh_path="robot.obj",
                    mesh_scale=[1.0, 1.0, 1.0],
                    rgba_color=[0.2, 0.2, 0.2, 1.0]
                ),
                collision_shape=ShapeParams(
                    shape_type="box",
                    half_extents=[0.2, 0.1, 0.2]
                ),
                initial_pose=Pose.from_xyz(1.0, 2.0, 0.0),
                max_linear_vel=3.0
            )
            robot = Agent.from_params(spawn_params=params)

        Example (URDF)::

            params = AgentSpawnParams(
                urdf_path="arm_robot.urdf",
                initial_pose=Pose.from_xyz(0.0, 0.0, 0.0),
                use_fixed_base=True
            )
            robot = Agent.from_params(spawn_params=params)

        Example (Virtual Agent - invisible, no collision)::

            params = AgentSpawnParams(
                initial_pose=Pose.from_xyz(0.0, 0.0, 0.0),
                max_linear_vel=2.0
            )
            virtual_agent = Agent.from_params(spawn_params=params)
        """
        # Choose mesh or URDF path
        if spawn_params.urdf_path is not None:
            # URDF robot
            agent = cls.from_urdf(
                **_forward_spawn_params(
                    cls.from_urdf,
                    spawn_params,
                    aliases={"initial_pose": "pose"},
                    extra_kwargs={"sim_core": sim_core},
                )
            )
        else:
            # Mesh robot
            agent = cls.from_mesh(
                **_forward_spawn_params(
                    cls.from_mesh,
                    spawn_params,
                    aliases={"initial_pose": "pose"},
                    extra_kwargs={"sim_core": sim_core},
                )
            )

        # Auto-create controller from config if specified
        if spawn_params.controller_config:
            from pybullet_fleet.controller import create_controller

            ctrl_config = spawn_params.controller_config
            ctrl = create_controller(ctrl_config["type"], ctrl_config)
            agent.set_controller(ctrl)

        # Default controller: if none specified, create one based on motion_mode
        if not agent._controllers:
            from pybullet_fleet.controller import DifferentialController, OmniController

            if agent._motion_mode == MotionMode.OMNIDIRECTIONAL:
                agent._controllers.append(
                    OmniController(
                        max_linear_vel=float(np.max(agent.max_linear_vel)),
                        max_angular_vel=float(np.max(agent.max_angular_vel)),
                        navigation_2d=spawn_params.navigation_2d,
                    )
                )
            else:
                agent._controllers.append(
                    DifferentialController(
                        max_linear_vel=float(np.max(agent.max_linear_vel)),
                        max_angular_vel=float(np.max(agent.max_angular_vel)),
                        navigation_2d=spawn_params.navigation_2d,
                    )
                )

        # Propagate navigation_2d to base controller (set_motion_mode may have
        # created a default controller before from_params could configure it)
        if agent._controllers and hasattr(agent._controllers[0], "_navigation_2d"):
            agent._controllers[0]._navigation_2d = spawn_params.navigation_2d

        return agent

    @classmethod
    def from_dict(cls, config: Dict[str, Any], sim_core=None) -> "Agent":
        """Create an Agent from a raw config dict.

        Combines :meth:`AgentSpawnParams.from_dict` and :meth:`from_params`
        in a single call.  Uses ``cls._spawn_params_cls`` so subclasses
        (e.g. Door, Elevator) automatically use their own SpawnParams.

        Args:
            config: Entity definition dict (as parsed from YAML).
            sim_core: Reference to simulation core (optional).

        Returns:
            ``cls`` instance.
        """
        params = cls._spawn_params_cls.from_dict(config)
        return cls.from_params(params, sim_core)

    @classmethod
    def from_mesh(
        cls,
        visual_shape: Optional[ShapeParams] = None,
        collision_shape: Optional[ShapeParams] = None,
        pose: Pose = None,
        mass: float = 0.0,
        max_linear_vel: Union[float, List[float]] = _AGT_D["max_linear_vel"],
        max_linear_accel: Union[float, List[float]] = _AGT_D["max_linear_accel"],
        max_angular_vel: Union[float, List[float]] = _AGT_D["max_angular_vel"],
        max_angular_accel: Union[float, List[float]] = _AGT_D["max_angular_accel"],
        motion_mode: Union[MotionMode, str] = MotionMode(_AGT_D["motion_mode"]),
        use_fixed_base: bool = _AGT_D["use_fixed_base"],
        collision_mode: CollisionMode = CollisionMode.NORMAL_3D,
        sim_core=None,
        name: Optional[str] = None,
        user_data: Optional[Dict[str, Any]] = None,
    ) -> "Agent":
        """
        Create a mesh-based Agent with flexible shape control.

        Args:
            visual_shape: ShapeParams for visual geometry
            collision_shape: ShapeParams for collision geometry
            pose: Initial Pose (position and orientation)
            mass: Robot mass (kg), 0.0 for kinematic control
            max_linear_vel: Maximum velocity (m/s) - float or [vx, vy, vz]
            max_linear_accel: Maximum acceleration (m/s²) - float or [ax, ay, az]
            max_angular_vel: Maximum angular velocity (rad/s)
            max_angular_accel: Maximum angular acceleration (rad/s²)
            motion_mode: MotionMode.OMNIDIRECTIONAL or MotionMode.DIFFERENTIAL
            use_fixed_base: If True, robot base is fixed and doesn't move
            sim_core: Reference to simulation core

        Returns:
            Agent instance

        Example::

            # Mesh visual with rotated frame + box collision
            agent = Agent.from_mesh(
                visual_shape=ShapeParams(
                    shape_type="mesh",
                    mesh_path="robot.obj",
                    mesh_scale=[1.0, 1.0, 1.0],
                    rgba_color=[0.2, 0.2, 0.2, 1.0],
                    frame_pose=Pose.from_euler(0, 0, 0, roll=np.pi/2, yaw=np.pi/2)
                ),
                collision_shape=ShapeParams(
                    shape_type="box",
                    half_extents=[0.2, 0.1, 0.2]
                ),
                pose=Pose.from_xyz(0, 0, 0.5),
                max_linear_vel=2.0,
                motion_mode=MotionMode.DIFFERENTIAL
            )
        """
        if pose is None:
            pose = Pose.from_xyz(0.0, 0.0, 0.0)

        # Use parent class's create_shared_shapes() for optimization
        # This dramatically reduces OpenGL object count when spawning many robots
        visual_id, collision_id = cls.create_shared_shapes(
            visual_shape=visual_shape,
            collision_shape=collision_shape,
        )

        # Create multibody using helper method
        body_id = cls._create_body_from_shapes(
            visual_id=visual_id,
            collision_id=collision_id,
            pose=pose,
            mass=mass,
        )

        # Create agent instance with Agent-specific parameters
        # collision_mode is passed through __init__ -> super().__init__() -> add_object
        # so the object is registered with the correct mode directly (no post-hoc transition)
        agent = cls(
            body_id=body_id,
            max_linear_vel=max_linear_vel,
            max_linear_accel=max_linear_accel,
            max_angular_vel=max_angular_vel,
            max_angular_accel=max_angular_accel,
            motion_mode=motion_mode,
            use_fixed_base=use_fixed_base,
            collision_mode=collision_mode,
            sim_core=sim_core,
            name=name,
            user_data=user_data,
            mass=mass,
        )

        return agent

    @classmethod
    def from_urdf(
        cls,
        urdf_path: str,
        pose: Pose = None,
        mass: Optional[float] = None,
        max_linear_vel: Union[float, List[float]] = _AGT_D["max_linear_vel"],
        max_linear_accel: Union[float, List[float]] = _AGT_D["max_linear_accel"],
        max_angular_vel: Union[float, List[float]] = _AGT_D["max_angular_vel"],
        max_angular_accel: Union[float, List[float]] = _AGT_D["max_angular_accel"],
        motion_mode: Union[MotionMode, str] = MotionMode(_AGT_D["motion_mode"]),
        use_fixed_base: bool = _AGT_D["use_fixed_base"],
        collision_mode: CollisionMode = CollisionMode.NORMAL_3D,
        sim_core=None,
        name: Optional[str] = None,
        user_data: Optional[Dict[str, Any]] = None,
        ik_params: Optional[IKParams] = None,
        joint_tolerance: Optional[Union[float, list, dict]] = None,
    ) -> "Agent":
        """
        Create a URDF-based Agent (with joints).

        Args:
            urdf_path: Robot model name (e.g. ``"panda"``) or path to URDF file.
                Names are resolved via :func:`~pybullet_fleet.robot_models.resolve_model`
            pose: Initial Pose (position and orientation). Defaults to origin
            mass: Mass override.
                - None (default): Use URDF file's mass values as-is
                - 0.0: Override all links to mass=0 for kinematic control (no physics)
            max_linear_vel: Maximum velocity (m/s) - float or [vx, vy, vz]
            max_linear_accel: Maximum acceleration (m/s²) - float or [ax, ay, az]
            max_angular_vel: Maximum angular velocity (rad/s)
            max_angular_accel: Maximum angular acceleration (rad/s²)
            motion_mode: MotionMode.OMNIDIRECTIONAL or MotionMode.DIFFERENTIAL
            use_fixed_base: If True, robot base is fixed in space
            sim_core: Reference to simulation core
            ik_params: IK solver configuration (default: ``IKParams()``).
            joint_tolerance: Default joint tolerance for convergence.
                See :attr:`joint_tolerance` property for details.

        Returns:
            Agent instance

        Note:
            - mass=None (default): Uses URDF file's mass values for physics simulation
            - mass=0.0: Override all links to mass=0 for kinematic control (no physics)

        Example::

            # Use URDF mass values (physics enabled) — mass defaults to None
            robot = Agent.from_urdf(urdf_path="arm_robot.urdf")

            # Resolve by model name
            robot = Agent.from_urdf(urdf_path="panda", use_fixed_base=True)

            # Kinematic control (no physics)
            robot = Agent.from_urdf(urdf_path="arm_robot.urdf", mass=0.0)
        """
        urdf_path = resolve_model(urdf_path)

        if pose is None:
            pose = Pose.from_xyz(0.0, 0.0, 0.0)

        # Get position and orientation from Pose
        position, orientation = pose.as_position_orientation()

        _client = sim_core._client if sim_core is not None else 0
        body_id = p.loadURDF(
            urdf_path,
            position,
            orientation,
            useFixedBase=use_fixed_base,
            flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
            physicsClientId=_client,
        )

        # Override mass if explicitly set to 0.0 (kinematic control)
        # mass=None (default) means use URDF's mass values as-is
        if mass == 0.0:
            # Kinematic control: set mass to 0 for all links
            # This prevents gravity and inertia from affecting the robot
            p.changeDynamics(body_id, -1, mass=0.0, physicsClientId=_client)
            num_joints = p.getNumJoints(body_id, physicsClientId=_client)
            for joint_idx in range(num_joints):
                p.changeDynamics(body_id, joint_idx, mass=0.0, physicsClientId=_client)

        # Resolve the mass value to pass to the constructor.
        # When mass=None, compute total URDF mass (all links) so that
        # is_kinematic is correctly False for physics robots, even when
        # useFixedBase=True makes the base link mass 0.
        if mass is None:
            num_joints = p.getNumJoints(body_id, physicsClientId=_client)
            total_mass = p.getDynamicsInfo(body_id, -1, physicsClientId=_client)[0]
            for j in range(num_joints):
                total_mass += p.getDynamicsInfo(body_id, j, physicsClientId=_client)[0]
            resolved_mass = total_mass
        else:
            resolved_mass = mass

        # Create agent instance (SimObject.__init__ handles auto-registration)
        # collision_mode is passed through __init__ -> super().__init__() -> add_object
        agent = cls(
            body_id=body_id,
            urdf_path=urdf_path,
            max_linear_vel=max_linear_vel,
            max_linear_accel=max_linear_accel,
            max_angular_vel=max_angular_vel,
            max_angular_accel=max_angular_accel,
            motion_mode=motion_mode,
            use_fixed_base=use_fixed_base,
            collision_mode=collision_mode,
            sim_core=sim_core,
            name=name,
            user_data=user_data,
            mass=resolved_mass,
            ik_params=ik_params,
            joint_tolerance=joint_tolerance,
        )

        return agent

    def set_goal_pose(
        self,
        goal: Pose,
        direction: Union[MovementDirection, str, None] = None,
    ):
        """
        Set goal pose for the robot to move to.

        Note: This is ignored if the robot has a fixed base.

        Args:
            goal: Target Pose (compatible with ROS2 geometry_msgs/Pose)
            direction: Movement direction override, or ``None`` to use
                the controller's ``default_direction``.
        """
        # Single goal is just a path with one waypoint
        self.set_path([goal], direction=direction)

    def set_motion_mode(self, mode: Union[MotionMode, str]) -> bool:
        """
        Set motion mode for the robot.

        Creates and assigns the matching controller (OmniController or
        DifferentialController).  Also initializes legacy TPI instances
        for backward compatibility.

        Note: Motion mode cannot be changed while the robot is moving.

        Args:
            mode: MotionMode.OMNIDIRECTIONAL or MotionMode.DIFFERENTIAL (or string)

        Returns:
            True if mode was changed successfully, False if robot is moving
        """
        if self._is_moving:
            self._log.warning("Cannot change motion mode while robot is moving. Call stop() first.")
            return False

        # Normalize to enum
        if isinstance(mode, str):
            mode = MotionMode(mode)

        self._motion_mode = mode

        # Create matching controller
        from pybullet_fleet.controller import DifferentialController, OmniController

        if mode == MotionMode.OMNIDIRECTIONAL:
            new_ctrl = OmniController(
                max_linear_vel=float(np.max(self.max_linear_vel)),
                max_angular_vel=float(np.max(self.max_angular_vel)),
            )
        elif mode == MotionMode.DIFFERENTIAL:
            new_ctrl = DifferentialController(
                max_linear_vel=float(np.max(self.max_linear_vel)),
                max_angular_vel=float(np.max(self.max_angular_vel)),
            )
        else:
            new_ctrl = None
        self.set_controller(new_ctrl)

        return True

    def _clear_path_visualization(self):
        """Clear all path visualization debug lines."""
        for debug_id in self._path_debug_ids:
            try:
                p.removeUserDebugItem(debug_id, physicsClientId=self._pid)
            except Exception:  # noqa: B902
                pass  # Ignore errors if item was already removed
        self._path_debug_ids.clear()

    def _visualize_path(self, path: List[Pose]):
        """
        Visualize a path as debug lines in PyBullet.

        Args:
            path: List of waypoints to visualize
        """
        if not self.path_visualize or len(path) < 2:
            return

        # Default color: cyan [0, 1, 1]
        color = self.path_visualize_color if self.path_visualize_color is not None else [0, 1, 1]
        width = self.path_visualize_width

        # Draw lines between consecutive waypoints
        for i in range(len(path) - 1):
            start_pos = path[i].position
            end_pos = path[i + 1].position

            debug_id = p.addUserDebugLine(
                start_pos,
                end_pos,
                lineColorRGB=color,
                lineWidth=width,
                lifeTime=0,  # Permanent until manually removed
                physicsClientId=self._pid,
            )
            self._path_debug_ids.append(debug_id)

    def set_path(
        self,
        path: List[Pose],
        auto_approach: bool = True,
        final_orientation_align: bool = True,
        direction: Union[MovementDirection, str, None] = None,
    ):
        """
        Set a path (list of waypoints) for the robot to follow.

        Args:
            path: List of Pose waypoints to follow in sequence
            auto_approach: If True, automatically add straight-line approach from current position
                          to first waypoint if distance > threshold (default: True)
            final_orientation_align: If True, add final rotation to match last waypoint's orientation
                                    after reaching the position (default: True)
            direction: Movement direction override, or ``None`` to use the controller's
                      ``default_direction``.  Can be MovementDirection enum or string
                      ("auto", "forward", "backward").
                      Only used in differential drive mode (MotionMode.DIFFERENTIAL).
                      In omnidirectional mode, this parameter is ignored.
        """
        if self.use_fixed_base:
            self._log.warning("Cannot set path for fixed-base robot")
            return

        if len(path) == 0:
            self._log.warning("Empty path provided")
            return

        # Normalize direction string → enum (None passes through to controller)
        if isinstance(direction, str):
            direction = MovementDirection(direction)

        # Get current position
        current_pose = self.get_pose()
        current_pos = np.array(current_pose.position)
        first_waypoint_pos = np.array(path[0].position)

        # Check if we need to add approach waypoint
        distance_to_first = np.linalg.norm(first_waypoint_pos - current_pos)
        approach_threshold = 0.5  # meters - add approach if farther than this

        final_path = []

        # Add approach waypoint if needed
        if auto_approach and distance_to_first > approach_threshold:
            # Add current position as approach waypoint
            # This creates a path: current_pose -> path[0] -> path[1] -> ...
            final_path.append(current_pose)
            self._log.debug(
                lambda: f"Added approach waypoint at current position (distance to first: {distance_to_first:.2f}m)"
            )

        # Add all path waypoints
        final_path.extend(path)

        self._is_moving = True

        # Clear previous path visualization and visualize new path
        self._clear_path_visualization()
        self._visualize_path(final_path)

        # Delegate all path state and trajectory init to the base controller
        self._controllers[0].set_path(
            self,
            final_path,
            final_orientation_align=final_orientation_align,
            direction=direction,
        )

    def _reset_pybullet_velocity(self) -> None:
        """Zero residual physics velocity (prevents drift in kinematic mode)."""
        p.resetBaseVelocity(self.body_id, [0, 0, 0], [0, 0, 0], physicsClientId=self._pid)

    # ============================================================================
    # Action Queue System
    # ============================================================================

    def add_action(self, action: Action):
        """
        Add an action to the execution queue.

        Actions are executed sequentially in the order they are added.

        Args:
            action: Action instance to add to queue

        Example::

            agent.add_action(MoveAction(path=my_path))
            agent.add_action(WaitAction(duration=5.0))
        """
        self._action_queue.append(action)
        self._log.info(f"Added {action.__class__.__name__} to queue (queue size: {len(self._action_queue)})")

    def add_action_sequence(self, actions: List[Action]):
        """
        Add multiple actions to the execution queue.

        Args:
            actions: List of Action instances to add

        Example::

            agent.add_action_sequence([
                MoveAction(path=path1),
                PickAction(target_object_id=obj.body_id),
                MoveAction(path=path2),
                DropAction(drop_pose=Pose(position=[10, 5, 0]))
            ])
        """
        self._action_queue.extend(actions)
        self._log.info(f"Added {len(actions)} actions to queue (queue size: {len(self._action_queue)})")

    def clear_actions(self):
        """
        Clear all actions from the queue and cancel current action.
        """
        self._action_queue.clear()
        if self._current_action is not None:
            self._current_action.cancel()
            self._current_action = None
        self._log.info("Cleared all actions")

    def get_current_action(self) -> Optional[Action]:
        """
        Get the currently executing action.

        Returns:
            Current Action instance, or None if no action is executing
        """
        return self._current_action

    def get_action_queue_size(self) -> int:
        """
        Get the number of actions waiting in the queue.

        Returns:
            Number of actions in queue
        """
        return len(self._action_queue)

    def is_action_queue_empty(self) -> bool:
        """
        Check if action queue is empty and no action is currently executing.

        Returns:
            True if no actions are queued or executing, False otherwise
        """
        return len(self._action_queue) == 0 and self._current_action is None

    def _update_actions(self, dt: float):
        """
        Update action execution system.

        This is called internally by update() to process the action queue.

        Args:
            dt: Time step (seconds)
        """
        # If no current action, try to get next one from queue
        if self._current_action is None:
            if self._action_queue:
                self._current_action = self._action_queue.pop(0)
                self._log.info(
                    f"Starting {self._current_action.__class__.__name__} " f"(remaining in queue: {len(self._action_queue)})"
                )
                # --- action_started event ---
                if self.sim_core is not None:
                    self.sim_core.events.emit(SimEvents.ACTION_STARTED, agent=self, action=self._current_action)
                if self._has_entity_events():
                    self.events.emit(SimEvents.ACTION_STARTED, action=self._current_action)
            else:
                return  # No actions to execute

        # Execute current action
        is_complete = self._current_action.execute(self, dt)

        if is_complete:
            action_name = self._current_action.__class__.__name__
            status = self._current_action.status

            if status == ActionStatus.COMPLETED:
                self._log.info(f"{action_name} completed successfully")
            elif status == ActionStatus.FAILED:
                self._log.error(f"{action_name} failed: {self._current_action.error_message}")
            elif status == ActionStatus.CANCELLED:
                self._log.warning(f"{action_name} was cancelled")

            # --- action_completed event ---
            finished_action = self._current_action
            if self.sim_core is not None:
                self.sim_core.events.emit(SimEvents.ACTION_COMPLETED, agent=self, action=finished_action, status=status)
            if self._has_entity_events():
                self.events.emit(SimEvents.ACTION_COMPLETED, action=finished_action, status=status)

            # Move to next action
            self._current_action = None

    def _compute_use_kinematic_joints(self) -> bool:
        """Compute whether joints need kinematic interpolation.

        True when:
        - The agent itself is kinematic (mass=0), OR
        - The simulation has physics disabled (sim_core.physics=False),
          meaning p.stepSimulation() is never called so motor control
          (setJointMotorControl2) would have no effect.

        Called once at init; result cached in ``_use_kinematic_joints``.
        """
        if self.is_kinematic:
            return True
        if self.sim_core is not None:
            if not hasattr(self.sim_core, "_params"):
                self._log.warning(
                    "sim_core has no '_params' attribute; " "assuming physics=True (joints will use motor control)"
                )
                return False
            if not self.sim_core._params.physics:
                if not Agent._kinematic_joints_physics_off_logged:
                    Agent._kinematic_joints_physics_off_logged = True
                    self._log.info(
                        "sim_core has physics=False; URDF joints will use kinematic "
                        "interpolation instead of motor control (resetJointState "
                        "instead of setJointMotorControl2)"
                    )
                return True
        return False

    def _update_kinematic_joints(self, dt: float) -> bool:
        """Interpolate joints toward targets for kinematic robots (mass=0).

        Called from update() each step. Mirrors the role of stepSimulation()
        for physics-mode motor control.

        Each joint moves at most ``velocity_limit * dt`` per step, where
        velocity_limit comes from the URDF ``<limit velocity="...">`` attribute.
        Falls back to 2.0 rad/s for revolute or 0.5 m/s for prismatic
        if the URDF limit is 0 or missing.

        Iterates ``_last_joint_targets`` and skips joints that have already
        reached their target (``abs(diff) < 1e-7``).  Entries are **never**
        deleted so that ``are_joints_at_targets()`` can query them later.

        Returns:
            True if any joint position changed this step, False otherwise.
        """
        if not self._last_joint_targets:
            return False

        any_moved = False
        for joint_index, target in self._last_joint_targets.items():
            current_pos = self._kinematic_joint_positions.get(joint_index, 0.0)
            diff = target - current_pos
            if abs(diff) < 1e-7:
                continue  # Already at target — skip
            # URDF velocity limit: joint_info[joint_index][11] is maxVelocity
            max_vel = self.joint_info[joint_index][11]
            if max_vel <= 0:
                joint_type = self.joint_info[joint_index][2]
                if joint_type == p.JOINT_PRISMATIC:
                    max_vel = self._KINEMATIC_PRISMATIC_FALLBACK_VELOCITY
                else:
                    max_vel = self._KINEMATIC_JOINT_FALLBACK_VELOCITY
            max_step = max_vel * dt
            if abs(diff) <= max_step:
                new_pos = target
            else:
                new_pos = current_pos + math.copysign(max_step, diff)
            p.resetJointState(self.body_id, joint_index, new_pos, physicsClientId=self._pid)
            self._kinematic_joint_positions[joint_index] = new_pos
            any_moved = True
        return any_moved

    def update(self, dt: float) -> bool:
        """
        Update robot position towards goal with velocity/acceleration constraints.

        Supports two motion modes:
        - MotionMode.OMNIDIRECTIONAL: Move in any direction without rotating first (smooth trajectories)
        - MotionMode.DIFFERENTIAL: Rotate to face target, then move forward (smooth trajectories)

        Also processes action queue for high-level task execution.

        Note: This does nothing if the robot has a fixed base.

        Args:
            dt: Time step (seconds)

        Returns:
            True if the robot moved (position, orientation, or joint state changed),
            False otherwise
        """
        # Per-entity pre_update event (inline check — ~30ns when no handlers)
        if self._events is not None:
            self._events.emit(SimEvents.PRE_UPDATE, dt=dt)

        # Process action queue first (actions may set goals/paths)
        self._update_actions(dt)

        # Kinematic joint interpolation (mirrors stepSimulation role for mass=0 URDF robots
        # or when physics is disabled in sim_core)
        joints_moved = False
        if self.is_urdf_robot() and self._use_kinematic_joints:
            joints_moved = self._update_kinematic_joints(dt)

        moved = False
        if not self.use_fixed_base:
            if self._controllers:
                # Execute controllers in reversed order (high-level first)
                for ctrl in reversed(self._controllers):
                    ctrl_moved = ctrl.compute(self, dt)
                    moved = moved or ctrl_moved
            elif self._is_moving:
                self._log.warning("No controller set — movement ignored. Use set_controller().")
                moved = False

        # Always update attached objects for URDF robots AFTER base motion
        # so that objects attached to links track correctly both when
        # the base moves and when only arm joints move.
        if self.is_urdf_robot() and self.attached_objects:
            self.update_attached_objects_kinematics()

        result = moved or joints_moved

        # Per-entity post_update event (inline check — ~30ns when no handlers)
        if self._events is not None:
            self._events.emit(SimEvents.POST_UPDATE, dt=dt, moved=result)

        # Return True if any geometry changed (base moved OR joints moved)
        # so that sim_core adds this agent to _moved_this_step for collision checks.
        return result

    def get_velocity(self) -> np.ndarray:
        """
        Get current velocity vector.

        Returns:
            Velocity [vx, vy, vz] in m/s
        """
        return self._current_velocity.copy()

    def set_controller(self, controller: Optional[Any] = None) -> None:
        """Set or replace the base (index-0) movement controller.

        When a controller is set, :meth:`update` delegates movement to
        ``controller.compute(agent, dt)``.

        Note: A default controller is always assigned in ``from_params()``.
        Setting ``None`` disables all movement.

        Args:
            controller: A :class:`~pybullet_fleet.controller.Controller`
                instance, or ``None`` to disable movement.
        """
        if controller is None:
            self._controllers.clear()
        elif self._controllers:
            self._controllers[0] = controller
        else:
            self._controllers.append(controller)

    def add_controller(self, controller) -> None:
        """Append a high-level controller to the chain.

        High-level controllers (patrol, random walk) are executed *before*
        the base navigation controller in :meth:`update`.

        Args:
            controller: A :class:`~pybullet_fleet.controller.Controller` instance.
        """
        self._controllers.append(controller)

    def remove_controller(self, controller) -> None:
        """Remove a non-base controller from the chain.

        The base controller (index 0) cannot be removed via this method.
        Use :meth:`set_controller` to replace it.

        Args:
            controller: Controller instance to remove.
        """
        if controller in self._controllers[1:]:
            self._controllers.remove(controller)

    @property
    def controller(self):
        """Return the base (index-0) controller, or ``None`` if the chain is empty."""
        return self._controllers[0] if self._controllers else None

    @property
    def _controller(self):
        """Backward-compatible alias for the base controller."""
        return self._controllers[0] if self._controllers else None

    @_controller.setter
    def _controller(self, value):
        """Backward-compatible setter — delegates to set_controller()."""
        self.set_controller(value)

    def stop(self):
        """Stop robot movement and clear goal and path."""
        self._is_moving = False
        self._current_velocity[:] = 0.0
        self._current_angular_velocity = 0.0
        self._reset_pybullet_velocity()
        self._clear_path_visualization()
        # Notify base controller to reset all internal state (velocity, pose, path)
        if self._controllers:
            self._controllers[0].on_stop(self)

    # ========================================
    # URDF-specific methods (joint control)
    # ========================================

    def is_urdf_robot(self) -> bool:
        """Check if this robot is URDF-based (has joints)."""
        return self.urdf_path is not None

    def get_num_joints(self) -> int:
        """Get number of joints (0 for mesh robots)."""
        return len(self.joint_info)

    def get_joint_state(self, joint_index: int) -> Tuple[float, float]:
        """
        Get joint state (position and velocity).

        For kinematic robots (mass=0), returns the cached position and
        velocity=0.0, avoiding a PyBullet C call.  For physics robots
        the value is read from ``p.getJointState()``.

        Args:
            joint_index: Joint index (0-based)

        Returns:
            (position, velocity) tuple

        Example::

            pos, vel = robot.get_joint_state(0)
        """
        if not self.is_urdf_robot():
            self._log.warning("get_joint_state() only works for URDF robots")
            return (0.0, 0.0)

        if self._use_kinematic_joints and joint_index in self._kinematic_joint_positions:
            return (self._kinematic_joint_positions[joint_index], 0.0)

        joint_state = p.getJointState(self.body_id, joint_index, physicsClientId=self._pid)
        return (joint_state[0], joint_state[1])  # position, velocity

    def get_joint_state_by_name(self, joint_name: str) -> tuple:
        """
        Get (position, velocity) for a single joint by name.
        Returns (position, velocity) or (0.0, 0.0) if not found.
        """
        idx = resolve_joint_index(self.body_id, joint_name)
        if idx == -1:
            self._log.warning(f"Joint name '{joint_name}' not found.")
            return (0.0, 0.0)
        return self.get_joint_state(idx)

    def get_all_joints_state(self) -> list:
        """
        Return a list of (position, velocity) for all joints.
        Example: [(pos0, vel0), (pos1, vel1), ...]
        """
        if not self.is_urdf_robot():
            self._log.warning("get_all_joints_state() only works for URDF robots")
            return []
        return [self.get_joint_state(i) for i in range(self.get_num_joints())]

    def get_all_joints_state_by_name(self) -> dict:
        """
        Return a dict of {joint_name: (position, velocity)} for all joints.
        Example: {joint_name: (pos, vel), ...}
        """
        if not self.is_urdf_robot():
            self._log.warning("get_all_joints_state_by_name() only works for URDF robots")
            return {}
        joint_names = [self.joint_info[i][1].decode("utf-8") for i in range(self.get_num_joints())]
        result = {}
        for name in joint_names:
            result[name] = self.get_joint_state_by_name(name)
        return result

    def get_joints_state_by_name(self, joint_names: list) -> dict:
        """
        Return a dict of {joint_name: (position, velocity)} for the specified joint_names.
        Example: {joint_name: (pos, vel), ...}
        """
        result = {}
        for name in joint_names:
            idx = resolve_joint_index(self.body_id, name)
            if idx == -1:
                self._log.warning(f"Joint name '{name}' not found.")
                continue
            result[name] = self.get_joint_state(idx)
        return result

    def set_joint_target(self, joint_index: int, target_position: float, max_force: float = 500.0):
        """
        Set target position for a joint (for URDF robots only).

        Args:
            joint_index: Joint index (0-based)
            target_position: Target position (radians for revolute, meters for prismatic)
            max_force: Maximum force to apply

        Example::

            robot.set_joint_target(0, 1.57)  # Move first joint to 90 degrees
        """
        if not self.is_urdf_robot():
            self._log.warning("set_joint_target() only works for URDF robots")
            return

        if joint_index >= len(self.joint_info):
            self._log.warning(f"joint_index {joint_index} out of range (max: {len(self.joint_info)-1})")
            return

        # Skip fixed joints — they cannot be driven
        if self.joint_info[joint_index][2] == p.JOINT_FIXED:
            return

        # Always record for are_joints_at_targets() / PoseAction completion
        self._last_joint_targets[joint_index] = target_position

        if not self._use_kinematic_joints:
            # Physics mode (mass>0 with stepSimulation): motor control
            p.setJointMotorControl2(
                bodyUniqueId=self.body_id,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_position,
                force=max_force,
                physicsClientId=self._pid,
            )

    def set_joint_target_by_name(self, joint_name: str, target_position: float, max_force: float = 500.0):
        """
        Set target position for a joint by joint name.
        """
        joint_index = resolve_joint_index(self.body_id, joint_name)
        if joint_index == -1:
            self._log.warning(f"Joint name '{joint_name}' not found.")
            return
        self.set_joint_target(joint_index, target_position, max_force)

    def set_all_joints_targets(self, target_positions: List[float], max_force: float = 500.0):
        """
        Set target positions for all joints at once.

        Args:
            target_positions: List of target positions for each joint
            max_force: Maximum force to apply

        Example::

            robot.set_all_joints_targets([0.0, 1.57, -1.57, 0.0])
        """
        if not self.is_urdf_robot():
            self._log.warning("set_all_joints_targets() only works for URDF robots")
            return

        if len(target_positions) != len(self.joint_info):
            self._log.warning(f"Expected {len(self.joint_info)} targets, got {len(target_positions)}")
            return

        for i, target in enumerate(target_positions):
            self.set_joint_target(i, target, max_force)

    def set_joints_targets_by_name(self, joint_targets: dict, max_force: float = 500.0):
        """
        Set multiple joint targets by dict {joint_name: target_position} (partial update allowed).
        """
        for joint_name, target in joint_targets.items():
            joint_index = resolve_joint_index(self.body_id, joint_name)
            if joint_index == -1:
                self._log.warning(f"Joint name '{joint_name}' not found.")
                continue
            self.set_joint_target(joint_index, target, max_force)

    def set_joints_targets(self, targets: Union[list, dict], max_force: float = 500.0):
        """
        Set joint targets (accepts both list and dict).

        Args:
            targets: List of target positions for all joints, or dict {joint_name: position}
            max_force: Maximum force to apply

        Example::

            # Using list
            robot.set_joints_targets([0.0, 1.57, -1.57, 0.0])

            # Using dict
            robot.set_joints_targets({"joint1": 1.57, "joint2": -1.57})
        """
        if isinstance(targets, dict):
            self.set_joints_targets_by_name(targets, max_force)
        else:
            self.set_all_joints_targets(targets, max_force)

    def is_joint_at_target(
        self, joint_index: int, target: float, tolerance: Optional[Union[float, list, dict]] = None
    ) -> bool:
        """
        Check if a single joint (by index) is within tolerance of the target position.

        Args:
            joint_index: Joint index (0-based).
            target: Target position (radians for revolute, metres for prismatic).
            tolerance: Position tolerance — ``float``, ``list``, ``dict``,
                or ``None`` (→ ``self.joint_tolerance``).  Resolved via
                ``_resolve_joint_tolerance(tolerance, joint_index)``.
        """
        if not self.is_urdf_robot():
            self._log.warning("is_joint_at_target() only works for URDF robots")
            return False
        if joint_index < 0 or joint_index >= len(self.joint_info):
            self._log.warning(f"joint_index {joint_index} out of range (max: {len(self.joint_info)-1})")
            return False
        tol = self._resolve_joint_tolerance(tolerance, joint_index)
        pos, _ = self.get_joint_state(joint_index)
        return abs(pos - target) <= tol

    def is_joint_at_target_by_name(
        self, joint_name: str, target: float, tolerance: Optional[Union[float, list, dict]] = None
    ) -> bool:
        """
        Check if a single joint (by name) is within tolerance of the target position.

        Args:
            joint_name: Joint name string.
            target: Target position (radians for revolute, metres for prismatic).
            tolerance: Position tolerance — ``float``, ``list``, ``dict``,
                or ``None`` (→ ``self.joint_tolerance``).  Resolved via
                ``_resolve_joint_tolerance()`` using the joint's index.
        """
        joint_index = resolve_joint_index(self.body_id, joint_name)
        if joint_index == -1:
            return False
        return self.is_joint_at_target(joint_index, target, tolerance)

    @property
    def last_joint_targets(self) -> Dict[int, float]:
        """Read-only copy of last commanded joint targets."""
        return dict(self._last_joint_targets)

    def are_all_joints_at_targets(
        self, target_positions: Optional[list] = None, tolerance: Optional[Union[float, list, dict]] = None
    ) -> bool:
        """Check if joints are at target positions.

        Args:
            target_positions: List of target positions for each joint.
                If None, uses ``_last_joint_targets`` (last commanded targets).
            tolerance: ``float``, ``list``, ``dict``, or ``None``.
                If None, uses ``self.joint_tolerance``.
                Passed through to ``_resolve_joint_tolerance()`` per joint.

                .. note::

                    A scalar tolerance applies the same value to all joints.
                    For mixed prismatic (metres) / revolute (radians) robots,
                    consider a ``dict`` keyed by joint name or set
                    ``self.joint_tolerance`` to a dict.

        Returns:
            True if all joints are within tolerance of their targets.
        """
        if not self.is_urdf_robot():
            self._log.warning("are_all_joints_at_targets() only works for URDF robots")
            return False

        if target_positions is None:
            targets = self._last_joint_targets
        else:
            targets = {i: t for i, t in enumerate(target_positions)}

        if not targets:
            if target_positions is None:
                self._log.warning("are_all_joints_at_targets() called but no targets have been set")
            return True  # Nothing to check → vacuously true

        for idx, target in targets.items():
            if not self.is_joint_at_target(idx, target, tolerance):
                return False
        return True

    def are_joints_at_targets_by_name(self, joint_targets: dict, tolerance: Optional[Union[float, list, dict]] = None) -> bool:
        """
        Check if all specified joints (by name) are within tolerance.

        Args:
            joint_targets: ``{joint_name: target}``
            tolerance: ``float`` (all), ``dict`` (``{joint_name: tol}``),
                or ``None`` (→ ``self.joint_tolerance``).

                A ``list`` is also accepted but is resolved by **absolute
                joint index** (not by iteration order of *joint_targets*).
                For named-joint subsets, prefer a ``dict`` to avoid
                unexpected index-based fallback.

                For mixed prismatic / revolute chains, use a ``dict`` so
                prismatic joints (metres) can have a tighter tolerance
                than revolute joints (radians).

        Returns:
            True if all joints are within tolerance, False otherwise.
        """
        for joint_name, target in joint_targets.items():
            joint_index = resolve_joint_index(self.body_id, joint_name)
            if joint_index == -1:
                return False
            if not self.is_joint_at_target(joint_index, target, tolerance):
                return False
        return True

    def are_joints_at_targets(
        self,
        targets: Union[list, dict, None] = None,
        tolerance: Optional[Union[float, list, dict]] = None,
    ) -> bool:
        """Check if joints are at target positions.

        Args:
            targets: List of target positions for all joints,
                dict ``{joint_name: position}``, or **None** to use the
                last commanded targets (``_last_joint_targets``).
            tolerance: Tolerance value(s) — ``float``, ``list``, ``dict``,
                or **None** (→ ``self.joint_tolerance``).

                For mixed prismatic / revolute chains, use a ``dict`` so
                prismatic joints (metres) can have a tighter tolerance
                than revolute joints (radians)::

                    agent.are_joints_at_targets(
                        targets,
                        tolerance={"rail_joint": 0.005, "elbow_to_wrist": 0.05},
                    )

        Returns:
            True if all joints are within tolerance, False otherwise.

        Example::

            # No args — check last commanded targets with agent's default tolerance
            if robot.are_joints_at_targets():
                print("All joints settled")

            # Using list
            if robot.are_joints_at_targets([0.0, 1.57, -1.57, 0.0], tolerance=0.01):
                print("Reached target")

            # Using dict
            if robot.are_joints_at_targets({"joint1": 1.57, "joint2": -1.57}, tolerance=0.01):
                print("Reached target")
        """
        if targets is None:
            return self.are_all_joints_at_targets(None, tolerance)
        if isinstance(targets, dict):
            return self.are_joints_at_targets_by_name(targets, tolerance)
        return self.are_all_joints_at_targets(targets, tolerance)

    # ========================================
    # Inverse Kinematics (IK)
    # ========================================

    def _get_end_effector_link_index(
        self,
        end_effector_link: Union[int, str, None] = None,
    ) -> int:
        """Resolve end-effector link index.

        Args:
            end_effector_link: Link index (int), link name (str), or None.
                If None, returns the last link index (last joint's child link).

        Returns:
            Link index (int). Returns -1 if robot has no joints.
        """
        if end_effector_link is not None:
            return resolve_link_index(self.body_id, end_effector_link)
        if not self.joint_info:
            self._log.warning("_get_end_effector_link_index: robot has no joints")
            return -1
        return len(self.joint_info) - 1

    @property
    def ik_params(self) -> IKParams:
        """Read-only access to the IK solver configuration."""
        return self._ik_params

    def _solve_ik(
        self,
        target_position: List[float],
        target_orientation: Optional[Tuple[float, float, float, float]] = None,
        ee_link_index: Optional[int] = None,
    ) -> List[float]:
        """Solve inverse kinematics for the given end-effector target.

        Uses a multi-seed iterative scheme on top of PyBullet's
        ``calculateInverseKinematics``.  The following seed strategies
        are tried:

        1. **Current joint positions** — works well when the robot is
           already close to the desired configuration.
        2. **Quartile seeds** — joint-range fractions specified by
           ``ik_params.seed_quartiles`` (default 25 %, 50 %, and 75 %),
           covering diverse regions of the joint space.

        For each seed, up to ``ik_params.max_outer_iterations`` refinement
        passes are performed (feeding the previous IK solution back as
        ``restPoses``).  The loop returns early when the FK end-effector
        position is within ``ik_params.convergence_threshold`` of the target.

        Args:
            target_position: Target EE position [x, y, z] in world frame.
            target_orientation: Target EE orientation as quaternion
                [qx, qy, qz, qw].  If None, only position is constrained.
            ee_link_index: Resolved end-effector link index (int).
                If None, auto-detects (last link).

        Returns:
            List of joint angles (one per joint).  Empty list if IK
            cannot be attempted (non-URDF robot, invalid EE link).
        """
        if not self.is_urdf_robot() or not self.joint_info:
            self._log.warning("_solve_ik: only works for URDF robots with joints")
            return []

        ee_index = ee_link_index if ee_link_index is not None else self._get_end_effector_link_index()
        if ee_index < 0:
            self._log.warning("_solve_ik: invalid end-effector link index")
            return []

        num_joints = len(self.joint_info)
        saved_positions = [self.get_joint_state(i)[0] for i in range(num_joints)]
        cfg = self._ik_params

        # Build movable-joint mapping.  PyBullet's calculateInverseKinematics
        # returns values only for movable joints (skips JOINT_FIXED).
        movable_indices = self._cached_movable_indices

        # Build limit arrays for movable joints only.
        # Joints NOT in the IK "free set" are locked at their current
        # position via a tiny range around the saved value.
        _LOCK_EPS = 1e-4  # tiny range to effectively freeze the joint

        # Determine which movable joints the IK solver is allowed to move.
        ik_names = cfg.ik_joint_names
        if ik_names is not None:
            # Resolve names to a set of joint indices for O(1) lookup.
            ik_free_indices: Optional[set] = set()
            for name in ik_names:
                idx = resolve_joint_index(self.body_id, name)
                if idx >= 0:
                    ik_free_indices.add(idx)
                else:
                    self._log.warning("_solve_ik: ik_joint_names entry '%s' not found", name)
        else:
            ik_free_indices = None  # auto-detect mode

        lower_limits: List[float] = []
        upper_limits: List[float] = []
        joint_ranges: List[float] = []
        for idx in movable_indices:
            lo, hi = self.joint_info[idx][8], self.joint_info[idx][9]
            if ik_free_indices is not None:
                # Explicit mode: lock if not in the free set
                should_lock = idx not in ik_free_indices
            else:
                # Auto mode: lock continuous joints (lower >= upper)
                should_lock = lo >= hi
            if should_lock:
                cur = saved_positions[idx]
                lo, hi = cur - _LOCK_EPS, cur + _LOCK_EPS
            lower_limits.append(lo)
            upper_limits.append(hi)
            joint_ranges.append(hi - lo)

        # Seeds in movable-joint space
        movable_saved = [saved_positions[i] for i in movable_indices]
        seed_strategies: List[List[float]] = [list(movable_saved)]
        for q in cfg.seed_quartiles:
            seed_strategies.append([lo + q * r for lo, r in zip(lower_limits, joint_ranges)])

        target_arr = np.array(target_position)
        best_angles: List[float] = []
        best_distance = float("inf")

        for seed in seed_strategies:
            rest_poses = list(seed)
            prev_angles: Optional[List[float]] = None
            for _ in range(cfg.max_outer_iterations):
                ik_kwargs = dict(
                    bodyUniqueId=self.body_id,
                    endEffectorLinkIndex=ee_index,
                    targetPosition=target_position,
                    lowerLimits=lower_limits,
                    upperLimits=upper_limits,
                    jointRanges=joint_ranges,
                    restPoses=rest_poses,
                    maxNumIterations=cfg.max_inner_iterations,
                    residualThreshold=cfg.residual_threshold,
                    physicsClientId=self._pid,
                )
                if target_orientation is not None:
                    ik_kwargs["targetOrientation"] = target_orientation

                joint_angles = list(p.calculateInverseKinematics(**ik_kwargs))

                # Early exit: solution unchanged from previous iteration
                if prev_angles is not None and joint_angles == prev_angles:
                    break

                # Map movable-joint IK output to actual joint indices
                for k, angle in enumerate(joint_angles):
                    p.resetJointState(self.body_id, movable_indices[k], angle, physicsClientId=self._pid)
                link_state = p.getLinkState(self.body_id, ee_index, computeForwardKinematics=1, physicsClientId=self._pid)
                distance = float(np.linalg.norm(np.array(link_state[0]) - target_arr))

                if distance < best_distance:
                    best_distance = distance
                    best_angles = joint_angles

                if distance < cfg.convergence_threshold:
                    break

                prev_angles = joint_angles
                rest_poses = joint_angles

            if best_distance < cfg.convergence_threshold:
                break

        # Restore original joint positions
        for i, pos in enumerate(saved_positions):
            p.resetJointState(self.body_id, i, pos, physicsClientId=self._pid)

        # Map movable-joint result back to full joint list
        full_angles = list(saved_positions)
        for k, angle in enumerate(best_angles):
            full_angles[movable_indices[k]] = angle
        return full_angles

    def _check_ee_pose(
        self,
        ee_link_index: int,
        target_position: List[float],
        target_orientation: Optional[Tuple[float, float, float, float]] = None,
        tolerance: float = 0.02,
        orientation_tolerance: float = 0.15,
    ) -> bool:
        """Core FK check: compare current EE pose against a target.

        Reads the EE link state via forward kinematics and checks whether
        the position (and optionally orientation) is within tolerance.

        This is the shared implementation used by both
        :meth:`_check_ik_reachability` (with temporary joint states) and
        :meth:`are_ee_at_target` (with current joint states).

        Args:
            ee_link_index: End-effector link index.  Returns False if < 0.
            target_position: Desired EE position [x, y, z].
            target_orientation: Desired EE orientation as quaternion
                [qx, qy, qz, qw].  If None, only position is checked.
            tolerance: Maximum Euclidean distance (m) to consider reached.
            orientation_tolerance: Maximum ``1 - |dot|`` between actual and
                target quaternions to consider orientation achieved (default 0.15).

        Returns:
            True if the EE is within *tolerance* of the target.
        """
        if ee_link_index < 0:
            return False

        link_state = p.getLinkState(self.body_id, ee_link_index, computeForwardKinematics=1, physicsClientId=self._pid)
        actual_pos = np.array(link_state[0])
        distance = float(np.linalg.norm(actual_pos - np.array(target_position)))

        if distance > tolerance:
            return False

        if target_orientation is not None:
            actual_orn = np.array(link_state[1])
            target_orn = np.array(target_orientation, dtype=float)
            # Normalize to handle non-unit quaternions from callers
            norm = np.linalg.norm(target_orn)
            if norm > 0:
                target_orn = target_orn / norm
            dot = float(min(abs(np.dot(actual_orn, target_orn)), 1.0))
            if (1.0 - dot) > orientation_tolerance:
                return False

        return True

    def _check_ik_reachability(
        self,
        joint_angles: List[float],
        target_position: List[float],
        ee_link_index: int,
        tolerance: float = 0.02,
        target_orientation: Optional[Tuple[float, float, float, float]] = None,
        orientation_tolerance: float = 0.15,
    ) -> bool:
        """Check whether an IK solution actually reaches the target.

        Temporarily applies ``joint_angles`` via ``resetJointState``,
        delegates to :meth:`_check_ee_pose` for the FK comparison, then
        restores the original joint positions.

        Args:
            joint_angles: IK solution (one angle per joint).
            target_position: Desired EE position [x, y, z].
            ee_link_index: End-effector link index.
            tolerance: Maximum Euclidean distance (m) to consider reachable.
            target_orientation: Desired EE orientation as quaternion
                [qx, qy, qz, qw].  If provided, orientation is also checked.
            orientation_tolerance: Maximum ``1 - |dot|`` between actual and
                target quaternions to consider orientation achieved (default 0.15).

        Returns:
            True if EE position (and optionally orientation) is within tolerance.
        """
        if ee_link_index < 0:
            return False

        # Save current positions
        num_joints = len(self.joint_info)
        saved = [self.get_joint_state(i)[0] for i in range(num_joints)]

        # Apply IK solution temporarily
        for i, angle in enumerate(joint_angles):
            p.resetJointState(self.body_id, i, angle, physicsClientId=self._pid)

        result = self._check_ee_pose(ee_link_index, target_position, target_orientation, tolerance, orientation_tolerance)

        # Restore original positions
        for i, pos in enumerate(saved):
            p.resetJointState(self.body_id, i, pos, physicsClientId=self._pid)

        return result

    def move_end_effector(
        self,
        target_position: List[float],
        target_orientation: Optional[Tuple[float, float, float, float]] = None,
        end_effector_link: Union[int, str, None] = None,
        max_force: float = 500.0,
        tolerance: Optional[float] = None,
    ) -> bool:
        """Set end-effector target position via inverse kinematics.

        Solves IK, checks reachability, and calls
        ``set_all_joints_targets()`` internally.  Actual movement happens
        in ``update()`` (fire-and-forget, like ``set_joint_target()``).

        The joint targets are **always** set (best-effort), even when the
        target is unreachable.  The return value tells the caller whether
        the IK solution actually reaches the desired position.

        Args:
            target_position: Target EE position [x, y, z] in world frame.
            target_orientation: Target EE orientation as quaternion [qx, qy, qz, qw].
            end_effector_link: End-effector link (int, str, or None for auto-detect).
            max_force: Maximum force for motor control (physics mode).
            tolerance: Euclidean distance (m) for reachability check.
                If None, uses ``ik_params.reachability_tolerance``.

        Returns:
            True if the IK solution places the EE within *tolerance* of
            the target (reachable).  False otherwise (best-effort targets
            are still set).

        Example::

            reachable = arm.move_end_effector([0.3, 0.0, 0.5])
            if not reachable:
                print("Target may not be fully reachable")
        """
        if not self.is_urdf_robot():
            self._log.warning("move_end_effector() only works for URDF robots")
            return False

        ee_index = self._get_end_effector_link_index(end_effector_link)

        # Disable rendering during IK solve + reachability check to prevent
        # visual flicker from temporary resetJointState calls.
        try:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=self._pid)
        except Exception:
            pass  # DIRECT mode or already disabled

        try:
            joint_angles = self._solve_ik(target_position, target_orientation, ee_index)
            if not joint_angles:
                return False

            tol = tolerance if tolerance is not None else self._ik_params.reachability_tolerance
            reachable = self._check_ik_reachability(
                joint_angles,
                target_position,
                ee_index,
                tol,
                target_orientation=target_orientation,
            )
        finally:
            try:
                p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=self._pid)
            except Exception:
                pass

        self.set_all_joints_targets(joint_angles, max_force=max_force)
        return reachable

    def are_ee_at_target(
        self,
        target_position: List[float],
        target_orientation: Optional[Tuple[float, float, float, float]] = None,
        end_effector_link: Union[int, str, None] = None,
        tolerance: float = 0.02,
        orientation_tolerance: float = 0.15,
    ) -> bool:
        """Check whether the end-effector is near the target pose.

        Resolves the EE link and delegates to :meth:`_check_ee_pose`.

        Args:
            target_position: Target EE position [x, y, z] in world frame.
            target_orientation: Target EE orientation as quaternion
                [qx, qy, qz, qw].  If None, only position is checked.
            end_effector_link: Link index, name, or None (auto-detect).
            tolerance: Maximum Euclidean distance (m) to consider reached.
            orientation_tolerance: Maximum ``1 - |dot|`` between actual and
                target quaternions (default 0.15).

        Returns:
            True if the EE is within *tolerance* of the target.
        """
        if not self.is_urdf_robot():
            self._log.warning("are_ee_at_target() only works for URDF robots")
            return False

        ee_index = self._get_end_effector_link_index(end_effector_link)
        return self._check_ee_pose(ee_index, target_position, target_orientation, tolerance, orientation_tolerance)

    def update_attached_objects_kinematics(self):
        """
        For URDF robots: update attached objects' pose to follow the parent link (kinematics attach).
        This is called every step for kinematic (mass=0) attached objects.
        """
        for obj in self.attached_objects:
            # Only update if attached to a link (not base) and no constraint (mass=0)
            if obj._attached_link_index >= 0 and getattr(obj, "_constraint_id", None) is None:
                # Get parent link's world coordinates
                link_state = p.getLinkState(
                    self.body_id, obj._attached_link_index, computeForwardKinematics=1, physicsClientId=self._pid
                )
                parent_pos, parent_orn = link_state[0], link_state[1]
                # Apply relative offset
                new_pos, new_orn = p.multiplyTransforms(
                    parent_pos, parent_orn, obj._attach_offset.position, obj._attach_offset.orientation
                )
                obj.set_pose_raw(new_pos, new_orn)

    def __repr__(self):
        pose = self.get_pose()
        fixed_str = " [FIXED]" if self.use_fixed_base else ""
        urdf_str = " [URDF]" if self.is_urdf_robot() else " [MESH]"
        return (
            f"Agent(id={self.body_id}{fixed_str}{urdf_str}, "
            f"pos={pose.position[:2]}, "
            f"vel={self._current_velocity[:2].tolist()}, "
            f"moving={self._is_moving})"
        )
