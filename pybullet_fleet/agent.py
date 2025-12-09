"""
core/agent.py
Agent class for goal-based position control with max velocity and acceleration constraints.
Supports both mobile (use_fixed_base=False) and fixed (use_fixed_base=True) robots.
Supports both Mesh and URDF loading.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from two_point_interpolation import TwoPointInterpolation

from pybullet_fleet.sim_object import Pose, SimObject, SimObjectSpawnParams


class MotionMode(str, Enum):
    """
    Motion mode for robot movement control.

    Attributes:
        OMNIDIRECTIONAL: Robot can move in any direction without rotating first (holonomic)
        DIFFERENTIAL: Robot must rotate to face target, then move forward (non-holonomic)
    """

    OMNIDIRECTIONAL = "omnidirectional"
    DIFFERENTIAL = "differential"


class DifferentialPhase(str, Enum):
    """
    Phase for differential drive motion control.

    Attributes:
        ROTATE: Robot is rotating to face the target direction
        FORWARD: Robot is moving forward towards the target
    """

    ROTATE = "rotate"
    FORWARD = "forward"


@dataclass
class AgentSpawnParams(SimObjectSpawnParams):
    """
    Agent spawn parameters extending SimObjectSpawnParams.

    These parameters define the physical properties, appearance, and initial state of an agent.
    They are used by Agent.from_params() and AgentManager.spawn_agents_grid().

    Supports both Mesh and URDF robots. Specify either mesh_path or urdf_path (not both).

    Attributes (in addition to SimObjectSpawnParams):
        urdf_path: Path to robot URDF file (for URDF-based robots with joints)
        max_linear_vel: Maximum velocity in m/s - float or [vx, vy, vz] (ignored if use_fixed_base=True)
        max_linear_accel: Maximum acceleration in m/s² - float or [ax, ay, az] (ignored if use_fixed_base=True)
        max_angular_vel: Maximum angular velocity in rad/s (for differential drive)
        max_angular_accel: Maximum angular acceleration in rad/s² (for differential drive, default: 10.0)
        motion_mode: MotionMode.OMNIDIRECTIONAL (move in any direction) or MotionMode.DIFFERENTIAL (rotate then move forward)
        visual_mesh_path: Optional separate visual mesh path (for Agent.from_mesh)
        use_fixed_base: If True, robot base is fixed and doesn't move (default: False)
        user_data: Optional dictionary for custom metadata (default: empty dict)

    Note:
        initial_pose is optional and mainly used by Agent.from_params().
        AgentManager.spawn_agents_grid() calculates positions automatically and ignores this field.
    """

    urdf_path: Optional[str] = None
    max_linear_vel: Union[float, List[float]] = 2.0
    max_linear_accel: Union[float, List[float]] = 5.0
    max_angular_vel: Union[float, List[float]] = 3.0
    max_angular_accel: Union[float, List[float]] = 10.0
    motion_mode: Union[MotionMode, str] = MotionMode.OMNIDIRECTIONAL
    visual_mesh_path: Optional[str] = None
    use_fixed_base: bool = False
    user_data: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        """Validate that either mesh_path or urdf_path is specified."""
        if self.mesh_path is None and self.urdf_path is None:
            raise ValueError("Either mesh_path or urdf_path must be specified")
        if self.mesh_path is not None and self.urdf_path is not None:
            raise ValueError("Cannot specify both mesh_path and urdf_path")

    @classmethod
    def from_dict(cls, config: Dict[str, Any]):
        """
        Create AgentSpawnParams from configuration dictionary.

        Args:
            config: Dictionary with robot parameters including:
                    - mesh_path or urdf_path (required)
                    - initial_pose: Pose object (optional)
                    - max_linear_vel, max_linear_accel, use_fixed_base, etc. (optional)

        Returns:
            AgentSpawnParams instance

        Example:
            config = {
                'urdf_path': 'robots/mobile_robot.urdf',
                'initial_pose': Pose.from_xyz(1.0, 2.0, 0.0),
                'max_linear_vel': 2.0,
                'max_linear_accel': 5.0,
                'use_fixed_base': False
            }
            params = AgentSpawnParams.from_dict(config)
        """
        # Get motion_mode and convert string to enum if needed
        motion_mode_value = config.get("motion_mode", MotionMode.OMNIDIRECTIONAL)
        if isinstance(motion_mode_value, str):
            motion_mode_value = MotionMode(motion_mode_value)

        return cls(
            mesh_path=config.get("mesh_path"),
            urdf_path=config.get("urdf_path"),
            initial_pose=config.get("initial_pose"),
            max_linear_vel=config.get("max_linear_vel", 2.0),
            max_linear_accel=config.get("max_linear_accel", 5.0),
            max_angular_vel=config.get("max_angular_vel", 3.0),
            max_angular_accel=config.get("max_angular_accel", 10.0),
            motion_mode=motion_mode_value,
            mesh_scale=config.get("mesh_scale", [1.0, 1.0, 1.0]),
            collision_half_extents=config.get("collision_half_extents", [0.2, 0.1, 0.2]),
            rgba_color=config.get("rgba_color", [0.2, 0.2, 0.2, 1.0]),
            mass=config.get("mass", 0.0),
            use_fixed_base=config.get("use_fixed_base", False),
        )


class Agent(SimObject):
    """
    Agent class with goal-based position control.

    Inherits from SimObject to get attach/detach and callback functionality.
    Supports both mobile robots (use_fixed_base=False) and fixed robots (use_fixed_base=True).
    Supports both Mesh and URDF loading.

    Features:
    - Accepts Pose goals (compatible with ROS2 geometry_msgs/Pose)
    - Max velocity and acceleration constraints (for mobile robots)
    - Current pose query API
    - Shared shape optimization for fast spawning
    - Fixed robot support (non-moving objects)
    - URDF support with joint control
    - Attach/detach objects (inherited from SimObject)

    This class is designed to be eventually controlled by ROS2 nodes,
    so the API follows ROS message conventions.
    """

    # Class-level shared shapes (for optimization)
    _shared_shapes: Dict[str, Tuple[int, int]] = {}

    @staticmethod
    def _normalize_vector_param(value: Union[float, List[float]], param_name: str, dim: int = 3) -> np.ndarray:
        """
        Normalize a parameter to a numpy array of specified dimension.

        Args:
            value: Float (applied to all dimensions) or list of floats (per-dimension)
            param_name: Parameter name for error messages
            dim: Number of dimensions (default: 3)

        Returns:
            Numpy array of shape (dim,)

        Raises:
            ValueError: If list length doesn't match dimension
        """
        if isinstance(value, (int, float)):
            return np.array([value] * dim, dtype=float)
        else:
            arr = np.array(value, dtype=float)
            if len(arr) != dim:
                raise ValueError(f"{param_name} must be a float or a list of {dim} floats")
            return arr

    def __init__(
        self,
        body_id: int,
        mesh_path: Optional[str] = None,
        urdf_path: Optional[str] = None,
        max_linear_vel: Union[float, List[float]] = 2.0,
        max_linear_accel: Union[float, List[float]] = 5.0,
        max_angular_vel: Union[float, List[float]] = 3.0,
        max_angular_accel: Union[float, List[float]] = 10.0,
        motion_mode: Union[MotionMode, str] = MotionMode.OMNIDIRECTIONAL,
        use_fixed_base: bool = False,
        sim_core=None,
    ):
        """
        Initialize Agent.

        Args:
            body_id: PyBullet body ID
            mesh_path: Path to robot mesh file (if mesh-based)
            urdf_path: Path to robot URDF file (if URDF-based)
            max_linear_vel: Maximum velocity (m/s) - float or [vx, vy, vz] for per-axis limits
            max_linear_accel: Maximum acceleration (m/s²) - float or [ax, ay, az] for per-axis limits
            max_angular_vel: Maximum angular velocity (rad/s) - float or [yaw, pitch, roll] for per-axis limits
            max_angular_accel: Maximum angular acceleration (rad/s²) - float or [yaw, pitch, roll] for per-axis limits
            motion_mode: MotionMode.OMNIDIRECTIONAL or MotionMode.DIFFERENTIAL drive
            use_fixed_base: If True, robot base is fixed and doesn't move
            sim_core: Reference to simulation core (optional)

        Note:
            For spawning robots from AgentSpawnParams, use Agent.from_params() instead.
            For mesh robots, use Agent.from_mesh() factory method.
            For URDF robots, use Agent.from_urdf() factory method.
        """
        # Initialize SimObject base class
        super().__init__(body_id, sim_core=sim_core)

        self.mesh_path = mesh_path
        self.urdf_path = urdf_path

        # Normalize max_linear_vel and max_linear_accel to numpy arrays [vx, vy, vz]
        self.max_linear_vel = self._normalize_vector_param(max_linear_vel, "max_linear_vel", 3)
        self.max_linear_accel = self._normalize_vector_param(max_linear_accel, "max_linear_accel", 3)

        # Normalize max_angular_vel and max_angular_accel to numpy arrays [yaw, pitch, roll]
        self.max_angular_vel = self._normalize_vector_param(max_angular_vel, "max_angular_vel", 3)
        self.max_angular_accel = self._normalize_vector_param(max_angular_accel, "max_angular_accel", 3)

        self.use_fixed_base = use_fixed_base

        # URDF-specific: joint information
        self.joint_info = []
        if urdf_path is not None:
            num_joints = p.getNumJoints(body_id)
            self.joint_info = [p.getJointInfo(body_id, j) for j in range(num_joints)]

        # Goal tracking (only used for non-static robots) - Private internal state
        self._goal_pose: Optional[Pose] = None
        self._current_velocity = np.array([0.0, 0.0, 0.0])
        self._current_angular_velocity = 0.0  # rad/s (for differential drive)
        self._is_moving = False  # Private: use property for read-only access

        # Path following - Private internal state
        self._path: List[Pose] = []  # Empty list when no path (simpler than None)
        self._current_waypoint_index = 0

        # Two-point interpolation trajectory planners (initialized by set_motion_mode) - Private
        # Position trajectories [X, Y, Z] - shared by both omnidirectional and differential modes
        self._tpi_pos: Optional[List[TwoPointInterpolation]] = None
        # Rotation using scipy Slerp combined with TPI for angle progression - used by differential mode
        self._rotation_slerp: Optional[Slerp] = None
        self._tpi_rotation_angle: Optional[TwoPointInterpolation] = None  # TPI for rotation angle (0 to total_angle)
        self._rotation_total_angle: float = 0.0  # Total rotation angle in radians
        # Forward distance for differential mode (tracks progress along path)
        self._tpi_forward: Optional[TwoPointInterpolation] = None

        # Quaternion state for differential rotation phase
        self._rotation_start_quat: Optional[np.ndarray] = None
        self._rotation_target_quat: Optional[np.ndarray] = None

        # Differential drive state - Private
        self._differential_phase: DifferentialPhase = DifferentialPhase.ROTATE

        # User-extensible data storage
        self.user_data: Dict[str, Any] = {}

        # Initialize motion mode and TPI instances (must be after all attributes are set)
        # Note: This will set _motion_mode and create TPI instances
        self._motion_mode = None  # Temporary: will be set by set_motion_mode()
        self.set_motion_mode(motion_mode)  # This normalizes mode and creates TPI instances

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
        return self._goal_pose

    @classmethod
    def create_shared_shapes(
        cls,
        mesh_path: str,
        mesh_scale: List[float] = None,
        collision_half_extents: List[float] = None,
        rgba_color: List[float] = None,
    ) -> Tuple[int, int]:
        """
        Create shared visual and collision shapes for fast robot spawning.

        Args:
            mesh_path: Path to robot mesh file
            mesh_scale: Mesh scaling [x, y, z]
            collision_half_extents: Collision box half extents [x, y, z]
            rgba_color: RGBA color [r, g, b, a]

        Returns:
            (visual_id, collision_id) tuple
        """
        if mesh_path in cls._shared_shapes:
            return cls._shared_shapes[mesh_path]

        if mesh_scale is None:
            mesh_scale = [1.0, 1.0, 1.0]
        if collision_half_extents is None:
            collision_half_extents = [0.2, 0.1, 0.2]
        if rgba_color is None:
            rgba_color = [0.2, 0.2, 0.2, 1.0]

        # Create visual shape
        visual_id = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=mesh_path, rgbaColor=rgba_color, meshScale=mesh_scale)

        # Create collision shape (simple box)
        collision_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=collision_half_extents)

        cls._shared_shapes[mesh_path] = (visual_id, collision_id)
        return visual_id, collision_id

    @classmethod
    def from_params(cls, spawn_params: AgentSpawnParams, sim_core=None) -> "Agent":
        """
        Create an Agent from AgentSpawnParams.

        This is the recommended way to spawn robots when using AgentSpawnParams.
        Automatically detects whether to use mesh or URDF based on spawn_params.

        Args:
            spawn_params: AgentSpawnParams instance with all robot parameters
            sim_core: Reference to simulation core (optional)

        Returns:
            Agent instance

        Example (Mesh):
            params = AgentSpawnParams(
                mesh_path="robot.obj",
                initial_pose=Pose.from_xyz(1.0, 2.0, 0.0),
                max_linear_vel=3.0
            )
            robot = Agent.from_params(spawn_params=params)

        Example (URDF):
            params = AgentSpawnParams(
                urdf_path="arm_robot.urdf",
                initial_pose=Pose.from_xyz(0.0, 0.0, 0.0),
                use_fixed_base=True
            )
            robot = Agent.from_params(spawn_params=params)
        """
        # Validate required parameters

        # Choose mesh or URDF path
        if spawn_params.urdf_path is not None:
            # URDF robot
            agent = cls.from_urdf(
                urdf_path=spawn_params.urdf_path,
                pose=spawn_params.initial_pose,
                mass=spawn_params.mass,
                max_linear_vel=spawn_params.max_linear_vel,
                max_linear_accel=spawn_params.max_linear_accel,
                max_angular_vel=spawn_params.max_angular_vel,
                max_angular_accel=spawn_params.max_angular_accel,
                motion_mode=spawn_params.motion_mode,
                use_fixed_base=spawn_params.use_fixed_base,
                sim_core=sim_core,
            )
        else:
            # Mesh robot
            agent = cls.from_mesh(
                mesh_path=spawn_params.mesh_path,
                pose=spawn_params.initial_pose,
                mesh_scale=spawn_params.mesh_scale,
                collision_half_extents=spawn_params.collision_half_extents,
                rgba_color=spawn_params.rgba_color,
                base_mass=spawn_params.mass,
                max_linear_vel=spawn_params.max_linear_vel,
                max_linear_accel=spawn_params.max_linear_accel,
                max_angular_vel=spawn_params.max_angular_vel,
                max_angular_accel=spawn_params.max_angular_accel,
                motion_mode=spawn_params.motion_mode,
                use_fixed_base=spawn_params.use_fixed_base,
                visual_mesh_path=spawn_params.visual_mesh_path,
                sim_core=sim_core,
            )

        # Set user_data if provided
        if spawn_params.user_data:
            agent.user_data.update(spawn_params.user_data)

        return agent

    @classmethod
    def from_mesh(
        cls,
        mesh_path: str,
        pose: Pose = None,
        mesh_scale: List[float] = None,
        collision_half_extents: List[float] = None,
        rgba_color: List[float] = None,
        base_mass: float = 0.0,
        max_linear_vel: Union[float, List[float]] = 2.0,
        max_linear_accel: Union[float, List[float]] = 5.0,
        max_angular_vel: Union[float, List[float]] = 3.0,
        max_angular_accel: Union[float, List[float]] = 10.0,
        motion_mode: Union[MotionMode, str] = MotionMode.OMNIDIRECTIONAL,
        use_fixed_base: bool = False,
        visual_mesh_path: Optional[str] = None,
        sim_core=None,
    ) -> "Agent":
        """
        Create a mesh-based Agent.

        This method extends SimObject.from_mesh() by adding Agent-specific parameters
        (max_linear_vel, max_linear_accel, use_fixed_base, visual_mesh_path).

        Args:
            mesh_path: Path to robot mesh file (.obj, .dae, etc.) for collision
            pose: Initial Pose (position and orientation). Defaults to origin
            mesh_scale: Mesh scaling [sx, sy, sz]
            collision_half_extents: Collision box half extents [hx, hy, hz]
            rgba_color: RGBA color [r, g, b, a]
            base_mass: Robot mass (kg), 0.0 for kinematic control
            max_linear_vel: Maximum velocity (m/s) - float or [vx, vy, vz] - Agent-specific
            max_linear_accel: Maximum acceleration (m/s²) - float or [ax, ay, az] - Agent-specific
            max_angular_vel: Maximum angular velocity (rad/s) - Agent-specific
            max_angular_accel: Maximum angular acceleration (rad/s²) - Agent-specific
            motion_mode: MotionMode.OMNIDIRECTIONAL or MotionMode.DIFFERENTIAL - Agent-specific
            use_fixed_base: If True, robot base is fixed and doesn't move - Agent-specific
            visual_mesh_path: Optional path to separate visual mesh (if None, uses mesh_path) - Agent-specific
            sim_core: Reference to simulation core

        Returns:
            Agent instance

        Example:
            agent = Agent.from_mesh(
                mesh_path="robot.obj",
                pose=Pose.from_xyz(0, 0, 0.5),
                max_linear_vel=2.0,
                motion_mode=MotionMode.DIFFERENTIAL,
                visual_mesh_path="robot_visual.obj"  # Optional separate visual mesh
            )
        """
        if pose is None:
            pose = Pose.from_xyz(0.0, 0.0, 0.0)
        if mesh_scale is None:
            mesh_scale = [1.0, 1.0, 1.0]
        if collision_half_extents is None:
            collision_half_extents = [0.2, 0.1, 0.2]
        if rgba_color is None:
            rgba_color = [0.2, 0.2, 0.2, 1.0]

        # Use visual_mesh_path if provided, otherwise use mesh_path
        visual_mesh = visual_mesh_path if visual_mesh_path is not None else mesh_path

        # Create collision shape (box)
        collision_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=collision_half_extents)

        # Create visual shape (from visual_mesh or mesh_path)
        visual_id = p.createVisualShape(
            shapeType=p.GEOM_MESH, fileName=visual_mesh, rgbaColor=rgba_color, meshScale=mesh_scale
        )

        # Get position and orientation from Pose
        position, orientation = pose.as_position_orientation()

        # Create multibody
        body_id = p.createMultiBody(
            baseMass=base_mass,
            baseCollisionShapeIndex=collision_id,
            baseVisualShapeIndex=visual_id,
            basePosition=position,
            baseOrientation=orientation,
        )

        # Create agent instance with Agent-specific parameters
        agent = cls(
            body_id=body_id,
            mesh_path=mesh_path,
            max_linear_vel=max_linear_vel,
            max_linear_accel=max_linear_accel,
            max_angular_vel=max_angular_vel,
            max_angular_accel=max_angular_accel,
            motion_mode=motion_mode,
            use_fixed_base=use_fixed_base,
            sim_core=sim_core,
        )

        # Auto-register to sim_core if provided (handled by SimObject.__init__)
        # Note: No need to manually append here, SimObject.__init__ handles it

        return agent

    @classmethod
    def from_urdf(
        cls,
        urdf_path: str,
        pose: Pose = None,
        mass: float = 1.0,
        max_linear_vel: Union[float, List[float]] = 2.0,
        max_linear_accel: Union[float, List[float]] = 5.0,
        max_angular_vel: Union[float, List[float]] = 3.0,
        max_angular_accel: Union[float, List[float]] = 10.0,
        motion_mode: Union[MotionMode, str] = MotionMode.OMNIDIRECTIONAL,
        use_fixed_base: bool = False,
        sim_core=None,
    ) -> "Agent":
        """
        Create a URDF-based Agent (with joints).

        Args:
            urdf_path: Path to robot URDF file
            pose: Initial Pose (position and orientation). Defaults to origin
            mass: Mass override (default: 1.0 uses URDF values, 0.0 for kinematic control)
            max_linear_vel: Maximum velocity (m/s) - float or [vx, vy, vz]
            max_linear_accel: Maximum acceleration (m/s²) - float or [ax, ay, az]
            max_angular_vel: Maximum angular velocity (rad/s)
            max_angular_accel: Maximum angular acceleration (rad/s²)
            motion_mode: MotionMode.OMNIDIRECTIONAL or MotionMode.DIFFERENTIAL
            use_fixed_base: If True, robot base is fixed in space
            sim_core: Reference to simulation core

        Returns:
            Agent instance

        Note:
            - mass=1.0 (default): Uses URDF file's mass values for physics simulation
            - mass=0.0: Override all links to mass=0 for kinematic control (no physics)

        Example:
            # Use URDF mass values (physics enabled)
            robot = Agent.from_urdf(urdf_path="arm_robot.urdf")

            # Kinematic control (no physics)
            robot = Agent.from_urdf(urdf_path="arm_robot.urdf", mass=0.0)
        """
        if pose is None:
            pose = Pose.from_xyz(0.0, 0.0, 0.0)

        # Get position and orientation from Pose
        position, orientation = pose.as_position_orientation()

        body_id = p.loadURDF(urdf_path, position, orientation, useFixedBase=use_fixed_base)

        # Override mass if explicitly set to 0.0 (kinematic control)
        # mass=1.0 (default) means use URDF's mass values
        if mass == 0.0:
            # Kinematic control: set mass to 0 for all links
            # This prevents gravity and inertia from affecting the robot
            p.changeDynamics(body_id, -1, mass=0.0)
            num_joints = p.getNumJoints(body_id)
            for joint_idx in range(num_joints):
                p.changeDynamics(body_id, joint_idx, mass=0.0)

        # Create agent instance (SimObject.__init__ handles auto-registration)
        agent = cls(
            body_id=body_id,
            urdf_path=urdf_path,
            max_linear_vel=max_linear_vel,
            max_linear_accel=max_linear_accel,
            max_angular_vel=max_angular_vel,
            max_angular_accel=max_angular_accel,
            motion_mode=motion_mode,
            use_fixed_base=use_fixed_base,
            sim_core=sim_core,
        )

        return agent

    def set_goal_pose(self, goal: Pose):
        """
        Set goal pose for the robot to move to.

        Note: This is ignored if the robot has a fixed base.

        Args:
            goal: Target Pose (compatible with ROS2 geometry_msgs/Pose)
        """
        # Single goal is just a path with one waypoint
        self.set_path([goal])

    def set_motion_mode(self, mode: Union[MotionMode, str]) -> bool:
        """
        Set motion mode for the robot.

        Note: Motion mode cannot be changed while the robot is moving.

        Args:
            mode: MotionMode.OMNIDIRECTIONAL or MotionMode.DIFFERENTIAL (or string)

        Returns:
            True if mode was changed successfully, False if robot is moving
        """
        if self._is_moving:
            print(
                "[Agent] Warning: Cannot change motion mode while robot is moving "
                f"(body_id={self.body_id}). Call stop() first."
            )
            return False

        # Normalize to enum
        if isinstance(mode, str):
            mode = MotionMode(mode)

        self._motion_mode = mode

        # Create TPI instances for the new mode (actual trajectory will be calculated in set_path)
        if mode == MotionMode.OMNIDIRECTIONAL:
            # Position trajectories: X, Y, Z
            self._tpi_pos = [TwoPointInterpolation(), TwoPointInterpolation(), TwoPointInterpolation()]
            self._rotation_slerp = None
            self._tpi_forward = None
        elif mode == MotionMode.DIFFERENTIAL:
            # Position trajectories: X, Y, Z (shared with omnidirectional)
            self._tpi_pos = [TwoPointInterpolation(), TwoPointInterpolation(), TwoPointInterpolation()]
            # Rotation: TPI for angle progression + Slerp for quaternion interpolation
            self._rotation_slerp = None
            self._tpi_rotation_angle = None  # Will be initialized in set_path
            self._tpi_forward = TwoPointInterpolation()  # Forward distance
            self._differential_phase = DifferentialPhase.ROTATE

        return True

    def set_path(self, path: List[Pose]):
        """
        Set a path (list of waypoints) for the robot to follow.

        Args:
            path: List of Pose waypoints to follow in sequence
        """
        if self.use_fixed_base:
            print(f"[Agent] Warning: Cannot set path for fixed-base robot (body_id={self.body_id})")
            return

        if len(path) == 0:
            print("[Agent] Warning: Empty path provided")
            return

        self._path = path
        self._current_waypoint_index = 0
        self._goal_pose = path[0]
        self._is_moving = True

        # Initialize trajectory based on motion mode
        if self._motion_mode == MotionMode.OMNIDIRECTIONAL:
            self._init_omnidirectional_trajectory(path[0])
        elif self._motion_mode == MotionMode.DIFFERENTIAL:
            self._init_differential_rotation_trajectory(path[0])

    def _handle_goal_reached(self, current_pose: Pose):
        """
        Handle goal reached event: teleport to exact position and move to next waypoint.
        This is common logic for both omnidirectional and differential drive.

        Args:
            current_pose: Current robot pose
        """
        # Teleport to exact goal position before updating waypoint
        p.resetBasePositionAndOrientation(self.body_id, self._goal_pose.position, current_pose.orientation)

        # Stop moving
        self._current_velocity = np.array([0.0, 0.0, 0.0])
        self._current_angular_velocity = 0.0
        self._is_moving = False

        # Move to next waypoint if following path
        if self._path:  # Simpler: empty list is falsy
            self._current_waypoint_index += 1
            if self._current_waypoint_index < len(self._path):
                self._goal_pose = self._path[self._current_waypoint_index]
                self._is_moving = True
                # Initialize trajectory based on motion mode
                if self._motion_mode == MotionMode.OMNIDIRECTIONAL:
                    self._init_omnidirectional_trajectory(self._goal_pose)
                elif self._motion_mode == MotionMode.DIFFERENTIAL:
                    self._init_differential_rotation_trajectory(self._goal_pose)
            else:
                # Path complete - stop all motion in PyBullet
                self._path = []  # Clear path (was None)
                self._goal_pose = None
                # Reset velocity in PyBullet to stop any residual motion
                p.resetBaseVelocity(self.body_id, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])

    def _init_omnidirectional_trajectory(self, goal: Pose):
        """
        Initialize two-point interpolation trajectory for omnidirectional motion (X, Y, Z axes).
        Reuses existing TPI instances created by set_motion_mode().
        Uses per-axis max_linear_vel and max_linear_accel if configured.

        Args:
            goal: Target pose
        """
        current_pose = self.get_pose()
        current_pos = np.array(current_pose.position)
        goal_pos = np.array(goal.position)

        # Get current simulation time (from sim_core if available, otherwise 0)
        t0 = self.sim_core.sim_time if self.sim_core is not None else 0.0

        # Reuse existing TPI instances (created by set_motion_mode)
        if self._tpi_pos is None:
            raise RuntimeError("_tpi_pos is not initialized. Call set_motion_mode() before setting path.")

        # 3D: X, Y, Z with per-axis limits
        for axis in range(3):
            self._tpi_pos[axis].init(
                p0=current_pos[axis],
                pe=goal_pos[axis],
                acc_max=self.max_linear_accel[axis],  # Per-axis acceleration
                vmax=self.max_linear_vel[axis],  # Per-axis velocity
                t0=t0,
                v0=self._current_velocity[axis],
                ve=0.0,  # Stop at goal
                dec_max=self.max_linear_accel[axis],  # Per-axis deceleration
            )
            self._tpi_pos[axis].calc_trajectory()  # Calculate trajectory

    def _update_omnidirectional(self, dt: float):
        """
        Update robot position using omnidirectional motion (holonomic).
        Robot can move in any direction without rotating first.
        Uses two-point interpolation for smooth acceleration/deceleration.

        Args:
            dt: Time step (seconds)
        """
        current_pose = self.get_pose()
        current_pos = np.array(current_pose.position)
        goal_pos = np.array(self._goal_pose.position)

        # Get current simulation time
        current_time = self.sim_core.sim_time if self.sim_core is not None else 0.0

        # Get interpolated position and velocity from trajectories
        if self._tpi_pos is None:
            # This should not happen if set_motion_mode and set_path were called correctly
            return

        p_xyz = []
        v_xyz = []
        for tpi in self._tpi_pos:
            p_val, v_val, a_val = tpi.get_point(current_time)
            p_xyz.append(p_val)
            v_xyz.append(v_val)

        # Check if trajectory is complete using get_end_time()
        # Use the longest of the three trajectories
        trajectory_complete = all(current_time >= tpi.get_end_time() for tpi in self._tpi_pos)

        if trajectory_complete:
            # Reached goal - use common handler
            self._handle_goal_reached(current_pose)
            return

        # Update position (full 3D)
        new_pos = np.array([p_xyz[0], p_xyz[1], p_xyz[2]])

        # Update velocity (3D)
        self._current_velocity = np.array([v_xyz[0], v_xyz[1], v_xyz[2]])

        # Set new position (omnidirectional: keep original orientation, don't rotate)
        p.resetBasePositionAndOrientation(self.body_id, new_pos.tolist(), current_pose.orientation)

    def _init_differential_rotation_trajectory(self, goal: Pose):
        """
        Initialize rotation trajectory for differential drive (Phase 1: Rotate to face target).
        Uses TPI for angle progression with acceleration/deceleration, combined with Slerp for
        smooth quaternion interpolation.

        Differential drive has two phases:
        1. Rotation phase: Rotate to face the target using TPI + Slerp
           - TPI calculates angle progression considering max_angular_vel and max_angular_accel
           - Slerp provides smooth shortest-path quaternion interpolation
        2. Forward phase: Move forward to the target

        Args:
            goal: Target pose
        """
        current_pose = self.get_pose()
        current_pos = np.array(current_pose.position)
        goal_pos = np.array(goal.position)

        # Calculate direction vector
        direction_vec = goal_pos - current_pos

        # Cache for forward phase
        self._forward_start_pos = current_pos.copy()
        self._forward_goal_pos = goal_pos.copy()
        self._forward_goal_orientation = np.array(goal.orientation)  # Save goal orientation

        # Get current simulation time
        t0 = self.sim_core.sim_time if self.sim_core is not None else 0.0

        # Calculate 3D distance
        self._forward_total_distance_3d = float(np.linalg.norm(direction_vec))

        # Calculate target orientation
        self._differential_phase = DifferentialPhase.ROTATE

        # Get current and target quaternions
        self._rotation_start_quat = np.array(current_pose.orientation)  # [x, y, z, w]

        # Calculate target orientation:
        # - X-axis (robot forward) should point along path direction
        # - Z-axis (robot up) should match goal pose Z-axis
        if self._forward_total_distance_3d > 1e-6:
            # Path direction (will be robot's X-axis)
            x_axis_target = direction_vec / self._forward_total_distance_3d

            # Get goal pose Z-axis
            goal_rotation = R.from_quat(goal.orientation)
            z_axis_goal = goal_rotation.as_matrix()[:, 2]  # Extract Z column from rotation matrix

            # Calculate target quaternion from X and Z axes
            self._rotation_target_quat = np.array(self._direction_and_up_to_quaternion(x_axis_target, z_axis_goal))
        else:
            # No movement needed, use goal orientation directly
            self._rotation_target_quat = np.array(goal.orientation)

        # Calculate rotation angle between quaternions
        r_current = R.from_quat(self._rotation_start_quat)
        r_target = R.from_quat(self._rotation_target_quat)
        r_delta = r_target * r_current.inv()
        rotation_angle = r_delta.magnitude()  # Total rotation angle in radians

        # Use TPI to calculate rotation trajectory with angular acceleration and velocity constraints
        if rotation_angle > 1e-6:
            # Store total rotation angle for Slerp interpolation (ensure float type)
            self._rotation_total_angle = float(rotation_angle)

            # Use first element of max_angular_vel and max_angular_accel for overall rotation
            angular_vel = self.max_angular_vel[0]
            angular_accel = self.max_angular_accel[0]

            # Initialize TPI for rotation angle (from 0 to rotation_angle)
            self._tpi_rotation_angle = TwoPointInterpolation()
            self._tpi_rotation_angle.init(
                p0=0.0,  # Start angle
                pe=rotation_angle,  # End angle
                acc_max=angular_accel,  # Angular acceleration
                vmax=angular_vel,  # Angular velocity
                t0=t0,
                v0=0.0,  # Start from rest
                ve=0.0,  # Stop at target
                dec_max=angular_accel,
            )
            self._tpi_rotation_angle.calc_trajectory()

            # Create Slerp interpolator (we'll use angle ratio from TPI to query Slerp)
            # Slerp will be queried with ratio = current_angle / total_angle
            key_rots = R.from_quat([self._rotation_start_quat, self._rotation_target_quat])
            # Slerp with normalized time [0, 1]
            self._rotation_slerp = Slerp([0.0, 1.0], key_rots)
        else:
            # No rotation needed, teleport to target orientation and skip to FORWARD phase
            p.resetBasePositionAndOrientation(self.body_id, current_pos.tolist(), self._rotation_target_quat.tolist())

            # Calculate 3D distance for forward phase
            distance_3d = self._forward_total_distance_3d

            # Skip to FORWARD phase
            self._differential_phase = DifferentialPhase.FORWARD
            self._init_differential_forward_distance_trajectory(distance_3d)

    def _init_differential_forward_distance_trajectory(self, distance: float):
        """
        Initialize forward distance trajectory for differential drive (Phase 2: Move forward).

        Args:
            distance: Distance to travel forward (3D distance)
        """
        # Get current simulation time (from sim_core if available, otherwise 0)
        t0 = self.sim_core.sim_time if self.sim_core is not None else 0.0

        # Use average of XYZ for 3D motion
        avg_vel = np.mean(self.max_linear_vel)
        avg_accel = np.mean(self.max_linear_accel)

        # Initialize forward distance trajectory
        self._tpi_forward = TwoPointInterpolation()
        self._tpi_forward.init(
            p0=0.0,  # Start at 0 distance
            pe=distance,  # End at target distance
            acc_max=avg_accel,
            vmax=avg_vel,
            t0=t0,
            v0=0.0,  # Start from rest
            ve=0.0,  # Stop at goal
            dec_max=avg_accel,
        )
        self._tpi_forward.calc_trajectory()  # Calculate trajectory

    def _update_differential(self, dt: float):
        """
        Update robot position using differential drive motion (non-holonomic).
        Robot must first rotate to face the target, then move forward.
        Uses TPI + Slerp for smooth rotation with acceleration/deceleration,
        and TPI for smooth forward motion.

        Rotation Phase:
        - TPI calculates angle progression (0 to total_angle) with angular acceleration/deceleration
        - Slerp interpolates quaternion using angle ratio from TPI
        - Provides smooth angular velocity profile respecting max_angular_vel and max_angular_accel

        Forward Phase:
        - TPI calculates distance progression with linear acceleration/deceleration
        - Maintains fixed orientation from rotation phase

        Args:
            dt: Time step (seconds)
        """
        current_pose = self.get_pose()
        current_pos = np.array(current_pose.position)
        goal_pos = np.array(self._goal_pose.position)

        # Get current simulation time
        current_time = self.sim_core.sim_time if self.sim_core is not None else 0.0

        if self._differential_phase == DifferentialPhase.ROTATE:
            # Phase 1: Rotate towards target using TPI + Slerp
            # TPI controls angle progression with acceleration/deceleration
            # Slerp provides smooth quaternion interpolation

            # Get current angle from TPI trajectory
            if self._tpi_rotation_angle is None:
                # Fallback: no rotation needed
                self._differential_phase = DifferentialPhase.FORWARD
                self._init_differential_forward_distance_trajectory(self._forward_total_distance_3d)
                return

            current_angle, angular_vel, angular_accel = self._tpi_rotation_angle.get_point(current_time)

            # Check if rotation is complete using TPI end time
            if current_time >= self._tpi_rotation_angle.get_end_time():
                # Rotation complete, switch to forward phase
                # Set exact final rotation (target quaternion)
                p.resetBasePositionAndOrientation(self.body_id, current_pos.tolist(), self._rotation_target_quat.tolist())

                # Initialize forward trajectory
                self._differential_phase = DifferentialPhase.FORWARD
                self._init_differential_forward_distance_trajectory(self._forward_total_distance_3d)
                return

            # Apply Slerp interpolation with TPI-calculated angle ratio
            # Ratio = current_angle / total_angle (normalized to [0, 1])
            if self._rotation_slerp is not None and self._rotation_total_angle > 1e-6:
                angle_ratio = current_angle / self._rotation_total_angle
                angle_ratio = np.clip(angle_ratio, 0.0, 1.0)  # Ensure [0, 1] range

                r_interpolated = self._rotation_slerp(angle_ratio)
                new_orientation = r_interpolated.as_quat().tolist()
                p.resetBasePositionAndOrientation(self.body_id, current_pos.tolist(), new_orientation)
            else:
                # Fallback: use target orientation directly
                p.resetBasePositionAndOrientation(self.body_id, current_pos.tolist(), self._rotation_target_quat.tolist())

            # Update angular velocity from TPI
            self._current_angular_velocity = angular_vel
            self._current_velocity = np.array([0.0, 0.0, 0.0])

        elif self._differential_phase == DifferentialPhase.FORWARD:
            # Phase 2: Move forward using TPI
            # Get interpolated distance from trajectory (scalar progress)
            distance_traveled, forward_vel, forward_acc = self._tpi_forward.get_point(current_time)

            # Check if forward motion is complete using get_end_time()
            trajectory_complete = current_time >= self._tpi_forward.get_end_time()

            if trajectory_complete:
                # Reached goal - use common handler
                self._handle_goal_reached(current_pose)
                return

            # Move along straight 3D line from start to goal
            direction_3d = self._forward_goal_pos - self._forward_start_pos
            total_distance = self._forward_total_distance_3d
            ratio = distance_traveled / total_distance if total_distance > 1e-6 else 0.0

            # Position: interpolate along 3D line
            new_pos = self._forward_start_pos + direction_3d * ratio

            # Orientation: Keep the target orientation from rotation phase (no need to recalculate)
            new_orientation = self._rotation_target_quat.tolist()

            # Velocity: along 3D direction
            direction_3d_unit = direction_3d / total_distance if total_distance > 1e-6 else np.array([0.0, 0.0, 0.0])
            self._current_velocity = direction_3d_unit * forward_vel

            # Update position/orientation and reset angular velocity
            p.resetBasePositionAndOrientation(self.body_id, new_pos.tolist(), new_orientation)
            self._current_angular_velocity = 0.0

    def _direction_and_up_to_quaternion(self, x_axis: np.ndarray, z_axis: np.ndarray) -> List[float]:
        """
        Create quaternion from desired X-axis (forward) and Z-axis (up) directions.

        The Y-axis is computed as Z × X to form a right-handed coordinate system.
        If X and Z are not perpendicular, Z-axis takes priority and X is adjusted.

        Args:
            x_axis: Desired forward direction (will be normalized)
            z_axis: Desired up direction (will be normalized, takes priority)

        Returns:
            Quaternion [x, y, z, w] representing the orientation
        """
        # Normalize inputs
        z_norm = z_axis / np.linalg.norm(z_axis)
        x_initial = x_axis / np.linalg.norm(x_axis)

        # Y-axis = Z × X (right-hand rule)
        y_axis = np.cross(z_norm, x_initial)
        y_norm_magnitude = np.linalg.norm(y_axis)

        # Check if X and Z are parallel (or nearly so)
        if y_norm_magnitude < 1e-6:
            # X and Z are parallel, cannot form a valid coordinate system
            # Fall back to a default Y-axis
            # Choose Y perpendicular to Z
            if abs(z_norm[2]) < 0.9:
                # Z is not vertical, use world up × Z
                y_axis = np.cross(np.array([0, 0, 1]), z_norm)
            else:
                # Z is nearly vertical, use world X × Z
                y_axis = np.cross(np.array([1, 0, 0]), z_norm)
            y_axis = y_axis / np.linalg.norm(y_axis)
        else:
            y_axis = y_axis / y_norm_magnitude

        # Recompute X-axis to ensure orthogonality: X = Y × Z
        x_final = np.cross(y_axis, z_norm)

        # Build rotation matrix with [X, Y, Z] as columns
        rotation_matrix = np.column_stack([x_final, y_axis, z_norm])

        # Convert to quaternion
        r = R.from_matrix(rotation_matrix)
        return r.as_quat().tolist()  # [x, y, z, w]

    def update(self, dt: float):
        """
        Update robot position towards goal with velocity/acceleration constraints.

        Supports two motion modes:
        - MotionMode.OMNIDIRECTIONAL: Move in any direction without rotating first (smooth trajectories)
        - MotionMode.DIFFERENTIAL: Rotate to face target, then move forward (smooth trajectories)

        Note: This does nothing if the robot has a fixed base.

        Args:
            dt: Time step (seconds)
        """
        if self.use_fixed_base:
            return

        if not self._is_moving or self._goal_pose is None:
            # Ensure robot is completely stopped when not following a path
            # Reset velocity to zero every frame to counteract physics (gravity, etc.)
            p.resetBaseVelocity(self.body_id, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])
            # Also fix position to prevent any drift from physics simulation
            current_pos, current_orn = p.getBasePositionAndOrientation(self.body_id)
            p.resetBasePositionAndOrientation(self.body_id, current_pos, current_orn)
            return

        # Dispatch to appropriate motion controller
        if self._motion_mode == MotionMode.OMNIDIRECTIONAL:
            self._update_omnidirectional(dt)
        elif self._motion_mode == MotionMode.DIFFERENTIAL:
            self._update_differential(dt)
        else:
            raise ValueError(f"Unknown motion_mode: {self._motion_mode}")

    def get_velocity(self) -> np.ndarray:
        """
        Get current velocity vector.

        Returns:
            Velocity [vx, vy, vz] in m/s
        """
        return self._current_velocity.copy()

    def stop(self):
        """Stop robot movement and clear goal and path."""
        self._goal_pose = None
        self._path = []  # Clear path (empty list instead of None)
        self._current_waypoint_index = 0
        self._is_moving = False
        self._current_velocity = np.array([0.0, 0.0, 0.0])
        self._current_angular_velocity = 0.0
        self._differential_phase = DifferentialPhase.ROTATE

    # ========================================
    # URDF-specific methods (joint control)
    # ========================================

    def is_urdf_robot(self) -> bool:
        """Check if this robot is URDF-based (has joints)."""
        return self.urdf_path is not None

    def get_num_joints(self) -> int:
        """Get number of joints (0 for mesh robots)."""
        return len(self.joint_info)

    def set_joint_target(self, joint_index: int, target_position: float, max_force: float = 500.0):
        """
        Set target position for a joint (for URDF robots only).

        Args:
            joint_index: Joint index (0-based)
            target_position: Target position (radians for revolute, meters for prismatic)
            max_force: Maximum force to apply

        Example:
            robot.set_joint_target(0, 1.57)  # Move first joint to 90 degrees
        """
        if not self.is_urdf_robot():
            print("[Agent] Warning: set_joint_target() only works for URDF robots")
            return

        if joint_index >= len(self.joint_info):
            print(f"[Agent] Warning: joint_index {joint_index} out of range (max: {len(self.joint_info)-1})")
            return

        p.setJointMotorControl2(
            bodyUniqueId=self.body_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_position,
            force=max_force,
        )

    def get_joint_state(self, joint_index: int) -> Tuple[float, float]:
        """
        Get joint state (position and velocity).

        Args:
            joint_index: Joint index (0-based)

        Returns:
            (position, velocity) tuple

        Example:
            pos, vel = robot.get_joint_state(0)
        """
        if not self.is_urdf_robot():
            print("[Agent] Warning: get_joint_state() only works for URDF robots")
            return (0.0, 0.0)

        joint_state = p.getJointState(self.body_id, joint_index)
        return (joint_state[0], joint_state[1])  # position, velocity

    def set_all_joint_targets(self, target_positions: List[float], max_force: float = 500.0):
        """
        Set target positions for all joints at once.

        Args:
            target_positions: List of target positions for each joint
            max_force: Maximum force to apply

        Example:
            robot.set_all_joint_targets([0.0, 1.57, -1.57, 0.0])
        """
        if not self.is_urdf_robot():
            print("[Agent] Warning: set_all_joint_targets() only works for URDF robots")
            return

        if len(target_positions) != len(self.joint_info):
            print(f"[Agent] Warning: Expected {len(self.joint_info)} targets, got {len(target_positions)}")
            return

        for i, target in enumerate(target_positions):
            self.set_joint_target(i, target, max_force)

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
