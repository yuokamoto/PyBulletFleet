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
from two_point_interpolation import TwoPointInterpolation, TwoAngleInterpolation

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


class Differential3DMode(str, Enum):
    """
    3D behavior mode for differential drive motion.

    Attributes:
        TWO_D_WITH_Z: Rotate only yaw in XY plane, then move with XY differential + Z TPI interpolation
        FULL_3D: Rotate yaw and pitch to face 3D direction, then move along straight 3D line
    """

    TWO_D_WITH_Z = "2d_with_z"
    FULL_3D = "full_3d"


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
    differential_3d_mode: Union[Differential3DMode, str] = Differential3DMode.TWO_D_WITH_Z
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

        # Get differential_3d_mode and convert string to enum if needed
        differential_3d_mode_value = config.get("differential_3d_mode", Differential3DMode.TWO_D_WITH_Z)
        if isinstance(differential_3d_mode_value, str):
            differential_3d_mode_value = Differential3DMode(differential_3d_mode_value)

        return cls(
            mesh_path=config.get("mesh_path"),
            urdf_path=config.get("urdf_path"),
            initial_pose=config.get("initial_pose"),
            max_linear_vel=config.get("max_linear_vel", 2.0),
            max_linear_accel=config.get("max_linear_accel", 5.0),
            max_angular_vel=config.get("max_angular_vel", 3.0),
            max_angular_accel=config.get("max_angular_accel", 10.0),
            motion_mode=motion_mode_value,
            differential_3d_mode=differential_3d_mode_value,
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
        differential_3d_mode: Union[Differential3DMode, str] = Differential3DMode.TWO_D_WITH_Z,
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
            differential_3d_mode: Differential3DMode.TWO_D_WITH_Z or Differential3DMode.FULL_3D
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
        # Orientation trajectories [yaw, pitch, roll] - used by differential mode during rotation phase
        self._tpi_orientation: Optional[List[TwoAngleInterpolation]] = None
        # Forward distance for differential mode (tracks progress along path)
        self._tpi_forward: Optional[TwoPointInterpolation] = None

        # Differential drive state - Private
        self._differential_phase: DifferentialPhase = DifferentialPhase.ROTATE

        # Normalize differential_3d_mode to enum
        if isinstance(differential_3d_mode, str):
            differential_3d_mode = Differential3DMode(differential_3d_mode)
        self.differential_3d_mode: Differential3DMode = differential_3d_mode

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
                max_linear_vel=spawn_params.max_linear_vel,
                max_linear_accel=spawn_params.max_linear_accel,
                max_angular_vel=spawn_params.max_angular_vel,
                max_angular_accel=spawn_params.max_angular_accel,
                motion_mode=spawn_params.motion_mode,
                differential_3d_mode=spawn_params.differential_3d_mode,
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
                differential_3d_mode=spawn_params.differential_3d_mode,
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
        differential_3d_mode: Union[Differential3DMode, str] = Differential3DMode.TWO_D_WITH_Z,
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
            differential_3d_mode: Differential3DMode for differential drive (TWO_D_WITH_Z or FULL_3D) - Agent-specific
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
            differential_3d_mode=differential_3d_mode,
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
        max_linear_vel: Union[float, List[float]] = 2.0,
        max_linear_accel: Union[float, List[float]] = 5.0,
        max_angular_vel: Union[float, List[float]] = 3.0,
        max_angular_accel: Union[float, List[float]] = 10.0,
        motion_mode: Union[MotionMode, str] = MotionMode.OMNIDIRECTIONAL,
        differential_3d_mode: Union[Differential3DMode, str] = Differential3DMode.TWO_D_WITH_Z,
        use_fixed_base: bool = False,
        sim_core=None,
    ) -> "Agent":
        """
        Create a URDF-based Agent (with joints).

        Args:
            urdf_path: Path to robot URDF file
            pose: Initial Pose (position and orientation). Defaults to origin
            max_linear_vel: Maximum velocity (m/s) - float or [vx, vy, vz]
            max_linear_accel: Maximum acceleration (m/s²) - float or [ax, ay, az]
            max_angular_vel: Maximum angular velocity (rad/s)
            max_angular_accel: Maximum angular acceleration (rad/s²)
            motion_mode: MotionMode.OMNIDIRECTIONAL or MotionMode.DIFFERENTIAL
            differential_3d_mode: Differential3DMode for differential drive (TWO_D_WITH_Z or FULL_3D)
            use_fixed_base: If True, robot base is fixed in space
            sim_core: Reference to simulation core

        Returns:
            Agent instance

        Example:
            robot = Agent.from_urdf(
                urdf_path="arm_robot.urdf",
                pose=Pose.from_xyz(0, 0, 0),
                use_fixed_base=True
            )
            robot.set_joint_target(0, 1.57)  # Control first joint
        """
        if pose is None:
            pose = Pose.from_xyz(0.0, 0.0, 0.0)

        # Get position and orientation from Pose
        position, orientation = pose.as_position_orientation()

        body_id = p.loadURDF(urdf_path, position, orientation, useFixedBase=use_fixed_base)

        # Create agent instance (SimObject.__init__ handles auto-registration)
        agent = cls(
            body_id=body_id,
            urdf_path=urdf_path,
            max_linear_vel=max_linear_vel,
            max_linear_accel=max_linear_accel,
            max_angular_vel=max_angular_vel,
            max_angular_accel=max_angular_accel,
            motion_mode=motion_mode,
            differential_3d_mode=differential_3d_mode,
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
            self._tpi_orientation = None
            self._tpi_forward = None
        elif mode == MotionMode.DIFFERENTIAL:
            # Position trajectories: X, Y, Z (shared with omnidirectional)
            self._tpi_pos = [TwoPointInterpolation(), TwoPointInterpolation(), TwoPointInterpolation()]
            # Orientation trajectories: yaw, pitch, roll
            self._tpi_orientation = [TwoAngleInterpolation(), TwoAngleInterpolation(), TwoAngleInterpolation()]
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
                # Path complete
                self._path = []  # Clear path (was None)
                self._goal_pose = None

    def _init_differential_rotation_trajectory(self, goal: Pose):
        """
        Initialize rotation trajectory for differential drive (Phase 1: Rotate to face target).

        Differential drive has two phases:
        1. Rotation phase: Rotate to face the target (yaw, and pitch for full_3d mode)
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

        # Get current simulation time
        t0 = self.sim_core.sim_time if self.sim_core is not None else 0.0

        # ========== Calculate Target Angles ==========
        # Roll: Use the goal point's roll value directly (both modes)
        desired_roll = goal.roll

        # Calculate distances and yaw/pitch based on mode
        if self.differential_3d_mode == Differential3DMode.FULL_3D:
            # ========== FULL_3D MODE ==========
            # Calculate 3D distance
            self._forward_total_distance_3d = float(np.linalg.norm(direction_vec))
            self._forward_total_distance_xy = None  # Not used in FULL_3D mode

            # Calculate yaw and pitch from 3D direction vector
            direction_xy = direction_vec[:2]
            distance_xy = np.linalg.norm(direction_xy)

            # Yaw: Direction in XY plane
            desired_yaw = np.arctan2(direction_xy[1], direction_xy[0]) if distance_xy > 1e-6 else current_pose.yaw

            # Pitch: Angle from XY plane to 3D direction
            if self._forward_total_distance_3d > 1e-6:
                desired_pitch = np.arctan2(direction_vec[2], distance_xy)
            else:
                desired_pitch = current_pose.pitch

        else:  # TWO_D_WITH_Z MODE
            # ========== TWO_D_WITH_Z MODE ==========
            # Calculate XY distance and direction
            direction_xy = direction_vec[:2]
            distance_xy = np.linalg.norm(direction_xy)
            self._forward_total_distance_xy = float(distance_xy)
            self._forward_total_distance_3d = None  # Not used in TWO_D_WITH_Z mode

            # Yaw: Direction in XY plane
            desired_yaw = np.arctan2(direction_xy[1], direction_xy[0]) if distance_xy > 1e-6 else current_pose.yaw

            # Pitch: Keep current value (no pitch rotation in 2D mode)
            desired_pitch = current_pose.pitch

        # ========== Initialize Rotation Trajectories ==========
        self._differential_phase = DifferentialPhase.ROTATE

        # Store target roll for use in FORWARD phase
        self._target_roll = desired_roll

        # Get current angles
        current_yaw = current_pose.yaw
        current_pitch = current_pose.pitch
        current_roll = current_pose.roll

        # Normalize all angles to [-pi, pi] for shortest rotation
        current_yaw = np.arctan2(np.sin(current_yaw), np.cos(current_yaw))
        current_pitch = np.arctan2(np.sin(current_pitch), np.cos(current_pitch))
        current_roll = np.arctan2(np.sin(current_roll), np.cos(current_roll))
        desired_yaw = np.arctan2(np.sin(desired_yaw), np.cos(desired_yaw))
        desired_pitch = np.arctan2(np.sin(desired_pitch), np.cos(desired_pitch))
        desired_roll = np.arctan2(np.sin(desired_roll), np.cos(desired_roll))

        # Initialize yaw trajectory (both modes)
        self._tpi_orientation[0].init(  # Yaw
            p0=current_yaw,
            pe=desired_yaw,
            acc_max=self.max_angular_accel[0],  # Yaw angular acceleration (rad/s²)
            vmax=self.max_angular_vel[0],  # Yaw angular velocity
            t0=t0,
            v0=0.0,  # Start from rest
            ve=0.0,  # Stop at target yaw
            dec_max=self.max_angular_accel[0],
        )
        self._tpi_orientation[0].calc_trajectory()

        # Initialize pitch trajectory (only for FULL_3D mode)
        if self.differential_3d_mode == Differential3DMode.FULL_3D:
            self._tpi_orientation[1].init(  # Pitch
                p0=current_pitch,
                pe=desired_pitch,
                acc_max=self.max_angular_accel[1],  # Pitch angular acceleration
                vmax=self.max_angular_vel[1],  # Pitch angular velocity
                t0=t0,
                v0=0.0,
                ve=0.0,
                dec_max=self.max_angular_accel[1],
            )
            self._tpi_orientation[1].calc_trajectory()

        # Initialize roll trajectory (both modes)
        self._tpi_orientation[2].init(  # Roll
            p0=current_roll,
            pe=desired_roll,
            acc_max=self.max_angular_accel[2] if len(self.max_angular_accel) > 2 else self.max_angular_accel[0],
            vmax=self.max_angular_vel[2] if len(self.max_angular_vel) > 2 else self.max_angular_vel[0],
            t0=t0,
            v0=0.0,
            ve=0.0,
            dec_max=self.max_angular_accel[2] if len(self.max_angular_accel) > 2 else self.max_angular_accel[0],
        )
        self._tpi_orientation[2].calc_trajectory()

        # Synchronize rotation durations based on mode
        if self.differential_3d_mode == Differential3DMode.FULL_3D:
            # Full 3D: Synchronize yaw, pitch, and roll
            yaw_duration = self._tpi_orientation[0].get_end_time() - t0
            pitch_duration = self._tpi_orientation[1].get_end_time() - t0
            roll_duration = self._tpi_orientation[2].get_end_time() - t0
            max_duration = max(yaw_duration, pitch_duration, roll_duration)

            # Recalculate trajectories with synchronized timing if needed
            if max_duration > 0.01:  # More than 10ms
                # Yaw synchronization
                if abs(yaw_duration - max_duration) > 0.01:
                    yaw_diff = desired_yaw - current_yaw
                    yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
                    yaw_angle_change = abs(yaw_diff)
                    if yaw_angle_change > 1e-6:
                        sync_vmax_yaw = yaw_angle_change / max_duration * 2.0
                        self._tpi_orientation[0].init(
                            p0=current_yaw,
                            pe=desired_yaw,
                            acc_max=self.max_angular_accel[0],
                            vmax=min(sync_vmax_yaw, self.max_angular_vel[0]),
                            t0=t0,
                            v0=0.0,
                            ve=0.0,
                            dec_max=self.max_angular_accel[0],
                        )
                        self._tpi_orientation[0].calc_trajectory()

                # Pitch synchronization
                if abs(pitch_duration - max_duration) > 0.01:
                    pitch_angle_change = abs(desired_pitch - current_pitch)
                    if pitch_angle_change > 1e-6:
                        sync_vmax_pitch = pitch_angle_change / max_duration * 2.0
                        self._tpi_orientation[1].init(
                            p0=current_pitch,
                            pe=desired_pitch,
                            acc_max=self.max_angular_accel[1],
                            vmax=min(sync_vmax_pitch, self.max_angular_vel[1]),
                            t0=t0,
                            v0=0.0,
                            ve=0.0,
                            dec_max=self.max_angular_accel[1],
                        )
                        self._tpi_orientation[1].calc_trajectory()

                # Roll synchronization
                if abs(roll_duration - max_duration) > 0.01:
                    roll_diff = desired_roll - current_roll
                    roll_diff = np.arctan2(np.sin(roll_diff), np.cos(roll_diff))
                    roll_angle_change = abs(roll_diff)
                    if roll_angle_change > 1e-6:
                        sync_vmax_roll = roll_angle_change / max_duration * 2.0
                        roll_accel = (
                            self.max_angular_accel[2] if len(self.max_angular_accel) > 2 else self.max_angular_accel[0]
                        )
                        roll_vel = self.max_angular_vel[2] if len(self.max_angular_vel) > 2 else self.max_angular_vel[0]
                        self._tpi_orientation[2].init(
                            p0=current_roll,
                            pe=desired_roll,
                            acc_max=roll_accel,
                            vmax=min(sync_vmax_roll, roll_vel),
                            t0=t0,
                            v0=0.0,
                            ve=0.0,
                            dec_max=roll_accel,
                        )
                        self._tpi_orientation[2].calc_trajectory()
        else:
            # 2D mode: Synchronize yaw and roll only (pitch stays constant)
            yaw_duration = self._tpi_orientation[0].get_end_time() - t0
            roll_duration = self._tpi_orientation[2].get_end_time() - t0
            max_duration = max(yaw_duration, roll_duration)

            if max_duration > 0.01:
                # Yaw synchronization
                if abs(yaw_duration - max_duration) > 0.01:
                    yaw_diff = desired_yaw - current_yaw
                    yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
                    yaw_angle_change = abs(yaw_diff)
                    if yaw_angle_change > 1e-6:
                        sync_vmax_yaw = yaw_angle_change / max_duration * 2.0
                        self._tpi_orientation[0].init(
                            p0=current_yaw,
                            pe=desired_yaw,
                            acc_max=self.max_angular_accel[0],
                            vmax=min(sync_vmax_yaw, self.max_angular_vel[0]),
                            t0=t0,
                            v0=0.0,
                            ve=0.0,
                            dec_max=self.max_angular_accel[0],
                        )
                        self._tpi_orientation[0].calc_trajectory()

                # Roll synchronization
                if abs(roll_duration - max_duration) > 0.01:
                    roll_diff = desired_roll - current_roll
                    roll_diff = np.arctan2(np.sin(roll_diff), np.cos(roll_diff))
                    roll_angle_change = abs(roll_diff)
                    if roll_angle_change > 1e-6:
                        sync_vmax_roll = roll_angle_change / max_duration * 2.0
                        roll_accel = (
                            self.max_angular_accel[2] if len(self.max_angular_accel) > 2 else self.max_angular_accel[0]
                        )
                        roll_vel = self.max_angular_vel[2] if len(self.max_angular_vel) > 2 else self.max_angular_vel[0]
                        self._tpi_orientation[2].init(
                            p0=current_roll,
                            pe=desired_roll,
                            acc_max=roll_accel,
                            vmax=min(sync_vmax_roll, roll_vel),
                            t0=t0,
                            v0=0.0,
                            ve=0.0,
                            dec_max=roll_accel,
                        )
                        self._tpi_orientation[2].calc_trajectory()

    def _init_differential_forward_distance_trajectory(self, distance: float):
        """
        Initialize forward distance trajectory for differential drive (Phase 2: Move forward).
        For 2d_with_z mode, also initializes Z axis TPI.

        Args:
            distance: Distance to travel forward (XY distance for 2d_with_z, 3D distance for full_3d)
        """
        # Get current simulation time (from sim_core if available, otherwise 0)
        t0 = self.sim_core.sim_time if self.sim_core is not None else 0.0

        # Determine which velocity/acceleration to use based on mode
        if self.differential_3d_mode == Differential3DMode.FULL_3D:
            # Use average of XYZ for 3D motion
            avg_vel = np.mean(self.max_linear_vel)
            avg_accel = np.mean(self.max_linear_accel)
        else:  # 2d_with_z
            # Use average of XY for 2D motion
            avg_vel = np.mean(self.max_linear_vel[:2])
            avg_accel = np.mean(self.max_linear_accel[:2])

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

        # For 2d_with_z mode, initialize Z trajectory using _tpi_pos[2]
        if self.differential_3d_mode == Differential3DMode.TWO_D_WITH_Z:
            start_z = self._forward_start_pos[2]
            goal_z = self._forward_goal_pos[2]

            self._tpi_pos[2].init(
                p0=start_z,
                pe=goal_z,
                acc_max=self.max_linear_accel[2],  # Z axis acceleration
                vmax=self.max_linear_vel[2],  # Z axis velocity
                t0=t0,
                v0=0.0,  # Start from rest
                ve=0.0,  # Stop at goal
                dec_max=self.max_linear_accel[2],
            )
            self._tpi_pos[2].calc_trajectory()

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

    def _update_differential(self, dt: float):
        """
        Update robot position using differential drive motion (non-holonomic).
        Robot must first rotate to face the target, then move forward.
        Uses two-point interpolation for smooth acceleration/deceleration.

        Args:
            dt: Time step (seconds)
        """
        current_pose = self.get_pose()
        current_pos = np.array(current_pose.position)
        goal_pos = np.array(self._goal_pose.position)

        # Get current simulation time
        current_time = self.sim_core.sim_time if self.sim_core is not None else 0.0

        if self._differential_phase == DifferentialPhase.ROTATE:
            # Phase 1: Rotate towards target using TPI
            # Get interpolated yaw from trajectory
            yaw_target, yaw_vel, yaw_acc = self._tpi_orientation[0].get_point(current_time)

            # For full_3d mode, also get pitch
            if self.differential_3d_mode == Differential3DMode.FULL_3D:
                pitch_target, pitch_vel, pitch_acc = self._tpi_orientation[1].get_point(current_time)
                # Check if both yaw and pitch rotations are complete
                trajectory_complete = (
                    current_time >= self._tpi_orientation[0].get_end_time()
                    and current_time >= self._tpi_orientation[1].get_end_time()
                )
            else:
                pitch_target = 0.0
                # Check if yaw rotation is complete
                trajectory_complete = current_time >= self._tpi_orientation[0].get_end_time()

            if trajectory_complete:
                # Rotation complete, switch to forward phase
                # Recalculate distances (might have drifted slightly) based on mode
                direction_vec = goal_pos - current_pos
                self._forward_start_pos = current_pos.copy()
                self._forward_goal_pos = goal_pos.copy()

                # Calculate distance based on mode
                if self.differential_3d_mode == Differential3DMode.FULL_3D:
                    # FULL_3D: only need 3D distance
                    distance_3d = np.linalg.norm(direction_vec)
                    self._forward_total_distance_3d = float(distance_3d)
                    forward_distance = distance_3d
                else:  # TWO_D_WITH_Z
                    # TWO_D_WITH_Z: only need XY distance
                    direction_xy = direction_vec[:2]
                    distance_xy = np.linalg.norm(direction_xy)
                    self._forward_total_distance_xy = float(distance_xy)
                    forward_distance = distance_xy

                # Initialize forward trajectory
                self._differential_phase = DifferentialPhase.FORWARD
                self._init_differential_forward_distance_trajectory(forward_distance)

                # Set exact final rotation using direction vector method with target roll
                if self.differential_3d_mode == Differential3DMode.FULL_3D:
                    # For full_3d mode, use direction vector with target roll
                    if self._forward_total_distance_3d > 1e-6:
                        new_orientation = self._direction_to_quaternion(direction_vec, roll=self._target_roll)
                    else:
                        new_orientation = current_pose.orientation
                else:
                    # For 2d_with_z mode, use XY direction with target roll
                    if self._forward_total_distance_xy > 1e-6:
                        direction_2d = np.array([direction_xy[0], direction_xy[1], 0.0])
                    else:
                        direction_2d = np.array([1.0, 0.0, 0.0])
                    new_orientation = self._direction_to_quaternion(direction_2d, roll=self._target_roll)

                p.resetBasePositionAndOrientation(self.body_id, current_pos.tolist(), new_orientation)
                return

            # Apply rotation during interpolation
            # For full_3d mode, use direction vector method to avoid orientation artifacts
            if self.differential_3d_mode == Differential3DMode.FULL_3D:
                # Reconstruct direction vector from interpolated yaw and pitch
                dx = np.cos(pitch_target) * np.cos(yaw_target)
                dy = np.cos(pitch_target) * np.sin(yaw_target)
                dz = np.sin(pitch_target)
                interpolated_direction = np.array([dx, dy, dz])
                # Use target roll (from path waypoint) during rotation
                new_orientation = self._direction_to_quaternion(interpolated_direction, roll=self._target_roll)
            else:
                # For 2d_with_z mode, rotate yaw with target roll
                # Create direction vector for yaw (in XY plane)
                direction_2d = np.array([np.cos(yaw_target), np.sin(yaw_target), 0.0])
                # Use target roll (from path waypoint) during rotation
                new_orientation = self._direction_to_quaternion(direction_2d, roll=self._target_roll)

            p.resetBasePositionAndOrientation(self.body_id, current_pos.tolist(), new_orientation)

            self._current_angular_velocity = yaw_vel
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

            # Compute new position based on selected 3D mode
            if self.differential_3d_mode == Differential3DMode.FULL_3D:
                # Move along straight 3D line from start to goal.
                if self._forward_total_distance_3d > 1e-6:
                    ratio = distance_traveled / self._forward_total_distance_3d
                else:
                    ratio = 0.0

                new_pos = self._forward_start_pos + (self._forward_goal_pos - self._forward_start_pos) * ratio

                # Calculate direction_3d (needed for velocity and orientation calculation)
                direction_3d = self._forward_goal_pos - self._forward_start_pos

                # Orientation: maintain target roll during forward motion
                if np.linalg.norm(direction_3d) > 1e-6:
                    new_orientation = self._direction_to_quaternion(direction_3d, roll=self._target_roll)
                else:
                    new_orientation = current_pose.orientation

                p.resetBasePositionAndOrientation(self.body_id, new_pos.tolist(), new_orientation)

                # Velocity: along the straight 3D direction
                if self._forward_total_distance_3d > 1e-6:
                    direction_3d_unit = direction_3d / self._forward_total_distance_3d
                else:
                    direction_3d_unit = np.array([0.0, 0.0, 0.0])

                self._current_velocity = direction_3d_unit * forward_vel
                self._current_angular_velocity = 0.0

            else:  # "2d_with_z": yaw in XY, plus Z TPI interpolation
                # Calculate new position based on current yaw for XY, and
                # interpolate Z using TPI.
                current_yaw = current_pose.yaw
                forward_direction_xy = np.array([np.cos(current_yaw), np.sin(current_yaw)])

                # XY movement follows differential forward direction from start
                start_xy = self._forward_start_pos[:2]
                new_xy = start_xy + forward_direction_xy * distance_traveled

                # Z moves using TPI
                if self._tpi_pos[2] is not None:
                    new_z, vel_z, acc_z = self._tpi_pos[2].get_point(current_time)
                else:
                    # Fallback to linear if TPI not initialized
                    if self._forward_total_distance_xy > 1e-6:
                        ratio = distance_traveled / self._forward_total_distance_xy
                    else:
                        ratio = 0.0
                    start_z = self._forward_start_pos[2]
                    goal_z = self._forward_goal_pos[2]
                    new_z = start_z + (goal_z - start_z) * ratio
                    vel_z = 0.0

                new_pos = np.array([new_xy[0], new_xy[1], new_z])

                # Orientation: maintain yaw direction with target roll
                current_yaw = current_pose.yaw
                forward_direction_3d = np.array([np.cos(current_yaw), np.sin(current_yaw), 0.0])
                new_orientation = self._direction_to_quaternion(forward_direction_3d, roll=self._target_roll)

                # Update position and orientation
                p.resetBasePositionAndOrientation(self.body_id, new_pos.tolist(), new_orientation)

                # Velocity: XY along forward direction, Z from TPI
                vel_xy = forward_direction_xy * forward_vel
                self._current_velocity = np.array([vel_xy[0], vel_xy[1], vel_z])
                self._current_angular_velocity = 0.0

    def _yaw_pitch_to_quaternion(self, yaw: float, pitch: float, roll: float = 0.0) -> List[float]:
        """
        Convert yaw and pitch angles to quaternion that orients robot's X+ axis
        toward the direction defined by (yaw, pitch), with specified roll.

        Args:
            yaw: Yaw angle in radians (rotation around Z axis)
            pitch: Pitch angle in radians (rotation around Y axis)
            roll: Roll angle in radians (rotation around X axis, default: 0.0)

        Returns:
            Quaternion [x, y, z, w]
        """
        # Create direction vector from yaw and pitch
        # X+ should point in this direction
        dx = np.cos(pitch) * np.cos(yaw)
        dy = np.cos(pitch) * np.sin(yaw)
        dz = np.sin(pitch)

        target_dir = np.array([dx, dy, dz])
        return self._direction_to_quaternion(target_dir, roll=roll)

    def _direction_to_quaternion(self, direction: np.ndarray, roll: float = 0.0) -> List[float]:
        """
        Create quaternion that rotates robot's X+ axis to point along given direction,
        with Z+ axis determined by roll angle (perpendicular to path plane).

        Args:
            direction: 3D direction vector (will be normalized) - robot's X+ axis
            roll: Roll angle in radians - determines Z+ axis orientation

        Returns:
            Quaternion [x, y, z, w] that aligns X+ with direction and Z+ perpendicular to plane
        """
        # Normalize direction
        dir_norm = direction / np.linalg.norm(direction)

        # X-axis = normalized direction (forward direction)
        x_axis = dir_norm

        # Build initial frame with natural up vector
        # Choose a temporary up vector (prefer world Z+, but avoid parallel case)
        if abs(np.dot(x_axis, np.array([0, 0, 1]))) < 0.99:
            temp_up = np.array([0, 0, 1])
        else:
            temp_up = np.array([0, 1, 0])

        # Build initial Y and Z axes (before roll rotation)
        # Y-axis points to the right, Z-axis points up (right-hand rule)
        y_initial = np.cross(temp_up, x_axis)
        if np.linalg.norm(y_initial) < 1e-6:
            # Edge case: x_axis parallel to temp_up
            y_initial = np.array([0, 1, 0]) if temp_up[0] == 1 else np.array([1, 0, 0])
        else:
            y_initial = y_initial / np.linalg.norm(y_initial)
        z_initial = np.cross(x_axis, y_initial)

        # Apply roll rotation around X-axis
        # This rotates Y and Z axes while keeping X fixed
        cos_roll = np.cos(roll)
        sin_roll = np.sin(roll)
        y_final = cos_roll * y_initial + sin_roll * z_initial
        z_final = -sin_roll * y_initial + cos_roll * z_initial

        # Build rotation matrix [X, Y, Z] as columns
        rot_matrix = np.column_stack([x_axis, y_final, z_final])

        # Convert rotation matrix to quaternion using scipy
        rot = R.from_matrix(rot_matrix)
        quat = rot.as_quat()  # [x, y, z, w]
        return quat

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
