"""
core/agent.py
Agent class for goal-based position control with max velocity and acceleration constraints.
Supports both mobile (use_fixed_base=False) and fixed (use_fixed_base=True) robots.
Supports both Mesh and URDF loading.
"""

import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

from .geometry import Pose
from .sim_object import SimObject, SimObjectSpawnParams, ShapeParams
from two_point_interpolation import TwoPointInterpolation
from .action import Action
from .types import MotionMode, DifferentialPhase, MovementDirection, ActionStatus
from .tools import normalize_vector_param

# Create logger for this module
logger = logging.getLogger(__name__)


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
    use_fixed_base: bool = False
    user_data: Dict[str, Any] = field(default_factory=dict)

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
                    - mesh_path or urdf_path (required)
                    - initial_pose: Pose object (optional)
                    - max_linear_vel, max_linear_accel, use_fixed_base, etc. (optional)

        Returns:
            AgentSpawnParams instance

        Example:
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
        # Get motion_mode and convert string to enum if needed
        motion_mode_value = config.get("motion_mode", MotionMode.OMNIDIRECTIONAL)
        if isinstance(motion_mode_value, str):
            motion_mode_value = MotionMode(motion_mode_value)

        # Create visual_shape from legacy parameters if provided
        visual_shape = config.get("visual_shape")
        if visual_shape is None and config.get("mesh_path"):
            visual_shape = ShapeParams(
                shape_type="mesh",
                mesh_path=config["mesh_path"],
                mesh_scale=config.get("mesh_scale", [1.0, 1.0, 1.0]),
                rgba_color=config.get("rgba_color", [0.2, 0.2, 0.2, 1.0]),
            )

        # Create collision_shape from legacy parameters if provided
        collision_shape = config.get("collision_shape")
        if collision_shape is None and config.get("mesh_path"):
            collision_shape = ShapeParams(shape_type="box", half_extents=config.get("collision_half_extents", [0.2, 0.1, 0.2]))

        return cls(
            visual_shape=visual_shape,
            collision_shape=collision_shape,
            urdf_path=config.get("urdf_path"),
            initial_pose=config.get("initial_pose"),
            max_linear_vel=config.get("max_linear_vel", 2.0),
            max_linear_accel=config.get("max_linear_accel", 5.0),
            max_angular_vel=config.get("max_angular_vel", 3.0),
            max_angular_accel=config.get("max_angular_accel", 10.0),
            motion_mode=motion_mode_value,
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
        self.max_linear_vel = normalize_vector_param(max_linear_vel, "max_linear_vel", 3)
        self.max_linear_accel = normalize_vector_param(max_linear_accel, "max_linear_accel", 3)

        # Normalize max_angular_vel and max_angular_accel to numpy arrays [yaw, pitch, roll]
        self.max_angular_vel = normalize_vector_param(max_angular_vel, "max_angular_vel", 3)
        self.max_angular_accel = normalize_vector_param(max_angular_accel, "max_angular_accel", 3)

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
        self._final_target_orientation: Optional[np.ndarray] = None  # Final orientation to align to
        self._align_final_orientation: bool = False  # Whether to align to final orientation
        self._movement_direction: MovementDirection = MovementDirection.FORWARD  # Movement direction (differential mode only)

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

        # Action queue system for high-level task execution
        self._action_queue: List[Action] = []
        self._current_action: Optional[Action] = None

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

        Example (Mesh):
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

        Example (URDF):
            params = AgentSpawnParams(
                urdf_path="arm_robot.urdf",
                initial_pose=Pose.from_xyz(0.0, 0.0, 0.0),
                use_fixed_base=True
            )
            robot = Agent.from_params(spawn_params=params)

        Example (Virtual Agent - invisible, no collision):
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
                visual_shape=spawn_params.visual_shape,
                collision_shape=spawn_params.collision_shape,
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

        # Set user_data if provided
        if spawn_params.user_data:
            agent.user_data.update(spawn_params.user_data)

        return agent

    @classmethod
    def from_mesh(
        cls,
        visual_shape: Optional[ShapeParams] = None,
        collision_shape: Optional[ShapeParams] = None,
        pose: Pose = None,
        mass: float = 0.0,
        max_linear_vel: Union[float, List[float]] = 2.0,
        max_linear_accel: Union[float, List[float]] = 5.0,
        max_angular_vel: Union[float, List[float]] = 3.0,
        max_angular_accel: Union[float, List[float]] = 10.0,
        motion_mode: Union[MotionMode, str] = MotionMode.OMNIDIRECTIONAL,
        use_fixed_base: bool = False,
        sim_core=None,
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

        Example:
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

        # Store mesh_path for backward compatibility
        stored_mesh_path = visual_shape.mesh_path if (visual_shape and visual_shape.shape_type == "mesh") else None

        # Create agent instance with Agent-specific parameters
        agent = cls(
            body_id=body_id,
            mesh_path=stored_mesh_path,
            max_linear_vel=max_linear_vel,
            max_linear_accel=max_linear_accel,
            max_angular_vel=max_angular_vel,
            max_angular_accel=max_angular_accel,
            motion_mode=motion_mode,
            use_fixed_base=use_fixed_base,
            sim_core=sim_core,
        )

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
            logger.warning("Cannot change motion mode while robot is moving " f"(body_id={self.body_id}). Call stop() first.")
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

    def _clear_path_visualization(self):
        """Clear all path visualization debug lines."""
        for debug_id in self._path_debug_ids:
            try:
                p.removeUserDebugItem(debug_id)
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
                start_pos, end_pos, lineColorRGB=color, lineWidth=width, lifeTime=0  # Permanent until manually removed
            )
            self._path_debug_ids.append(debug_id)

    def set_path(
        self,
        path: List[Pose],
        auto_approach: bool = True,
        final_orientation_align: bool = True,
        direction: Union[MovementDirection, str] = MovementDirection.FORWARD,
    ):
        """
        Set a path (list of waypoints) for the robot to follow.

        Args:
            path: List of Pose waypoints to follow in sequence
            auto_approach: If True, automatically add straight-line approach from current position
                          to first waypoint if distance > threshold (default: True)
            final_orientation_align: If True, add final rotation to match last waypoint's orientation
                                    after reaching the position (default: True)
            direction: Movement direction - MovementDirection.FORWARD (face movement direction) or
                      MovementDirection.BACKWARD (maintain current orientation, move backward).
                      Can also accept string "forward" or "backward" for convenience.
                      Note: Only used in differential drive mode (MotionMode.DIFFERENTIAL).
                            In omnidirectional mode, this parameter is ignored.
        """
        if self.use_fixed_base:
            logger.warning(f"Cannot set path for fixed-base robot (body_id={self.body_id})")
            return

        if len(path) == 0:
            logger.warning("Empty path provided")
            return

        # Normalize direction to enum
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
            logger.debug(f"Added approach waypoint at current position (distance to first: {distance_to_first:.2f}m)")

        # Add all path waypoints
        final_path.extend(path)

        # Add final orientation alignment if requested
        if final_orientation_align and len(path) > 0:
            last_waypoint = path[-1]
            # Check if final waypoint orientation differs from current
            # (This will be checked when we reach the last position)
            self._final_target_orientation = np.array(last_waypoint.orientation)
            self._align_final_orientation = True
        else:
            self._final_target_orientation = None
            self._align_final_orientation = False

        self._path = final_path
        self._current_waypoint_index = 0
        self._goal_pose = final_path[0]
        self._is_moving = True
        self._movement_direction = direction  # Store movement direction

        # Clear previous path visualization and visualize new path
        self._clear_path_visualization()
        self._visualize_path(final_path)

        # Initialize trajectory based on motion mode
        if self._motion_mode == MotionMode.OMNIDIRECTIONAL:
            self._init_omnidirectional_trajectory(final_path[0])
        elif self._motion_mode == MotionMode.DIFFERENTIAL:
            self._init_differential_rotation_trajectory(final_path[0])

    def _handle_goal_reached(self, current_pose: Pose):
        """
        Handle goal reached event: teleport to exact position and move to next waypoint.
        This is common logic for both omnidirectional and differential drive.

        Args:
            current_pose: Current robot pose
        """
        # Teleport to exact goal position before updating waypoint
        p.resetBasePositionAndOrientation(self.body_id, self._goal_pose.position, current_pose.orientation)

        logger.debug(f"Agent {self.body_id} reached waypoint at position {self._goal_pose.position[:2]}")
        r = R.from_quat(current_pose.orientation)
        yaw_current = r.as_euler("xyz", degrees=False)[2]
        logger.debug(f"  Current orientation: yaw={np.degrees(yaw_current):.1f}°, quat={current_pose.orientation}")

        # Stop moving
        self._current_velocity = np.array([0.0, 0.0, 0.0])
        self._current_angular_velocity = 0.0
        self._is_moving = False

        # Move to next waypoint if following path
        if self._path:  # Simpler: empty list is falsy
            self._current_waypoint_index += 1
            if self._current_waypoint_index < len(self._path):
                logger.debug(f"Agent {self.body_id} moving to next waypoint {self._current_waypoint_index}")
                self._goal_pose = self._path[self._current_waypoint_index]
                self._is_moving = True
                # Initialize trajectory based on motion mode
                if self._motion_mode == MotionMode.OMNIDIRECTIONAL:
                    self._init_omnidirectional_trajectory(self._goal_pose)
                elif self._motion_mode == MotionMode.DIFFERENTIAL:
                    self._init_differential_rotation_trajectory(self._goal_pose)
            else:
                # Path complete - check if we need final orientation alignment
                if self._align_final_orientation and self._final_target_orientation is not None:
                    # Start final rotation to match target orientation
                    logger.info("Path complete, aligning to final orientation...")
                    self._start_final_orientation_alignment()
                else:
                    # Path complete - stop all motion in PyBullet
                    self._path = []  # Clear path (was None)
                    self._goal_pose = None
                    # Reset velocity in PyBullet to stop any residual motion
                    p.resetBaseVelocity(self.body_id, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])
                    logger.info(f"Path complete (body_id={self.body_id})")

    def _start_final_orientation_alignment(self):
        """Start rotation to align with final target orientation after reaching last position."""
        current_pose = self.get_pose()

        # Create a goal pose at current position with target orientation
        final_goal = Pose(position=current_pose.position, orientation=self._final_target_orientation.tolist())

        # Set as goal and start rotation trajectory
        self._goal_pose = final_goal
        self._is_moving = True

        # Use differential rotation trajectory for the final alignment
        # (even for omnidirectional robots, we just rotate in place)
        if self._motion_mode == MotionMode.DIFFERENTIAL:
            self._init_differential_rotation_trajectory(final_goal)
        else:
            # For omnidirectional, we also use rotation trajectory but skip forward phase
            self._init_differential_rotation_trajectory(final_goal)

        # Mark that we're done with the path (just doing final rotation)
        self._path = []
        self._align_final_orientation = False  # Don't repeat this

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
            try:
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
            except ValueError as e:
                # TPI error: same position with different velocity (dp=0, dv!=0)
                # This can happen when setting a new goal at the current position
                # Warning only - robot will stay at current position
                logger.warning(
                    f"Agent {self.body_id}: Failed to initialize trajectory for axis {axis}: {e}. "
                    f"Robot will stay at current position."
                )
                # Initialize with zero movement (stay at current position)
                self._tpi_pos[axis].init(
                    p0=current_pos[axis],
                    pe=current_pos[axis],  # Same position
                    acc_max=self.max_linear_accel[axis],
                    vmax=self.max_linear_vel[axis],
                    t0=t0,
                    v0=0.0,  # Zero velocity
                    ve=0.0,
                    dec_max=self.max_linear_accel[axis],
                )
                self._tpi_pos[axis].calc_trajectory()

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
        # Use set_pose() to ensure attached objects are updated
        new_pose = Pose(position=new_pos.tolist(), orientation=current_pose.orientation)
        self.set_pose(new_pose)

    def _init_differential_rotation_trajectory(self, goal: Pose):
        """
        Initialize rotation trajectory for differential drive (Phase 1: Rotate to face target).
        Uses TPI for angle progression with acceleration/deceleration, combined with Slerp for
        smooth quaternion interpolation.

        Differential drive has two phases:
        1. Rotation phase: Rotate to align X-axis with movement direction using TPI + Slerp
           - TPI calculates angle progression considering max_angular_vel and max_angular_accel
           - Slerp provides smooth shortest-path quaternion interpolation
           - FORWARD: X+ points towards target (robot faces target)
           - BACKWARD: X- points towards target (robot faces away, X+ points away from target)
        2. Forward phase: Move along the path (position interpolates from start to goal regardless of direction)

        Args:
            goal: Target pose
        """
        current_pose = self.get_pose()
        current_pos = np.array(current_pose.position)
        goal_pos = np.array(goal.position)

        logger.debug(f"Agent {self.body_id} initializing differential rotation trajectory")
        logger.debug(f"  Current pos: {current_pos[:2]}, Goal pos: {goal_pos[:2]}")
        logger.debug(f"  Goal orientation (quat): {goal.orientation}")

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

        # Forward movement: rotate to face target (or opposite for backward)
        # Calculate target orientation
        self._differential_phase = DifferentialPhase.ROTATE

        # Get current and target quaternions
        self._rotation_start_quat = np.array(current_pose.orientation)  # [x, y, z, w]

        # Calculate target orientation:
        # For differential drive:
        # - FORWARD: X-axis (robot forward) points along path direction
        # - BACKWARD: X-axis points opposite to path direction (X- towards target)
        # - Roll (rotation around X-axis) has two modes:
        #   1. If goal point's X-axis is aligned with movement direction: use goal's roll
        #   2. Otherwise: minimize roll (natural orientation with Z pointing up)
        if self._forward_total_distance_3d > 1e-6:
            # Path direction (normalized)
            movement_direction_vec = direction_vec / self._forward_total_distance_3d

            # Determine X-axis direction based on forward/backward movement
            if self._movement_direction == MovementDirection.BACKWARD:
                # Backward: X-axis points opposite to movement (robot faces away from target)
                x_axis_target = -movement_direction_vec
            else:
                # Forward: X-axis points along movement (robot faces target)
                x_axis_target = movement_direction_vec

            # Get goal pose orientation axes
            goal_rotation = R.from_quat(goal.orientation)
            goal_rot_matrix = goal_rotation.as_matrix()
            x_axis_goal = goal_rot_matrix[:, 0]  # Goal's X-axis
            z_axis_goal = goal_rot_matrix[:, 2]  # Goal's Z-axis (desired up direction)

            # Check if goal's X-axis is aligned with movement direction
            # Use dot product to measure alignment (1.0 = perfectly aligned)
            alignment = np.dot(x_axis_goal, x_axis_target)

            logger.debug(f"Agent {self.body_id} checking orientation alignment:")
            logger.debug(f"  Movement direction: {x_axis_target}")
            logger.debug(f"  Goal's X-axis: {x_axis_goal}")
            logger.debug(f"  Alignment: {alignment:.3f} (threshold: 0.95)")

            # Threshold: cos(18°) ≈ 0.95 (allow up to 18 degrees deviation)
            if alignment > 0.95:
                # Goal's X-axis is aligned with movement direction
                # Use goal's complete orientation (including roll around X-axis)
                self._rotation_target_quat = np.array(goal.orientation)
                logger.debug(f"Agent {self.body_id} using goal's complete orientation")
                logger.info(f"Using goal's complete orientation (alignment={alignment:.3f})")
            else:
                # Goal's X-axis is NOT aligned with movement direction
                # Build orientation with X pointing in movement direction
                # and minimize roll (Z should point mostly up)

                # Y-axis = Z_goal × X_target (right-hand rule)
                y_axis = np.cross(z_axis_goal, x_axis_target)
                y_norm_magnitude = np.linalg.norm(y_axis)

                if y_norm_magnitude < 1e-6:
                    # X and Z are parallel (or nearly so)
                    # Keep X-axis, choose a perpendicular Y-axis
                    if abs(x_axis_target[2]) < 0.9:
                        # X is not vertical, use world up × X
                        y_axis = np.cross(np.array([0, 0, 1]), x_axis_target)
                    else:
                        # X is nearly vertical, use world Y × X
                        y_axis = np.cross(np.array([0, 1, 0]), x_axis_target)
                    y_axis = y_axis / np.linalg.norm(y_axis)
                else:
                    y_axis = y_axis / y_norm_magnitude

                # Recompute Z-axis to ensure orthogonality: Z = X × Y
                z_axis_final = np.cross(x_axis_target, y_axis)

                # Build rotation matrix with [X, Y, Z] as columns
                rotation_matrix = np.column_stack([x_axis_target, y_axis, z_axis_final])

                # Convert to quaternion
                r = R.from_matrix(rotation_matrix)
                self._rotation_target_quat = np.array(r.as_quat())  # [x, y, z, w]
        else:
            # No movement needed - this happens when waypoint is at current position
            # Keep current orientation (don't rotate to waypoint orientation)
            self._rotation_target_quat = self._rotation_start_quat

        # Calculate rotation angle between quaternions
        r_current = R.from_quat(self._rotation_start_quat)
        r_target = R.from_quat(self._rotation_target_quat)
        r_delta = r_target * r_current.inv()
        rotation_angle = r_delta.magnitude()  # Total rotation angle in radians

        logger.debug(f"Agent {self.body_id} rotation: {rotation_angle:.3f} rad ({np.degrees(rotation_angle):.1f}°)")

        # Use TPI to calculate rotation trajectory with angular acceleration and velocity constraints
        if rotation_angle > 1e-6:
            # Store total rotation angle for Slerp interpolation (ensure float type)
            self._rotation_total_angle = float(rotation_angle)

            # Use first element of max_angular_vel and max_angular_accel for overall rotation
            angular_vel = self.max_angular_vel[0]
            angular_accel = self.max_angular_accel[0]

            # Initialize TPI for rotation angle (from 0 to rotation_angle)
            try:
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
            except ValueError as e:
                # TPI error: skip rotation and go directly to forward phase
                logger.warning(
                    f"Agent {self.body_id}: Failed to initialize rotation trajectory: {e}. " f"Skipping rotation phase."
                )
                # Teleport to target orientation
                p.resetBasePositionAndOrientation(self.body_id, current_pos.tolist(), self._rotation_target_quat.tolist())
                # Skip to FORWARD phase
                self._differential_phase = DifferentialPhase.FORWARD
                self._init_differential_forward_distance_trajectory(self._forward_total_distance_3d)
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
        try:
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
        except ValueError as e:
            # TPI error: distance is zero or invalid
            logger.warning(
                f"Agent {self.body_id}: Failed to initialize forward trajectory: {e}. " f"Goal may already be reached."
            )
            # Initialize with zero movement (already at goal)
            self._tpi_forward = TwoPointInterpolation()
            self._tpi_forward.init(
                p0=0.0,
                pe=0.0,  # Zero distance
                acc_max=avg_accel,
                vmax=avg_vel,
                t0=t0,
                v0=0.0,
                ve=0.0,
                dec_max=avg_accel,
            )
            self._tpi_forward.calc_trajectory()

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
                final_pose = Pose(position=current_pos.tolist(), orientation=self._rotation_target_quat.tolist())
                self.set_pose(final_pose)

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
                new_pose = Pose(position=current_pos.tolist(), orientation=new_orientation)
                self.set_pose(new_pose)
            else:
                # Fallback: use target orientation directly
                fallback_pose = Pose(position=current_pos.tolist(), orientation=self._rotation_target_quat.tolist())
                self.set_pose(fallback_pose)

            # Update angular velocity from TPI
            self._current_angular_velocity = angular_vel
            self._current_velocity = np.array([0.0, 0.0, 0.0])

            # Log current yaw during rotation (debug level)
            logger.debug(f"Agent {self.body_id} rotating, current yaw: {self.get_pose().yaw:.1f}°")

        elif self._differential_phase == DifferentialPhase.FORWARD:
            # Phase 2: Move forward/backward using TPI
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

            # Orientation: use orientation from rotation phase
            # (For FORWARD: X+ towards target, for BACKWARD: X- towards target)
            new_orientation = self._rotation_target_quat.tolist()

            # Velocity: along 3D direction (same for both forward and backward)
            # Note: Position interpolates from start to goal regardless of orientation
            direction_3d_unit = direction_3d / total_distance if total_distance > 1e-6 else np.array([0.0, 0.0, 0.0])
            self._current_velocity = direction_3d_unit * forward_vel

            # Update position/orientation - use set_pose() to update attached objects
            new_pose = Pose(position=new_pos.tolist(), orientation=new_orientation)
            self.set_pose(new_pose)
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

    # ============================================================================
    # Action Queue System
    # ============================================================================

    def add_action(self, action: Action):
        """
        Add an action to the execution queue.

        Actions are executed sequentially in the order they are added.

        Args:
            action: Action instance to add to queue

        Example:
            agent.add_action(MoveAction(path=my_path))
            agent.add_action(WaitAction(duration=5.0))
        """
        self._action_queue.append(action)
        logger.info(f"Added {action.__class__.__name__} to queue (queue size: {len(self._action_queue)})")

    def add_action_sequence(self, actions: List[Action]):
        """
        Add multiple actions to the execution queue.

        Args:
            actions: List of Action instances to add

        Example:
            agent.add_action_sequence([
                MoveAction(path=path1),
                PickAction(target_object_id=obj.body_id),
                MoveAction(path=path2),
                DropAction(drop_position=[10, 5, 0])
            ])
        """
        self._action_queue.extend(actions)
        logger.info(f"Added {len(actions)} actions to queue (queue size: {len(self._action_queue)})")

    def clear_actions(self):
        """
        Clear all actions from the queue and cancel current action.
        """
        self._action_queue.clear()
        if self._current_action is not None:
            self._current_action.cancel()
            self._current_action = None
        logger.info("Cleared all actions")

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
                logger.info(
                    f"Starting {self._current_action.__class__.__name__} " f"(remaining in queue: {len(self._action_queue)})"
                )
            else:
                return  # No actions to execute

        # Execute current action
        is_complete = self._current_action.execute(self, dt)

        if is_complete:
            action_name = self._current_action.__class__.__name__
            status = self._current_action.status

            if status == ActionStatus.COMPLETED:
                logger.info(f"{action_name} completed successfully")
            elif status == ActionStatus.FAILED:
                logger.error(f"{action_name} failed: {self._current_action.error_message}")
            elif status == ActionStatus.CANCELLED:
                logger.warning(f"{action_name} was cancelled")

            # Move to next action
            self._current_action = None

    def update(self, dt: float):
        """
        Update robot position towards goal with velocity/acceleration constraints.

        Supports two motion modes:
        - MotionMode.OMNIDIRECTIONAL: Move in any direction without rotating first (smooth trajectories)
        - MotionMode.DIFFERENTIAL: Rotate to face target, then move forward (smooth trajectories)

        Also processes action queue for high-level task execution.

        Note: This does nothing if the robot has a fixed base.

        Args:
            dt: Time step (seconds)
        """
        # Process action queue first (actions may set goals/paths)
        self._update_actions(dt)

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
        self._final_target_orientation = None
        self._align_final_orientation = False
        self._movement_direction = MovementDirection.FORWARD
        self._clear_path_visualization()  # Clear visualization when stopping

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
            logger.warning("set_joint_target() only works for URDF robots")
            return

        if joint_index >= len(self.joint_info):
            logger.warning(f"joint_index {joint_index} out of range (max: {len(self.joint_info)-1})")
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
            logger.warning("get_joint_state() only works for URDF robots")
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
            logger.warning("set_all_joint_targets() only works for URDF robots")
            return

        if len(target_positions) != len(self.joint_info):
            logger.warning(f"Expected {len(self.joint_info)} targets, got {len(target_positions)}")
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
