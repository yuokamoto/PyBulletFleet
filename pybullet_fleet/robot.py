"""
core/robot.py
Agent class for goal-based position control with max velocity and acceleration constraints.
Supports both mobile (use_fixed_base=False) and fixed (use_fixed_base=True) robots.
Supports both Mesh and URDF loading.
Designed with ROS2 geometry_msgs/Pose compatibility in mind.
"""
import pybullet as p
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Dict, Any

# Import SimObject and Pose from sim_object module
from pybullet_fleet.sim_object import SimObject, Pose


@dataclass
class AgentSpawnParams:
    """
    Robot spawn parameters for Agent class.
    
    These parameters define the physical properties, appearance, and initial state of a robot.
    They are used by Agent.from_params() and RobotManager.spawn_robots_grid().
    
    Supports both Mesh and URDF robots. Specify either mesh_path or urdf_path (not both).
    
    Attributes:
        mesh_path: Path to robot mesh file (for mesh-based robots)
        urdf_path: Path to robot URDF file (for URDF-based robots with joints)
        initial_position: Initial [x, y, z] position in world coordinates (optional, used by from_params)
        max_vel: Maximum velocity in m/s (ignored if use_fixed_base=True)
        max_accel: Maximum acceleration in m/s² (ignored if use_fixed_base=True)
        orientation_euler: Orientation in Euler angles [roll, pitch, yaw] radians
        mesh_scale: Mesh scaling factors [sx, sy, sz] (for mesh robots only)
        collision_half_extents: Collision box half extents [hx, hy, hz] in meters (for mesh robots only)
        rgba_color: RGBA color [r, g, b, a] (0.0-1.0)
        base_mass: Robot mass in kg (0.0 for kinematic control)
        use_fixed_base: If True, robot base is fixed and doesn't move (default: False)
    
    Note:
        initial_position is optional and mainly used by Agent.from_params().
        RobotManager.spawn_robots_grid() calculates positions automatically and ignores this field.
    """
    mesh_path: Optional[str] = None
    urdf_path: Optional[str] = None
    initial_position: Optional[List[float]] = None  # [x, y, z]
    max_vel: float = 2.0
    max_accel: float = 5.0
    orientation_euler: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    mesh_scale: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    collision_half_extents: List[float] = field(default_factory=lambda: [0.2, 0.1, 0.2])
    rgba_color: List[float] = field(default_factory=lambda: [0.2, 0.2, 0.2, 1.0])
    base_mass: float = 0.0
    use_fixed_base: bool = False
    
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
                    - max_vel, max_accel, use_fixed_base, etc. (optional)
        
        Returns:
            AgentSpawnParams instance
        
        Example:
            config = {
                'urdf_path': 'robots/mobile_robot.urdf',
                'max_vel': 2.0,
                'max_accel': 5.0,
                'use_fixed_base': False
            }
            params = AgentSpawnParams.from_dict(config)
        """
        return cls(
            mesh_path=config.get('mesh_path'),
            urdf_path=config.get('urdf_path'),
            initial_position=config.get('initial_position'),
            max_vel=config.get('max_vel', 2.0),
            max_accel=config.get('max_accel', 5.0),
            orientation_euler=config.get('orientation_euler', [0.0, 0.0, 0.0]),
            mesh_scale=config.get('mesh_scale', [1.0, 1.0, 1.0]),
            collision_half_extents=config.get('collision_half_extents', [0.2, 0.1, 0.2]),
            rgba_color=config.get('rgba_color', [0.2, 0.2, 0.2, 1.0]),
            base_mass=config.get('base_mass', 0.0),
            use_fixed_base=config.get('use_fixed_base', False)
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
    _shared_shapes = {}
    
    def __init__(self, body_id: int, 
                 mesh_path: Optional[str] = None,
                 urdf_path: Optional[str] = None,
                 max_vel: float = 2.0,
                 max_accel: float = 5.0,
                 use_fixed_base: bool = False,
                 sim_core=None):
        """
        Initialize Agent.
        
        Args:
            body_id: PyBullet body ID
            mesh_path: Path to robot mesh file (if mesh-based)
            urdf_path: Path to robot URDF file (if URDF-based)
            max_vel: Maximum velocity (m/s)
            max_accel: Maximum acceleration (m/s²)
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
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.use_fixed_base = use_fixed_base
        
        # URDF-specific: joint information
        self.joint_info = []
        if urdf_path is not None:
            num_joints = p.getNumJoints(body_id)
            self.joint_info = [p.getJointInfo(body_id, j) for j in range(num_joints)]
        
        # Goal tracking (only used for non-static robots)
        self.goal_pose: Optional[Pose] = None
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.is_moving = False
        
        # User-extensible data storage
        self.user_data: Dict[str, Any] = {}
        
    @classmethod
    def create_shared_shapes(cls, mesh_path: str, mesh_scale: List[float] = None,
                           collision_half_extents: List[float] = None,
                           rgba_color: List[float] = None) -> Tuple[int, int]:
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
        visual_id = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=mesh_path,
            rgbaColor=rgba_color,
            meshScale=mesh_scale
        )
        
        # Create collision shape (simple box)
        collision_id = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=collision_half_extents
        )
        
        cls._shared_shapes[mesh_path] = (visual_id, collision_id)
        return visual_id, collision_id
    
    @classmethod
    def from_params(cls, spawn_params: AgentSpawnParams, 
                   sim_core=None) -> 'Robot':
        """
        Create a Robot from AgentSpawnParams.
        
        This is the recommended way to spawn robots when using AgentSpawnParams.
        Automatically detects whether to use mesh or URDF based on spawn_params.
        
        Args:
            spawn_params: AgentSpawnParams instance with all robot parameters
            sim_core: Reference to simulation core (optional)
        
        Returns:
            Robot instance
        
        Example (Mesh):
            params = AgentSpawnParams(
                mesh_path="robot.obj",
                initial_position=[1.0, 2.0, 0.0],
                max_vel=3.0
            )
            robot = Agent.from_params(spawn_params=params)
            
        Example (URDF):
            params = AgentSpawnParams(
                urdf_path="arm_robot.urdf",
                initial_position=[0.0, 0.0, 0.0],
                use_fixed_base=True
            )
            robot = Agent.from_params(spawn_params=params)
        """
        # Validate required parameters
        if spawn_params.initial_position is None:
            raise ValueError("spawn_params.initial_position must be provided")
        
        orientation = p.getQuaternionFromEuler(spawn_params.orientation_euler)
        
        # Choose mesh or URDF path
        if spawn_params.urdf_path is not None:
            # URDF robot
            return cls.from_urdf(
                urdf_path=spawn_params.urdf_path,
                position=spawn_params.initial_position,
                orientation=orientation,
                max_vel=spawn_params.max_vel,
                max_accel=spawn_params.max_accel,
                use_fixed_base=spawn_params.use_fixed_base,
                sim_core=sim_core
            )
        else:
            # Mesh robot
            return cls.from_mesh(
                mesh_path=spawn_params.mesh_path,
                position=spawn_params.initial_position,
                orientation=orientation,
                mesh_scale=spawn_params.mesh_scale,
                collision_half_extents=spawn_params.collision_half_extents,
                rgba_color=spawn_params.rgba_color,
                base_mass=spawn_params.base_mass,
                max_vel=spawn_params.max_vel,
                max_accel=spawn_params.max_accel,
                use_fixed_base=spawn_params.use_fixed_base,
                sim_core=sim_core
            )
    
    @classmethod
    def from_mesh(cls, mesh_path: str, position: List[float], orientation: List[float],
                 mesh_scale: List[float] = None,
                 collision_half_extents: List[float] = None,
                 rgba_color: List[float] = None,
                 base_mass: float = 0.0,
                 max_vel: float = 2.0,
                 max_accel: float = 5.0,
                 use_fixed_base: bool = False,
                 sim_core=None) -> 'Robot':
        """
        Create a mesh-based Agent.
        
        Args:
            mesh_path: Path to robot mesh file (.obj, .dae, etc.)
            position: Initial [x, y, z] position
            orientation: Orientation as quaternion [x, y, z, w]
            mesh_scale: Mesh scaling [sx, sy, sz]
            collision_half_extents: Collision box half extents [hx, hy, hz]
            rgba_color: RGBA color [r, g, b, a]
            base_mass: Robot mass (kg), 0.0 for kinematic control
            max_vel: Maximum velocity (m/s)
            max_accel: Maximum acceleration (m/s²)
            use_fixed_base: If True, robot base is fixed and doesn't move
            sim_core: Reference to simulation core
        
        Returns:
            Robot instance
        
        Example:
            robot = Agent.from_mesh(
                mesh_path="robot.obj",
                position=[0, 0, 0.5],
                orientation=[0, 0, 0, 1],
                max_vel=2.0
            )
        """
        # Get or create shared shapes
        visual_id, collision_id = cls.create_shared_shapes(
            mesh_path=mesh_path,
            mesh_scale=mesh_scale,
            collision_half_extents=collision_half_extents,
            rgba_color=rgba_color
        )
        
        # Create multibody
        body_id = p.createMultiBody(
            baseMass=base_mass,
            baseCollisionShapeIndex=collision_id,
            baseVisualShapeIndex=visual_id,
            basePosition=position,
            baseOrientation=orientation
        )
        
        return cls(
            body_id=body_id,
            mesh_path=mesh_path,
            max_vel=max_vel,
            max_accel=max_accel,
            use_fixed_base=use_fixed_base,
            sim_core=sim_core
        )
    
    @classmethod
    def from_urdf(cls, urdf_path: str, position: List[float], orientation: List[float],
                 max_vel: float = 2.0,
                 max_accel: float = 5.0,
                 use_fixed_base: bool = False,
                 sim_core=None) -> 'Robot':
        """
        Create a URDF-based Robot (with joints).
        
        Args:
            urdf_path: Path to robot URDF file
            position: Initial [x, y, z] position
            orientation: Orientation as quaternion [x, y, z, w]
            max_vel: Maximum velocity (m/s)
            max_accel: Maximum acceleration (m/s²)
            use_fixed_base: If True, robot base is fixed in space
            sim_core: Reference to simulation core
        
        Returns:
            Robot instance
        
        Example:
            robot = Agent.from_urdf(
                urdf_path="arm_robot.urdf",
                position=[0, 0, 0],
                orientation=[0, 0, 0, 1],
                use_fixed_base=True
            )
            robot.set_joint_target(0, 1.57)  # Control first joint
        """
        body_id = p.loadURDF(
            urdf_path,
            position,
            orientation,
            useFixedBase=use_fixed_base
        )
        
        return cls(
            body_id=body_id,
            urdf_path=urdf_path,
            max_vel=max_vel,
            max_accel=max_accel,
            use_fixed_base=use_fixed_base,
            sim_core=sim_core
        )
    
    def get_pose(self) -> Pose:
        """
        Get current robot pose (compatible with ROS2 geometry_msgs/Pose).
        
        Returns:
            Pose object with current position and orientation
        """
        position, orientation = p.getBasePositionAndOrientation(self.body_id)
        return Pose.from_pybullet(position, orientation)
    
    def set_pose(self, pose: Pose):
        """
        Directly set robot pose (teleport) via PyBullet.
        
        Args:
            pose: Target Pose to teleport to
        """
        p.resetBasePositionAndOrientation(
            self.body_id,
            pose.position,
            pose.orientation
        )
        # Reset velocity when teleporting
        p.resetBaseVelocity(self.body_id, [0, 0, 0], [0, 0, 0])
        self.velocity = [0.0, 0.0, 0.0]
    
    def set_goal_pose(self, goal: Pose):
        """
        Set goal pose for the robot to move to.
        
        Note: This is ignored if the robot has a fixed base.
        
        Args:
            goal: Target Pose (compatible with ROS2 geometry_msgs/Pose)
        """
        if self.use_fixed_base:
            print(f"[Agent] Warning: Cannot set goal for fixed-base robot (body_id={self.body_id})")
            return
        
        self.goal_pose = goal
        self.is_moving = True
    
    def update(self, dt: float):
        """
        Update robot position towards goal with velocity/acceleration constraints.
        
        Note: This does nothing if the robot has a fixed base.
        
        Args:
            dt: Time step (seconds)
        """
        if self.use_fixed_base:
            return
        
        if not self.is_moving or self.goal_pose is None:
            return
        
        # Get current state
        current_pose = self.get_pose()
        current_pos = np.array(current_pose.position)
        goal_pos = np.array(self.goal_pose.position)
        
        # Calculate direction and distance
        direction = goal_pos - current_pos
        distance = np.linalg.norm(direction)
        
        # Check if reached goal
        if distance < 0.01:  # 1cm threshold
            # Stop at goal
            self.current_velocity = np.array([0.0, 0.0, 0.0])
            self.is_moving = False
            
            # Set exact goal position and orientation
            p.resetBasePositionAndOrientation(
                self.body_id,
                self.goal_pose.position,
                self.goal_pose.orientation
            )
            return
        
        # Normalize direction
        direction = direction / distance
        
        # Calculate desired velocity (max velocity towards goal)
        desired_velocity = direction * self.max_vel
        
        # Apply acceleration constraint (simplified - no actual acceleration tracking yet)
        # For now, just limit velocity magnitude
        velocity_magnitude = np.linalg.norm(desired_velocity)
        if velocity_magnitude > self.max_vel:
            desired_velocity = (desired_velocity / velocity_magnitude) * self.max_vel
        
        # Calculate movement for this timestep
        move_distance = np.linalg.norm(desired_velocity) * dt
        if move_distance > distance:
            # Would overshoot, just move to goal
            new_pos = goal_pos
        else:
            new_pos = current_pos + desired_velocity * dt
        
        # Update position (keep current orientation for now)
        p.resetBasePositionAndOrientation(
            self.body_id,
            new_pos.tolist(),
            current_pose.orientation
        )
        
        self.current_velocity = desired_velocity
    
    def get_velocity(self) -> np.ndarray:
        """
        Get current velocity vector.
        
        Returns:
            Velocity [vx, vy, vz] in m/s
        """
        return self.current_velocity.copy()
    
    def stop(self):
        """Stop robot movement and clear goal."""
        self.goal_pose = None
        self.is_moving = False
        self.current_velocity = np.array([0.0, 0.0, 0.0])
    
    # ========================================
    # URDF-specific methods (joint control)
    # ========================================
    
    def is_urdf_robot(self) -> bool:
        """Check if this robot is URDF-based (has joints)."""
        return self.urdf_path is not None
    
    def get_num_joints(self) -> int:
        """Get number of joints (0 for mesh robots)."""
        return len(self.joint_info)
    
    def set_joint_target(self, joint_index: int, target_position: float, 
                        max_force: float = 500.0):
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
            print(f"[Agent] Warning: set_joint_target() only works for URDF robots")
            return
        
        if joint_index >= len(self.joint_info):
            print(f"[Agent] Warning: joint_index {joint_index} out of range (max: {len(self.joint_info)-1})")
            return
        
        p.setJointMotorControl2(
            bodyUniqueId=self.body_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_position,
            force=max_force
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
            print(f"[Agent] Warning: get_joint_state() only works for URDF robots")
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
            print(f"[Agent] Warning: set_all_joint_targets() only works for URDF robots")
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
        return (f"Agent(id={self.body_id}{fixed_str}{urdf_str}, "
                f"pos={pose.position[:2]}, "
                f"vel={self.current_velocity[:2].tolist()}, "
                f"moving={self.is_moving})")

