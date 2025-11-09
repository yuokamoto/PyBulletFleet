"""
core/robot.py
Robot class for goal-based position control with max velocity and acceleration constraints.
Supports both mobile (static=False) and static (static=True) robots.
Designed with ROS2 geometry_msgs/Pose compatibility in mind.
"""
import pybullet as p
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Dict, Any


@dataclass
class RobotPose:
    """
    Position and orientation representation compatible with ROS2 geometry_msgs/Pose.
    
    Note: Named 'RobotPose' to avoid conflict with core_simulation.Pose and future ROS2 imports.
    
    Attributes:
        position: [x, y, z] in world coordinates
        orientation: [x, y, z, w] quaternion (optional, defaults to no rotation)
    """
    position: List[float]  # [x, y, z]
    orientation: Optional[List[float]] = None  # [x, y, z, w] quaternion
    
    def __post_init__(self):
        if self.orientation is None:
            # Default: no rotation (identity quaternion)
            self.orientation = [0.0, 0.0, 0.0, 1.0]
    
    @classmethod
    def from_xyz(cls, x: float, y: float, z: float):
        """Create RobotPose from x, y, z coordinates."""
        return cls(position=[x, y, z])
    
    @classmethod
    def from_pybullet(cls, position: Tuple[float, float, float], 
                     orientation: Tuple[float, float, float, float]):
        """Create RobotPose from PyBullet position and orientation tuples."""
        return cls(position=list(position), orientation=list(orientation))


@dataclass
class RobotSpawnParams:
    """
    Robot spawn parameters for Robot class.
    
    These parameters define the physical properties, appearance, and initial state of a robot.
    They are used by Robot.from_params() and RobotManager.spawn_robots_grid().
    
    Attributes:
        mesh_path: Path to robot mesh file
        initial_position: Initial [x, y, z] position in world coordinates (optional, used by from_params)
        max_vel: Maximum velocity in m/s (ignored if static=True)
        max_accel: Maximum acceleration in m/s² (ignored if static=True)
        orientation_euler: Orientation in Euler angles [roll, pitch, yaw] radians
        mesh_scale: Mesh scaling factors [sx, sy, sz]
        collision_half_extents: Collision box half extents [hx, hy, hz] in meters
        rgba_color: RGBA color [r, g, b, a] (0.0-1.0)
        base_mass: Robot mass in kg (0.0 for kinematic control)
        static: If True, robot is static and doesn't move (default: False)
    
    Note:
        initial_position is optional and mainly used by Robot.from_params().
        RobotManager.spawn_robots_grid() calculates positions automatically and ignores this field.
    """
    mesh_path: str
    initial_position: Optional[List[float]] = None  # [x, y, z]
    max_vel: float = 2.0
    max_accel: float = 5.0
    orientation_euler: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    mesh_scale: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    collision_half_extents: List[float] = field(default_factory=lambda: [0.2, 0.1, 0.2])
    rgba_color: List[float] = field(default_factory=lambda: [0.2, 0.2, 0.2, 1.0])
    base_mass: float = 0.0
    static: bool = False


class Robot:
    """
    Robot class with goal-based position control.
    
    Supports both mobile robots (static=False) and static robots (static=True).
    
    Features:
    - Accepts RobotPose goals (compatible with ROS2 geometry_msgs/Pose)
    - Max velocity and acceleration constraints (for mobile robots)
    - Current pose query API
    - Shared shape optimization for fast spawning
    - Static robot support (non-moving objects)
    
    This class is designed to be eventually controlled by ROS2 nodes,
    so the API follows ROS message conventions.
    """
    
    # Class-level shared shapes (for optimization)
    _shared_shapes = {}
    
    def __init__(self, body_id: int, mesh_path: str, 
                 max_vel: float = 2.0,
                 max_accel: float = 5.0,
                 static: bool = False,
                 sim_core=None):
        """
        Initialize Robot.
        
        Args:
            body_id: PyBullet body ID
            mesh_path: Path to robot mesh file
            max_vel: Maximum velocity (m/s)
            max_accel: Maximum acceleration (m/s²)
            static: If True, robot is static and doesn't move
            sim_core: Reference to simulation core (optional)
        
        Note:
            For spawning robots from RobotSpawnParams, use Robot.from_params() instead.
        """
        self.body_id = body_id
        self.mesh_path = mesh_path
        self.sim_core = sim_core
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.static = static
        
        # Goal tracking (only used for non-static robots)
        self.goal_pose: Optional[RobotPose] = None
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
    def from_params(cls, spawn_params: RobotSpawnParams, 
                   sim_core=None) -> 'Robot':
        """
        Create a Robot from RobotSpawnParams (similar to MultiRobotSimulationCore.from_params()).
        
        This is the recommended way to spawn robots when using RobotSpawnParams.
        
        Args:
            spawn_params: RobotSpawnParams instance with all robot parameters
            sim_core: Reference to simulation core (optional)
        
        Returns:
            Robot instance
        
        Example:
            params = RobotSpawnParams(
                mesh_path="robot.obj",
                initial_position=[1.0, 2.0, 0.0],
                max_vel=3.0,
                static=False
            )
            robot = Robot.from_params(spawn_params=params)
        """
        # Validate required parameters
        if spawn_params.initial_position is None:
            raise ValueError("spawn_params.initial_position must be provided")
        if not spawn_params.mesh_path:
            raise ValueError("spawn_params.mesh_path must be provided")
        
        # Get or create shared shapes
        visual_id, collision_id = cls.create_shared_shapes(
            mesh_path=spawn_params.mesh_path,
            mesh_scale=spawn_params.mesh_scale,
            collision_half_extents=spawn_params.collision_half_extents,
            rgba_color=spawn_params.rgba_color
        )
        
        # Convert euler to quaternion
        orientation = p.getQuaternionFromEuler(spawn_params.orientation_euler)
        
        # Create multibody
        body_id = p.createMultiBody(
            baseMass=spawn_params.base_mass,
            baseCollisionShapeIndex=collision_id,
            baseVisualShapeIndex=visual_id,
            basePosition=spawn_params.initial_position,
            baseOrientation=orientation
        )
        
        # Create robot instance with simplified __init__
        robot = cls(
            body_id=body_id,
            mesh_path=spawn_params.mesh_path,
            max_vel=spawn_params.max_vel,
            max_accel=spawn_params.max_accel,
            static=spawn_params.static,
            sim_core=sim_core
        )
        
        return robot
    
    def get_pose(self) -> RobotPose:
        """
        Get current robot pose (compatible with ROS2 geometry_msgs/Pose).
        
        Returns:
            RobotPose object with current position and orientation
        """
        position, orientation = p.getBasePositionAndOrientation(self.body_id)
        return RobotPose.from_pybullet(position, orientation)
    
    def set_pose(self, pose: RobotPose):
        """
        Directly set robot pose (teleport) via PyBullet.
        
        Args:
            pose: Target RobotPose to teleport to
        """
        p.resetBasePositionAndOrientation(
            self.body_id,
            pose.position,
            pose.orientation
        )
        # Reset velocity when teleporting
        p.resetBaseVelocity(self.body_id, [0, 0, 0], [0, 0, 0])
        self.velocity = [0.0, 0.0, 0.0]
    
    def set_goal_pose(self, goal: RobotPose):
        """
        Set goal pose for the robot to move to.
        
        Note: This is ignored if the robot is static.
        
        Args:
            goal: Target RobotPose (compatible with ROS2 geometry_msgs/Pose)
        """
        if self.static:
            print(f"[Robot] Warning: Cannot set goal for static robot (body_id={self.body_id})")
            return
        
        self.goal_pose = goal
        self.is_moving = True
    
    def update(self, dt: float):
        """
        Update robot position towards goal with velocity/acceleration constraints.
        
        Note: This does nothing if the robot is static.
        
        Args:
            dt: Time step (seconds)
        """
        if self.static:
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
    
    def __repr__(self):
        pose = self.get_pose()
        static_str = " [STATIC]" if self.static else ""
        return (f"Robot(id={self.body_id}{static_str}, "
                f"pos={pose.position}, "
                f"vel={self.current_velocity.tolist()}, "
                f"moving={self.is_moving})")
