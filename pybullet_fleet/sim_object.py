"""
sim_object.py
Base class for simulation objects with attachment support.
"""

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import numpy as np
import pybullet as p


@dataclass
class Pose:
    """
    Position and orientation representation for any object in the simulation.
    Compatible with ROS2 geometry_msgs/Pose and PyBullet's (position, orientation) tuples.

    Supports both quaternion and Euler angle representations.

    Attributes:
        position: [x, y, z] in world coordinates
        orientation: [x, y, z, w] quaternion (defaults to no rotation)
    """

    position: List[float]  # [x, y, z]
    orientation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])  # [x, y, z, w] quaternion

    # Convenient property accessors
    @property
    def x(self) -> float:
        """X coordinate."""
        return self.position[0]

    @property
    def y(self) -> float:
        """Y coordinate."""
        return self.position[1]

    @property
    def z(self) -> float:
        """Z coordinate."""
        return self.position[2]

    @property
    def roll(self) -> float:
        """Roll angle in radians (rotation around x-axis)."""
        return p.getEulerFromQuaternion(self.orientation)[0]

    @property
    def pitch(self) -> float:
        """Pitch angle in radians (rotation around y-axis)."""
        return p.getEulerFromQuaternion(self.orientation)[1]

    @property
    def yaw(self) -> float:
        """Yaw angle in radians (rotation around z-axis)."""
        return p.getEulerFromQuaternion(self.orientation)[2]

    @classmethod
    def from_xyz(cls, x: float, y: float, z: float):
        """Create Pose from x, y, z coordinates."""
        return cls(position=[x, y, z])

    @classmethod
    def from_euler(cls, x: float, y: float, z: float, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0):
        """
        Create Pose from position and Euler angles.

        Args:
            x, y, z: Position in world coordinates
            roll, pitch, yaw: Euler angles in radians
        """
        quat = p.getQuaternionFromEuler([roll, pitch, yaw])
        return cls(position=[x, y, z], orientation=list(quat))

    @classmethod
    def from_pybullet(cls, position: Tuple[float, float, float], orientation: Tuple[float, float, float, float]):
        """Create Pose from PyBullet position and orientation tuples."""
        return cls(position=list(position), orientation=list(orientation))

    def as_position(self) -> List[float]:
        """Return position as list [x, y, z]."""
        return self.position

    def as_orientation(self) -> List[float]:
        """Return orientation as quaternion [x, y, z, w]."""
        return self.orientation

    def as_position_orientation(self) -> Tuple[List[float], List[float]]:
        """Return (position, orientation) tuple for PyBullet."""
        return self.position, self.orientation

    def as_tuple(self) -> Tuple[List[float], List[float]]:
        """
        Return (position, orientation) tuple.
        Alias for as_position_orientation() for convenience.
        """
        return self.position, self.orientation

    def as_euler(self) -> Tuple[float, float, float]:
        """Return orientation as Euler angles (roll, pitch, yaw)."""
        return p.getEulerFromQuaternion(self.orientation)

    @classmethod
    def from_yaw(cls, x: float, y: float, z: float, yaw: float):
        """
        Create Pose from position and yaw angle (rotation around z-axis).

        Args:
            x, y, z: Position in world coordinates
            yaw: Yaw angle in radians

        Returns:
            Pose instance
        """
        # Use PyBullet to convert yaw to quaternion (roll=0, pitch=0, yaw=yaw)
        quaternion = p.getQuaternionFromEuler([0.0, 0.0, yaw])
        return cls(position=[x, y, z], orientation=list(quaternion))

    def to_yaw_quaternion(self, yaw: float) -> List[float]:
        """
        Create quaternion from yaw angle (rotation around z-axis).

        Args:
            yaw: Yaw angle in radians

        Returns:
            [x, y, z, w] quaternion
        """
        # Use PyBullet to convert yaw to quaternion (roll=0, pitch=0, yaw=yaw)
        return list(p.getQuaternionFromEuler([0.0, 0.0, yaw]))


@dataclass
class Path:
    """
    Represents a path as a sequence of waypoints (Poses).

    Attributes:
        waypoints: List of Pose objects representing the path

    Example:
        # Create from Pose list (recommended)
        path = Path([
            Pose.from_xyz(0, 0, 0),
            Pose.from_xyz(1, 0, 0),
            Pose.from_xyz(1, 1, 0)
        ])

        # Create from positions
        path = Path.from_positions([[0, 0, 0], [1, 0, 0], [1, 1, 0]])

        # Create predefined shapes
        circle = Path.create_circle(center=[0, 0], radius=1.5)
        square = Path.create_square(center=[0, 0], side_length=2.0)
    """

    waypoints: List[Pose]

    def __len__(self) -> int:
        """Return number of waypoints."""
        return len(self.waypoints)

    def __getitem__(self, index: int) -> Pose:
        """Get waypoint at index."""
        return self.waypoints[index]

    def __iter__(self):
        """Iterate over waypoints."""
        return iter(self.waypoints)

    @classmethod
    def from_positions(cls, positions: List[List[float]], orientation: Optional[List[float]] = None) -> "Path":
        """
        Create Path from list of positions with optional shared orientation.

        Args:
            positions: List of [x, y, z] positions
            orientation: Optional quaternion [x, y, z, w] applied to all waypoints
                        (default: [0, 0, 0, 1] - no rotation)

        Returns:
            Path instance

        Example:
            path = Path.from_positions([
                [0, 0, 0],
                [1, 0, 0],
                [1, 1, 0]
            ])
        """
        if orientation is None:
            orientation = [0.0, 0.0, 0.0, 1.0]

        waypoints = [Pose(position=list(pos), orientation=list(orientation)) for pos in positions]
        return cls(waypoints=waypoints)

    @classmethod
    def create_square(cls, center: List[float], side_length: float, height: float = 0.0) -> "Path":
        """
        Create a square path.

        Args:
            center: [x, y] center position
            side_length: Length of square sides
            height: Z-coordinate for all waypoints

        Returns:
            Path forming a square

        Example:
            path = Path.create_square(center=[0, 0], side_length=2.0, height=0.1)
        """
        half = side_length / 2.0
        cx, cy = center[0], center[1]

        positions = [
            [cx - half, cy - half, height],  # Bottom-left
            [cx + half, cy - half, height],  # Bottom-right
            [cx + half, cy + half, height],  # Top-right
            [cx - half, cy + half, height],  # Top-left
            [cx - half, cy - half, height],  # Back to start
        ]

        return cls.from_positions(positions)

    @classmethod
    def create_circle(cls, center: List[float], radius: float, num_points: int = 16, height: float = 0.0) -> "Path":
        """
        Create a circular path.

        Args:
            center: [x, y] center position
            radius: Circle radius
            num_points: Number of waypoints around the circle
            height: Z-coordinate for all waypoints

        Returns:
            Path forming a circle

        Example:
            path = Path.create_circle(center=[0, 0], radius=1.5, num_points=20)
        """
        cx, cy = center[0], center[1]
        positions = []

        for i in range(num_points + 1):  # +1 to close the circle
            angle = 2.0 * np.pi * i / num_points
            x = cx + radius * np.cos(angle)
            y = cy + radius * np.sin(angle)
            positions.append([x, y, height])

        return cls.from_positions(positions)

    def get_total_distance(self) -> float:
        """
        Calculate total path length.

        Returns:
            Total distance in meters
        """
        if len(self.waypoints) < 2:
            return 0.0

        total = 0.0
        for i in range(len(self.waypoints) - 1):
            p1 = np.array(self.waypoints[i].position)
            p2 = np.array(self.waypoints[i + 1].position)
            total += np.linalg.norm(p2 - p1)

        return total


@dataclass
class SimObjectSpawnParams:
    """
    Base spawn parameters for SimObject class.

    These parameters define the physical properties, appearance, and initial state of a simulation object.

    Attributes:
        mesh_path: Path to mesh file (obj, dae, stl, etc.) for visual representation
        initial_pose: Initial Pose (position and orientation) in world coordinates (optional, used by from_params)
        collision_shape_type: 'box', 'mesh', or None for collision shape (None = no collision)
        collision_half_extents: [hx, hy, hz] for box collision
        mesh_scale: Mesh scaling factors [sx, sy, sz]
        rgba_color: RGBA color [r, g, b, a] (0.0-1.0)
        mass: Object mass in kg (0.0 for static objects)

    Note:
        initial_pose is optional and mainly used by SimObject.from_params().
        Managers calculate positions automatically and may ignore this field.
    """

    mesh_path: Optional[str] = None
    initial_pose: Optional[Pose] = None
    collision_shape_type: str = "box"
    collision_half_extents: List[float] = field(default_factory=lambda: [0.5, 0.5, 0.5])
    mesh_scale: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    rgba_color: List[float] = field(default_factory=lambda: [0.8, 0.8, 0.8, 1.0])
    mass: float = 0.0


class SimObject:
    """
    Base class for simulation objects (robots, pallets, obstacles).
    Supports attachment/detachment and kinematic teleportation.

    Args:
        body_id: PyBullet body ID
        sim_core: Reference to simulation core (optional)
        mesh_path: Optional path to visual mesh file (obj, dae, stl, etc.)
                   If provided, visual shape will be updated with this mesh.
    """

    def __init__(self, body_id: int, sim_core=None, mesh_path: Optional[str] = None):
        self.body_id = body_id
        self.sim_core = sim_core
        self.attached_objects: List["SimObject"] = []
        self.callbacks: List[dict] = []
        self.mesh_path = mesh_path

        # Attachment state (initialized when attach() is called)
        self._attach_offset: Optional[Pose] = None
        self._constraint_id: Optional[int] = None

        # Auto-register to sim_core if provided
        if sim_core is not None:
            sim_core.sim_objects.append(self)

    @classmethod
    def from_mesh(
        cls,
        mesh_path: Optional[str] = None,
        pose: Pose = None,
        collision_shape_type: str = "box",
        collision_half_extents: List[float] = None,
        mesh_scale: List[float] = None,
        rgba_color: List[float] = None,
        mass: float = 0.0,
        sim_core=None,
    ) -> "SimObject":
        """
        Create a SimObject with optional visual mesh and collision.

        Args:
            mesh_path: Path to mesh file (obj, dae, stl, etc.) or None for no visual
            pose: Pose object (position and orientation). Defaults to origin with no rotation
            collision_shape_type: 'box', 'mesh', or None for collision shape (None = no collision)
            collision_half_extents: [hx, hy, hz] for box collision (required if collision_shape_type='box')
            mesh_scale: [sx, sy, sz] mesh scaling
            rgba_color: [r, g, b, a] color (only used when mesh_path is provided)
            mass: Object mass (0.0 for static)
            sim_core: Reference to simulation core

        Returns:
            SimObject instance

        Example:
            # With mesh and collision
            pallet = SimObject.from_mesh(
                mesh_path="11pallet.obj",
                pose=Pose.from_xyz(-2, 0, 0.1),
                collision_half_extents=[0.5, 0.4, 0.1],
                mesh_scale=[0.5, 0.5, 0.5],
                rgba_color=[0.8, 0.6, 0.4, 1.0],
                sim_core=sim_core
            )

            # Visual only (no collision)
            decoration = SimObject.from_mesh(
                mesh_path="decorative.obj",
                pose=Pose.from_xyz(0, 0, 0),
                collision_shape_type=None,
                rgba_color=[1.0, 0.0, 0.0, 1.0],
                sim_core=sim_core
            )

            # Invisible collision object (no visual)
            invisible_wall = SimObject.from_mesh(
                mesh_path=None,  # No visual
                pose=Pose.from_xyz(1, 0, 0),
                collision_shape_type='box',
                collision_half_extents=[1.0, 0.1, 2.0],
                sim_core=sim_core
            )

            # Pure marker (no visual, no collision)
            marker = SimObject.from_mesh(
                mesh_path=None,
                pose=Pose.from_euler(2, 0, 0, yaw=1.57),
                collision_shape_type=None,
                sim_core=sim_core
            )
        """
        if pose is None:
            pose = Pose.from_xyz(0.0, 0.0, 0.0)
        if mesh_scale is None:
            mesh_scale = [1.0, 1.0, 1.0]
        if rgba_color is None:
            rgba_color = [0.8, 0.8, 0.8, 1.0]
        if collision_half_extents is None:
            collision_half_extents = [0.5, 0.5, 0.5]

        # Create collision shape
        if collision_shape_type is None:
            collision_id = -1  # No collision shape
        elif collision_shape_type == "box":
            collision_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=collision_half_extents)
        elif collision_shape_type == "mesh":
            if mesh_path is None:
                raise ValueError("mesh_path is required when collision_shape_type='mesh'")
            collision_id = p.createCollisionShape(p.GEOM_MESH, fileName=mesh_path, meshScale=mesh_scale)
        else:
            # Unknown type: default to box collision
            collision_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=collision_half_extents)

        # Create visual shape
        if mesh_path is not None:
            # Use mesh for visual
            visual_id = p.createVisualShape(p.GEOM_MESH, fileName=mesh_path, meshScale=mesh_scale, rgbaColor=rgba_color)
        else:
            # No visual shape when mesh_path is None
            visual_id = -1

        # Get position and orientation from Pose
        position, orientation = pose.as_position_orientation()

        # Create multibody
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_id,
            baseVisualShapeIndex=visual_id,
            basePosition=position,
            baseOrientation=orientation,
        )

        return cls(body_id=body_id, sim_core=sim_core, mesh_path=mesh_path)

    @classmethod
    def from_params(cls, spawn_params: SimObjectSpawnParams, sim_core=None) -> "SimObject":
        """
        Create a SimObject from SimObjectSpawnParams.

        Args:
            spawn_params: SimObjectSpawnParams instance
            sim_core: Reference to simulation core

        Returns:
            SimObject instance

        Example:
            params = SimObjectSpawnParams(
                mesh_path="pallet.obj",
                initial_pose=Pose.from_xyz(0, 0, 0.1),
                collision_half_extents=[0.5, 0.4, 0.1],
                rgba_color=[0.8, 0.6, 0.4, 1.0]
            )
            obj = SimObject.from_params(params, sim_core)
        """
        return cls.from_mesh(
            mesh_path=spawn_params.mesh_path,
            pose=spawn_params.initial_pose,
            collision_shape_type=spawn_params.collision_shape_type,
            collision_half_extents=spawn_params.collision_half_extents,
            mesh_scale=spawn_params.mesh_scale,
            rgba_color=spawn_params.rgba_color,
            mass=spawn_params.mass,
            sim_core=sim_core,
        )

    def get_pose(self) -> Pose:
        """Return current position and orientation as Pose object."""
        position, orientation = p.getBasePositionAndOrientation(self.body_id)
        return Pose.from_pybullet(position, orientation)

    def set_pose(self, pose: Pose):
        """
        Set position and orientation from a Pose object.
        Preserves current velocity (useful for kinematic control with physics enabled).

        Args:
            pose: Pose object containing position and orientation
        """
        # Get current velocities to preserve them
        linear_vel, angular_vel = p.getBaseVelocity(self.body_id)

        position, orientation = pose.as_position_orientation()
        p.resetBasePositionAndOrientation(self.body_id, position, orientation)
        p.resetBaseVelocity(self.body_id, linear_vel, angular_vel)

        # Recursively apply the same coordinates and velocity to attached_objects
        for obj in getattr(self, "attached_objects", []):
            # Follow using relative position and orientation from attachment
            if hasattr(obj, "_attach_offset"):
                offset_pos, offset_orn = obj._attach_offset
                new_pos, new_orn = p.multiplyTransforms(position, orientation, offset_pos, offset_orn)
                new_pose = Pose.from_pybullet(new_pos, new_orn)
                obj.set_pose(new_pose)

    def attach_object(
        self,
        obj: "SimObject",
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0],
        jointAxis=[0, 0, 0],
        jointType=p.JOINT_FIXED,
        parentLinkIndex=-1,
        childLinkIndex=-1,
    ):
        """Attach another SimObject to this object."""
        if obj not in self.attached_objects:
            self.attached_objects.append(obj)
            # Save initial relative position and orientation
            parent_pos, parent_orn = self.get_pose().as_tuple()
            child_pos, child_orn = obj.get_pose().as_tuple()

            rel_pos, rel_orn = p.invertTransform(parent_pos, parent_orn)
            offset_pos, offset_orn = p.multiplyTransforms(rel_pos, rel_orn, child_pos, child_orn)
            obj._attach_offset = Pose.from_pybullet(offset_pos, offset_orn)
            mass = p.getDynamicsInfo(obj.body_id, -1)[0]
            if mass != 0:
                obj._constraint_id = p.createConstraint(
                    parentBodyUniqueId=self.body_id,
                    parentLinkIndex=parentLinkIndex,
                    childBodyUniqueId=obj.body_id,
                    childLinkIndex=childLinkIndex,
                    jointType=jointType,
                    jointAxis=jointAxis,
                    parentFramePosition=parentFramePosition,
                    childFramePosition=childFramePosition,
                )

    def detach_object(self, obj: "SimObject"):
        """Detach an object from this object."""
        if obj in self.attached_objects:
            self.attached_objects.remove(obj)
            mass = p.getDynamicsInfo(obj.body_id, -1)[0]
            if mass != 0 and hasattr(obj, "_constraint_id"):
                p.removeConstraint(obj._constraint_id)
                obj._constraint_id = None

    def register_callback(self, callback: Callable, frequency: float = 0.25):
        """Register a callback function to be executed periodically."""
        self.callbacks.append({"func": callback, "frequency": frequency, "last_executed": 0.0})

    def execute_callbacks(self, current_time: float):
        """Execute registered callbacks based on their frequency."""
        for cbinfo in self.callbacks:
            if current_time - cbinfo["last_executed"] >= cbinfo["frequency"]:
                cbinfo["func"](self)
                cbinfo["last_executed"] = current_time


# =============================================================================
# Legacy classes (deprecated - use Agent class instead)
# =============================================================================


class MeshObject(SimObject):
    """
    Deprecated: Use Agent.from_mesh() instead.
    Legacy class for mesh-based objects.
    """

    @classmethod
    def from_mesh(
        cls, mesh_path, position, orientation, base_mass=0.0, mesh_scale=[1, 1, 1], rgbaColor=[1, 1, 1, 1], sim_core=None
    ):
        vis_id = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=mesh_path, meshScale=mesh_scale, rgbaColor=rgbaColor)
        col_id = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=mesh_path, meshScale=mesh_scale)
        body_id = p.createMultiBody(
            baseMass=base_mass,
            baseCollisionShapeIndex=col_id,
            baseVisualShapeIndex=vis_id,
            basePosition=position,
            baseOrientation=orientation,
        )
        return cls(body_id=body_id, mesh_path=mesh_path, visual_id=vis_id, collision_id=col_id, sim_core=sim_core)

    def __init__(self, body_id, mesh_path, visual_id=None, collision_id=None, sim_core=None):
        super().__init__(body_id, sim_core=sim_core)
        self.mesh_path = mesh_path
        self.visual_id = visual_id
        self.collision_id = collision_id

    def set_color(self, rgbaColor, linkIndex=-1):
        p.changeVisualShape(self.body_id, linkIndex, rgbaColor=rgbaColor)


class URDFObject(SimObject):
    """
    Deprecated: Use Agent.from_urdf() instead.
    Legacy class for URDF-based objects with navigation capabilities.
    """

    @classmethod
    def from_urdf(cls, urdf_path, position, orientation, useFixedBase=False, set_mass_zero=False, meta_data={}, sim_core=None):
        body_id = p.loadURDF(urdf_path, position, orientation, useFixedBase=useFixedBase)
        return cls(body_id, urdf_path, set_mass_zero=set_mass_zero, meta_data=meta_data, sim_core=sim_core)

    def __init__(
        self,
        body_id,
        urdf_path,
        set_mass_zero=False,
        meta_data={},
        max_accel=1.0,
        max_speed=1.0,
        goal_threshold=0.01,
        sim_core=None,
    ):
        import numpy as np

        super().__init__(body_id, sim_core=sim_core)
        self.urdf_path = urdf_path
        self.joint_info = [p.getJointInfo(body_id, j) for j in range(p.getNumJoints(body_id))]
        if set_mass_zero:
            self.set_all_masses_to_zero()
        self.meta_data = meta_data
        self.max_accel = max_accel
        self.max_speed = max_speed
        self.goal_threshold = goal_threshold
        self.target_actions = []
        self._current_nav_index = 0
        self._nav_velocity = np.array([0.0, 0.0, 0.0])
        self._nav_last_update = None
        self._motion_completed = True

    def set_navigation_params(self, max_accel, max_speed, goal_threshold=None):
        self.max_accel = max_accel
        self.max_speed = max_speed
        if goal_threshold is not None:
            self.goal_threshold = goal_threshold

    def set_action(self, pose_callback_list):
        """
        pose_callback_list: List[Tuple[Pose, Optional[Callable], Optional[float]]]
        Example: [(Pose, callback, wait_time), (Pose, None, 0.0), ...]
        callback is executed on arrival, wait_time is seconds to wait after reaching Pose
        """
        self.target_actions = pose_callback_list
        self._current_nav_index = 0
        self._nav_last_update = None
        self._wait_until = None
        self._motion_completed = False

    def update_action(self, dt=0.01):
        """
        Move sequentially to each Pose in target_actions, considering max acceleration and max speed.
        dt: simulation timestep
        sim_time: simulation time (seconds). Required for wait functionality.
        Used in pallet_carrier_demo.py. Can be replaced with Robot class.
        """
        import numpy as np

        # Wait feature: if _wait_until is set, stop until sim_time reaches that time
        sim_time = self.sim_core.sim_time if self.sim_core is not None else None
        if self._wait_until is not None:
            if sim_time is not None and sim_time < self._wait_until:
                self._nav_velocity = np.array([0.0, 0.0, 0.0])
                return
            else:
                self._wait_until = None

        if not self.target_actions or self._current_nav_index >= len(self.target_actions):
            self._motion_completed = True
            return  # Path finished

        # Get current position
        pose = self.get_pose()
        pos, orn = pose.as_tuple()
        current_pos = np.array(pos)
        # nav_path extension: (Pose, callback, wait_time)
        target_tuple = self.target_actions[self._current_nav_index]
        if isinstance(target_tuple, Pose):
            target_pose = target_tuple
            callback = None
            wait_time = 0.0
        elif len(target_tuple) == 2:
            target_pose, callback = target_tuple
            wait_time = 0.0
        else:
            target_pose, callback, wait_time = target_tuple
        target_pos, target_orn = target_pose.as_position_orientation()
        target_pos = np.array(target_pos)
        # Calculate velocity and acceleration
        direction = target_pos - current_pos
        distance = np.linalg.norm(direction)
        if distance < self.goal_threshold:
            # Goal reached: teleport if within threshold
            self.set_pose(target_pose)
            self._current_nav_index += 1
            self._nav_velocity = np.array([0.0, 0.0, 0.0])
            # Execute callback on arrival
            if callback:
                callback()
            # If wait_time is specified, wait based on sim_time
            if wait_time and sim_time is not None:
                self._wait_until = sim_time + wait_time
            return
        direction = direction / (distance + 1e-6)
        # Target speed: do not exceed the distance to the goal
        desired_speed = min(self.max_speed, distance / dt, distance)
        desired_velocity = direction * desired_speed
        # Acceleration limit
        accel = (desired_velocity - self._nav_velocity) / dt
        accel_norm = np.linalg.norm(accel)
        if accel_norm > self.max_accel:
            accel = accel / (accel_norm + 1e-6) * self.max_accel
        self._nav_velocity += accel * dt
        speed = np.linalg.norm(self._nav_velocity)
        if speed > self.max_speed:
            self._nav_velocity = self._nav_velocity / (speed + 1e-6) * self.max_speed
        # Update position
        new_pos = current_pos + self._nav_velocity * dt
        # Orientation matches target Pose
        _, target_orn = target_pose.as_position_orientation()
        new_pose = Pose.from_pybullet(new_pos.tolist(), target_orn)
        self.set_pose(new_pose)

    def set_all_masses_to_zero(self):
        p.changeDynamics(self.body_id, -1, mass=0)
        for j in range(p.getNumJoints(self.body_id)):
            p.changeDynamics(self.body_id, j, mass=0)

    def kinematic_teleport_joint(self, joint_index, target_pos):
        p.resetJointState(self.body_id, joint_index, target_pos)

    def set_joint_target(self, joint_index, target_pos):
        mass = p.getDynamicsInfo(self.body_id, joint_index)[0]
        if mass == 0:
            self.kinematic_teleport_joint(joint_index, target_pos)
        else:
            p.setJointMotorControl2(self.body_id, joint_index, p.POSITION_CONTROL, targetPosition=target_pos)
