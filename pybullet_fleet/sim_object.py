"""
sim_object.py
Base class for simulation objects with attachment support.
"""

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Dict, Tuple

import numpy as np
import pybullet as p

from .geometry import Pose


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
        mass: Object mass in kg (1.0 default, use 0.0 for kinematic control)

    Note:
        initial_pose is optional and mainly used by SimObject.from_params().
        Managers calculate positions automatically and may ignore this field.

        For URDF robots:
        - mass=1.0 (default): Uses URDF file's mass values
        - mass=0.0: Override all links to mass=0 (kinematic control, no physics)
    """

    mesh_path: Optional[str] = None
    initial_pose: Optional[Pose] = None
    collision_shape_type: str = "box"
    collision_half_extents: List[float] = field(default_factory=lambda: [0.5, 0.5, 0.5])
    mesh_scale: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    rgba_color: List[float] = field(default_factory=lambda: [0.8, 0.8, 0.8, 1.0])
    mass: float = 1.0
    pickable: bool = True  # Can this object be picked by robots


class SimObject:
    """
    Base class for simulation objects (robots, pallets, obstacles).
    Supports attachment/detachment and kinematic teleportation.

    Args:
        body_id: PyBullet body ID
        sim_core: Reference to simulation core (optional)
        mesh_path: Optional path to visual mesh file (obj, dae, stl, etc.)
                   If provided, visual shape will be updated with this mesh.
        pickable: Whether this object can be picked by robots (default: True)
    """

    # Class-level shared shapes cache (for optimization)
    _shared_shapes: Dict[str, Tuple[int, int]] = {}

    def __init__(self, body_id: int, sim_core=None, mesh_path: Optional[str] = None, pickable: bool = True):
        self.body_id = body_id
        self.sim_core = sim_core
        self.attached_objects: List["SimObject"] = []
        self.callbacks: List[dict] = []
        self.mesh_path = mesh_path
        self.pickable = pickable

        # Attachment state (initialized with default zero offset)
        self._attach_offset: Pose = Pose(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
        self._constraint_id: Optional[int] = None
        self._attached_to: Optional["SimObject"] = None  # Which object is this attached to
        self._attached_link_index: int = -1  # Which link is this attached to

        # Auto-register to sim_core if provided
        if sim_core is not None:
            sim_core.sim_objects.append(self)

    @classmethod
    def create_shared_shapes(
        cls,
        mesh_path: str,
        mesh_scale: List[float] = None,
        collision_shape_type: str = "box",
        collision_half_extents: List[float] = None,
        rgba_color: List[float] = None,
    ) -> Tuple[int, int]:
        """
        Create shared visual and collision shapes for fast object spawning.

        This method caches shapes by their parameters, so multiple objects with
        identical parameters will reuse the same PyBullet shape IDs.

        Args:
            mesh_path: Path to mesh file
            mesh_scale: Mesh scaling [x, y, z]
            collision_shape_type: "box", "mesh", or None
            collision_half_extents: Collision box half extents [x, y, z]
            rgba_color: RGBA color [r, g, b, a]

        Returns:
            (visual_id, collision_id) tuple
        """
        # Set defaults
        if mesh_scale is None:
            mesh_scale = [1.0, 1.0, 1.0]
        if collision_half_extents is None:
            collision_half_extents = [0.5, 0.5, 0.5]
        if rgba_color is None:
            rgba_color = [0.8, 0.8, 0.8, 1.0]

        # Create unique cache key from all parameters
        shape_key = (
            f"{mesh_path}_{collision_shape_type}_{tuple(collision_half_extents)}_{tuple(mesh_scale)}_{tuple(rgba_color)}"
        )

        # Check if shapes already exist in cache
        if shape_key in cls._shared_shapes:
            return cls._shared_shapes[shape_key]

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
            visual_id = p.createVisualShape(p.GEOM_MESH, fileName=mesh_path, meshScale=mesh_scale, rgbaColor=rgba_color)
        else:
            visual_id = -1

        # Cache the shapes for future reuse
        cls._shared_shapes[shape_key] = (visual_id, collision_id)
        return visual_id, collision_id

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
        pickable: bool = True,
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
            pickable: Whether this object can be picked by robots (default: True)
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

        # Use create_shared_shapes() to get or create cached shapes
        visual_id, collision_id = cls.create_shared_shapes(
            mesh_path=mesh_path,
            mesh_scale=mesh_scale,
            collision_shape_type=collision_shape_type,
            collision_half_extents=collision_half_extents,
            rgba_color=rgba_color,
        )

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

        return cls(body_id=body_id, sim_core=sim_core, mesh_path=mesh_path, pickable=pickable)

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
            pickable=spawn_params.pickable,
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
        for obj in self.attached_objects:
            # Follow using relative position and orientation from attachment
            # _attach_offset is always initialized with default (0,0,0) if not set
            new_pos, new_orn = p.multiplyTransforms(
                position, orientation, obj._attach_offset.position, obj._attach_offset.orientation
            )
            new_pose = Pose.from_pybullet(new_pos, new_orn)
            obj.set_pose(new_pose)

    def attach_object(
        self,
        obj: "SimObject",
        parent_link_index: int = -1,
        relative_pose: Optional[Pose] = None,
        joint_type: int = p.JOINT_FIXED,
    ) -> bool:
        """
        Attach another SimObject to this object.

        Args:
            obj: Object to attach
            parent_link_index: Link index to attach to (-1 for base link)
            relative_pose: Position and orientation offset in parent link's frame as Pose object
                          (default: None, which uses Pose(position=[0, 0, 0], orientation=[0, 0, 0, 1]))
            joint_type: PyBullet joint type (default: JOINT_FIXED)

        Returns:
            True if attachment successful, False otherwise

        Example:
            # Attach to base link at 0.5m in front
            agent.attach_object(pallet, relative_pose=Pose.from_xyz(0.5, 0, 0))

            # Attach with position and orientation
            agent.attach_object(pallet, relative_pose=Pose.from_euler(0.6, 0, -0.2,
                                                                       roll=np.pi/2, pitch=0, yaw=0))

            # Attach to specific link with default zero offset
            agent.attach_object(box, parent_link_index=2)
        """
        # Default to zero offset if not specified
        if relative_pose is None:
            relative_pose = Pose(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])

        # Check if object is pickable
        if not obj.pickable:
            print(f"[SimObject] Cannot attach: object {obj.body_id} is not pickable")
            return False

        if obj in self.attached_objects:
            print(f"[SimObject] Object {obj.body_id} already attached")
            return False

        # Add to attached list
        self.attached_objects.append(obj)

        # Get parent link position and orientation
        if parent_link_index == -1:
            parent_pos, parent_orn = p.getBasePositionAndOrientation(self.body_id)
        else:
            link_state = p.getLinkState(self.body_id, parent_link_index)
            parent_pos, parent_orn = link_state[0], link_state[1]

        # Save attachment state with user-specified relative pose
        obj._attach_offset = relative_pose
        obj._attached_to = self
        obj._attached_link_index = parent_link_index

        # Create constraint if object has mass (physics-based attachment)
        mass = p.getDynamicsInfo(obj.body_id, -1)[0]
        if mass > 0:
            obj._constraint_id = p.createConstraint(
                parentBodyUniqueId=self.body_id,
                parentLinkIndex=parent_link_index,
                childBodyUniqueId=obj.body_id,
                childLinkIndex=-1,
                jointType=joint_type,
                jointAxis=[0, 0, 0],
                parentFramePosition=relative_pose.position,
                childFramePosition=[0, 0, 0],
                parentFrameOrientation=relative_pose.orientation,
                childFrameOrientation=[0, 0, 0, 1],
            )

        print(f"[SimObject] Attached object {obj.body_id} to link {parent_link_index}")
        return True

    def detach_object(self, obj: "SimObject") -> bool:
        """
        Detach an object from this object.

        Args:
            obj: Object to detach

        Returns:
            True if detachment successful, False otherwise

        Example:
            agent.detach_object(pallet)
        """
        if obj not in self.attached_objects:
            print(f"[SimObject] Object {obj.body_id} is not attached")
            return False

        # Remove from attached list
        self.attached_objects.remove(obj)

        # Remove constraint if it exists
        if obj._constraint_id is not None:
            p.removeConstraint(obj._constraint_id)
            obj._constraint_id = None

        # Reset attachment state to default values
        obj._attach_offset = Pose(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
        obj._attached_to = None
        obj._attached_link_index = -1

        print(f"[SimObject] Detached object {obj.body_id}")
        return True

    def get_attached_objects(self) -> List["SimObject"]:
        """
        Return list of objects attached to this object.

        Returns:
            List of attached SimObject instances
        """
        return self.attached_objects.copy()

    def is_attached(self) -> bool:
        """
        Check if this object is attached to another object.

        Returns:
            True if attached to another object, False otherwise
        """
        return self._attached_to is not None

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
        max_linear_accel=1.0,
        max_speed=1.0,
        goal_threshold=0.01,
        sim_core=None,
    ):
        super().__init__(body_id, sim_core=sim_core)
        self.urdf_path = urdf_path
        self.joint_info = [p.getJointInfo(body_id, j) for j in range(p.getNumJoints(body_id))]
        if set_mass_zero:
            self.set_all_masses_to_zero()
        self.meta_data = meta_data
        self.max_linear_accel = max_linear_accel
        self.max_speed = max_speed
        self.goal_threshold = goal_threshold
        self.target_actions = []
        self._current_nav_index = 0
        self._nav_velocity = np.array([0.0, 0.0, 0.0])
        self._nav_last_update = None
        self._motion_completed = True

    def set_navigation_params(self, max_linear_accel, max_speed, goal_threshold=None):
        self.max_linear_accel = max_linear_accel
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
        if accel_norm > self.max_linear_accel:
            accel = accel / (accel_norm + 1e-6) * self.max_linear_accel
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
