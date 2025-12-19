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
class ShapeParams:
    """
    Parameters for visual or collision shape.

    Similar to URDF's <visual> and <collision> tags, this allows full control
    over shape type, geometry, appearance, and frame offset.

    Attributes:
        shape_type: Shape type - "box", "sphere", "cylinder", "mesh", or None (no shape)
        mesh_path: Path to mesh file (required if shape_type="mesh")
        mesh_scale: Mesh scaling [sx, sy, sz] (for mesh shapes)
        half_extents: Box half extents [hx, hy, hz] (for box shapes)
        radius: Sphere/cylinder radius (for sphere/cylinder shapes)
        height: Cylinder height (for cylinder shapes)
        rgba_color: RGBA color [r, g, b, a] (for visual shapes, ignored for collision)
        frame_pose: Shape offset as Pose object (relative to body frame)

    Example:
        # Box visual
        visual = ShapeParams(
            shape_type="box",
            half_extents=[0.5, 0.3, 0.2],
            rgba_color=[0.8, 0.2, 0.2, 1.0]
        )

        # Mesh collision
        collision = ShapeParams(
            shape_type="mesh",
            mesh_path="collision.obj",
            mesh_scale=[1.0, 1.0, 1.0]
        )

        # Sphere with offset
        sphere = ShapeParams(
            shape_type="sphere",
            radius=0.5,
            rgba_color=[0.0, 1.0, 0.0, 1.0],
            frame_pose=Pose.from_xyz(0, 0, 0.5)
        )
    """

    shape_type: Optional[str] = None  # "box", "sphere", "cylinder", "mesh", or None
    mesh_path: Optional[str] = None
    mesh_scale: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    half_extents: List[float] = field(default_factory=lambda: [0.5, 0.5, 0.5])
    radius: float = 0.5
    height: float = 1.0
    rgba_color: List[float] = field(default_factory=lambda: [0.8, 0.8, 0.8, 1.0])
    frame_pose: Optional[Pose] = None


@dataclass
class SimObjectSpawnParams:
    """
    Parameters for spawning a SimObject.

    Attributes:
        visual_shape: Visual shape parameters (ShapeParams or None)
        collision_shape: Collision shape parameters (ShapeParams or None)
        initial_pose: Initial Pose (position and orientation) in world coordinates
        mass: Object mass in kg (0.0 for static objects, >0 for dynamic)
        pickable: Whether this object can be picked by robots (default: True)

    Note:
        initial_pose is optional and mainly used by SimObject.from_params().
        Managers calculate positions automatically and may ignore this field.

    Example:
        # Box visual + sphere collision
        params = SimObjectSpawnParams(
            visual_shape=ShapeParams(
                shape_type="box",
                half_extents=[0.5, 0.3, 0.2],
                rgba_color=[1.0, 0.0, 0.0, 1.0]
            ),
            collision_shape=ShapeParams(
                shape_type="sphere",
                radius=0.3
            ),
            initial_pose=Pose.from_xyz(0, 0, 1),
            mass=1.0
        )
    """

    visual_shape: Optional[ShapeParams] = None
    collision_shape: Optional[ShapeParams] = None
    initial_pose: Optional[Pose] = None
    mass: float = 0.0
    pickable: bool = True
    visual_mesh_path: Optional[str] = None
    visual_frame_pose: Optional[Pose] = None
    collision_frame_pose: Optional[Pose] = None
    is_structure: bool = False  # If True, automatically registers as structure body


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
        visual_shape: Optional[ShapeParams] = None,
        collision_shape: Optional[ShapeParams] = None,
    ) -> Tuple[int, int]:
        """
        Create shared visual and collision shapes for fast object spawning.

        This method caches mesh shapes by their parameters, so multiple objects with
        identical mesh parameters will reuse the same PyBullet shape IDs.

        Note: Primitive shapes (box, sphere, cylinder) are NOT cached, as PyBullet
        handles them very efficiently and caching adds unnecessary overhead.

        Args:
            visual_shape: ShapeParams for visual geometry
            collision_shape: ShapeParams for collision geometry

        Returns:
            (visual_id, collision_id) tuple

        Example:
            visual_id, collision_id = create_shared_shapes(
                visual_shape=ShapeParams(
                    shape_type="mesh",
                    mesh_path="visual.obj",
                    mesh_scale=[1.0, 1.0, 1.0],
                    rgba_color=[0.8, 0.2, 0.2, 1.0]
                ),
                collision_shape=ShapeParams(
                    shape_type="box",
                    half_extents=[0.5, 0.3, 0.2]
                )
            )
        """
        # Only cache mesh shapes (primitives are cheap to create)
        visual_needs_cache = visual_shape and visual_shape.shape_type == "mesh"
        collision_needs_cache = collision_shape and collision_shape.shape_type == "mesh"

        # Create cache key only for mesh shapes
        if visual_needs_cache or collision_needs_cache:
            visual_key = cls._shape_params_to_key(visual_shape) if visual_needs_cache else "none"
            collision_key = cls._shape_params_to_key(collision_shape) if collision_needs_cache else "none"
            shape_key = f"v:{visual_key}_c:{collision_key}"

            # Check cache
            if shape_key in cls._shared_shapes:
                return cls._shared_shapes[shape_key]

        # Create visual shape
        visual_id = cls._create_visual_shape(visual_shape) if visual_shape and visual_shape.shape_type else -1

        # Create collision shape
        collision_id = cls._create_collision_shape(collision_shape) if collision_shape and collision_shape.shape_type else -1

        # Cache only if at least one is a mesh shape
        if visual_needs_cache or collision_needs_cache:
            cls._shared_shapes[shape_key] = (visual_id, collision_id)

        return visual_id, collision_id

    @staticmethod
    def _shape_params_to_key(shape: ShapeParams) -> str:
        """Convert ShapeParams to a unique cache key string."""
        frame_pos = shape.frame_pose.position if shape.frame_pose else [0, 0, 0]
        frame_orn = shape.frame_pose.orientation if shape.frame_pose else [0, 0, 0, 1]

        return (
            f"{shape.shape_type}_{shape.mesh_path}_{tuple(shape.mesh_scale)}_"
            f"{tuple(shape.half_extents)}_{shape.radius}_{shape.height}_"
            f"{tuple(shape.rgba_color)}_{tuple(frame_pos)}_{tuple(frame_orn)}"
        )

    @staticmethod
    def _create_visual_shape(shape: ShapeParams) -> int:
        """Create a PyBullet visual shape from ShapeParams."""
        # Get frame offset
        frame_position = [0.0, 0.0, 0.0]
        frame_orientation = [0.0, 0.0, 0.0, 1.0]
        if shape.frame_pose:
            frame_position, frame_orientation = shape.frame_pose.as_position_orientation()

        if shape.shape_type == "mesh":
            if not shape.mesh_path:
                raise ValueError("mesh_path is required for shape_type='mesh'")
            return p.createVisualShape(
                p.GEOM_MESH,
                fileName=shape.mesh_path,
                meshScale=shape.mesh_scale,
                rgbaColor=shape.rgba_color,
                visualFramePosition=frame_position,
                visualFrameOrientation=frame_orientation,
            )
        elif shape.shape_type == "box":
            return p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=shape.half_extents,
                rgbaColor=shape.rgba_color,
                visualFramePosition=frame_position,
                visualFrameOrientation=frame_orientation,
            )
        elif shape.shape_type == "sphere":
            return p.createVisualShape(
                p.GEOM_SPHERE,
                radius=shape.radius,
                rgbaColor=shape.rgba_color,
                visualFramePosition=frame_position,
                visualFrameOrientation=frame_orientation,
            )
        elif shape.shape_type == "cylinder":
            return p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=shape.radius,
                length=shape.height,
                rgbaColor=shape.rgba_color,
                visualFramePosition=frame_position,
                visualFrameOrientation=frame_orientation,
            )
        else:
            raise ValueError(f"Unknown visual shape_type: {shape.shape_type}")

    @staticmethod
    def _create_collision_shape(shape: ShapeParams) -> int:
        """Create a PyBullet collision shape from ShapeParams."""
        # Get frame offset
        frame_position = [0.0, 0.0, 0.0]
        frame_orientation = [0.0, 0.0, 0.0, 1.0]
        if shape.frame_pose:
            frame_position, frame_orientation = shape.frame_pose.as_position_orientation()

        if shape.shape_type == "mesh":
            if not shape.mesh_path:
                raise ValueError("mesh_path is required for shape_type='mesh'")
            return p.createCollisionShape(
                p.GEOM_MESH,
                fileName=shape.mesh_path,
                meshScale=shape.mesh_scale,
                collisionFramePosition=frame_position,
                collisionFrameOrientation=frame_orientation,
            )
        elif shape.shape_type == "box":
            return p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=shape.half_extents,
                collisionFramePosition=frame_position,
                collisionFrameOrientation=frame_orientation,
            )
        elif shape.shape_type == "sphere":
            return p.createCollisionShape(
                p.GEOM_SPHERE,
                radius=shape.radius,
                collisionFramePosition=frame_position,
                collisionFrameOrientation=frame_orientation,
            )
        elif shape.shape_type == "cylinder":
            return p.createCollisionShape(
                p.GEOM_CYLINDER,
                radius=shape.radius,
                height=shape.height,
                collisionFramePosition=frame_position,
                collisionFrameOrientation=frame_orientation,
            )
        else:
            raise ValueError(f"Unknown collision shape_type: {shape.shape_type}")

    @classmethod
    def _create_body_from_shapes(
        cls,
        visual_id: int,
        collision_id: int,
        pose: Pose,
        mass: float = 0.0,
    ) -> int:
        """
        Create a PyBullet multibody from pre-created visual and collision shapes.

        This is a helper method to reduce code duplication between SimObject and Agent.

        Args:
            visual_id: Visual shape ID from create_shared_shapes()
            collision_id: Collision shape ID from create_shared_shapes()
            pose: Initial Pose (position and orientation)
            mass: Body mass (kg)

        Returns:
            PyBullet body ID
        """
        position, orientation = pose.as_position_orientation()
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_id,
            baseVisualShapeIndex=visual_id,
            basePosition=position,
            baseOrientation=orientation,
        )
        return body_id

    @classmethod
    def from_mesh(
        cls,
        visual_shape: Optional[ShapeParams] = None,
        collision_shape: Optional[ShapeParams] = None,
        pose: Pose = None,
        mass: float = 0.0,
        pickable: bool = True,
        sim_core=None,
    ) -> "SimObject":
        """
        Create a SimObject with optional visual and collision shapes.

        Args:
            visual_shape: ShapeParams for visual geometry
            collision_shape: ShapeParams for collision geometry
            pose: Initial Pose (position and orientation)
            mass: Object mass (0.0 for static)
            pickable: Whether object can be picked by robots
            sim_core: Reference to simulation core

        Returns:
            SimObject instance

        Example:
            # Box visual + sphere collision
            obj = SimObject.from_mesh(
                visual_shape=ShapeParams(
                    shape_type="box",
                    half_extents=[0.5, 0.3, 0.2],
                    rgba_color=[0.8, 0.2, 0.2, 1.0]
                ),
                collision_shape=ShapeParams(
                    shape_type="sphere",
                    radius=0.4
                ),
                pose=Pose.from_xyz(0, 0, 1),
                mass=1.0,
                sim_core=sim_core
            )
        """
        if pose is None:
            pose = Pose.from_xyz(0.0, 0.0, 0.0)

        # Use create_shared_shapes() to get or create cached shapes
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

        # Store mesh_path if available for backward compatibility
        stored_mesh_path = visual_shape.mesh_path if (visual_shape and visual_shape.shape_type == "mesh") else None

        return cls(body_id=body_id, sim_core=sim_core, mesh_path=stored_mesh_path, pickable=pickable)

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
                visual_shape=ShapeParams(
                    shape_type="mesh",
                    mesh_path="pallet.obj",
                    mesh_scale=[0.5, 0.5, 0.5],
                    rgba_color=[0.8, 0.6, 0.4, 1.0]
                ),
                collision_shape=ShapeParams(
                    shape_type="box",
                    half_extents=[0.5, 0.4, 0.1]
                ),
                initial_pose=Pose.from_xyz(0, 0, 0.1),
                mass=1.0
            )
            obj = SimObject.from_params(params, sim_core)
        """
        obj = cls.from_mesh(
            visual_shape=spawn_params.visual_shape,
            collision_shape=spawn_params.collision_shape,
            pose=spawn_params.initial_pose,
            mass=spawn_params.mass,
            pickable=spawn_params.pickable,
            sim_core=sim_core,
        )

        # Auto-register as structure if is_structure flag is set
        if spawn_params.is_structure and sim_core is not None:
            sim_core.register_structure_body(obj.body_id)

        return obj

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
