"""
sim_object.py
Base class for simulation objects with attachment support.
"""
import pybullet as p
from typing import Optional, List, Callable, Tuple
from dataclasses import dataclass


@dataclass
class Pose:
    """
    Position and orientation representation for any object in the simulation.
    Compatible with ROS2 geometry_msgs/Pose and PyBullet's (position, orientation) tuples.
    
    Supports both quaternion and Euler angle representations.
    
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
    
    @classmethod
    def from_xyz(cls, x: float, y: float, z: float):
        """Create Pose from x, y, z coordinates."""
        return cls(position=[x, y, z])
    
    @classmethod
    def from_euler(cls, x: float, y: float, z: float, 
                   roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0):
        """
        Create Pose from position and Euler angles.
        
        Args:
            x, y, z: Position in world coordinates
            roll, pitch, yaw: Euler angles in radians
        """
        quat = p.getQuaternionFromEuler([roll, pitch, yaw])
        return cls(position=[x, y, z], orientation=list(quat))
    
    @classmethod
    def from_pybullet(cls, position: Tuple[float, float, float], 
                     orientation: Tuple[float, float, float, float]):
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


class SimObject:
    """
    Base class for simulation objects (robots, pallets, obstacles).
    Supports attachment/detachment and kinematic teleportation.
    """
    def __init__(self, body_id: int, sim_core=None):
        self.body_id = body_id
        self.sim_core = sim_core
        self.attached_objects: List['SimObject'] = []
        self.callbacks: List[dict] = []
    
    def get_pose(self) -> Pose:
        """Return current position and orientation as Pose object."""
        position, orientation = p.getBasePositionAndOrientation(self.body_id)
        return Pose.from_pybullet(position, orientation)
    
    def kinematic_teleport_base(self, position, orientation, linear_vel=None, angular_vel=None):
        p.resetBasePositionAndOrientation(self.body_id, position, orientation)
        if linear_vel is not None and angular_vel is not None:
            p.resetBaseVelocity(self.body_id, linear_vel, angular_vel)
        # Recursively apply the same coordinates and velocity to attached_objects
        for obj in getattr(self, 'attached_objects', []):
            # Follow using relative position and orientation from attachment
            if hasattr(obj, '_attach_offset'):
                offset_pos, offset_orn = obj._attach_offset
                new_pos, new_orn = p.multiplyTransforms(position, orientation, offset_pos, offset_orn)
                obj.kinematic_teleport_base(new_pos, new_orn, linear_vel, angular_vel)
    
    def attach_object(self, obj: 'SimObject', parentFramePosition=[0,0,0], 
                     childFramePosition=[0,0,0], jointAxis=[0,0,0], 
                     jointType=p.JOINT_FIXED, parentLinkIndex=-1, childLinkIndex=-1):
        """Attach another SimObject to this object."""
        if obj not in self.attached_objects:
            self.attached_objects.append(obj)
            # Save initial relative position and orientation
            parent_pos, parent_orn = self.get_pose().as_tuple()
            child_pos, child_orn = obj.get_pose().as_tuple()
            
            rel_pos, rel_orn = p.invertTransform(parent_pos, parent_orn)
            offset_pos, offset_orn = p.multiplyTransforms(rel_pos, rel_orn, child_pos, child_orn)
            obj._attach_offset = (offset_pos, offset_orn)
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
                    childFramePosition=childFramePosition
                )
    
    def detach_object(self, obj: 'SimObject'):
        """Detach an object from this object."""
        if obj in self.attached_objects:
            self.attached_objects.remove(obj)
            mass = p.getDynamicsInfo(obj.body_id, -1)[0]
            if mass != 0 and hasattr(obj, '_constraint_id'):
                p.removeConstraint(obj._constraint_id)
                obj._constraint_id = None
    
    def register_callback(self, callback: Callable, frequency: float = 0.25):
        """Register a callback function to be executed periodically."""
        self.callbacks.append({
            'func': callback,
            'frequency': frequency,
            'last_executed': 0.0
        })
    
    def execute_callbacks(self, current_time: float):
        """Execute registered callbacks based on their frequency."""
        for cbinfo in self.callbacks:
            if current_time - cbinfo['last_executed'] >= cbinfo['frequency']:
                cbinfo['func'](self)
                cbinfo['last_executed'] = current_time


# =============================================================================
# Legacy classes (deprecated - use Agent class instead)
# =============================================================================

class MeshObject(SimObject):
    """
    Deprecated: Use Agent.from_mesh() instead.
    Legacy class for mesh-based objects.
    """
    @classmethod
    def from_mesh(cls, mesh_path, position, orientation, base_mass=0.0, mesh_scale=[1,1,1], rgbaColor=[1,1,1,1], sim_core=None):
        vis_id = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=mesh_path,
            meshScale=mesh_scale,
            rgbaColor=rgbaColor
        )
        col_id = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=mesh_path,
            meshScale=mesh_scale
        )
        body_id = p.createMultiBody(
            baseMass=base_mass,
            baseCollisionShapeIndex=col_id,
            baseVisualShapeIndex=vis_id,
            basePosition=position,
            baseOrientation=orientation
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
    
    def __init__(self, body_id, urdf_path, set_mass_zero=False, meta_data={}, max_accel=1.0, max_speed=1.0, goal_threshold=0.01, sim_core=None):
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
            self.kinematic_teleport_base(target_pos.tolist(), target_orn)
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
        self.kinematic_teleport_base(new_pos.tolist(), target_orn)
    
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
