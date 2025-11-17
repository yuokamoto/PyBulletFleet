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
