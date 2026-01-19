"""
types.py
Common types and enumerations used across the package.

This module contains shared type definitions to avoid circular imports
and provide a single source of truth for common types.
"""

from enum import Enum


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


class MovementDirection(str, Enum):
    """
    Movement direction for robot motion.

    Note: This is only used in differential drive mode (MotionMode.DIFFERENTIAL).
          In omnidirectional mode, the robot always faces the movement direction.

    Attributes:
        FORWARD: Robot X+ axis points towards target, moves forward along path
        BACKWARD: Robot X- axis points towards target (X+ points away), moves along path
                 (robot appears to move "backward" relative to its X+ orientation)
    """

    FORWARD = "forward"
    BACKWARD = "backward"


class ActionStatus(Enum):
    """
    Action execution status.

    Attributes:
        NOT_STARTED: Action has not begun execution
        IN_PROGRESS: Action is currently executing
        COMPLETED: Action completed successfully
        FAILED: Action failed to complete
        CANCELLED: Action was cancelled before completion
    """

    NOT_STARTED = "not_started"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class SpatialHashCellSizeMode(Enum):
    """
    Collision detection spatial hash cell size calculation mode.
    
    Determines how the spatial hash grid cell size is calculated for
    efficient broad-phase collision detection.
    
    Attributes:
        CONSTANT: Fixed cell size provided by user (fastest, requires manual tuning)
        AUTO_ADAPTIVE: Recalculate on object add/remove (adaptive, some overhead)
        AUTO_INITIAL: Calculate once at simulation start (default, balanced performance)
    """
    CONSTANT = "constant"
    AUTO_ADAPTIVE = "auto_adaptive"
    AUTO_INITIAL = "auto_initial"


class CollisionDetectionMethod(Enum):
    """
    Collision detection method for narrow-phase collision checking.
    
    Determines which PyBullet API to use for detecting actual contacts
    between objects after broad-phase filtering (AABB + spatial hashing).
    
    Design Philosophy (from design document):
    - Physics OFF (kinematics): Use CLOSEST_POINTS for safety margin detection
    - Physics ON (physics): Use CONTACT_POINTS for actual contact logging
    
    Attributes:
        CLOSEST_POINTS: Use getClosestPoints() - distance-based, kinematics-safe (RECOMMENDED DEFAULT)
                       Best for: kinematics motion, safety clearance, collision avoidance
                       Works with: Physics ON/OFF, stable with resetBasePositionAndOrientation
        
        CONTACT_POINTS: Use getContactPoints() - physics contact manifold (PHYSICS MODE)
                       Best for: physics simulation, actual contact logging, debug/reproduction
                       Requires: stepSimulation() to be called regularly
                       Note: Unstable for kinematic-kinematic pairs
        
        HYBRID: Use getContactPoints for physics, getClosestPoints for kinematic (ADVANCED)
               Best for: Mixed physics/kinematics with different detection needs
               Slower due to branching overhead
    
    Recommended defaults:
        - physics_enabled=False → CLOSEST_POINTS (default)
        - physics_enabled=True  → CONTACT_POINTS or HYBRID
    """
    CLOSEST_POINTS = "closest_points"  # Default for kinematics
    CONTACT_POINTS = "contact_points"  # For physics simulation
    HYBRID = "hybrid"                  # Advanced mixed mode

