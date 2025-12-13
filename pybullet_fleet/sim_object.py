"""
sim_object.py
Base class for simulation objects with attachment support.
"""

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple, Iterable

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R


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

    @staticmethod
    def _calculate_orientation_for_plane(direction: np.ndarray, plane_normal: np.ndarray) -> List[float]:
        """
        Calculate quaternion orientation where X+ points in direction and Z+ points in plane_normal.

        Args:
            direction: 3D direction vector (will be normalized) - robot's X+ axis
            plane_normal: 3D plane normal vector (will be normalized) - robot's Z+ axis

        Returns:
            Quaternion [x, y, z, w] that aligns X+ with direction and Z+ with plane_normal
        """
        # Normalize inputs
        x_axis = direction / np.linalg.norm(direction)
        z_axis = plane_normal / np.linalg.norm(plane_normal)

        # Build Y-axis (right direction) using right-hand rule: Y = Z × X
        y_axis = np.cross(z_axis, x_axis)
        y_norm = np.linalg.norm(y_axis)

        if y_norm < 1e-6:
            # Edge case: direction parallel to plane normal
            # Choose arbitrary perpendicular Y axis
            if abs(x_axis[0]) < 0.9:
                y_axis = np.cross(z_axis, np.array([1, 0, 0]))
            else:
                y_axis = np.cross(z_axis, np.array([0, 1, 0]))
            y_axis = y_axis / np.linalg.norm(y_axis)
        else:
            y_axis = y_axis / y_norm

        # Recompute Z to ensure orthogonality: Z = X × Y
        z_axis = np.cross(x_axis, y_axis)

        # Build rotation matrix [X, Y, Z] as columns
        rot_matrix = np.column_stack([x_axis, y_axis, z_axis])

        # Convert to quaternion using scipy
        rot = R.from_matrix(rot_matrix)
        quat = rot.as_quat()  # [x, y, z, w]
        return quat.tolist()

    @classmethod
    def create_rectangle(
        cls,
        center: List[float],
        width: float,
        height: float,
        rpy: List[float] = [0.0, 0.0, 0.0],
    ) -> "Path":
        """Create a rectangular path in 3D.

        Args:
            center: [x, y, z] center position in world coordinates
            width: Width of rectangle (X direction in local frame)
            height: Height of rectangle (Y direction in local frame)
            rpy: [roll, pitch, yaw] in radians. Defaults to [0, 0, 0].
                 When non-zero, the Z+ axis of the robot will be
                 perpendicular to the rectangle plane.

        Returns:
            Path instance with waypoints at rectangle corners

        Example:
            rect = Path.create_rectangle(center=[0, 0, 1], width=3.0, height=2.0)
            tilted_rect = Path.create_rectangle(center=[0, 0, 1], width=3.0, height=2.0,
                                                rpy=[0.1, 0.2, 0])
        """
        if len(center) != 3:
            raise ValueError("Path.create_rectangle: center must be [x, y, z]")

        if len(rpy) != 3:
            raise ValueError("Path.create_rectangle: rpy must be [roll, pitch, yaw]")

        roll, pitch, yaw = rpy

        half_width = width / 2.0
        half_height = height / 2.0

        # Local coordinates of rectangle corners in the XY plane (z=0) around origin
        local_points = np.array(
            [
                [-half_width, -half_height, 0.0],  # Bottom-left
                [half_width, -half_height, 0.0],  # Bottom-right
                [half_width, half_height, 0.0],  # Top-right
                [-half_width, half_height, 0.0],  # Top-left
                [-half_width, -half_height, 0.0],  # Back to start
            ]
        )

        # Build rotation using scipy (more reliable than manual calculation)
        rot = R.from_euler("XYZ", [roll, pitch, yaw])

        # Calculate plane normal from rotation (Z+ axis)
        plane_normal = rot.as_matrix()[:, 2]

        # Center position in world coordinates
        center_world = np.array(center)

        # Transform points to world coordinates
        positions = []
        for pt in local_points:
            world_pt = center_world + rot.as_matrix().dot(pt)
            positions.append(world_pt.tolist())

        # Calculate orientation for each waypoint based on travel direction
        waypoints = []
        for i in range(len(positions)):
            pos = positions[i]

            # Calculate direction to next waypoint
            next_i = (i + 1) % len(positions)
            direction = np.array(positions[next_i]) - np.array(pos)
            direction_norm = np.linalg.norm(direction)

            if direction_norm > 1e-6:
                # Calculate orientation with X+ pointing in travel direction, Z+ perpendicular to plane
                quat = cls._calculate_orientation_for_plane(direction, plane_normal)
            else:
                # For duplicate points (e.g., closing point of path), use previous waypoint's orientation
                if i > 0:
                    quat = waypoints[-1].orientation
                else:
                    quat = [0.0, 0.0, 0.0, 1.0]

            waypoints.append(Pose(position=pos, orientation=quat))

        return cls(waypoints=waypoints)

    @classmethod
    def create_square(
        cls,
        center: List[float],
        side_length: float,
        rpy: List[float] = [0.0, 0.0, 0.0],
    ) -> "Path":
        """Create a square path in 3D.

        Args:
            center: [x, y, z] center position in world coordinates
            side_length: Length of square sides
            rpy: [roll, pitch, yaw] in radians. Defaults to [0, 0, 0].
                 When non-zero, the Z+ axis of the robot will be
                 perpendicular to the square plane.

        Returns:
            Path instance with waypoints at square corners

        Example:
            square = Path.create_square(center=[0, 0, 0], side_length=2.0)
            tilted_square = Path.create_square(center=[0, 0, 1], side_length=2.0,
                                              rpy=[0.1, 0.2, 0])

        Note:
            This is a wrapper for create_rectangle with width == height.
        """
        return cls.create_rectangle(center=center, width=side_length, height=side_length, rpy=rpy)

    @classmethod
    def create_ellipse(
        cls,
        center: List[float],
        radius_x: float,
        radius_y: float,
        num_points: int = 16,
        rpy: List[float] = [0.0, 0.0, 0.0],
        start_angle: float = 0.0,
        end_angle: float = 2.0 * np.pi,
    ) -> "Path":
        """Create an elliptical path in 3D.

        Args:
            center: [x, y, z] center position in world coordinates
            radius_x: Radius along X axis in local frame
            radius_y: Radius along Y axis in local frame
            num_points: Number of waypoints along the arc
            rpy: [roll, pitch, yaw] in radians. Defaults to [0, 0, 0].
            start_angle: Start angle [rad] in the local XY-plane
            end_angle: End angle [rad] in the local XY-plane

        Returns:
            Path instance with waypoints along ellipse

        Example:
            ellipse = Path.create_ellipse(center=[0, 0, 0], radius_x=2.0, radius_y=1.0)
            tilted_ellipse = Path.create_ellipse(center=[0, 0, 1], radius_x=2.0, radius_y=1.0,
                                                 rpy=[0, 0.3, 0])
        """
        if len(center) != 3:
            raise ValueError("Path.create_ellipse: center must be [x, y, z]")

        if len(rpy) != 3:
            raise ValueError("Path.create_ellipse: rpy must be [roll, pitch, yaw]")

        roll, pitch, yaw = rpy

        # Local ellipse/arc points in XY-plane with z=0
        local_points = []
        # Ensure we always include the end point by sampling num_points
        # segments between start_angle and end_angle.
        for i in range(num_points + 1):
            t = i / float(num_points)
            angle = start_angle + (end_angle - start_angle) * t
            x = radius_x * np.cos(angle)
            y = radius_y * np.sin(angle)
            local_points.append([x, y, 0.0])

        local_points = np.array(local_points)

        # Build rotation using scipy
        rot = R.from_euler("XYZ", [roll, pitch, yaw])

        # Calculate plane normal from rotation (Z+ axis)
        plane_normal = rot.as_matrix()[:, 2]

        # Center position in world coordinates
        center_world = np.array(center)

        positions = []
        for pt in local_points:
            world_pt = center_world + rot.as_matrix().dot(pt)
            positions.append(world_pt.tolist())

        # Calculate orientation for each waypoint based on travel direction
        waypoints = []
        for i in range(len(positions)):
            pos = positions[i]

            # Calculate direction to next waypoint
            next_i = (i + 1) % len(positions)
            direction = np.array(positions[next_i]) - np.array(pos)
            direction_norm = np.linalg.norm(direction)

            if direction_norm > 1e-6:
                # Calculate orientation with X+ pointing in travel direction, Z+ perpendicular to plane
                quat = cls._calculate_orientation_for_plane(direction, plane_normal)
            else:
                # Fallback for duplicate points (e.g., last point == first point)
                if i > 0:
                    # Use previous waypoint's orientation
                    quat = waypoints[i - 1].orientation
                else:
                    quat = [0.0, 0.0, 0.0, 1.0]

            waypoints.append(Pose(position=pos, orientation=quat))

        return cls(waypoints=waypoints)

    @classmethod
    def create_circle(
        cls,
        center: List[float],
        radius: float,
        num_points: int = 16,
        rpy: List[float] = [0.0, 0.0, 0.0],
        start_angle: float = 0.0,
        end_angle: float = 2.0 * np.pi,
    ) -> "Path":
        """Create a circular path in 3D.

        Args:
            center: [x, y, z] center position in world coordinates
            radius: Circle radius
            num_points: Number of waypoints along the arc
            rpy: [roll, pitch, yaw] in radians. Defaults to [0, 0, 0].
            start_angle: Start angle [rad] in the local XY-plane
            end_angle: End angle [rad] in the local XY-plane

        Returns:
            Path instance with waypoints along circle

        Example:
            circle = Path.create_circle(center=[0, 0, 0], radius=1.5)
            arc = Path.create_circle(center=[0, 0, 0], radius=1.5,
                                    start_angle=0, end_angle=np.pi)

        Note:
            This is a wrapper for create_ellipse with radius_x == radius_y.
        """
        return cls.create_ellipse(
            center=center,
            radius_x=radius,
            radius_y=radius,
            num_points=num_points,
            rpy=rpy,
            start_angle=start_angle,
            end_angle=end_angle,
        )

    # --------------------------------------------------------------
    # Path composition utilities
    # --------------------------------------------------------------

    def append(self, other: "Path") -> None:
        """Append waypoints from another path in-place.

        Note:
            This mutates the current Path. For a non-mutating version,
            use `combined = path1 + path2`.
        """

        self.waypoints.extend(other.waypoints)

    def __add__(self, other: "Path") -> "Path":
        """Concatenate two paths and return a new Path.

        The resulting path simply has waypoints = self.waypoints + other.waypoints.
        """

        return Path(waypoints=list(self.waypoints) + list(other.waypoints))

    @classmethod
    def from_paths(cls, paths: Iterable["Path"]) -> "Path":
        """Create a single Path by concatenating multiple paths.

        Example:
            circle_arc = Path.create_circle(center=[0, 0, 0.5], radius=2.0,
                                           num_points=16,
                                           start_angle=0.0,
                                           end_angle=np.pi/2)
            line = Path.from_positions([[1, 1, 0.5], [3, 1, 0.5]])
            full = Path.from_paths([circle_arc, line])
        """

        waypoints: List[Pose] = []
        for path in paths:
            waypoints.extend(path.waypoints)
        return cls(waypoints=waypoints)

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

    def visualize(
        self,
        show_lines: bool = True,
        line_color: List[float] = None,
        line_width: float = 2.0,
        show_waypoints: bool = False,
        show_axes: bool = False,
        axis_length: float = 0.3,
        show_points: bool = False,
        point_size: float = 0.05,
        lifetime: float = 0.0,
    ) -> List[int]:
        """
        Visualize path with lines and optional waypoint markers.

        Args:
            show_lines: If True, draw lines connecting waypoints
            line_color: RGB color for path lines [r, g, b] (0-1 range), None for default
            line_width: Width of path lines
            show_waypoints: If True, show waypoint coordinate axes and points
            show_axes: If True, draw X, Y, Z axes at each waypoint
            axis_length: Length of coordinate axes in meters
            show_points: If True, draw spheres at waypoint positions
            point_size: Radius of waypoint spheres in meters
            lifetime: Debug line lifetime (0 = permanent until reset)

        Returns:
            List of debug item IDs (for later removal if needed)

        Example:
            # Just show path lines
            path = Path.create_circle(center=[0, 0, 0.5], radius=2.0)
            path.visualize(line_color=[1, 0, 0])

            # Show path with waypoint orientations
            path.visualize(show_waypoints=True, show_axes=True)
        """
        debug_ids = []

        # Draw path lines
        if show_lines and len(self.waypoints) >= 2:
            # Default color if not specified
            if line_color is None:
                line_color = [0.5, 0.5, 0.5]

            for i in range(len(self.waypoints) - 1):
                p1 = self.waypoints[i].position
                p2 = self.waypoints[i + 1].position
                debug_id = p.addUserDebugLine(p1, p2, lineColorRGB=line_color, lineWidth=line_width, lifeTime=lifetime)
                debug_ids.append(debug_id)

        # Draw waypoint markers if requested
        if show_waypoints:
            waypoint_ids = self.visualize_waypoints(
                show_axes=show_axes,
                axis_length=axis_length,
                show_points=show_points,
                point_size=point_size,
                lifetime=lifetime,
            )
            debug_ids.extend(waypoint_ids)

        return debug_ids

    def visualize_waypoints(
        self,
        show_axes: bool = True,
        axis_length: float = 0.3,
        show_points: bool = True,
        point_size: float = 0.05,
        lifetime: float = 0.0,
    ) -> List[int]:
        """
        Visualize waypoints with coordinate axes for debugging.

        Args:
            show_axes: If True, draw X, Y, Z axes at each waypoint
            axis_length: Length of coordinate axes in meters
            show_points: If True, draw spheres at waypoint positions
            point_size: Radius of waypoint spheres in meters
            lifetime: Debug line lifetime (0 = permanent until reset)

        Returns:
            List of debug item IDs (for later removal if needed)

        Example:
            path = Path.create_square(center=[0, 0, 1], side_length=2.0, rpy=[0.1, 0.2, 0])
            debug_ids = path.visualize_waypoints(axis_length=0.5)
        """
        debug_ids = []

        for i, waypoint in enumerate(self.waypoints):
            pos = waypoint.position

            # Draw point at waypoint
            if show_points:
                # Use small sphere for waypoint marker
                for offset in [
                    [-point_size, 0, 0],
                    [point_size, 0, 0],
                    [0, -point_size, 0],
                    [0, point_size, 0],
                    [0, 0, -point_size],
                    [0, 0, point_size],
                ]:
                    p1 = [pos[0] + offset[0], pos[1] + offset[1], pos[2] + offset[2]]
                    debug_id = p.addUserDebugLine(pos, p1, lineColorRGB=[1, 1, 1], lineWidth=3.0, lifeTime=lifetime)
                    debug_ids.append(debug_id)

            # Draw coordinate axes
            if show_axes:
                # Get rotation matrix from quaternion
                from scipy.spatial.transform import Rotation as R

                rot = R.from_quat(waypoint.orientation)
                rot_matrix = rot.as_matrix()

                # Extract axis vectors
                x_axis = rot_matrix[:, 0]  # X+ (forward/red)
                y_axis = rot_matrix[:, 1]  # Y+ (right/green)
                z_axis = rot_matrix[:, 2]  # Z+ (up/blue)

                # Draw X axis (red)
                x_end = [pos[0] + x_axis[0] * axis_length, pos[1] + x_axis[1] * axis_length, pos[2] + x_axis[2] * axis_length]
                debug_id = p.addUserDebugLine(
                    pos,
                    x_end,
                    lineColorRGB=[1, 0, 0],  # Red
                    lineWidth=5.0,  # Thicker line for better visibility
                    lifeTime=lifetime,
                )
                debug_ids.append(debug_id)

                # Draw Y axis (green)
                y_end = [pos[0] + y_axis[0] * axis_length, pos[1] + y_axis[1] * axis_length, pos[2] + y_axis[2] * axis_length]
                debug_id = p.addUserDebugLine(
                    pos,
                    y_end,
                    lineColorRGB=[0, 1, 0],  # Green
                    lineWidth=5.0,  # Thicker line for better visibility
                    lifeTime=lifetime,
                )
                debug_ids.append(debug_id)

                # Draw Z axis (blue)
                z_end = [pos[0] + z_axis[0] * axis_length, pos[1] + z_axis[1] * axis_length, pos[2] + z_axis[2] * axis_length]
                debug_id = p.addUserDebugLine(
                    pos,
                    z_end,
                    lineColorRGB=[0, 0, 1],  # Blue
                    lineWidth=5.0,  # Thicker line for better visibility
                    lifeTime=lifetime,
                )
                debug_ids.append(debug_id)

                # Add waypoint number label
                debug_id = p.addUserDebugText(
                    f"WP{i}",
                    [pos[0], pos[1], pos[2] + axis_length * 1.2],
                    textColorRGB=[1, 1, 1],
                    textSize=1.0,
                    lifeTime=lifetime,
                )
                debug_ids.append(debug_id)

        return debug_ids


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
        max_linear_accel=1.0,
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
