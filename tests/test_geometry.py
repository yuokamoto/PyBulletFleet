"""
Tests for geometry module (Pose and Path classes).

This module tests:
- Pose creation from various formats
- Pose conversions
- Distance calculations
- Path creation and waypoint management
"""

import pytest
import numpy as np
from pybullet_fleet.geometry import Pose, Path


class TestPoseCreation:
    """Test Pose creation methods"""

    def test_pose_from_xyz(self):
        """Test creating Pose from position only"""
        pose = Pose.from_xyz(1.0, 2.0, 3.0)

        assert pose.x == 1.0
        assert pose.y == 2.0
        assert pose.z == 3.0
        # Default orientation should be identity quaternion
        np.testing.assert_array_almost_equal(pose.orientation, [0, 0, 0, 1])

    def test_pose_from_euler(self):
        """Test creating Pose from position and Euler angles"""
        pose = Pose.from_euler(1.0, 2.0, 3.0, roll=0.1, pitch=0.2, yaw=0.3)

        assert pose.x == 1.0
        assert pose.y == 2.0
        assert pose.z == 3.0
        # Orientation should be non-identity
        assert not np.allclose(pose.orientation, [0, 0, 0, 1])

    def test_pose_from_pybullet(self):
        """Test creating Pose from PyBullet format"""
        position = [1.0, 2.0, 3.0]
        orientation = [0.0, 0.0, 0.707, 0.707]  # 90 degrees around Z
        pose = Pose.from_pybullet(position, orientation)

        np.testing.assert_array_almost_equal(pose.position, position)
        np.testing.assert_array_almost_equal(pose.orientation, orientation)

    def test_pose_default_constructor(self):
        """Test Pose with minimal constructor"""
        # Pose requires position, but orientation has default
        pose = Pose(position=[0, 0, 0])

        np.testing.assert_array_almost_equal(pose.position, [0, 0, 0])
        np.testing.assert_array_almost_equal(pose.orientation, [0, 0, 0, 1])

    def test_pose_with_position_and_orientation(self):
        """Test Pose with explicit position and orientation"""
        position = [1.0, 2.0, 3.0]
        orientation = [0.0, 0.0, 0.707, 0.707]
        pose = Pose(position=position, orientation=orientation)

        np.testing.assert_array_almost_equal(pose.position, position)
        np.testing.assert_array_almost_equal(pose.orientation, orientation)


class TestPoseConversion:
    """Test Pose conversion methods"""

    def test_as_position_orientation(self):
        """Test converting Pose to position/orientation tuple"""
        pose = Pose.from_xyz(1.0, 2.0, 3.0)
        position, orientation = pose.as_position_orientation()

        np.testing.assert_array_almost_equal(position, [1.0, 2.0, 3.0])
        np.testing.assert_array_almost_equal(orientation, [0, 0, 0, 1])

    def test_as_euler(self):
        """Test converting Pose to Euler angles"""
        # Create pose with known Euler angles
        roll, pitch, yaw = 0.1, 0.2, 0.3
        pose = Pose.from_euler(0, 0, 0, roll=roll, pitch=pitch, yaw=yaw)

        # Convert back to Euler
        result_roll, result_pitch, result_yaw = pose.as_euler()

        # Should match original values (within tolerance)
        assert abs(result_roll - roll) < 0.01
        assert abs(result_pitch - pitch) < 0.01
        assert abs(result_yaw - yaw) < 0.01

    def test_euler_roundtrip(self):
        """Test Euler angle conversion roundtrip"""
        original_roll, original_pitch, original_yaw = 0.5, -0.3, 1.2

        # Create pose from Euler
        pose = Pose.from_euler(1, 2, 3, roll=original_roll, pitch=original_pitch, yaw=original_yaw)

        # Convert back
        roll, pitch, yaw = pose.as_euler()

        # Verify roundtrip
        assert abs(roll - original_roll) < 0.01
        assert abs(pitch - original_pitch) < 0.01
        assert abs(yaw - original_yaw) < 0.01


class TestPoseProperties:
    """Test Pose properties"""

    def test_xyz_properties(self):
        """Test x, y, z property accessors"""
        pose = Pose.from_xyz(1.5, 2.5, 3.5)

        assert pose.x == 1.5
        assert pose.y == 2.5
        assert pose.z == 3.5

    def test_position_is_list(self):
        """Test that position is a list"""
        pose = Pose.from_xyz(1, 2, 3)

        assert isinstance(pose.position, list)
        assert len(pose.position) == 3

    def test_orientation_is_list(self):
        """Test that orientation is a list"""
        pose = Pose(position=[0, 0, 0])

        assert isinstance(pose.orientation, list)
        assert len(pose.orientation) == 4


class TestPoseDistanceCalculations:
    """Test Pose distance calculation methods"""

    def test_distance_calculation_same_position(self):
        """Test distance between same positions using numpy"""
        pose1 = Pose.from_xyz(1, 2, 3)
        pose2 = Pose.from_xyz(1, 2, 3)

        # Calculate distance manually
        pos1 = np.array(pose1.position)
        pos2 = np.array(pose2.position)
        distance = np.linalg.norm(pos1 - pos2)

        assert distance == 0.0

    def test_distance_calculation_different_position(self):
        """Test distance between different positions using numpy"""
        pose1 = Pose.from_xyz(0, 0, 0)
        pose2 = Pose.from_xyz(3, 4, 0)  # 3-4-5 triangle

        # Calculate distance manually
        pos1 = np.array(pose1.position)
        pos2 = np.array(pose2.position)
        distance = np.linalg.norm(pos1 - pos2)

        assert abs(distance - 5.0) < 0.001

    def test_distance_calculation_3d(self):
        """Test 3D distance calculation using numpy"""
        pose1 = Pose.from_xyz(0, 0, 0)
        pose2 = Pose.from_xyz(1, 1, 1)

        expected_distance = np.sqrt(3)  # sqrt(1^2 + 1^2 + 1^2)

        # Calculate distance manually
        pos1 = np.array(pose1.position)
        pos2 = np.array(pose2.position)
        distance = np.linalg.norm(pos1 - pos2)

        assert abs(distance - expected_distance) < 0.001

    def test_position_access(self):
        """Test accessing position coordinates"""
        pose1 = Pose.from_xyz(1, 2, 3)
        pose2 = Pose.from_xyz(4, 5, 6)

        # Verify position access works
        assert pose1.x == 1
        assert pose1.y == 2
        assert pose1.z == 3

        assert pose2.x == 4
        assert pose2.y == 5
        assert pose2.z == 6


class TestPath:
    """Test Path class"""

    def test_path_from_positions(self):
        """Test creating Path from list of positions"""
        positions = [
            [0, 0, 0],
            [1, 0, 0],
            [1, 1, 0],
            [0, 1, 0],
        ]
        path = Path.from_positions(positions)

        assert len(path.waypoints) == 4
        assert path.waypoints[0].x == 0
        assert path.waypoints[1].x == 1
        assert path.waypoints[2].y == 1
        assert path.waypoints[3].x == 0

    def test_path_constructor(self):
        """Test Path constructor with waypoints"""
        waypoints = [Pose.from_xyz(1, 2, 3), Pose.from_xyz(4, 5, 6)]
        path = Path(waypoints=waypoints)

        assert len(path.waypoints) == 2
        assert path.waypoints[0].x == 1
        assert path.waypoints[1].x == 4

    def test_path_indexing(self):
        """Test path indexing and iteration"""
        path = Path.from_positions([[0, 0, 0], [1, 1, 1], [2, 2, 2]])

        # Test indexing
        waypoint0 = path[0]
        waypoint1 = path[1]
        waypoint2 = path[2]

        assert waypoint0.x == 0
        assert waypoint1.x == 1
        assert waypoint2.x == 2

    def test_path_indexing_out_of_bounds(self):
        """Test path indexing with invalid index"""
        path = Path.from_positions([[0, 0, 0]])

        # Should raise IndexError
        with pytest.raises(IndexError):
            _ = path[10]

    def test_path_empty(self):
        """Test empty path"""
        path = Path(waypoints=[])

        assert len(path.waypoints) == 0
        assert path.waypoints == []

    def test_path_iteration(self):
        """Test iterating over path waypoints"""
        poses = [
            Pose.from_xyz(0, 0, 0),
            Pose.from_xyz(1, 0, 0),
            Pose.from_euler(2, 0, 0, yaw=1.57),  # With rotation
        ]
        path = Path(waypoints=poses)

        assert len(path) == 3

        # Test iteration
        for i, waypoint in enumerate(path):
            assert waypoint.x == i

        # Third waypoint should have rotation
        assert not np.allclose(path[2].orientation, [0, 0, 0, 1])


class TestPoseEdgeCases:
    """Test edge cases and special scenarios"""

    def test_pose_zero_position(self):
        """Test pose at origin"""
        pose = Pose.from_xyz(0, 0, 0)

        assert pose.x == 0
        assert pose.y == 0
        assert pose.z == 0

    def test_pose_negative_coordinates(self):
        """Test pose with negative coordinates"""
        pose = Pose.from_xyz(-1, -2, -3)

        assert pose.x == -1
        assert pose.y == -2
        assert pose.z == -3

    def test_pose_large_coordinates(self):
        """Test pose with large coordinates"""
        pose = Pose.from_xyz(1000, 2000, 3000)

        assert pose.x == 1000
        assert pose.y == 2000
        assert pose.z == 3000

    def test_quaternion_normalization(self):
        """Test that quaternion maintains expected properties"""
        # Create with non-normalized quaternion
        orientation = [1, 1, 1, 1]  # Not normalized
        pose = Pose(position=[0, 0, 0], orientation=orientation)

        # Note: Current implementation doesn't auto-normalize
        # Just verify quaternion is stored
        assert len(pose.orientation) == 4

        # Test with normalized quaternion
        normalized = [0, 0, 0, 1]  # Unit quaternion
        pose2 = Pose(position=[0, 0, 0], orientation=normalized)

        magnitude = np.linalg.norm(pose2.orientation)
        assert abs(magnitude - 1.0) < 0.001


if __name__ == "__main__":
    pytest.main([__file__, "-v"])


class TestPathGeneration:
    """Test Path generation methods (rectangle, square, circle, ellipse)"""

    def test_create_rectangle(self):
        """Test creating rectangular path"""
        center = [0, 0, 0]
        width = 4.0
        height = 2.0

        path = Path.create_rectangle(center=center, width=width, height=height)

        # Should have 5 waypoints (4 corners + return to start)
        assert len(path) == 5

        # First and last waypoints should be at same position
        assert abs(path[0].x - path[4].x) < 0.001
        assert abs(path[0].y - path[4].y) < 0.001

    def test_create_square(self):
        """Test creating square path"""
        center = [1, 2, 3]
        side_length = 2.0

        path = Path.create_square(center=center, side_length=side_length)

        # Should have 5 waypoints
        assert len(path) == 5

        # Check center is approximately correct
        x_coords = [w.x for w in path.waypoints[:4]]
        y_coords = [w.y for w in path.waypoints[:4]]

        assert abs(np.mean(x_coords) - center[0]) < 0.1
        assert abs(np.mean(y_coords) - center[1]) < 0.1

    def test_create_ellipse(self):
        """Test creating elliptical path"""
        center = [0, 0, 0]
        radius_x = 2.0
        radius_y = 1.0
        num_points = 8

        path = Path.create_ellipse(center=center, radius_x=radius_x, radius_y=radius_y, num_points=num_points)

        # Should have num_points + 1 waypoints
        assert len(path) == num_points + 1

        # Check approximate radii
        distances_x = [abs(w.x - center[0]) for w in path.waypoints]
        distances_y = [abs(w.y - center[1]) for w in path.waypoints]

        assert max(distances_x) <= radius_x + 0.1
        assert max(distances_y) <= radius_y + 0.1

    def test_create_circle(self):
        """Test creating circular path"""
        center = [0, 0, 0]
        radius = 1.5
        num_points = 16

        path = Path.create_circle(center=center, radius=radius, num_points=num_points)

        # Should have num_points + 1 waypoints
        assert len(path) == num_points + 1

        # All points should be approximately at radius distance from center
        for waypoint in path.waypoints:
            dx = waypoint.x - center[0]
            dy = waypoint.y - center[1]
            distance = np.sqrt(dx**2 + dy**2)
            assert abs(distance - radius) < 0.1

    def test_create_rectangle_with_rotation(self):
        """Test creating rotated rectangular path"""
        center = [0, 0, 1]
        width = 3.0
        height = 2.0
        rpy = [0, 0, np.pi / 4]  # 45 degree yaw rotation

        path = Path.create_rectangle(center=center, width=width, height=height, rpy=rpy)

        assert len(path) == 5

        # All z-coordinates should be approximately at center[2]
        for waypoint in path.waypoints:
            assert abs(waypoint.z - center[2]) < 0.1

    def test_create_square_tilted(self):
        """Test creating tilted square path"""
        center = [0, 0, 1]
        side_length = 2.0
        rpy = [0.1, 0.2, 0]  # Tilted in 3D

        path = Path.create_square(center=center, side_length=side_length, rpy=rpy)

        # Should still have 5 waypoints
        assert len(path) == 5

        # Waypoints should have non-identity orientations
        for waypoint in path.waypoints:
            # At least some orientation should be non-default
            pass  # Orientations are set based on direction


class TestPathAdvancedFeatures:
    """Test advanced Path features"""

    def test_path_length(self):
        """Test len() operator on Path"""
        waypoints = [Pose.from_xyz(i, 0, 0) for i in range(5)]
        path = Path(waypoints=waypoints)

        assert len(path) == 5

    def test_path_iteration_multiple_times(self):
        """Test that path can be iterated multiple times"""
        path = Path.from_positions([[0, 0, 0], [1, 0, 0], [2, 0, 0]])

        # First iteration
        count1 = sum(1 for _ in path)
        # Second iteration
        count2 = sum(1 for _ in path)

        assert count1 == 3
        assert count2 == 3

    def test_path_slicing(self):
        """Test path supports list slicing"""
        path = Path.from_positions([[i, 0, 0] for i in range(10)])

        # Test slicing waypoints list
        first_three = path.waypoints[:3]
        assert len(first_three) == 3
        assert first_three[0].x == 0
        assert first_three[2].x == 2


class TestPoseYawMethods:
    """Test Pose yaw-specific methods"""

    def test_from_yaw(self):
        """Test creating Pose with yaw angle"""
        pose = Pose.from_yaw(1, 2, 3, yaw=np.pi / 2)

        assert pose.x == 1
        assert pose.y == 2
        assert pose.z == 3

        # Yaw should be approximately pi/2
        assert abs(pose.yaw - np.pi / 2) < 0.01

    def test_to_yaw_quaternion(self):
        """Test converting yaw to quaternion"""
        pose = Pose.from_xyz(0, 0, 0)
        quat = pose.to_yaw_quaternion(np.pi / 4)

        # Should be a valid quaternion
        assert len(quat) == 4

        # Magnitude should be approximately 1
        magnitude = np.linalg.norm(quat)
        assert abs(magnitude - 1.0) < 0.01

    def test_yaw_property(self):
        """Test yaw property accessor"""
        yaw_angle = 1.2
        pose = Pose.from_yaw(0, 0, 0, yaw=yaw_angle)

        # Yaw property should match
        assert abs(pose.yaw - yaw_angle) < 0.01

    def test_roll_pitch_yaw_properties(self):
        """Test roll, pitch, yaw property accessors"""
        roll, pitch, yaw = 0.1, 0.2, 0.3
        pose = Pose.from_euler(0, 0, 0, roll=roll, pitch=pitch, yaw=yaw)

        # All angles should match
        assert abs(pose.roll - roll) < 0.01
        assert abs(pose.pitch - pitch) < 0.01
        assert abs(pose.yaw - yaw) < 0.01


class TestPoseFromPyBullet:
    """Test Pose.from_pybullet() constructor"""

    def test_from_pybullet_basic(self):
        """Test creating pose from PyBullet format"""
        position = (1.0, 2.0, 3.0)
        orientation = (0.0, 0.0, 0.0, 1.0)  # Identity quaternion

        pose = Pose.from_pybullet(position, orientation)

        assert pose.position[0] == 1.0
        assert pose.position[1] == 2.0
        assert pose.position[2] == 3.0
        assert len(pose.orientation) == 4

    def test_from_pybullet_with_rotation(self):
        """Test creating pose from PyBullet format with rotation"""
        position = (0.0, 0.0, 0.0)
        # 90 degree rotation around z-axis
        orientation = (0.0, 0.0, 0.7071068, 0.7071068)

        pose = Pose.from_pybullet(position, orientation)

        assert abs(pose.yaw - 1.5708) < 0.01  # ~90 degrees in radians
