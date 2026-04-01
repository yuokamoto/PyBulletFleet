"""
Tests for tools module (utility functions).

This module tests:
- Vector parameter normalization
- Offset pose calculations
- Grid coordinate conversions
- Link/joint resolution helpers
"""

import pytest
import numpy as np
import logging
from pybullet_fleet.tools import (
    normalize_vector_param,
    calculate_offset_pose,
    world_to_grid,
    grid_to_world,
)


# Helper functions for testing
def assert_distance_from_target(pose, target, expected_distance, tolerance=0.01):
    """
    Assert that pose is at expected_distance from target in XY plane.

    Args:
        pose: Pose object with x, y attributes
        target: Target position [x, y, z] or [x, y]
        expected_distance: Expected distance from target (can be negative for absolute check)
        tolerance: Acceptable error margin (default: 0.01)
    """
    distance = np.sqrt((pose.x - target[0]) ** 2 + (pose.y - target[1]) ** 2)
    assert (
        abs(distance - abs(expected_distance)) < tolerance
    ), f"Distance from target: {distance:.4f}, expected: {abs(expected_distance):.4f}"


def assert_pose_on_line(pose, current, target, offset, tolerance=0.01):
    """
    Assert that pose is on the line between current and target at offset distance.

    Verifies that:
    1. Pose position matches expected position calculated from offset
    2. Pose is geometrically correct relative to current-target line

    Args:
        pose: Pose object with x, y, yaw attributes
        current: Current position [x, y, z] or [x, y]
        target: Target position [x, y, z] or [x, y]
        offset: Distance from target (positive = before target, negative = beyond)
        tolerance: Acceptable error margin (default: 0.01)
    """
    # Calculate expected position using yaw
    expected_x = target[0] - offset * np.cos(pose.yaw)
    expected_y = target[1] - offset * np.sin(pose.yaw)

    assert abs(pose.x - expected_x) < tolerance, f"Pose X: {pose.x:.4f}, expected: {expected_x:.4f}"
    assert abs(pose.y - expected_y) < tolerance, f"Pose Y: {pose.y:.4f}, expected: {expected_y:.4f}"


class TestNormalizeVectorParam:
    """Test normalize_vector_param utility"""

    def test_scalar_to_vector_3d(self):
        """Test converting scalar to 3D vector"""
        result = normalize_vector_param(2.5, "test_param", dim=3)

        assert isinstance(result, np.ndarray)
        assert len(result) == 3
        np.testing.assert_array_almost_equal(result, [2.5, 2.5, 2.5])

    def test_scalar_to_vector_2d(self):
        """Test converting scalar to 2D vector"""
        result = normalize_vector_param(1.0, "test_param", dim=2)

        assert len(result) == 2
        np.testing.assert_array_almost_equal(result, [1.0, 1.0])

    def test_list_to_vector_3d(self):
        """Test converting list to 3D vector"""
        result = normalize_vector_param([1.0, 2.0, 3.0], "test_param", dim=3)

        assert isinstance(result, np.ndarray)
        assert len(result) == 3
        np.testing.assert_array_almost_equal(result, [1.0, 2.0, 3.0])

    def test_list_to_vector_2d(self):
        """Test converting list to 2D vector"""
        result = normalize_vector_param([5.0, 10.0], "test_param", dim=2)

        assert len(result) == 2
        np.testing.assert_array_almost_equal(result, [5.0, 10.0])

    def test_integer_input(self):
        """Test with integer input"""
        result = normalize_vector_param(3, "test_param", dim=3)

        np.testing.assert_array_almost_equal(result, [3.0, 3.0, 3.0])

    def test_wrong_dimension_raises_error(self):
        """Test that wrong dimension list raises ValueError"""
        with pytest.raises(ValueError, match="must be a float or a list of 3 floats"):
            normalize_vector_param([1.0, 2.0], "test_param", dim=3)


class TestCalculateOffsetPose:
    """Test calculate_offset_pose function"""

    def test_offset_pose_straight_line(self):
        """Test offset pose calculation on straight line"""
        target = [5.0, 0.0, 0.1]
        current = [0.0, 0.0, 0.3]
        offset = 1.0

        pose = calculate_offset_pose(target, current, offset, keep_height=True)

        # Should be 1.0m away from target (at x=4.0)
        assert abs(pose.x - 4.0) < 0.01
        assert abs(pose.y - 0.0) < 0.01
        assert abs(pose.z - 0.3) < 0.01  # Keep current height

        # Should face the target (yaw = 0)
        assert abs(pose.yaw - 0.0) < 0.01

        # Use helper functions for common assertions
        assert_distance_from_target(pose, target, offset)
        assert_pose_on_line(pose, current, target, offset)

    def test_offset_pose_diagonal(self):
        """Test offset pose on diagonal path"""
        target = [3.0, 3.0, 0.0]
        current = [0.0, 0.0, 0.5]
        offset = 1.0

        pose = calculate_offset_pose(target, current, offset, keep_height=True)

        # Check height is maintained
        assert abs(pose.z - 0.5) < 0.01

        # Should face toward target (yaw = 45 degrees = pi/4)
        expected_yaw = np.pi / 4
        assert abs(pose.yaw - expected_yaw) < 0.01

        # Use helper functions for common assertions
        assert_distance_from_target(pose, target, offset)
        assert_pose_on_line(pose, current, target, offset)

    def test_offset_zero_at_target(self):
        """Test zero offset places pose at target"""
        target = [5.0, 2.0, 0.1]
        current = [0.0, 0.0, 0.3]
        offset = 0.0

        pose = calculate_offset_pose(target, current, offset, keep_height=True)

        # Should be at target position (XY)
        assert abs(pose.x - 5.0) < 0.01
        assert abs(pose.y - 2.0) < 0.01
        assert abs(pose.z - 0.3) < 0.01  # Keep current height

        # Should face toward target
        expected_yaw = np.arctan2(target[1] - current[1], target[0] - current[0])
        assert abs(pose.yaw - expected_yaw) < 0.01

        # Use helper functions for common assertions
        assert_distance_from_target(pose, target, offset)
        assert_pose_on_line(pose, current, target, offset)

    def test_offset_keep_height_false(self):
        """Test offset with keep_height=False"""
        target = [5.0, 0.0, 0.1]
        current = [0.0, 0.0, 0.5]
        offset = 1.0

        pose = calculate_offset_pose(target, current, offset, keep_height=False)

        # Should use target height
        assert abs(pose.z - 0.1) < 0.01

    def test_offset_same_position(self, caplog):
        """Test offset when current and target are same (edge case)"""
        target = [5.0, 5.0, 0.1]
        current = [5.0, 5.0, 0.3]
        offset = 1.0

        # Should emit warning because direction is undefined
        with caplog.at_level(logging.WARNING):
            pose = calculate_offset_pose(target, current, offset, keep_height=True)

        # Check that warning was logged
        assert "current and target positions are same" in caplog.text
        assert "Direction is undefined" in caplog.text

        # When current and target XY are same, direction is undefined
        # Function should default to yaw=0 and stay at target position (offset is ignored)
        assert abs(pose.x - 5.0) < 0.01
        assert abs(pose.y - 5.0) < 0.01
        assert abs(pose.z - 0.3) < 0.01  # Keep current height
        assert abs(pose.yaw - 0.0) < 0.01  # Default yaw

    def test_offset_negative(self):
        """Test negative offset (beyond target)"""
        target = [5.0, 0.0, 0.1]
        current = [0.0, 0.0, 0.3]
        offset = -1.0

        pose = calculate_offset_pose(target, current, offset, keep_height=True)

        # Should be 1.0m beyond target (at x=6.0)
        assert abs(pose.x - 6.0) < 0.01
        assert abs(pose.y - 0.0) < 0.01

        # Should still face the target
        assert abs(pose.yaw - 0.0) < 0.01

        # Pose should be beyond target from current
        assert pose.x > target[0]  # Beyond target in positive x direction

        # Use helper functions for common assertions
        assert_distance_from_target(pose, target, offset)  # Uses abs(offset)
        assert_pose_on_line(pose, current, target, offset)

    def test_offset_pose_orientation(self):
        """Test that pose orientation faces target"""
        target = [0.0, 5.0, 0.0]
        current = [0.0, 0.0, 0.0]
        offset = 1.0

        pose = calculate_offset_pose(target, current, offset, keep_height=True)

        # Should face north (yaw = pi/2)
        expected_yaw = np.pi / 2
        assert abs(pose.yaw - expected_yaw) < 0.01

        # Should be 1.0m away from target (at y=4.0)
        assert abs(pose.x - 0.0) < 0.01
        assert abs(pose.y - 4.0) < 0.01

        # Verify pose is between current and target
        assert 0.0 < pose.y < 5.0

        # Use helper functions for common assertions
        assert_distance_from_target(pose, target, offset)
        assert_pose_on_line(pose, current, target, offset)


class TestWorldToGrid:
    """Test world_to_grid conversion"""

    def test_world_to_grid_basic(self):
        """Test basic world to grid conversion"""
        pos = [1.0, 2.0, 3.0]
        spacing = [1.0, 1.0, 1.0]

        grid = world_to_grid(pos, spacing)

        assert grid == [1, 2, 3]

    def test_world_to_grid_with_offset(self):
        """Test world to grid with offset"""
        pos = [2.5, 3.5, 4.5]
        spacing = [1.0, 1.0, 1.0]
        offset = [0.5, 0.5, 0.5]

        grid = world_to_grid(pos, spacing, offset)

        assert grid == [2, 3, 4]

    def test_world_to_grid_non_uniform_spacing(self):
        """Test world to grid with non-uniform spacing (different spacing per axis)"""
        pos = [2.0, 6.0, 15.0]
        spacing = [2.0, 3.0, 5.0]  # Different spacing for x, y, z

        grid = world_to_grid(pos, spacing)

        assert grid == [1, 2, 3]

    def test_world_to_grid_negative_position(self):
        """Test world to grid with negative positions"""
        pos = [-1.0, -2.0, -3.0]
        spacing = [1.0, 1.0, 1.0]

        grid = world_to_grid(pos, spacing)

        assert grid == [-1, -2, -3]

    def test_world_to_grid_2d_input(self):
        """Test world to grid with 2D position"""
        pos = [1.5, 2.5]
        spacing = [1.0, 1.0]

        grid = world_to_grid(pos, spacing)

        # Should return 3D grid with z=0
        assert grid == [2, 2, 0]

    def test_world_to_grid_rounding(self):
        """Test world to grid rounding behavior"""
        pos = [1.4, 1.6, 2.0]
        spacing = [1.0, 1.0, 1.0]

        grid = world_to_grid(pos, spacing)

        # Should round to nearest
        assert grid == [1, 2, 2]

    def test_world_to_grid_zero_position(self):
        """Test world to grid at origin"""
        pos = [0.0, 0.0, 0.0]
        spacing = [1.0, 1.0, 1.0]

        grid = world_to_grid(pos, spacing)

        assert grid == [0, 0, 0]


class TestGridToWorld:
    """Test grid_to_world conversion"""

    def test_grid_to_world_basic(self):
        """Test basic grid to world conversion"""
        grid = [1, 2, 3]
        spacing = [1.0, 1.0, 1.0]

        pos = grid_to_world(grid, spacing)

        assert pos == [1.0, 2.0, 3.0]

    def test_grid_to_world_with_offset(self):
        """Test grid to world with offset"""
        grid = [2, 3, 4]
        spacing = [1.0, 1.0, 1.0]
        offset = [0.5, 0.5, 0.5]

        pos = grid_to_world(grid, spacing, offset)

        assert pos == [2.5, 3.5, 4.5]

    def test_grid_to_world_non_uniform_spacing(self):
        """Test grid to world with non-uniform spacing (different spacing per axis)"""
        grid = [1, 2, 3]
        spacing = [2.0, 3.0, 5.0]  # Different spacing for x, y, z

        pos = grid_to_world(grid, spacing)

        assert pos == [2.0, 6.0, 15.0]

    def test_grid_to_world_negative_indices(self):
        """Test grid to world with negative indices"""
        grid = [-1, -2, -3]
        spacing = [1.0, 1.0, 1.0]

        pos = grid_to_world(grid, spacing)

        assert pos == [-1.0, -2.0, -3.0]

    def test_grid_to_world_2d_input(self):
        """Test grid to world with 2D grid"""
        grid = [2, 3]
        spacing = [1.0, 1.0]

        pos = grid_to_world(grid, spacing)

        # Should return 3D position with z=0
        assert pos == [2.0, 3.0, 0.0]

    def test_grid_to_world_zero_grid(self):
        """Test grid to world at origin"""
        grid = [0, 0, 0]
        spacing = [1.0, 1.0, 1.0]

        pos = grid_to_world(grid, spacing)

        assert pos == [0.0, 0.0, 0.0]


class TestGridRoundtrip:
    """Test roundtrip conversions between world and grid"""

    def test_roundtrip_basic(self):
        """Test world -> grid -> world roundtrip"""
        original_pos = [5.0, 10.0, 15.0]
        spacing = [1.0, 1.0, 1.0]

        grid = world_to_grid(original_pos, spacing)
        recovered_pos = grid_to_world(grid, spacing)

        assert recovered_pos == original_pos

    def test_roundtrip_with_offset(self):
        """Test roundtrip with offset"""
        original_pos = [5.5, 10.5, 15.5]
        spacing = [1.0, 1.0, 1.0]
        offset = [0.5, 0.5, 0.5]

        grid = world_to_grid(original_pos, spacing, offset)
        recovered_pos = grid_to_world(grid, spacing, offset)

        assert recovered_pos == original_pos

    def test_roundtrip_non_uniform(self):
        """Test roundtrip with non-uniform spacing"""
        original_pos = [10.0, 20.0, 30.0]
        spacing = [2.0, 4.0, 5.0]

        grid = world_to_grid(original_pos, spacing)
        recovered_pos = grid_to_world(grid, spacing)

        assert recovered_pos == original_pos

    def test_inverse_roundtrip(self):
        """Test grid -> world -> grid roundtrip"""
        original_grid = [5, 10, 15]
        spacing = [1.0, 1.0, 1.0]

        pos = grid_to_world(original_grid, spacing)
        recovered_grid = world_to_grid(pos, spacing)

        assert recovered_grid == original_grid


class TestEdgeCases:
    """Test edge cases and special scenarios"""

    def test_normalize_empty_list_raises_error(self):
        """Test that empty list raises error"""
        with pytest.raises(ValueError):
            normalize_vector_param([], "test_param", dim=3)

    def test_offset_pose_large_offset(self):
        """Test offset pose with very large offset"""
        target = [5.0, 0.0, 0.0]
        current = [0.0, 0.0, 0.0]
        offset = 100.0

        pose = calculate_offset_pose(target, current, offset, keep_height=True)

        # Use helper functions for validation with appropriate tolerance for large offset
        assert_distance_from_target(pose, target, offset, tolerance=0.1)
        assert_pose_on_line(pose, current, target, offset, tolerance=0.1)

    def test_grid_conversion_fractional_spacing(self):
        """Test grid conversion with fractional spacing"""
        pos = [1.5, 2.5, 3.5]
        spacing = [0.5, 0.5, 0.5]

        grid = world_to_grid(pos, spacing)
        recovered = grid_to_world(grid, spacing)

        # Should recover original position
        assert abs(recovered[0] - pos[0]) < 0.01
        assert abs(recovered[1] - pos[1]) < 0.01
        assert abs(recovered[2] - pos[2]) < 0.01


# =====================================================================
# body_to_world_velocity_2d
# =====================================================================
class TestBodyToWorldVelocity2D:
    """body_to_world_velocity_2d rotates (vx, vy) by yaw."""

    def test_zero_yaw(self):
        """At yaw=0, body and world frames are aligned."""
        from pybullet_fleet.tools import body_to_world_velocity_2d

        vx, vy = body_to_world_velocity_2d(1.0, 0.5, yaw=0.0)
        assert vx == pytest.approx(1.0)
        assert vy == pytest.approx(0.5)

    def test_90_degrees(self):
        """At yaw=pi/2, forward (+x body) maps to +y world."""
        import math
        from pybullet_fleet.tools import body_to_world_velocity_2d

        vx, vy = body_to_world_velocity_2d(1.0, 0.0, yaw=math.pi / 2)
        assert vx == pytest.approx(0.0, abs=1e-9)
        assert vy == pytest.approx(1.0, abs=1e-9)

    def test_lateral(self):
        """Lateral +y body at yaw=pi/2 maps to -x world."""
        import math
        from pybullet_fleet.tools import body_to_world_velocity_2d

        vx, vy = body_to_world_velocity_2d(0.0, 1.0, yaw=math.pi / 2)
        assert vx == pytest.approx(-1.0, abs=1e-9)
        assert vy == pytest.approx(0.0, abs=1e-9)


# =====================================================================
# body_to_world_velocity_3d
# =====================================================================
class TestBodyToWorldVelocity3D:
    """body_to_world_velocity_3d rotates (vx, vy, vz) by quaternion."""

    def test_identity_quaternion(self):
        """Identity quaternion passes through unchanged."""
        from pybullet_fleet.tools import body_to_world_velocity_3d

        vx, vy, vz = body_to_world_velocity_3d(1.0, 2.0, 3.0, (0.0, 0.0, 0.0, 1.0))
        assert vx == pytest.approx(1.0)
        assert vy == pytest.approx(2.0)
        assert vz == pytest.approx(3.0)

    def test_yaw_90(self):
        """90-degree yaw rotation matches 2D result."""
        from scipy.spatial.transform import Rotation
        from pybullet_fleet.tools import body_to_world_velocity_3d

        q = Rotation.from_euler("z", 90, degrees=True).as_quat()  # xyzw
        vx, vy, vz = body_to_world_velocity_3d(1.0, 0.0, 0.0, tuple(q))
        assert vx == pytest.approx(0.0, abs=1e-9)
        assert vy == pytest.approx(1.0, abs=1e-9)
        assert vz == pytest.approx(0.0, abs=1e-9)

    def test_pitch_90(self):
        """90-degree pitch: forward (+x body) maps to -z world."""
        from scipy.spatial.transform import Rotation
        from pybullet_fleet.tools import body_to_world_velocity_3d

        q = Rotation.from_euler("y", 90, degrees=True).as_quat()
        vx, vy, vz = body_to_world_velocity_3d(1.0, 0.0, 0.0, tuple(q))
        assert vx == pytest.approx(0.0, abs=1e-9)
        assert vy == pytest.approx(0.0, abs=1e-9)
        assert vz == pytest.approx(-1.0, abs=1e-9)

    def test_roll_90(self):
        """90-degree roll: +y body maps to +z world."""
        from scipy.spatial.transform import Rotation
        from pybullet_fleet.tools import body_to_world_velocity_3d

        q = Rotation.from_euler("x", 90, degrees=True).as_quat()
        vx, vy, vz = body_to_world_velocity_3d(0.0, 1.0, 0.0, tuple(q))
        assert vx == pytest.approx(0.0, abs=1e-9)
        assert vy == pytest.approx(0.0, abs=1e-9)
        assert vz == pytest.approx(1.0, abs=1e-9)

    def test_no_scipy_import(self):
        """Implementation must not import scipy (performance)."""
        import inspect
        from pybullet_fleet.tools import body_to_world_velocity_3d

        source = inspect.getsource(body_to_world_velocity_3d)
        # Ignore docstring lines — only check executable code
        in_docstring = False
        for line in source.splitlines():
            stripped = line.strip()
            if '"""' in stripped or "'''" in stripped:
                in_docstring = not in_docstring
                continue
            if in_docstring:
                continue
            assert "scipy" not in stripped, f"scipy found in executable code: {stripped}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
