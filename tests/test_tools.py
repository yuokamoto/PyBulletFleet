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
from pybullet_fleet.tools import (
    normalize_vector_param,
    calculate_offset_pose,
    world_to_grid,
    grid_to_world,
)


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

    def test_offset_keep_height_false(self):
        """Test offset with keep_height=False"""
        target = [5.0, 0.0, 0.1]
        current = [0.0, 0.0, 0.5]
        offset = 1.0

        pose = calculate_offset_pose(target, current, offset, keep_height=False)

        # Should use target height
        assert abs(pose.z - 0.1) < 0.01

    def test_offset_same_position(self):
        """Test offset when current and target are same"""
        target = [5.0, 5.0, 0.1]
        current = [5.0, 5.0, 0.3]
        offset = 1.0

        pose = calculate_offset_pose(target, current, offset, keep_height=True)

        # Should handle gracefully (no NaN)
        assert not np.isnan(pose.x)
        assert not np.isnan(pose.y)
        assert not np.isnan(pose.z)

    def test_offset_negative(self):
        """Test negative offset (beyond target)"""
        target = [5.0, 0.0, 0.1]
        current = [0.0, 0.0, 0.3]
        offset = -1.0

        pose = calculate_offset_pose(target, current, offset, keep_height=True)

        # Should be 1.0m beyond target (at x=6.0)
        assert abs(pose.x - 6.0) < 0.01

    def test_offset_pose_orientation(self):
        """Test that pose orientation faces target"""
        target = [0.0, 5.0, 0.0]
        current = [0.0, 0.0, 0.0]
        offset = 1.0

        pose = calculate_offset_pose(target, current, offset, keep_height=True)

        # Should face north (yaw = pi/2)
        expected_yaw = np.pi / 2
        assert abs(pose.yaw - expected_yaw) < 0.01


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
        """Test world to grid with non-uniform spacing"""
        pos = [2.0, 4.0, 6.0]
        spacing = [2.0, 2.0, 2.0]

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
        """Test grid to world with non-uniform spacing"""
        grid = [1, 2, 3]
        spacing = [2.0, 2.0, 2.0]

        pos = grid_to_world(grid, spacing)

        assert pos == [2.0, 4.0, 6.0]

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

        # Should be 100m away from target
        distance = np.sqrt((pose.x - target[0]) ** 2 + (pose.y - target[1]) ** 2)
        assert abs(distance - 100.0) < 0.1

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


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
