"""
Test multi-cell registration for large objects in spatial grid.

This test validates that:
1. Small objects are registered to single cells
2. Large objects are automatically registered to multiple overlapping cells
3. Collision detection works correctly with multi-cell registration
"""

import logging
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import pybullet as p
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import SimObject
from pybullet_fleet.types import CollisionMode, SpatialHashCellSizeMode

# Configure logging
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
logger = logging.getLogger(__name__)


def test_single_cell_registration():
    """Test that small objects use single cell registration"""
    logger.info("=" * 80)
    logger.info("TEST: Single cell registration for small objects")
    logger.info("=" * 80)

    # Create simulation with explicit cell size
    params = SimulationParams(
        gui=False,
        physics=False,
        monitor=False,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.0,  # 2m cell size
        multi_cell_threshold=1.5,  # 1.5x threshold = 3m
        log_level="debug",
    )

    sim_core = MultiRobotSimulationCore(params)
    sim_core.set_collision_spatial_hash_cell_size_mode()

    # Create small robot (0.5m x 0.5m, should use single cell)
    # Using simple box collision shape
    small_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25, 0.25, 0.1], physicsClientId=sim_core.client)
    small_visual = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[0.25, 0.25, 0.1], rgbaColor=[0, 0, 1, 1], physicsClientId=sim_core.client
    )
    small_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=small_collision,
        baseVisualShapeIndex=small_visual,
        basePosition=[0, 0, 0],
        physicsClientId=sim_core.client,
    )

    small_obj = SimObject(object_id=small_id, collision_mode=CollisionMode.NORMAL_3D)
    sim_core._sim_objects_dict[small_id] = small_obj

    # Update AABB and spatial grid
    sim_core._update_object_aabb(small_id, update_grid=True)

    # Check cell registration
    cells = sim_core._cached_object_to_cell.get(small_id, [])
    logger.info(f"Small object ({small_id}) registered to {len(cells)} cell(s): {cells}")

    assert len(cells) == 1, f"Expected single cell, got {len(cells)}"
    logger.info("✓ Small object uses single cell registration")

    sim_core.disconnect()


def test_multi_cell_registration():
    """Test that large objects use multi-cell registration"""
    logger.info("=" * 80)
    logger.info("TEST: Multi-cell registration for large objects")
    logger.info("=" * 80)

    # Create simulation with explicit cell size
    params = SimulationParams(
        gui=False,
        physics=False,
        monitor=False,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.0,  # 2m cell size
        multi_cell_threshold=1.5,  # 1.5x threshold = 3m
        log_level="debug",
    )

    sim_core = MultiRobotSimulationCore(params)
    sim_core.set_collision_spatial_hash_cell_size_mode()

    # Create large wall (5m x 5m, should use multi-cell)
    large_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[2.5, 2.5, 0.5], physicsClientId=sim_core.client)
    large_visual = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[2.5, 2.5, 0.5], rgbaColor=[1, 0, 0, 0.5], physicsClientId=sim_core.client
    )
    large_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=large_collision,
        baseVisualShapeIndex=large_visual,
        basePosition=[0, 0, 0],
        physicsClientId=sim_core.client,
    )

    large_obj = SimObject(object_id=large_id, collision_mode=CollisionMode.STATIC)
    sim_core._sim_objects_dict[large_id] = large_obj
    sim_core._static_objects.add(large_id)

    # Update AABB and spatial grid
    sim_core._update_object_aabb(large_id, update_grid=True)

    # Check cell registration
    cells = sim_core._cached_object_to_cell.get(large_id, [])
    logger.info(f"Large object ({large_id}) registered to {len(cells)} cell(s)")
    logger.info(f"Cells: {cells}")

    # 5m x 5m object with 2m cells should span 3x3 = 9 cells
    assert len(cells) > 1, f"Expected multiple cells, got {len(cells)}"
    logger.info(f"✓ Large object uses multi-cell registration ({len(cells)} cells)")

    sim_core.disconnect()


def test_large_object_collision_detection():
    """Test collision detection between small robot and large wall"""
    logger.info("=" * 80)
    logger.info("TEST: Collision detection with large objects")
    logger.info("=" * 80)

    # Create simulation with explicit cell size
    params = SimulationParams(
        gui=False,
        physics=False,
        monitor=False,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.0,  # 2m cell size
        multi_cell_threshold=1.5,  # 1.5x threshold = 3m
        log_level="info",
        ignore_static_collision=False,  # Enable static collision for this test
    )

    sim_core = MultiRobotSimulationCore(params)
    sim_core.set_collision_spatial_hash_cell_size_mode()

    # Create large wall at origin (5m x 5m)
    wall_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[2.5, 2.5, 0.5], physicsClientId=sim_core.client)
    wall_visual = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[2.5, 2.5, 0.5], rgbaColor=[1, 0, 0, 0.5], physicsClientId=sim_core.client
    )
    wall_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=wall_collision,
        baseVisualShapeIndex=wall_visual,
        basePosition=[0, 0, 0],
        physicsClientId=sim_core.client,
    )

    wall_obj = SimObject(object_id=wall_id, collision_mode=CollisionMode.STATIC)
    sim_core._sim_objects_dict[wall_id] = wall_obj
    sim_core._static_objects.add(wall_id)
    sim_core._cached_collision_modes[wall_id] = CollisionMode.STATIC

    # Create small robot near wall edge (0.5m x 0.5m)
    # Position at (3, 3, 0) - should be close to wall edge
    robot_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25, 0.25, 0.1], physicsClientId=sim_core.client)
    robot_visual = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[0.25, 0.25, 0.1], rgbaColor=[0, 0, 1, 1], physicsClientId=sim_core.client
    )
    robot_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=robot_collision,
        baseVisualShapeIndex=robot_visual,
        basePosition=[2.3, 2.3, 0],  # Near wall edge, within collision margin
        physicsClientId=sim_core.client,
    )

    robot_obj = SimObject(object_id=robot_id, collision_mode=CollisionMode.NORMAL_3D)
    sim_core._sim_objects_dict[robot_id] = robot_obj
    sim_core._cached_collision_modes[robot_id] = CollisionMode.NORMAL_3D

    # Update AABBs and spatial grids
    sim_core._update_object_aabb(wall_id, update_grid=True)
    sim_core._update_object_aabb(robot_id, update_grid=True)

    # Mark robot as moved
    sim_core._moved_this_step.add(robot_id)

    # Get potential collision pairs
    pairs = sim_core._get_potential_collision_pairs_spatial_hash(ignore_static=False)

    logger.info(f"Wall cells: {sim_core._cached_object_to_cell.get(wall_id, [])}")
    logger.info(f"Robot cells: {sim_core._cached_object_to_cell.get(robot_id, [])}")
    logger.info(f"Potential collision pairs: {pairs}")

    # Should detect potential collision
    expected_pair = (min(wall_id, robot_id), max(wall_id, robot_id))
    assert expected_pair in pairs, f"Expected collision pair {expected_pair} not found in {pairs}"
    logger.info(f"✓ Collision detected between large wall and small robot")

    sim_core.disconnect()


def test_threshold_configuration():
    """Test configurable multi-cell threshold"""
    logger.info("=" * 80)
    logger.info("TEST: Configurable multi-cell threshold")
    logger.info("=" * 80)

    # Create simulation with conservative threshold (1.2x)
    params = SimulationParams(
        gui=False,
        physics=False,
        monitor=False,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.0,  # 2m cell size
        multi_cell_threshold=1.2,  # Conservative: 1.2x = 2.4m
        log_level="info",
    )

    sim_core = MultiRobotSimulationCore(params)
    sim_core.set_collision_spatial_hash_cell_size_mode()

    # Create medium object (2.5m x 2.5m)
    # With threshold 1.2x (2.4m), this should use multi-cell
    # With threshold 1.5x (3.0m), this would use single cell
    medium_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1.25, 1.25, 0.1], physicsClientId=sim_core.client)
    medium_visual = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[1.25, 1.25, 0.1], rgbaColor=[0, 1, 0, 1], physicsClientId=sim_core.client
    )
    medium_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=medium_collision,
        baseVisualShapeIndex=medium_visual,
        basePosition=[0, 0, 0],
        physicsClientId=sim_core.client,
    )

    medium_obj = SimObject(object_id=medium_id, collision_mode=CollisionMode.NORMAL_3D)
    sim_core._sim_objects_dict[medium_id] = medium_obj

    # Update AABB and spatial grid
    sim_core._update_object_aabb(medium_id, update_grid=True)

    # Check cell registration
    cells = sim_core._cached_object_to_cell.get(medium_id, [])
    logger.info(f"Medium object (2.5m) with threshold 1.2x: {len(cells)} cell(s)")

    assert len(cells) > 1, f"Expected multi-cell with threshold 1.2x, got {len(cells)} cell(s)"
    logger.info(f"✓ Configurable threshold works correctly ({len(cells)} cells)")

    sim_core.disconnect()


if __name__ == "__main__":
    try:
        test_single_cell_registration()
        test_multi_cell_registration()
        test_large_object_collision_detection()
        test_threshold_configuration()

        logger.info("=" * 80)
        logger.info("ALL TESTS PASSED ✓")
        logger.info("=" * 80)

    except AssertionError as e:
        logger.error(f"TEST FAILED: {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"ERROR: {e}", exc_info=True)
        sys.exit(1)
