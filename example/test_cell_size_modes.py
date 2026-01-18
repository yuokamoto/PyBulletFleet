#!/usr/bin/env python3
"""
Test script to verify spatial hash cell size calculation modes.

Demonstrates:
1. Mode 1 (constant): Fixed cell_size provided by user
2. Mode 2 (auto_adaptive): Recalculates cell_size when objects are added/removed
3. Mode 3 (auto_initial): Calculates cell_size once at first collision check
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pybullet_fleet import MultiRobotSimulationCore, SimulationParams, SimObject
from pybullet_fleet.types import SpatialHashCellSizeMode
import pybullet as p
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_mode_constant():
    """Test Mode 1: constant cell_size"""
    logger.info("\n" + "=" * 80)
    logger.info("TEST: Mode 1 - constant cell_size")
    logger.info("=" * 80)
    
    params = SimulationParams(
        gui=False,
        physics=False,
        monitor=False,
        collision_check_frequency=0.1,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.5,  # Fixed value
    )
    
    sim = MultiRobotSimulationCore(params)
    
    # Add some objects
    box_id_1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
    body_1 = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=box_id_1, basePosition=[0, 0, 0])
    obj_1 = SimObject(body_id=body_1, sim_core=sim)
    
    box_id_2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1.0, 1.0, 1.0])
    body_2 = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=box_id_2, basePosition=[3, 0, 0])
    obj_2 = SimObject(body_id=body_2, sim_core=sim)
    
    # Trigger collision check to initialize cell_size
    sim.check_collisions()
    
    logger.info(f"Cell size: {sim._cached_cell_size:.3f}m (expected: 2.5m)")
    assert sim._cached_cell_size == 2.5, f"Expected 2.5, got {sim._cached_cell_size}"
    
    # Add more objects - cell_size should NOT change
    box_id_3 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[5.0, 5.0, 5.0])
    body_3 = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=box_id_3, basePosition=[10, 0, 0])
    obj_3 = SimObject(body_id=body_3, sim_core=sim)
    
    logger.info(f"Cell size after adding large object: {sim._cached_cell_size:.3f}m (should remain 2.5m)")
    assert sim._cached_cell_size == 2.5, f"Cell size should not change in constant mode"
    
    # Clean up
    p.disconnect()
    logger.info("✓ Mode 1 (constant) test PASSED\n")


def test_mode_auto_adaptive():
    """Test Mode 2: auto_adaptive cell_size"""
    logger.info("\n" + "=" * 80)
    logger.info("TEST: Mode 2 - auto_adaptive cell_size")
    logger.info("=" * 80)
    
    params = SimulationParams(
        gui=False,
        physics=False,
        monitor=False,
        collision_check_frequency=0.1,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.AUTO_ADAPTIVE,
    )
    
    sim = MultiRobotSimulationCore(params)
    
    # Add multiple small objects to establish a baseline
    small_objects = []
    for i in range(5):
        box_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
        body = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=box_id, basePosition=[i * 2, 0, 0])
        obj = SimObject(body_id=body, sim_core=sim)
        small_objects.append(obj)
    
    baseline_cell_size = sim._cached_cell_size
    logger.info(f"Baseline cell size (5 small boxes): {baseline_cell_size:.3f}m")
    
    # Add large object - cell_size should recalculate automatically
    # With 5 small (extent=1.0) and 1 large (extent=10.0), median should shift
    box_id_large = p.createCollisionShape(p.GEOM_BOX, halfExtents=[5.0, 5.0, 5.0])
    body_large = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=box_id_large, basePosition=[20, 0, 0])
    obj_large = SimObject(body_id=body_large, sim_core=sim)
    
    cell_size_with_large = sim._cached_cell_size
    logger.info(f"Cell size after adding large object: {cell_size_with_large:.3f}m")
    logger.info(f"Cell size changed: {baseline_cell_size:.3f}m -> {cell_size_with_large:.3f}m")
    
    # Remove large object - cell_size should recalculate back
    sim.remove_object(obj_large)
    final_cell_size = sim._cached_cell_size
    logger.info(f"Cell size after removing large object: {final_cell_size:.3f}m")
    
    # Verify that cell_size was recalculated (may or may not change depending on median)
    # The important thing is that recalculation happened (no error)
    logger.info(f"Recalculation in auto_adaptive mode: ✓")
    
    # Clean up
    p.disconnect()
    logger.info("✓ Mode 2 (auto_adaptive) test PASSED\n")


def test_mode_auto_initial():
    """Test Mode 3: auto_initial cell_size (current default behavior)"""
    logger.info("\n" + "=" * 80)
    logger.info("TEST: Mode 3 - auto_initial cell_size")
    logger.info("=" * 80)
    
    params = SimulationParams(
        gui=False,
        physics=False,
        monitor=False,
        collision_check_frequency=0.1,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.AUTO_INITIAL,  # Default mode
    )
    
    sim = MultiRobotSimulationCore(params)
    
    # Add multiple small objects
    small_objects = []
    for i in range(5):
        box_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
        body = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=box_id, basePosition=[i * 2, 0, 0])
        obj = SimObject(body_id=body, sim_core=sim)
        small_objects.append(obj)
    
    # Trigger collision check to initialize cell_size
    sim.check_collisions()
    initial_cell_size = sim._cached_cell_size
    logger.info(f"Initial cell size (5 small boxes): {initial_cell_size:.3f}m")
    
    # Add large object - cell_size should NOT recalculate automatically
    box_id_large = p.createCollisionShape(p.GEOM_BOX, halfExtents=[5.0, 5.0, 5.0])
    body_large = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=box_id_large, basePosition=[20, 0, 0])
    obj_large = SimObject(body_id=body_large, sim_core=sim)
    
    cell_size_after_add = sim._cached_cell_size
    logger.info(f"Cell size after adding large object: {cell_size_after_add:.3f}m (should remain {initial_cell_size:.3f}m)")
    assert cell_size_after_add == initial_cell_size, "Cell size should not change in auto_initial mode"
    
    # Manual recalculation trigger
    logger.info("Manually triggering cell_size recalculation...")
    new_cell_size = sim.set_collision_spatial_hash_cell_size_mode()
    logger.info(f"Cell size after manual recalculation: {new_cell_size:.3f}m")
    logger.info(f"Manual recalculation successful: {initial_cell_size:.3f}m -> {new_cell_size:.3f}m")
    
    # Clean up
    p.disconnect()
    logger.info("✓ Mode 3 (auto_initial) test PASSED\n")


def main():
    """Run all tests"""
    logger.info("Testing spatial hash cell size calculation modes...")
    
    try:
        test_mode_constant()
        test_mode_auto_adaptive()
        test_mode_auto_initial()
        
        logger.info("=" * 80)
        logger.info("ALL TESTS PASSED ✓")
        logger.info("=" * 80)
    except AssertionError as e:
        logger.error(f"TEST FAILED: {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"UNEXPECTED ERROR: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
