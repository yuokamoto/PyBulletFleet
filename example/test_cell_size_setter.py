#!/usr/bin/env python3
"""
Example demonstrating the setter function for collision spatial hash cell size mode.

Shows how to dynamically change mode and reinitialize the spatial hash grid.
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pybullet_fleet import (
    MultiRobotSimulationCore,
    SimulationParams,
    SpatialHashCellSizeMode,
    SimObject,
)
import pybullet as p
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    """Demonstrate dynamic mode switching with setter"""
    logger.info("=" * 80)
    logger.info("Testing collision spatial hash mode setter")
    logger.info("=" * 80 + "\n")
    
    # Start with AUTO_INITIAL mode
    params = SimulationParams(
        gui=False,
        physics=False,
        monitor=False,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.AUTO_INITIAL,
    )
    
    sim = MultiRobotSimulationCore(params)
    logger.info(f"Initial mode: {sim.params.spatial_hash_cell_size_mode.value}\n")
    
    # Add some objects
    logger.info("Adding 5 small objects...")
    for i in range(5):
        box_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
        body = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=box_id, basePosition=[i * 2, 0, 0])
        obj = SimObject(body_id=body, sim_core=sim)
    
    # Trigger initialization
    sim.check_collisions()
    logger.info(f"Cell size after initialization: {sim._cached_cell_size:.3f}m\n")
    
    # Add more objects
    logger.info("Adding 5 more objects...")
    for i in range(5, 10):
        box_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
        body = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=box_id, basePosition=[i * 2, 0, 0])
        obj = SimObject(body_id=body, sim_core=sim)
    
    logger.info(f"Cell size (no change in AUTO_INITIAL): {sim._cached_cell_size:.3f}m\n")
    
    # Switch to AUTO_ADAPTIVE mode using setter
    logger.info("Switching to AUTO_ADAPTIVE mode with setter...")
    sim.set_collision_spatial_hash_cell_size_mode(
        mode=SpatialHashCellSizeMode.AUTO_ADAPTIVE
    )
    logger.info(f"Cell size after mode switch: {sim._cached_cell_size:.3f}m\n")
    
    # Switch to CONSTANT mode with specific cell_size
    logger.info("Switching to CONSTANT mode with cell_size=3.0m...")
    sim.set_collision_spatial_hash_cell_size_mode(
        mode=SpatialHashCellSizeMode.CONSTANT,
        cell_size=3.0
    )
    logger.info(f"Cell size after mode switch: {sim._cached_cell_size:.3f}m\n")
    
    # Test switching back to AUTO_INITIAL
    logger.info("Switching back to AUTO_INITIAL...")
    sim.set_collision_spatial_hash_cell_size_mode(
        mode=SpatialHashCellSizeMode.AUTO_INITIAL
    )
    logger.info(f"Cell size after mode switch: {sim._cached_cell_size:.3f}m\n")
    
    # Clean up
    p.disconnect()
    
    logger.info("=" * 80)
    logger.info("Setter function test completed successfully ✓")
    logger.info("=" * 80)
    logger.info("\nKey features demonstrated:")
    logger.info("  ✓ Dynamic mode switching with setter")
    logger.info("  ✓ Automatic grid reinitialization")
    logger.info("  ✓ Enum-based mode specification")
    logger.info("  ✓ Collision spatial hash updates reflected in logs")


if __name__ == "__main__":
    main()
