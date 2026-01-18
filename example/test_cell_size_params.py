#!/usr/bin/env python3
"""
Simple unit test to verify spatial hash cell size mode configuration.

Tests parameter handling without running full simulations.
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pybullet_fleet import SimulationParams, SpatialHashCellSizeMode
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_params_mode_constant():
    """Test Mode 1: constant parameter configuration"""
    params = SimulationParams(
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.5,
    )
    
    assert params.spatial_hash_cell_size_mode == SpatialHashCellSizeMode.CONSTANT
    assert params.spatial_hash_cell_size == 2.5
    logger.info("✓ Mode 1 (CONSTANT) parameter test PASSED")


def test_params_mode_auto_adaptive():
    """Test Mode 2: auto_adaptive parameter configuration"""
    params = SimulationParams(
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.AUTO_ADAPTIVE,
    )
    
    assert params.spatial_hash_cell_size_mode == SpatialHashCellSizeMode.AUTO_ADAPTIVE
    logger.info("✓ Mode 2 (AUTO_ADAPTIVE) parameter test PASSED")


def test_params_mode_auto_initial():
    """Test Mode 3: auto_initial parameter configuration (default)"""
    params = SimulationParams(
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.AUTO_INITIAL,
    )
    
    assert params.spatial_hash_cell_size_mode == SpatialHashCellSizeMode.AUTO_INITIAL
    logger.info("✓ Mode 3 (AUTO_INITIAL) parameter test PASSED")


def test_params_default():
    """Test default mode"""
    params = SimulationParams()
    
    # Default should be auto_initial
    assert params.spatial_hash_cell_size_mode == SpatialHashCellSizeMode.AUTO_INITIAL
    logger.info("✓ Default mode test PASSED (defaults to AUTO_INITIAL)")


def main():
    """Run all parameter tests"""
    logger.info("=" * 80)
    logger.info("Testing spatial hash cell size mode parameters...")
    logger.info("=" * 80 + "\n")
    
    try:
        test_params_mode_constant()
        test_params_mode_auto_adaptive()
        test_params_mode_auto_initial()
        test_params_default()
        
        logger.info("\n" + "=" * 80)
        logger.info("ALL PARAMETER TESTS PASSED ✓")
        logger.info("=" * 80)
        logger.info("\nAvailable modes:")
        logger.info("  from pybullet_fleet import SpatialHashCellSizeMode")
        logger.info("  1. SpatialHashCellSizeMode.CONSTANT       - Fixed cell_size (fastest)")
        logger.info("  2. SpatialHashCellSizeMode.AUTO_ADAPTIVE  - Recalculate on add/remove")
        logger.info("  3. SpatialHashCellSizeMode.AUTO_INITIAL   - Calculate once at start (default)")
        logger.info("\nYAML config files support string values: 'constant', 'auto_adaptive', 'auto_initial'")
        
    except AssertionError as e:
        logger.error(f"TEST FAILED: {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"UNEXPECTED ERROR: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
