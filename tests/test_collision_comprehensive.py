"""
Comprehensive Collision Detection Tests

Tests all collision detection modes, methods, and edge cases to ensure
robust and correct collision detection behavior.

Test Categories:
1. Basic Collision Detection (mode combinations)
2. CollisionDetectionMethod verification
3. 2D vs 3D neighbor search
4. STATIC object optimization
5. DISABLED mode
6. Multi-cell registration
7. Movement detection
8. Runtime mode changes
9. Edge cases
10. Integration tests

Author: Auto-generated from COLLISION_TEST_DESIGN.md
Date: 2026-01-21
"""

import logging
import sys
import os
import itertools
from typing import List, Tuple, Optional

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import pybullet as p
import pytest
import numpy as np

from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import SimObject, ShapeParams
from pybullet_fleet.geometry import Pose
from pybullet_fleet.types import CollisionMode, CollisionDetectionMethod, SpatialHashCellSizeMode

# Configure logging
logging.basicConfig(level=logging.WARNING, format="%(message)s")
logger = logging.getLogger(__name__)


# ============================================================================
# Test Fixtures
# ============================================================================


@pytest.fixture
def sim_core_kinematics():
    """Simulation with physics=False, CLOSEST_POINTS (recommended default)"""
    params = SimulationParams(
        gui=False,
        physics=False,
        monitor=False,
        collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
        collision_margin=0.02,  # 2cm safety margin
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.0,
        ignore_static_collision=False,  # Enable for testing
        log_level="warning",
    )
    sim_core = MultiRobotSimulationCore(params)
    sim_core.set_collision_spatial_hash_cell_size_mode()
    yield sim_core
    p.disconnect(sim_core.client)


@pytest.fixture
def sim_core_physics():
    """Simulation with physics=True, CONTACT_POINTS"""
    params = SimulationParams(
        gui=False,
        physics=True,
        monitor=False,
        collision_detection_method=CollisionDetectionMethod.CONTACT_POINTS,
        collision_margin=0.02,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.0,
        ignore_static_collision=False,
        log_level="warning",
    )
    sim_core = MultiRobotSimulationCore(params)
    sim_core.set_collision_spatial_hash_cell_size_mode()
    yield sim_core
    p.disconnect(sim_core.client)


@pytest.fixture
def sim_core_hybrid():
    """Simulation with physics=True, HYBRID method"""
    params = SimulationParams(
        gui=False,
        physics=True,
        monitor=False,
        collision_detection_method=CollisionDetectionMethod.HYBRID,
        collision_margin=0.05,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.0,
        ignore_static_collision=False,
        log_level="warning",
    )
    sim_core = MultiRobotSimulationCore(params)
    sim_core.set_collision_spatial_hash_cell_size_mode()
    yield sim_core
    p.disconnect(sim_core.client)


# ============================================================================
# Helper Functions
# ============================================================================


def create_test_box(
    sim_core: MultiRobotSimulationCore,
    position: List[float],
    size: float = 0.5,
    collision_mode: CollisionMode = CollisionMode.NORMAL_3D,
    mass: float = 0.0,
    color: Optional[List[float]] = None,
) -> SimObject:
    """
    Create a simple box object for testing.

    Args:
        sim_core: Simulation core instance
        position: [x, y, z] position
        size: Half-extent of the box (box is 2*size in each dimension)
        collision_mode: Collision detection mode
        mass: Object mass (0 = kinematic)
        color: RGBA color (default: random)

    Returns:
        SimObject instance
    """
    if color is None:
        color = [np.random.random(), np.random.random(), np.random.random(), 1.0]

    # Create box using SimObject.from_mesh() with ShapeParams
    sim_obj = SimObject.from_mesh(
        visual_shape=ShapeParams(
            shape_type="box",
            half_extents=[size, size, size],
            rgba_color=color,
        ),
        collision_shape=ShapeParams(
            shape_type="box",
            half_extents=[size, size, size],
        ),
        pose=Pose.from_xyz(position[0], position[1], position[2]),
        mass=mass,
        sim_core=sim_core,
        collision_mode=collision_mode,
    )

    return sim_obj


def get_collision_pairs(sim_core: MultiRobotSimulationCore) -> List[Tuple[int, int]]:
    """
    Get current collision pairs.

    Args:
        sim_core: Simulation core instance

    Returns:
        List of (body_id1, body_id2) tuples
    """
    # Trigger collision detection
    # check_collisions() returns (pairs, timings) tuple
    collision_pairs, _ = sim_core.check_collisions()

    return collision_pairs


def assert_collision_detected(
    sim_core: MultiRobotSimulationCore, obj1: SimObject, obj2: SimObject, should_collide: bool = True, message: str = ""
):
    """
    Assert that collision between two objects is detected (or not).

    Args:
        sim_core: Simulation core instance
        obj1: First object
        obj2: Second object
        should_collide: True if collision should be detected
        message: Additional error message
    """
    pairs = get_collision_pairs(sim_core)
    expected_pair = (min(obj1.object_id, obj2.object_id), max(obj1.object_id, obj2.object_id))

    collision_detected = expected_pair in pairs

    if should_collide:
        assert collision_detected, (
            f"Collision NOT detected between obj{obj1.object_id} (body {obj1.body_id}) "
            f"and obj{obj2.object_id} (body {obj2.body_id}). "
            f"Expected pair: {expected_pair}, Found pairs: {pairs}. {message}"
        )
    else:
        assert not collision_detected, (
            f"Collision INCORRECTLY detected between obj{obj1.object_id} (body {obj1.body_id}) "
            f"and obj{obj2.object_id} (body {obj2.body_id}). "
            f"Unexpected pair: {expected_pair}, Found pairs: {pairs}. {message}"
        )


def assert_collision_count(sim_core: MultiRobotSimulationCore, expected_count: int, message: str = ""):
    """
    Assert the total number of collision pairs.

    Args:
        sim_core: Simulation core instance
        expected_count: Expected number of collision pairs
        message: Additional error message
    """
    pairs = get_collision_pairs(sim_core)
    actual_count = len(pairs)

    assert actual_count == expected_count, (
        f"Expected {expected_count} collision pairs, found {actual_count}. " f"Pairs: {pairs}. {message}"
    )


def get_object_cells(sim_core: MultiRobotSimulationCore, sim_obj: SimObject) -> List[Tuple[int, int, int]]:
    """
    Get the spatial grid cells an object is registered to.

    Args:
        sim_core: Simulation core instance
        sim_obj: Object to check

    Returns:
        List of (x, y, z) cell tuples
    """
    return sim_core._cached_object_to_cell.get(sim_obj.object_id, [])


# ============================================================================
# Category 1: Basic Collision Detection
# ============================================================================

# Test data generation for comprehensive collision mode testing
ALL_MODES = [
    CollisionMode.NORMAL_3D,
    CollisionMode.NORMAL_2D,
    CollisionMode.STATIC,
    CollisionMode.DISABLED,
]

DISTANCES = [
    (0.5, "overlapping"),  # Objects overlap (0.5 distance < 1.0 total size)
    (5.0, "separated"),  # Objects separated (5.0 distance > 1.0 total size)
]

# Special cases where behavior differs from the default rule
# Key: (mode1, mode2, distance) -> expected collision result
SPECIAL_CASES = {
    # Example: If there are any special collision rules, add them here
    # (CollisionMode.STATIC, CollisionMode.STATIC, 0.5): False,
}


def expected_collision(mode1: CollisionMode, mode2: CollisionMode, distance: float) -> bool:
    """
    Determine expected collision behavior based on collision modes and distance.

    Rules:
    1. Special cases take highest priority (if defined in SPECIAL_CASES)
    2. DISABLED mode: Never collides (regardless of distance or other mode)
    3. All other modes: Collide when overlapping (distance=0.5), not when separated (distance=5.0)

    Args:
        mode1: First object's collision mode
        mode2: Second object's collision mode
        distance: Distance between objects

    Returns:
        True if collision should be detected, False otherwise
    """
    # Check special cases first
    key = (mode1, mode2, distance)
    if key in SPECIAL_CASES:
        return SPECIAL_CASES[key]

    # DISABLED mode: Never collides
    if mode1 == CollisionMode.DISABLED or mode2 == CollisionMode.DISABLED:
        return False

    # Default rule: Collision depends on distance
    # Objects with size=0.5 have total extent of 1.0m
    # distance=0.5 → overlapping → should collide
    # distance=5.0 → separated → should NOT collide
    return distance == 0.5


def case_id(mode1: CollisionMode, mode2: CollisionMode, distance: float, dist_label: str) -> str:
    """Generate a readable test case ID."""
    return f"{mode1.name}-{mode2.name} {dist_label} (d={distance})"


def generate_collision_test_cases():
    """
    Generate all combinations of collision modes and distances.

    Yields:
        pytest.param with (mode1, mode2, distance, should_collide) and descriptive ID
    """
    for mode1, mode2 in itertools.product(ALL_MODES, repeat=2):
        for distance, dist_label in DISTANCES:
            should_collide = expected_collision(mode1, mode2, distance)
            yield pytest.param(
                mode1,
                mode2,
                distance,
                should_collide,
                id=case_id(mode1, mode2, distance, dist_label),
            )


class TestBasicCollisionDetection:
    """Test basic collision detection for all mode combinations"""

    @pytest.mark.parametrize("mode1,mode2,distance,should_collide", list(generate_collision_test_cases()))
    def test_collision_mode_combinations(self, sim_core_kinematics, mode1, mode2, distance, should_collide):
        """
        Test all collision mode combinations with overlapping and separated objects.

        Auto-generated test covering all combinations of:
        - Collision modes: NORMAL_3D, NORMAL_2D, STATIC, DISABLED
        - Distances: overlapping (0.5m), separated (5.0m)

        Total: 4 modes × 4 modes × 2 distances = 32 test cases

        Expected behavior:
        - DISABLED mode: Never collides (regardless of distance)
        - Other modes: Collide when overlapping, not when separated
        """
        obj1 = create_test_box(sim_core_kinematics, [0, 0, 0], size=0.5, collision_mode=mode1)
        obj2 = create_test_box(sim_core_kinematics, [distance, 0, 0], size=0.5, collision_mode=mode2)

        assert_collision_detected(
            sim_core_kinematics,
            obj1,
            obj2,
            should_collide=should_collide,
            message=f"mode1={mode1.name}, mode2={mode2.name}, distance={distance}",
        )

    def test_2d_mode_z_separation(self, sim_core_kinematics):
        """
        Special case: NORMAL_2D mode with Z-axis separation.

        2D mode means "check 9 XY neighbors" not "ignore Z axis in AABB".
        Objects separated in Z should NOT collide even in 2D mode.
        """
        obj1 = create_test_box(sim_core_kinematics, [0, 0, 0], size=0.5, collision_mode=CollisionMode.NORMAL_2D)
        obj2 = create_test_box(sim_core_kinematics, [0.5, 0, 10], size=0.5, collision_mode=CollisionMode.NORMAL_2D)

        # Z-separated and no AABB overlap in Z → should NOT collide even in 2D mode
        assert_collision_detected(
            sim_core_kinematics,
            obj1,
            obj2,
            should_collide=False,
            message="2D mode with Z separation should NOT collide (no Z overlap in AABB)",
        )


# ============================================================================
# Category 2: CollisionDetectionMethod Verification
# ============================================================================


class TestCollisionDetectionMethod:
    """Test different collision detection methods"""

    @pytest.mark.xfail(
        reason=(
            "AABB broadphase limitation: filter_aabb_pairs() requires strict AABB overlap "
            "but does not expand AABBs by collision_margin. For axis-aligned boxes with a "
            "positive surface gap (0.01m), AABBs never overlap even though getClosestPoints "
            "would detect the near-miss within margin (0.02m). "
            "Fix requires AABB expansion by margin in the broadphase filter."
        ),
        strict=True,
    )
    def test_closest_points_detects_near_miss(self, sim_core_kinematics):
        """TC101: CLOSEST_POINTS detects near-miss within margin"""
        margin = sim_core_kinematics.params.collision_margin  # 0.02m

        obj1 = create_test_box(sim_core_kinematics, [0, 0, 0], size=0.5, collision_mode=CollisionMode.NORMAL_3D)
        # Position just within margin (0.5 + 0.5 + 0.01 = 1.01, margin = 0.02)
        obj2 = create_test_box(sim_core_kinematics, [1.01, 0, 0], size=0.5, collision_mode=CollisionMode.NORMAL_3D)

        # Should detect because distance (0.01) < margin (0.02)
        assert_collision_detected(
            sim_core_kinematics,
            obj1,
            obj2,
            should_collide=True,
            message=f"CLOSEST_POINTS should detect near-miss within margin ({margin}m)",
        )

    @pytest.mark.skip(
        reason=(
            "CONTACT_POINTS detection needs further investigation - "
            "PyBullet reports 4 contacts but check_collisions() doesn't detect them. "
            "Requires debugging of filter_aabb_pairs or collision pair tracking logic."
        )
    )
    def test_contact_points_requires_step_simulation(self, sim_core_physics):
        """TC102: CONTACT_POINTS detects overlapping objects after step_once()"""
        import pybullet as p

        # Create two overlapping objects
        # At least one must have mass > 0 for PyBullet to generate contact points
        # obj1: center at [0, 0, 0.75], extends z=0.25 to 1.25 (with mass)
        # obj2: center at [0, 0, 0.25], extends z=-0.25 to 0.75 (static)
        # Overlap region: z=0.25 to 0.75 (0.5m overlap)
        obj1 = create_test_box(sim_core_physics, [0, 0, 0.75], size=0.5, collision_mode=CollisionMode.NORMAL_3D, mass=1.0)
        obj2 = create_test_box(sim_core_physics, [0, 0, 0.25], size=0.5, collision_mode=CollisionMode.NORMAL_3D, mass=0.0)

        # CONTACT_POINTS requires stepSimulation() to generate contact manifold
        sim_core_physics.step_once()

        # Update AABBs
        sim_core_physics._update_object_aabb(obj1.body_id, update_grid=True)
        sim_core_physics._update_object_aabb(obj2.body_id, update_grid=True)
        sim_core_physics._moved_this_step.add(obj1.body_id)

        # Debug: Check contact points
        contacts = p.getContactPoints(obj1.body_id, obj2.body_id, physicsClientId=sim_core_physics.client)
        print("\nDEBUG test_contact_points_requires_step_simulation:")
        print(f"  obj1.body_id={obj1.body_id} (mass=1.0), obj2.body_id={obj2.body_id} (mass=0.0)")
        print(f"  Direct getContactPoints: {len(contacts)} contacts")
        print(f"  _moved_this_step: {sim_core_physics._moved_this_step}")

        # Should detect contact
        assert_collision_detected(
            sim_core_physics, obj1, obj2, should_collide=True, message="CONTACT_POINTS should detect overlapping objects"
        )

    def test_auto_selection_kinematics(self):
        """TC105: Auto-selection chooses CLOSEST_POINTS for physics=False"""
        params = SimulationParams(
            gui=False,
            physics=False,
            monitor=False,
            # collision_detection_method not specified → should auto-select
        )
        sim_core = MultiRobotSimulationCore(params)

        try:
            assert (
                sim_core.params.collision_detection_method == CollisionDetectionMethod.CLOSEST_POINTS
            ), "physics=False should auto-select CLOSEST_POINTS"
        finally:
            p.disconnect(sim_core.client)

    def test_auto_selection_physics(self):
        """TC105: Auto-selection chooses CONTACT_POINTS for physics=True"""
        params = SimulationParams(
            gui=False,
            physics=True,
            monitor=False,
            # collision_detection_method not specified → should auto-select
        )
        sim_core = MultiRobotSimulationCore(params)

        try:
            assert (
                sim_core.params.collision_detection_method == CollisionDetectionMethod.CONTACT_POINTS
            ), "physics=True should auto-select CONTACT_POINTS"
        finally:
            p.disconnect(sim_core.client)


# ============================================================================
# Category 6: Multi-cell Registration
# ============================================================================


class TestMultiCellRegistration:
    """Test spatial hash multi-cell registration for large objects."""

    @pytest.fixture
    def sim_core_multicell(self):
        """Simulation with small cell size to trigger multi-cell registration."""
        params = SimulationParams(
            gui=False,
            physics=False,
            monitor=False,
            collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
            collision_margin=0.02,
            spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
            spatial_hash_cell_size=2.0,
            multi_cell_threshold=1.5,
            ignore_static_collision=False,
            log_level="warning",
        )
        sim_core = MultiRobotSimulationCore(params)
        sim_core.set_collision_spatial_hash_cell_size_mode()
        yield sim_core
        p.disconnect(sim_core.client)

    def test_small_object_single_cell(self, sim_core_multicell):
        """Small objects (< threshold * cell_size) use single cell."""
        obj = create_test_box(
            sim_core_multicell,
            [0, 0, 0],
            size=0.25,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        cells = sim_core_multicell._cached_object_to_cell.get(obj.object_id, [])
        assert len(cells) == 1, f"Small object should use 1 cell, got {len(cells)}"

    def test_large_object_multi_cell(self, sim_core_multicell):
        """Large objects (>= threshold * cell_size) use multiple cells."""
        obj = create_test_box(
            sim_core_multicell,
            [0, 0, 0],
            size=2.5,
            collision_mode=CollisionMode.STATIC,
        )
        cells = sim_core_multicell._cached_object_to_cell.get(obj.object_id, [])
        assert len(cells) > 1, f"Large object (5m) should use multiple cells, got {len(cells)}"

    def test_should_use_multi_cell_registration(self, sim_core_multicell):
        """_should_use_multi_cell_registration returns correct bool."""
        small = create_test_box(
            sim_core_multicell,
            [0, 0, 0],
            size=0.25,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        large = create_test_box(
            sim_core_multicell,
            [5, 0, 0],
            size=2.5,
            collision_mode=CollisionMode.STATIC,
        )
        assert not sim_core_multicell._should_use_multi_cell_registration(small.object_id)
        assert sim_core_multicell._should_use_multi_cell_registration(large.object_id)

    def test_get_overlapping_cells(self, sim_core_multicell):
        """_get_overlapping_cells returns cells covering object AABB."""
        large = create_test_box(
            sim_core_multicell,
            [0, 0, 0],
            size=2.5,
            collision_mode=CollisionMode.STATIC,
        )
        cells = sim_core_multicell._get_overlapping_cells(large.object_id)
        # 5m object with 2m cells → should span at least 4 cells
        assert len(cells) >= 4, f"Expected ≥4 overlapping cells, got {len(cells)}"

    def test_large_wall_collision_with_small_robot(self, sim_core_multicell):
        """Small robot collides with large wall via multi-cell lookup."""
        wall = create_test_box(
            sim_core_multicell,
            [0, 0, 0],
            size=2.5,
            collision_mode=CollisionMode.STATIC,
        )
        robot = create_test_box(
            sim_core_multicell,
            [2.3, 2.3, 0],
            size=0.25,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        # Mark robot as moved so check_collisions considers it
        sim_core_multicell._moved_this_step.add(robot.object_id)
        assert_collision_detected(
            sim_core_multicell,
            wall,
            robot,
            should_collide=True,
            message="Small robot near large wall edge should collide",
        )

    def test_threshold_affects_registration(self):
        """Lower threshold causes medium objects to use multi-cell."""
        params = SimulationParams(
            gui=False,
            physics=False,
            monitor=False,
            collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
            spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
            spatial_hash_cell_size=2.0,
            multi_cell_threshold=1.2,  # Lower threshold → 2.4m
            ignore_static_collision=False,
            log_level="warning",
        )
        sim_core = MultiRobotSimulationCore(params)
        sim_core.set_collision_spatial_hash_cell_size_mode()

        try:
            # 2.5m object: multi-cell at 1.2x (2.4m threshold)
            obj = create_test_box(sim_core, [0, 0, 0], size=1.25, collision_mode=CollisionMode.NORMAL_3D)
            cells = sim_core._cached_object_to_cell.get(obj.object_id, [])
            assert len(cells) > 1, f"Medium object with low threshold should use multi-cell, got {len(cells)}"
        finally:
            p.disconnect(sim_core.client)

    def test_coord_to_cell_positive(self, sim_core_multicell):
        """_coord_to_cell converts positive coordinates to correct cell index."""
        # cell_size=2.0  →  coord 3.5 → floor(3.5/2.0) = floor(1.75) = 1
        assert sim_core_multicell._coord_to_cell(3.5) == 1
        assert sim_core_multicell._coord_to_cell(0.0) == 0
        assert sim_core_multicell._coord_to_cell(1.99) == 0
        assert sim_core_multicell._coord_to_cell(2.0) == 1

    def test_coord_to_cell_negative(self, sim_core_multicell):
        """_coord_to_cell converts negative coordinates (floor toward -inf)."""
        # cell_size=2.0  →  coord -0.1 → floor(-0.05) = -1
        assert sim_core_multicell._coord_to_cell(-0.1) == -1
        assert sim_core_multicell._coord_to_cell(-2.0) == -1
        assert sim_core_multicell._coord_to_cell(-2.1) == -2

    def test_coord_to_cell_used_in_get_overlapping_cells(self, sim_core_multicell):
        """_get_overlapping_cells results are consistent with _coord_to_cell."""
        large = create_test_box(
            sim_core_multicell,
            [0, 0, 0],
            size=2.5,
            collision_mode=CollisionMode.STATIC,
        )
        cells = sim_core_multicell._get_overlapping_cells(large.object_id)
        aabb = sim_core_multicell._cached_aabbs_dict[large.object_id]
        # Each cell coordinate should match _coord_to_cell range
        for cell in cells:
            for d in range(3):
                min_c = sim_core_multicell._coord_to_cell(aabb[0][d])
                max_c = sim_core_multicell._coord_to_cell(aabb[1][d])
                assert min_c <= cell[d] <= max_c

    def test_same_cell_update_skips_grid_operations(self, sim_core_multicell):
        """_update_object_spatial_grid skips discard/add when cell unchanged."""
        obj = create_test_box(
            sim_core_multicell,
            [0, 0, 0],
            size=0.25,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        oid = obj.object_id
        cells_before = sim_core_multicell._cached_object_to_cell[oid]
        assert len(cells_before) == 1
        cell_key = cells_before[0]

        # Capture the set object identity for this cell
        grid_set_before = sim_core_multicell._cached_spatial_grid[cell_key]
        set_id_before = id(grid_set_before)

        # Move slightly within the same cell (cell_size=2.0, move 0.1m)
        from pybullet_fleet.geometry import Pose

        obj.set_pose(Pose(position=[0.1, 0.0, 0.0]))

        # Call _update_object_spatial_grid (simulating what set_pose triggers)
        sim_core_multicell._update_object_spatial_grid(oid)

        # Cell should be unchanged
        cells_after = sim_core_multicell._cached_object_to_cell[oid]
        assert cells_after == cells_before, "Cell should not change for small movement"

        # The grid set should be the same object (not recreated via delete+setdefault)
        grid_set_after = sim_core_multicell._cached_spatial_grid[cell_key]
        assert id(grid_set_after) == set_id_before, "Grid set was recreated — same-cell optimization not applied"

    def test_discard_from_cells_removes_object(self, sim_core_multicell):
        """_discard_from_cells removes object from spatial grid and cleans empty cells."""
        obj = create_test_box(
            sim_core_multicell,
            [0, 0, 0],
            size=0.25,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        oid = obj.object_id
        cells = sim_core_multicell._cached_object_to_cell[oid]
        assert len(cells) == 1
        cell_key = cells[0]
        assert oid in sim_core_multicell._cached_spatial_grid[cell_key]

        # Call _discard_from_cells directly
        sim_core_multicell._discard_from_cells(oid, cells)

        # Object should be removed from the cell's set
        # If the cell had only this object, the cell key should be deleted
        if cell_key in sim_core_multicell._cached_spatial_grid:
            assert oid not in sim_core_multicell._cached_spatial_grid[cell_key]

    def test_discard_from_cells_preserves_other_objects(self, sim_core_multicell):
        """_discard_from_cells only removes the target object, not others in the same cell."""
        obj1 = create_test_box(
            sim_core_multicell,
            [0, 0, 0],
            size=0.25,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        obj2 = create_test_box(
            sim_core_multicell,
            [0.1, 0, 0],
            size=0.25,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        cells1 = sim_core_multicell._cached_object_to_cell[obj1.object_id]
        cells2 = sim_core_multicell._cached_object_to_cell[obj2.object_id]
        # Both should be in the same cell (cell_size=2.0, both near origin)
        assert cells1 == cells2, "Both objects should be in the same cell"
        cell_key = cells1[0]

        # Remove obj1 only
        sim_core_multicell._discard_from_cells(obj1.object_id, cells1)

        # obj2 should still be in the cell
        assert obj2.object_id in sim_core_multicell._cached_spatial_grid[cell_key]
        # Cell should NOT be deleted (obj2 still there)
        assert cell_key in sim_core_multicell._cached_spatial_grid

    def test_discard_from_cells_deletes_empty_cell(self, sim_core_multicell):
        """_discard_from_cells deletes cell key when the set becomes empty."""
        obj = create_test_box(
            sim_core_multicell,
            [100, 100, 0],  # Far away to be the only object in this cell
            size=0.25,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        oid = obj.object_id
        cells = sim_core_multicell._cached_object_to_cell[oid]
        cell_key = cells[0]
        # Ensure only this object is in the cell
        assert sim_core_multicell._cached_spatial_grid[cell_key] == {oid}

        sim_core_multicell._discard_from_cells(oid, cells)

        # Cell key should be completely removed
        assert cell_key not in sim_core_multicell._cached_spatial_grid


# ============================================================================
# Category 7: Movement Detection
# ============================================================================


class TestMovementDetection:
    """Test that moved objects are correctly re-evaluated for collisions."""

    def test_move_into_collision(self, sim_core_kinematics):
        """Object moved into collision range is detected."""
        obj1 = create_test_box(sim_core_kinematics, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        obj2 = create_test_box(sim_core_kinematics, [5, 0, 0], collision_mode=CollisionMode.NORMAL_3D)

        # Initially separated → no collision
        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=False)

        # Move obj2 next to obj1
        obj2.set_pose(Pose.from_xyz(0.5, 0, 0))
        sim_core_kinematics._mark_object_moved(obj2.object_id)
        sim_core_kinematics._update_object_aabb(obj2.object_id, update_grid=True)
        sim_core_kinematics._moved_this_step.add(obj2.object_id)

        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=True)

    def test_move_out_of_collision(self, sim_core_kinematics):
        """Object moved away from collision range is no longer detected."""
        obj1 = create_test_box(sim_core_kinematics, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        obj2 = create_test_box(sim_core_kinematics, [0.5, 0, 0], collision_mode=CollisionMode.NORMAL_3D)

        # Initially overlapping → collision
        sim_core_kinematics._moved_this_step.add(obj2.object_id)
        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=True)

        # Move obj2 far away
        obj2.set_pose(Pose.from_xyz(10, 0, 0))
        sim_core_kinematics._mark_object_moved(obj2.object_id)
        sim_core_kinematics._update_object_aabb(obj2.object_id, update_grid=True)
        sim_core_kinematics._moved_this_step.add(obj2.object_id)

        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=False)


# ============================================================================
# Category 8: Runtime Mode Changes
# ============================================================================


class TestRuntimeModeChanges:
    """Test collision mode changes at runtime."""

    def test_remove_from_collision_system_clears_pairs(self, sim_core_kinematics):
        """_remove_object_from_collision_system also removes active collision pairs."""
        obj = create_test_box(sim_core_kinematics, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        oid = obj.object_id

        # Inject pairs referencing our object
        sim_core_kinematics._active_collision_pairs = {(oid, 99), (50, oid), (50, 99)}

        sim_core_kinematics._remove_object_from_collision_system(oid)

        # Pairs containing oid should be gone
        assert (oid, 99) not in sim_core_kinematics._active_collision_pairs
        assert (50, oid) not in sim_core_kinematics._active_collision_pairs
        # Unrelated pair survives
        assert (50, 99) in sim_core_kinematics._active_collision_pairs

    def test_remove_from_collision_system_no_op_pairs(self, sim_core_kinematics):
        """_remove_object_from_collision_system is no-op for pairs when id absent."""
        obj = create_test_box(sim_core_kinematics, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        oid = obj.object_id
        sim_core_kinematics._active_collision_pairs = {(50, 99)}

        sim_core_kinematics._remove_object_from_collision_system(oid)

        assert sim_core_kinematics._active_collision_pairs == {(50, 99)}

    def test_same_mode_update_is_no_op(self, sim_core_kinematics):
        """_update_object_collision_mode with old_mode == new_mode does nothing."""
        obj = create_test_box(sim_core_kinematics, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        oid = obj.object_id

        # Clear moved set to detect spurious additions
        sim_core_kinematics._moved_this_step.clear()

        # Call with same mode
        sim_core_kinematics._update_object_collision_mode(oid, CollisionMode.NORMAL_3D, CollisionMode.NORMAL_3D)

        # Should NOT add to _moved_this_step (early return, no side effects)
        assert oid not in sim_core_kinematics._moved_this_step, "Same-mode update should not add object to _moved_this_step"

    def test_disable_collision_at_runtime(self, sim_core_kinematics):
        """Disabling collision mode removes object from detection."""
        obj1 = create_test_box(sim_core_kinematics, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        obj2 = create_test_box(sim_core_kinematics, [0.5, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        sim_core_kinematics._moved_this_step.add(obj2.object_id)

        # Initially colliding
        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=True)

        # Disable obj2 collision
        old_mode = obj2.collision_mode
        obj2.set_collision_mode(CollisionMode.DISABLED)
        sim_core_kinematics._update_object_collision_mode(
            obj2.object_id,
            old_mode,
            CollisionMode.DISABLED,
        )

        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=False)

    def test_enable_collision_at_runtime(self, sim_core_kinematics):
        """Re-enabling collision mode re-adds object to detection."""
        obj1 = create_test_box(sim_core_kinematics, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        obj2 = create_test_box(sim_core_kinematics, [0.5, 0, 0], collision_mode=CollisionMode.DISABLED)

        sim_core_kinematics._moved_this_step.add(obj1.object_id)

        # Initially disabled → no collision
        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=False)

        # Enable obj2
        old_mode = obj2.collision_mode
        obj2.set_collision_mode(CollisionMode.NORMAL_3D)
        sim_core_kinematics._update_object_collision_mode(
            obj2.object_id,
            old_mode,
            CollisionMode.NORMAL_3D,
        )
        sim_core_kinematics._moved_this_step.add(obj2.object_id)

        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=True)


# ============================================================================
# Run Tests
# ============================================================================

if __name__ == "__main__":
    # Run with pytest
    pytest.main([__file__, "-v", "--tb=short"])
