"""
Tests for SimObjectManager and AgentManager.

Both managers share the same spawn / query logic (SimObjectManager is the base,
AgentManager sets ``_object_class = Agent``).  Parametrised tests run the same
assertions against both managers so the base class is directly covered.

AgentManager-specific features (goal setting, moving-count, aliases) are tested
in a separate section.
"""

import logging
import os
import random
import pybullet as p
import pybullet_data
import pytest

from pybullet_fleet.action import WaitAction
from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams, SimObjectManager
from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import ShapeParams, SimObject, SimObjectSpawnParams

MESH_PATH = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")


# =====================================================================
# MockSimCore
# =====================================================================
class MockSimCore:
    """Minimal sim_core mock that assigns object_ids."""

    def __init__(self, dt: float = 1.0 / 240.0):
        self.sim_time = 0.0
        self._dt = dt
        self.sim_objects = []
        self._next_object_id = 0
        self._kinematic_objects = set()
        self._client = 0

    @property
    def client(self):
        return self._client

    def add_object(self, obj):
        self.sim_objects.append(obj)

    def remove_object(self, obj):
        if obj in self.sim_objects:
            self.sim_objects.remove(obj)

    def _mark_object_moved(self, object_id):
        pass

    def register_callback(self, callback, frequency=None):
        """Record registered callbacks for verification."""
        if not hasattr(self, "_registered_callbacks"):
            self._registered_callbacks = []
        self._registered_callbacks.append({"func": callback, "frequency": frequency})

    def tick(self, n: int = 1):
        self.sim_time += self._dt * n


# =====================================================================
# Fixtures
# =====================================================================
@pytest.fixture
def mock_sim_core():
    return MockSimCore()


@pytest.fixture
def pybullet_env():
    physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    plane_id = p.loadURDF("plane.urdf")
    yield physics_client, plane_id
    p.disconnect()


# =====================================================================
# Helpers — spawn-params factories & grid assertion
# =====================================================================
def _make_spawn_params(manager_cls):
    """Return a single spawn-params appropriate for *manager_cls*."""
    visual = ShapeParams(shape_type="mesh", mesh_path=MESH_PATH, mesh_scale=[0.2, 0.2, 0.2])
    collision = ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1])
    if manager_cls is AgentManager:
        return AgentSpawnParams(visual_shape=visual, collision_shape=collision, mass=1.0)
    return SimObjectSpawnParams(visual_shape=visual, collision_shape=collision, mass=1.0)


def _make_colored_params(manager_cls, rgba):
    """Return spawn-params with a specific colour."""
    visual = ShapeParams(shape_type="mesh", mesh_path=MESH_PATH, mesh_scale=[0.2, 0.2, 0.2], rgba_color=rgba)
    collision = ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1])
    if manager_cls is AgentManager:
        return AgentSpawnParams(visual_shape=visual, collision_shape=collision, mass=1.0)
    return SimObjectSpawnParams(visual_shape=visual, collision_shape=collision, mass=1.0)


def _make_posed_params(manager_cls, pose):
    """Return spawn-params with an initial pose."""
    visual = ShapeParams(shape_type="mesh", mesh_path=MESH_PATH, mesh_scale=[0.2, 0.2, 0.2])
    collision = ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1])
    if manager_cls is AgentManager:
        return AgentSpawnParams(visual_shape=visual, collision_shape=collision, initial_pose=pose, mass=1.0)
    return SimObjectSpawnParams(visual_shape=visual, collision_shape=collision, initial_pose=pose, mass=1.0)


def _expected_object_cls(manager_cls):
    """Return the object class that the manager should produce."""
    return Agent if manager_cls is AgentManager else SimObject


def _create_manager(manager_cls, sim_core):
    """Instantiate the correct manager."""
    if manager_cls is AgentManager:
        return AgentManager(sim_core=sim_core)
    return SimObjectManager(sim_core=sim_core)


def _make_grid(x_max, y_max, spacing=1.0, z_offset=0.5, z_max=0, z_spacing=0.0):
    """Create GridSpawnParams.  For 3D grids, pass *z_max* > 0 and *z_spacing*."""
    return GridSpawnParams(
        x_min=0,
        x_max=x_max,
        y_min=0,
        y_max=y_max,
        z_min=0,
        z_max=z_max,
        spacing=[spacing, spacing, z_spacing],
        offset=[0, 0, z_offset],
    )


def assert_grid_positions(poses, expected_xs, expected_ys, expected_zs=None):
    """Assert every pose sits on the expected grid.

    *expected_zs* can be:
    - ``None``  — skip z check
    - a ``float`` — every z must equal that value (2D shortcut)
    - a ``set``  — z must be one of the values (3D)
    """
    for pose in poses:
        assert pose.x in expected_xs, f"x={pose.x} not in {expected_xs}"
        assert pose.y in expected_ys, f"y={pose.y} not in {expected_ys}"
        if expected_zs is None:
            pass
        elif isinstance(expected_zs, (int, float)):
            assert pose.z == pytest.approx(expected_zs), f"z={pose.z} != {expected_zs}"
        else:
            # set / frozenset
            assert any(pose.z == pytest.approx(ez) for ez in expected_zs), f"z={pose.z} not in {expected_zs}"


def unique_xy(poses):
    """Return the set of (x, y) rounded tuples."""
    return {(round(p.x, 6), round(p.y, 6)) for p in poses}


def unique_xyz(poses):
    """Return the set of (x, y, z) rounded tuples."""
    return {(round(p.x, 6), round(p.y, 6), round(p.z, 6)) for p in poses}


# =====================================================================
# Parametrised tests — run on BOTH SimObjectManager and AgentManager
# =====================================================================
MANAGER_CLASSES = [SimObjectManager, AgentManager]


@pytest.mark.parametrize("manager_cls", MANAGER_CLASSES, ids=["SimObjectManager", "AgentManager"])
class TestSpawningGrid:
    """Grid-based spawning: common behaviour for both managers."""

    def test_grid_basic(self, pybullet_env, mock_sim_core, manager_cls):
        """Spawn N objects on a 2×2 grid — count and cache."""
        mgr = _create_manager(manager_cls, mock_sim_core)
        params = _make_spawn_params(manager_cls)
        grid = _make_grid(x_max=1, y_max=1)

        objects = mgr.spawn_objects_grid(num_objects=4, grid_params=grid, spawn_params=params)

        assert len(objects) == 4
        assert mgr.get_object_count() == 4
        assert all(isinstance(o, _expected_object_cls(manager_cls)) for o in objects)
        # Same mesh → 1 cache entry
        assert len(SimObject._shared_shapes) == 1

    def test_grid_positions(self, pybullet_env, mock_sim_core, manager_cls):
        """9 objects on 3×3, spacing=2 → all positions valid and unique."""
        mgr = _create_manager(manager_cls, mock_sim_core)
        params = _make_spawn_params(manager_cls)
        grid = _make_grid(x_max=2, y_max=2, spacing=2.0)

        mgr.spawn_objects_grid(num_objects=9, grid_params=grid, spawn_params=params)

        poses = mgr.get_all_poses()
        assert len(poses) == 9
        assert_grid_positions(poses, {0.0, 2.0, 4.0}, {0.0, 2.0, 4.0}, expected_zs=0.5)
        assert len(unique_xy(poses)) == 9

    def test_fewer_than_grid(self, pybullet_env, mock_sim_core, manager_cls):
        """6 objects on a 4×4 grid — positions valid, all unique."""
        mgr = _create_manager(manager_cls, mock_sim_core)
        params = _make_spawn_params(manager_cls)
        grid = _make_grid(x_max=3, y_max=3)

        objects = mgr.spawn_objects_grid(num_objects=6, grid_params=grid, spawn_params=params)

        assert len(objects) == 6
        poses = mgr.get_all_poses()
        assert_grid_positions(poses, {0.0, 1.0, 2.0, 3.0}, {0.0, 1.0, 2.0, 3.0}, expected_zs=0.5)
        assert len(unique_xy(poses)) == 6

    def test_grid_3d(self, pybullet_env, mock_sim_core, manager_cls):
        """2×2×2 grid (8 objects) with z layers — positions valid and unique in 3D."""
        mgr = _create_manager(manager_cls, mock_sim_core)
        params = _make_spawn_params(manager_cls)
        # spacing=1.0 in xy, z_spacing=2.0, z_offset=0.5
        # expected z: 0*2.0+0.5=0.5,  1*2.0+0.5=2.5
        grid = _make_grid(x_max=1, y_max=1, spacing=1.0, z_offset=0.5, z_max=1, z_spacing=2.0)

        objects = mgr.spawn_objects_grid(num_objects=8, grid_params=grid, spawn_params=params)

        assert len(objects) == 8
        poses = mgr.get_all_poses()
        assert_grid_positions(poses, {0.0, 1.0}, {0.0, 1.0}, expected_zs={0.5, 2.5})
        assert len(unique_xyz(poses)) == 8


@pytest.mark.parametrize("manager_cls", MANAGER_CLASSES, ids=["SimObjectManager", "AgentManager"])
class TestSpawningMixed:
    """Mixed-type grid spawning."""

    def test_grid_mixed(self, pybullet_env, mock_sim_core, manager_cls):
        """prob sum == 1.0 → all spots filled; both types appear."""
        mgr = _create_manager(manager_cls, mock_sim_core)
        red = _make_colored_params(manager_cls, [1, 0, 0, 1])
        blue = _make_colored_params(manager_cls, [0, 0, 1, 1])
        grid = _make_grid(x_max=2, y_max=2)  # 9 spots

        random.seed(42)
        objects = mgr.spawn_grid_mixed(num_objects=9, grid_params=grid, spawn_params_list=[(red, 0.5), (blue, 0.5)])

        # prob sums to 1.0 → every spot must be filled
        assert len(objects) == 9
        assert mgr.get_object_count() == 9

        # Verify both types present via PyBullet visual colour
        red_count = blue_count = 0
        for obj in objects:
            rgba = p.getVisualShapeData(obj.body_id)[0][7]
            if rgba[0] > 0.5 and rgba[2] < 0.5:
                red_count += 1
            elif rgba[2] > 0.5 and rgba[0] < 0.5:
                blue_count += 1
        assert red_count > 0, "Expected at least 1 red object"
        assert blue_count > 0, "Expected at least 1 blue object"
        assert red_count + blue_count == 9

        poses = mgr.get_all_poses()
        assert_grid_positions(poses, {0.0, 1.0, 2.0}, {0.0, 1.0, 2.0}, expected_zs=0.5)
        assert len(unique_xy(poses)) == 9
        assert len(SimObject._shared_shapes) == 2

    def test_grid_mixed_law_of_large_numbers(self, pybullet_env, mock_sim_core, manager_cls):
        """Large grid (400 spots) — actual ratio converges to specified probability."""
        mgr = _create_manager(manager_cls, mock_sim_core)
        red = _make_colored_params(manager_cls, [1, 0, 0, 1])
        blue = _make_colored_params(manager_cls, [0, 0, 1, 1])
        grid = _make_grid(x_max=19, y_max=19)  # 20×20 = 400 spots

        random.seed(123)
        objects = mgr.spawn_grid_mixed(
            num_objects=400,
            grid_params=grid,
            spawn_params_list=[(red, 0.7), (blue, 0.3)],
        )

        assert len(objects) == 400

        red_count = blue_count = 0
        for obj in objects:
            rgba = p.getVisualShapeData(obj.body_id)[0][7]
            if rgba[0] > 0.5 and rgba[2] < 0.5:
                red_count += 1
            elif rgba[2] > 0.5 and rgba[0] < 0.5:
                blue_count += 1

        red_ratio = red_count / 400
        blue_ratio = blue_count / 400
        assert red_ratio == pytest.approx(0.7, abs=0.05), f"red ratio {red_ratio:.3f} not ≈ 0.7"
        assert blue_ratio == pytest.approx(0.3, abs=0.05), f"blue ratio {blue_ratio:.3f} not ≈ 0.3"

    def test_grid_mixed_partial_probability(self, pybullet_env, mock_sim_core, manager_cls):
        """prob sum < 1.0 → some spots are intentionally left empty."""
        mgr = _create_manager(manager_cls, mock_sim_core)
        red = _make_colored_params(manager_cls, [1, 0, 0, 1])
        blue = _make_colored_params(manager_cls, [0, 0, 1, 1])
        grid = _make_grid(x_max=19, y_max=19)  # 20×20 = 400 spots

        # prob sum = 0.6 → ~60% filled, ~40% empty
        random.seed(0)
        objects = mgr.spawn_grid_mixed(
            num_objects=400,
            grid_params=grid,
            spawn_params_list=[(red, 0.4), (blue, 0.2)],
        )

        assert len(objects) < 400, "Some spots must be empty when prob sum < 1.0"
        assert len(objects) > 0, "At least some objects should spawn"
        fill_ratio = len(objects) / 400
        assert fill_ratio == pytest.approx(0.6, abs=0.05), f"fill ratio {fill_ratio:.3f} not ≈ 0.6"
        assert mgr.get_object_count() == len(objects)

        # Per-type ratio among spawned objects
        red_count = blue_count = 0
        for obj in objects:
            rgba = p.getVisualShapeData(obj.body_id)[0][7]
            if rgba[0] > 0.5 and rgba[2] < 0.5:
                red_count += 1
            elif rgba[2] > 0.5 and rgba[0] < 0.5:
                blue_count += 1
        # Expected: red≈0.4/0.6≈0.667 of spawned, blue≈0.2/0.6≈0.333 of spawned
        spawned = len(objects)
        assert red_count / spawned == pytest.approx(0.4 / 0.6, abs=0.06)
        assert blue_count / spawned == pytest.approx(0.2 / 0.6, abs=0.06)

        # All spawned objects still sit on valid grid positions
        expected_xs = {float(x) for x in range(20)}
        expected_ys = {float(y) for y in range(20)}
        poses = mgr.get_all_poses()
        assert_grid_positions(poses, expected_xs, expected_ys, expected_zs=0.5)
        assert len(unique_xy(poses)) == len(objects)

    def test_grid_counts(self, pybullet_env, mock_sim_core, manager_cls):
        """Exact counts (3 red + 5 green) — verify per-type breakdown."""
        mgr = _create_manager(manager_cls, mock_sim_core)
        red = _make_colored_params(manager_cls, [1, 0, 0, 1])
        green = _make_colored_params(manager_cls, [0, 1, 0, 1])
        grid = _make_grid(x_max=3, y_max=3)

        objects = mgr.spawn_grid_counts(grid_params=grid, spawn_params_count_list=[(red, 3), (green, 5)])

        assert len(objects) == 8
        assert mgr.get_object_count() == 8

        # per-type breakdown via PyBullet visual colour
        red_count = green_count = 0
        for obj in objects:
            r, g = p.getVisualShapeData(obj.body_id)[0][7][:2]
            if r > 0.5 and g < 0.5:
                red_count += 1
            elif g > 0.5 and r < 0.5:
                green_count += 1
        assert red_count == 3
        assert green_count == 5

        poses = mgr.get_all_poses()
        assert_grid_positions(poses, {0.0, 1.0, 2.0, 3.0}, {0.0, 1.0, 2.0, 3.0}, expected_zs=0.5)
        assert len(unique_xy(poses)) == 8
        assert len(SimObject._shared_shapes) == 2

    def test_grid_counts_exceeds_grid(self, pybullet_env, mock_sim_core, manager_cls):
        """Total count > grid capacity → ValueError."""
        mgr = _create_manager(manager_cls, mock_sim_core)
        params = _make_spawn_params(manager_cls)
        grid = _make_grid(x_max=1, y_max=1)  # capacity = 4

        with pytest.raises(ValueError, match="exceeds grid cell count"):
            mgr.spawn_grid_counts(grid_params=grid, spawn_params_count_list=[(params, 5)])


@pytest.mark.parametrize("manager_cls", MANAGER_CLASSES, ids=["SimObjectManager", "AgentManager"])
class TestBatchSpawning:
    """Batch (non-grid) spawning."""

    def test_spawn_objects_batch(self, pybullet_env, mock_sim_core, manager_cls):
        """Batch spawn 5 objects with explicit poses."""
        mgr = _create_manager(manager_cls, mock_sim_core)
        params_list = [_make_posed_params(manager_cls, Pose.from_xyz(i, 0, 0.5)) for i in range(5)]

        objects = mgr.spawn_objects_batch(params_list)

        assert len(objects) == 5
        assert mgr.get_object_count() == 5
        assert all(isinstance(o, _expected_object_cls(manager_cls)) for o in objects)

        poses = mgr.get_all_poses()
        for i, pose in enumerate(poses):
            assert pose.x == pytest.approx(i, abs=1e-3)
            assert pose.y == pytest.approx(0, abs=1e-3)
            assert pose.z == pytest.approx(0.5, abs=1e-3)


@pytest.mark.parametrize("manager_cls", MANAGER_CLASSES, ids=["SimObjectManager", "AgentManager"])
class TestQueries:
    """Query methods: get_pose, get_all_poses, get_poses_dict, get_object_count."""

    def test_empty_manager(self, pybullet_env, manager_cls):
        """Empty manager returns sensible defaults."""
        mgr = _create_manager(manager_cls, sim_core=None)
        assert mgr.get_object_count() == 0
        assert mgr.get_all_poses() == []
        assert mgr.get_poses_dict() == {}

    def test_get_all_poses(self, pybullet_env, mock_sim_core, manager_cls):
        """get_all_poses returns correct positions for each object."""
        mgr = _create_manager(manager_cls, mock_sim_core)
        expected = [(i, i * 2, 0.5) for i in range(3)]
        params_list = [_make_posed_params(manager_cls, Pose.from_xyz(*xyz)) for xyz in expected]
        mgr.spawn_objects_batch(params_list)

        poses = mgr.get_all_poses()
        assert len(poses) == 3
        for pose, (ex, ey, ez) in zip(poses, expected):
            assert pose.x == pytest.approx(ex, abs=1e-3)
            assert pose.y == pytest.approx(ey, abs=1e-3)
            assert pose.z == pytest.approx(ez, abs=1e-3)

    def test_get_poses_dict(self, pybullet_env, mock_sim_core, manager_cls):
        """get_poses_dict keys are body_ids and values have correct positions."""
        mgr = _create_manager(manager_cls, mock_sim_core)
        expected = [(1.0, 2.0, 0.5), (3.0, 4.0, 0.5)]
        params_list = [_make_posed_params(manager_cls, Pose.from_xyz(*xyz)) for xyz in expected]
        mgr.spawn_objects_batch(params_list)

        poses_dict = mgr.get_poses_dict()
        assert len(poses_dict) == 2
        # Keys must be the actual body_ids of spawned objects
        assert set(poses_dict.keys()) == {obj.body_id for obj in mgr.objects}
        for obj, (ex, ey, ez) in zip(mgr.objects, expected):
            pose = poses_dict[obj.body_id]
            assert pose.x == pytest.approx(ex, abs=1e-3)
            assert pose.y == pytest.approx(ey, abs=1e-3)
            assert pose.z == pytest.approx(ez, abs=1e-3)

    def test_get_pose(self, pybullet_env, mock_sim_core, manager_cls):
        """get_pose returns correct position for valid index, None for invalid."""
        mgr = _create_manager(manager_cls, mock_sim_core)
        expected = [(10.0, 20.0, 0.5), (30.0, 40.0, 0.5)]
        params_list = [_make_posed_params(manager_cls, Pose.from_xyz(*xyz)) for xyz in expected]
        mgr.spawn_objects_batch(params_list)

        for idx, (ex, ey, ez) in enumerate(expected):
            pose = mgr.get_pose(idx)
            assert isinstance(pose, Pose)
            assert pose.x == pytest.approx(ex, abs=1e-3)
            assert pose.y == pytest.approx(ey, abs=1e-3)
            assert pose.z == pytest.approx(ez, abs=1e-3)

        assert mgr.get_pose(-1) is None
        assert mgr.get_pose(999) is None

    def test_add_object_directly(self, pybullet_env, mock_sim_core, manager_cls):
        """Manually add an object — tracked in objects/body_ids/object_ids."""
        mgr = _create_manager(manager_cls, mock_sim_core)
        params = _make_spawn_params(manager_cls)
        obj_cls = _expected_object_cls(manager_cls)
        obj = obj_cls.from_params(params, sim_core=mock_sim_core)

        mgr.add_object(obj)

        assert mgr.get_object_count() == 1
        assert mgr.body_ids[obj.body_id] is obj
        assert mgr.object_ids[obj.object_id] is obj


# =====================================================================
# AgentManager-specific tests
# =====================================================================


def _spawn_agents(mock_sim_core, pybullet_env, n=4):
    """Helper: create an AgentManager, spawn *n* agents on a grid, return (mgr, agents)."""
    mgr = AgentManager(sim_core=mock_sim_core)
    params = _make_spawn_params(AgentManager)
    grid = _make_grid(x_max=max(n - 1, 1), y_max=max(n - 1, 1))
    agents = mgr.spawn_agents_grid(num_agents=n, grid_params=grid, spawn_params=params)
    return mgr, agents


class TestAgentManagerSpecific:
    """Features that only exist on AgentManager (not SimObjectManager)."""

    # ------------------------------------------------------------------
    # get_moving_count
    # ------------------------------------------------------------------
    def test_get_moving_count(self, pybullet_env, mock_sim_core):
        """Moving count starts at 0 and increments when an agent has a goal."""
        mgr, agents = _spawn_agents(mock_sim_core, pybullet_env)

        assert mgr.get_moving_count() == 0
        agents[0].set_goal_pose(Pose.from_xyz(5, 0, 0.5))
        assert mgr.get_moving_count() == 1

    # ------------------------------------------------------------------
    # set_goal_pose (by index)
    # ------------------------------------------------------------------
    def test_set_goal_pose_valid_index(self, pybullet_env, mock_sim_core):
        """Valid index sets goal and marks agent as moving."""
        mgr, agents = _spawn_agents(mock_sim_core, pybullet_env)
        goal = Pose.from_xyz(10, 0, 0.5)

        mgr.set_goal_pose(0, goal)

        assert agents[0].is_moving

    def test_set_goal_pose_invalid_index(self, pybullet_env, mock_sim_core, caplog):
        """Invalid index logs a warning and does not crash."""
        mgr, agents = _spawn_agents(mock_sim_core, pybullet_env)
        goal = Pose.from_xyz(10, 0, 0.5)

        with caplog.at_level(logging.WARNING):
            mgr.set_goal_pose(999, goal)

        assert "Invalid agent index" in caplog.text
        assert mgr.get_moving_count() == 0

    def test_set_goal_pose_negative_index(self, pybullet_env, mock_sim_core, caplog):
        """Negative index is out of range and logs a warning."""
        mgr, agents = _spawn_agents(mock_sim_core, pybullet_env)

        with caplog.at_level(logging.WARNING):
            mgr.set_goal_pose(-1, Pose.from_xyz(0, 0, 0))

        assert "Invalid agent index" in caplog.text

    # ------------------------------------------------------------------
    # set_goal_pose_by_body_id
    # ------------------------------------------------------------------
    def test_set_goal_pose_by_body_id_valid(self, pybullet_env, mock_sim_core):
        """Valid body_id sets goal on the correct agent."""
        mgr, agents = _spawn_agents(mock_sim_core, pybullet_env)
        target_agent = agents[1]
        goal = Pose.from_xyz(20, 0, 0.5)

        mgr.set_goal_pose_by_body_id(target_agent.body_id, goal)

        assert target_agent.is_moving
        # Other agents remain stationary
        assert not agents[0].is_moving

    def test_set_goal_pose_by_body_id_invalid(self, pybullet_env, mock_sim_core, caplog):
        """Invalid body_id logs a warning."""
        mgr, _ = _spawn_agents(mock_sim_core, pybullet_env)

        with caplog.at_level(logging.WARNING):
            mgr.set_goal_pose_by_body_id(9999, Pose.from_xyz(0, 0, 0))

        assert "Unknown agent body_id" in caplog.text

    # ------------------------------------------------------------------
    # set_goal_pose_by_object_id
    # ------------------------------------------------------------------
    def test_set_goal_pose_by_object_id_valid(self, pybullet_env, mock_sim_core):
        """Valid object_id sets goal on the correct agent."""
        mgr, agents = _spawn_agents(mock_sim_core, pybullet_env)
        target_agent = agents[2]
        goal = Pose.from_xyz(30, 0, 0.5)

        mgr.set_goal_pose_by_object_id(target_agent.object_id, goal)

        assert target_agent.is_moving
        assert not agents[0].is_moving

    def test_set_goal_pose_by_object_id_invalid(self, pybullet_env, mock_sim_core, caplog):
        """Invalid object_id logs a warning."""
        mgr, _ = _spawn_agents(mock_sim_core, pybullet_env)

        with caplog.at_level(logging.WARNING):
            mgr.set_goal_pose_by_object_id(9999, Pose.from_xyz(0, 0, 0))

        assert "Unknown agent object_id" in caplog.text

    # ------------------------------------------------------------------
    # set_goal_pose_all (factory)
    # ------------------------------------------------------------------
    def test_set_goal_pose_all(self, pybullet_env, mock_sim_core):
        """Factory function is called for every agent and each receives a goal."""
        mgr, agents = _spawn_agents(mock_sim_core, pybullet_env)
        called_agents = []

        def goal_factory(agent):
            called_agents.append(agent)
            return Pose.from_xyz(agent.body_id * 10, 0, 0.5)

        mgr.set_goal_pose_all(goal_factory)

        assert len(called_agents) == len(agents)
        assert set(called_agents) == set(agents)
        assert mgr.get_moving_count() == len(agents)

    # ------------------------------------------------------------------
    # stop_all
    # ------------------------------------------------------------------
    def test_stop_all(self, pybullet_env, mock_sim_core):
        """stop_all clears goals and sets moving count to 0."""
        mgr, agents = _spawn_agents(mock_sim_core, pybullet_env)
        # Give all agents a goal first
        for a in agents:
            a.set_goal_pose(Pose.from_xyz(50, 50, 0.5))
        assert mgr.get_moving_count() == len(agents)

        mgr.stop_all()

        assert mgr.get_moving_count() == 0
        assert all(not a.is_moving for a in agents)

    def test_stop_all_idempotent(self, pybullet_env, mock_sim_core):
        """Calling stop_all on already-stopped agents does not crash."""
        mgr, _ = _spawn_agents(mock_sim_core, pybullet_env)
        mgr.stop_all()  # All already stopped
        assert mgr.get_moving_count() == 0

    # ------------------------------------------------------------------
    # set_joints_targets_all (factory)
    # ------------------------------------------------------------------
    def test_set_joints_targets_all(self, pybullet_env, mock_sim_core):
        """Factory function is invoked for each agent with set_joints_targets."""
        mgr, agents = _spawn_agents(mock_sim_core, pybullet_env)
        called_agents = []

        def targets_factory(agent):
            called_agents.append(agent)
            return [0.0, 0.0]  # Targets (will be a no-op for mesh robots)

        # Should not raise even for non-URDF agents (set_joints_targets handles that)
        mgr.set_joints_targets_all(targets_factory, max_force=100.0)

        assert len(called_agents) == len(agents)
        assert set(called_agents) == set(agents)

    # ------------------------------------------------------------------
    # add_action_all (factory)
    # ------------------------------------------------------------------
    def test_add_action_all(self, pybullet_env, mock_sim_core):
        """Factory produces one action per agent; each is added to the queue."""
        mgr, agents = _spawn_agents(mock_sim_core, pybullet_env)
        called_agents = []

        def action_factory(agent):
            called_agents.append(agent)
            return WaitAction(duration=1.0)

        mgr.add_action_all(action_factory)

        assert len(called_agents) == len(agents)
        for agent in agents:
            assert agent.get_action_queue_size() == 1

    # ------------------------------------------------------------------
    # add_action_sequence_all (factory)
    # ------------------------------------------------------------------
    def test_add_action_sequence_all(self, pybullet_env, mock_sim_core):
        """Factory produces a list of actions per agent; all are enqueued."""
        mgr, agents = _spawn_agents(mock_sim_core, pybullet_env)
        called_agents = []

        def sequence_factory(agent):
            called_agents.append(agent)
            return [WaitAction(duration=1.0), WaitAction(duration=2.0)]

        mgr.add_action_sequence_all(sequence_factory)

        assert len(called_agents) == len(agents)
        for agent in agents:
            assert agent.get_action_queue_size() == 2

    # ------------------------------------------------------------------
    # register_callback
    # ------------------------------------------------------------------
    def test_register_callback_with_sim_core(self, pybullet_env, mock_sim_core):
        """Callback is stored and forwarded to sim_core."""
        mgr = AgentManager(sim_core=mock_sim_core, update_frequency=10.0)

        def my_callback(manager, dt):
            pass

        mgr.register_callback(my_callback, frequency=5.0)

        # Stored internally
        assert len(mgr._callbacks) == 1
        assert mgr._callbacks[0]["func"] is my_callback
        assert mgr._callbacks[0]["frequency"] == 5.0
        # Forwarded to sim_core
        assert len(mock_sim_core._registered_callbacks) == 1
        assert mock_sim_core._registered_callbacks[0]["frequency"] == 5.0

    def test_register_callback_default_frequency(self, pybullet_env, mock_sim_core):
        """Omitting frequency uses the manager's default update_frequency."""
        mgr = AgentManager(sim_core=mock_sim_core, update_frequency=20.0)

        mgr.register_callback(lambda m, dt: None)

        assert mgr._callbacks[0]["frequency"] == 20.0

    def test_register_callback_without_sim_core(self, pybullet_env, caplog):
        """Without sim_core, callback is stored but a warning is logged."""
        mgr = AgentManager(sim_core=None)

        with caplog.at_level(logging.WARNING):
            mgr.register_callback(lambda m, dt: None, frequency=5.0)

        assert len(mgr._callbacks) == 1
        assert "sim_core not set" in caplog.text

    def test_register_multiple_callbacks(self, pybullet_env, mock_sim_core):
        """Multiple callbacks can be registered independently."""
        mgr = AgentManager(sim_core=mock_sim_core)

        mgr.register_callback(lambda m, dt: None, frequency=1.0)
        mgr.register_callback(lambda m, dt: None, frequency=2.0)
        mgr.register_callback(lambda m, dt: None, frequency=3.0)

        assert len(mgr._callbacks) == 3
        assert [cb["frequency"] for cb in mgr._callbacks] == [1.0, 2.0, 3.0]

    # ------------------------------------------------------------------
    # __repr__
    # ------------------------------------------------------------------
    def test_repr(self, pybullet_env, mock_sim_core):
        """repr includes total and moving count."""
        mgr, agents = _spawn_agents(mock_sim_core, pybullet_env)
        agents[0].set_goal_pose(Pose.from_xyz(99, 0, 0.5))

        result = repr(mgr)

        assert "AgentManager" in result
        assert "total=4" in result
        assert "moving=1" in result


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
