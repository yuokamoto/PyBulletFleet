"""Per-axis collision clearance (#105).

``collision_margin`` takes a scalar, as before, or an ``(x, y, z)`` triple in
world axes. With ``CLOSEST_POINTS`` (and the kinematic pairs of ``HYBRID``) a
pair is reported when the bodies touch, or when the gap between their closest
points is within the largest component and, along each world axis, within that
axis's clearance.
"""

import itertools
import math
import random

import pybullet as p
import pytest

from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import ShapeParams, SimObject
from pybullet_fleet.types import CollisionDetectionMethod, CollisionMode, SpatialHashCellSizeMode

XY_ONLY = (0.05, 0.05, 0.0)


def _sim(margin, method=CollisionDetectionMethod.CLOSEST_POINTS, cell=1.0):
    return MultiRobotSimulationCore(
        SimulationParams(
            gui=False,
            physics=False,
            monitor=False,
            collision_detection_method=method,
            collision_margin=margin,
            spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
            spatial_hash_cell_size=cell,
            ignore_static_collision=False,
            log_level="warning",
        )
    )


def _shape(kind, size):
    if kind == "sphere":
        return ShapeParams(shape_type="sphere", radius=size)
    return ShapeParams(shape_type="box", half_extents=list(size))


def _body(sim, xyz, size=(0.1, 0.1, 0.1), kind="box", mode=CollisionMode.NORMAL_3D):
    return SimObject.from_mesh(
        visual_shape=_shape(kind, size),
        collision_shape=_shape(kind, size),
        pose=Pose.from_xyz(*xyz),
        mass=0.0,
        sim_core=sim,
        collision_mode=mode,
    )


def _pair(a, b):
    return (min(a.object_id, b.object_id), max(a.object_id, b.object_id))


@pytest.fixture
def make_sim():
    sims = []

    def make(*args, **kwargs):
        sims.append(_sim(*args, **kwargs))
        return sims[-1]

    yield make
    for sim in sims:
        p.disconnect(sim.client)


# --- Configuration ---------------------------------------------------------


@pytest.mark.parametrize("value", [(0.05, 0.05, 0.0), [0.05, 0.05, 0]])
def test_params_accept_a_triple(value):
    params = SimulationParams(gui=False, monitor=False, collision_margin=value)
    assert params.collision_margin == (0.05, 0.05, 0.0)
    assert isinstance(params.collision_margin, tuple)


def test_params_keep_a_scalar_as_it_is():
    params = SimulationParams(gui=False, monitor=False, collision_margin=0.03)
    assert params.collision_margin == 0.03
    assert isinstance(params.collision_margin, float)


@pytest.mark.parametrize("value", [(0.05, 0.05), (0.1, 0.1, 0.1, 0.1), (0.05, -0.01, 0.0), (0.05, math.inf, 0.0)])
def test_params_reject_a_bad_triple(value):
    with pytest.raises(ValueError, match="collision_margin"):
        SimulationParams(gui=False, monitor=False, collision_margin=value)


def test_from_dict_reads_a_list(tmp_path):
    assert SimulationParams.from_dict({"collision_margin": [0.05, 0.05, 0.0]}).collision_margin == XY_ONLY
    config = tmp_path / "sim.yaml"
    config.write_text("simulation:\n  collision_margin: [0.05, 0.05, 0.0]\n")
    assert SimulationParams.from_config(str(config)).collision_margin == XY_ONLY


# --- Narrow and broad phase -------------------------------------------------


@pytest.mark.parametrize("method", [CollisionDetectionMethod.CLOSEST_POINTS, CollisionDetectionMethod.HYBRID])
def test_side_by_side_within_the_xy_clearance_is_reported(make_sim, method):
    sim = make_sim(XY_ONLY, method=method)
    a = _body(sim, (0.0, 0.0, 0.1))
    b = _body(sim, (0.23, 0.0, 0.1))  # 0.03 apart in x
    sim.check_collisions()
    assert sim.get_active_collision_pairs() == [_pair(a, b)]


@pytest.mark.parametrize("method", [CollisionDetectionMethod.CLOSEST_POINTS, CollisionDetectionMethod.HYBRID])
def test_under_a_shelf_within_the_scalar_clearance_is_not_reported_with_no_z_clearance(make_sim, method):
    scalar = make_sim(0.05, method=method)
    _body(scalar, (0.0, 0.0, 0.1))
    _body(scalar, (0.05, 0.0, 0.33), size=(0.3, 0.3, 0.1))  # 0.03 above
    scalar.check_collisions()
    assert len(scalar.get_active_collision_pairs()) == 1

    sim = make_sim(XY_ONLY, method=method)
    a = _body(sim, (0.0, 0.0, 0.1))
    shelf = _body(sim, (0.05, 0.0, 0.33), size=(0.3, 0.3, 0.1))
    sim._moved_this_step = {a.object_id, shelf.object_id}
    assert sim.filter_aabb_pairs()[0] == []
    sim.check_collisions()
    assert sim.get_active_collision_pairs() == []


def test_touching_is_reported_whatever_the_clearance(make_sim):
    sim = make_sim((0.0, 0.0, 0.0))
    a = _body(sim, (0.0, 0.0, 0.1))
    b = _body(sim, (0.15, 0.05, 0.15))  # overlapping in every axis
    sim.check_collisions()
    assert sim.get_active_collision_pairs() == [_pair(a, b)]


def test_each_axis_has_its_own_clearance(make_sim):
    """Boxes 0.03 apart in x only: within (0.05, 0.02, 0) but not (0.02, 0.05, 0.05)."""
    wide_x = make_sim((0.05, 0.02, 0.0))
    a, b = _body(wide_x, (0.0, 0.0, 0.1)), _body(wide_x, (0.23, 0.0, 0.1))
    wide_x.check_collisions()
    assert wide_x.get_active_collision_pairs() == [_pair(a, b)]

    narrow_x = make_sim((0.02, 0.05, 0.05))
    _body(narrow_x, (0.0, 0.0, 0.1)), _body(narrow_x, (0.23, 0.0, 0.1))
    narrow_x.check_collisions()
    assert narrow_x.get_active_collision_pairs() == []


def test_the_narrow_phase_filters_by_the_closest_points_separation(make_sim):
    """Spheres whose AABBs overlap in z but whose closest points are 1.5 mm
    apart in z: the broad phase passes them, the z clearance decides."""
    for z_clearance, expected in [(0.0, False), (0.002, True)]:
        sim = make_sim((0.05, 0.05, z_clearance))
        a = _body(sim, (0.0, 0.0, 0.2), size=0.1, kind="sphere")
        b = _body(sim, (0.2, 0.0, 0.25), size=0.1, kind="sphere")
        sim._moved_this_step = {a.object_id, b.object_id}
        assert sim.filter_aabb_pairs()[0] == [_pair(a, b)]
        sim.check_collisions()
        assert (sim.get_active_collision_pairs() == [_pair(a, b)]) is expected


def test_upright_cylinders_side_by_side_are_level(make_sim):
    """PyBullet puts a hair of z (about 1e-7 m) between the closest points of
    two upright cylinders at different heights; a zero z clearance still
    counts them as level."""
    sim = make_sim(XY_ONLY)
    shape = ShapeParams(shape_type="cylinder", radius=0.25, height=0.3)
    a = SimObject.from_mesh(visual_shape=shape, collision_shape=shape, pose=Pose.from_xyz(0, 0, 0.15), sim_core=sim)
    b = SimObject.from_mesh(visual_shape=shape, collision_shape=shape, pose=Pose.from_xyz(0.53, 0.1, 0.25), sim_core=sim)
    sim.check_collisions()
    assert sim.get_active_collision_pairs() == [_pair(a, b)]


def test_contact_points_ignore_the_triple(make_sim):
    sim = make_sim((0.5, 0.5, 0.5), method=CollisionDetectionMethod.CONTACT_POINTS)
    a, b = _body(sim, (0.0, 0.0, 0.1)), _body(sim, (0.23, 0.0, 0.1))
    sim._moved_this_step = {a.object_id, b.object_id}
    assert sim.filter_aabb_pairs()[0] == []


def test_observation_reports_the_configured_triple(make_sim):
    sim = make_sim(XY_ONLY)
    _body(sim, (0.0, 0.0, 0.1)), _body(sim, (0.23, 0.0, 0.1))
    sim.check_collisions()
    observation = sim.get_collision_observation()
    assert observation is not None and observation.margin == XY_ONLY
    assert observation.pairs[0].signed_distance == pytest.approx(0.03, abs=1e-4)


@pytest.mark.parametrize("seed", range(3))
def test_an_equal_triple_behaves_as_the_scalar(seed):
    """(m, m, m) finds the same pairs, candidates and signed distances as m."""
    results = []
    for margin in (0.05, (0.05, 0.05, 0.05)):
        rng = random.Random(seed)
        sim = _sim(margin, cell=0.5)
        try:
            objects = []
            for _ in range(40):
                xyz = [rng.uniform(0.0, 1.5) for _ in range(3)]
                kind = rng.choice(["box", "sphere"])
                size = rng.uniform(0.03, 0.1) if kind == "sphere" else [rng.uniform(0.03, 0.1) for _ in range(3)]
                objects.append(_body(sim, xyz, size=size, kind=kind))
            sim._moved_this_step = {o.object_id for o in objects}
            candidates = sorted(sim.filter_aabb_pairs()[0])
            sim._moved_this_step = {o.object_id for o in objects}
            sim.check_collisions()
            observation = sim.get_collision_observation()
            assert observation is not None
            results.append((candidates, [(o.object_ids, o.signed_distance) for o in observation.pairs]))
        finally:
            p.disconnect(sim.client)
    assert results[0] == results[1]
    assert results[0][1]  # the scene has near misses


@pytest.mark.parametrize("seed", range(4))
@pytest.mark.parametrize("margin", [(0.1, 0.02, 0.0), (0.0, 0.0, 0.25), (0.02, 0.1, 0.05)])
def test_broadphase_returns_every_pair_within_the_per_axis_clearance(seed, margin):
    """Against brute force: boxes on and off cell boundaries; a pair whose
    AABBs, each axis enlarged by that axis's clearance, overlap is a
    candidate, and one whose do not is not."""
    rng = random.Random(seed)
    cell = rng.choice([0.2, 0.5, 1.0])
    sim = _sim(margin, cell=cell)
    try:
        sim.set_collision_spatial_hash_cell_size_mode()
        objects = []
        for _ in range(30):
            half = [0.5 * cell * rng.choice([0.04, 0.1, 0.9, 1.4, 2.8]) for _ in range(3)]
            offsets = [0.0, 0.001, -0.001, rng.uniform(-cell, cell)]
            xyz = [round(rng.uniform(0.0, 3.0) / cell) * cell + rng.choice(offsets) for _ in range(3)]
            objects.append(_body(sim, xyz, size=half))
        sim._moved_this_step = {o.object_id for o in objects}
        found = set(sim.filter_aabb_pairs()[0])
        aabbs = sim._cached_aabbs_dict
        expected = set()
        for a, b in itertools.combinations([o.object_id for o in objects], 2):
            box_a, box_b = aabbs[a], aabbs[b]
            if all(box_a[1][d] + margin[d] >= box_b[0][d] and box_a[0][d] - margin[d] <= box_b[1][d] for d in range(3)):
                expected.add((min(a, b), max(a, b)))
        assert expected and found == expected
    finally:
        p.disconnect(sim.client)


# --- Changing the clearance -------------------------------------------------


@pytest.mark.parametrize("change", ["setter", "params"])
def test_a_clearance_change_rechecks_a_stationary_pair(make_sim, change):
    """Two STATIC bodies 0.03 apart in z: reported with 0.05, resolved when the
    z clearance drops to 0, reported again when it is restored."""
    sim = make_sim(0.05)
    a = _body(sim, (0.0, 0.0, 0.1), mode=CollisionMode.STATIC)
    shelf = _body(sim, (0.05, 0.0, 0.33), size=(0.3, 0.3, 0.1), mode=CollisionMode.STATIC)
    sim._moved_this_step = {a.object_id, shelf.object_id}
    sim.check_collisions()
    assert sim.get_active_collision_pairs() == [_pair(a, shelf)]

    def set_margin(value):
        if change == "setter":
            sim.set_collision_margin(value)
        else:
            sim.params.collision_margin = value

    set_margin(XY_ONLY)
    sim.check_collisions()
    assert sim.get_active_collision_pairs() == []

    set_margin((0.05, 0.05, 0.05))
    sim.check_collisions()
    assert sim.get_active_collision_pairs() == [_pair(a, shelf)]


def test_set_collision_margin_stores_a_tuple(make_sim):
    sim = make_sim(0.02)
    sim.set_collision_margin([0.05, 0.05, 0])
    assert sim.params.collision_margin == XY_ONLY


@pytest.mark.parametrize("value", [(0.05, 0.05), (0.05, -0.01, 0.0), (0.05, math.nan, 0.0)])
def test_set_collision_margin_rejects_a_bad_triple(make_sim, value):
    sim = make_sim(0.02)
    with pytest.raises(ValueError):
        sim.set_collision_margin(value)
    assert sim.params.collision_margin == 0.02


# --- Against NORMAL_2D ------------------------------------------------------


def _active_pairs_in_scene(seed, margin, mode):
    """Active pairs among 40 random upright boxes, all of ``mode``, by spawn index."""
    rng = random.Random(seed)
    sim = _sim(margin, cell=0.5)
    try:
        objects = [
            _body(sim, [rng.uniform(0.0, 1.2) for _ in range(3)], size=[rng.uniform(0.03, 0.12) for _ in range(3)], mode=mode)
            for _ in range(40)
        ]
        index = {o.object_id: k for k, o in enumerate(objects)}
        sim.check_collisions()
        return sorted(tuple(sorted((index[a], index[b]))) for a, b in sim.get_active_collision_pairs())
    finally:
        p.disconnect(sim.client)


@pytest.mark.filterwarnings("ignore:CollisionMode.NORMAL_2D is deprecated:FutureWarning")
@pytest.mark.parametrize("seed", range(3))
def test_a_zero_z_clearance_matches_normal_2d_for_upright_boxes(seed):
    """Every body NORMAL_2D with margin m reports what every body NORMAL_3D
    with (m, m, 0) does when the near misses are level, as between upright
    boxes; and fewer pairs than NORMAL_3D with m, which also reports the ones
    above or below."""
    normal_2d = _active_pairs_in_scene(seed, 0.05, CollisionMode.NORMAL_2D)
    per_axis = _active_pairs_in_scene(seed, (0.05, 0.05, 0.0), CollisionMode.NORMAL_3D)
    normal_3d = _active_pairs_in_scene(seed, 0.05, CollisionMode.NORMAL_3D)
    assert normal_2d == per_axis
    assert set(per_axis) < set(normal_3d)
