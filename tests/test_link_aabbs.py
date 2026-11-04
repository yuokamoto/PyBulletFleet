"""The broadphase AABB of an articulated body covers all of its links.

``p.getAABB(body)`` returns the base link's box only. Caching that made every
child link of an arm invisible to the broadphase: a wrist touching a box never
reached the narrow phase, and ``query_aabb`` could not find the arm by its upper
links. The cached box is now the union of the link boxes, refreshed whenever a
joint moves, while a single-link body still costs one ``getAABB`` per update.
"""

import pybullet as p
import pytest

from pybullet_fleet import Agent, AgentSpawnParams, MultiRobotSimulationCore, Pose, SimObject, SimulationParams
from pybullet_fleet.sim_object import ShapeParams
from pybullet_fleet.types import CollisionMode

ARM = "robots/arm_robot.urdf"  # upright 4-joint arm: base box reaches z=0.05, end effector z=0.81
ELBOW = "shoulder_to_elbow"  # revolute about +x: a positive angle swings the upper arm towards -y
BENT = 1.57
DT = 0.02


def make_sim(physics: bool = False) -> MultiRobotSimulationCore:
    sim = MultiRobotSimulationCore(
        SimulationParams(gui=False, physics=physics, timestep=DT, monitor=False, log_level="warning")
    )
    sim.initialize_simulation()
    return sim


@pytest.fixture
def sim():
    sim = make_sim()
    yield sim
    p.disconnect(sim.client)


@pytest.fixture
def physics_sim():
    sim = make_sim(physics=True)
    yield sim
    p.disconnect(sim.client)


def arm(sim, mass=0.0):
    params = AgentSpawnParams(urdf_path=ARM, initial_pose=Pose.from_xyz(0.0, 0.0, 0.0), use_fixed_base=True, mass=mass)
    return Agent.from_params(params, sim_core=sim)


def box(sim, x, y, z, half=0.05, name="box"):
    shape = ShapeParams(shape_type="box", half_extents=[half, half, half])
    return SimObject.from_mesh(
        visual_shape=shape,
        collision_shape=shape,
        pose=Pose.from_xyz(x, y, z),
        sim_core=sim,
        collision_mode=CollisionMode.NORMAL_3D,
        name=name,
    )


def whole_body_aabb(sim, obj):
    """Union of every link's PyBullet AABB, computed independently of the sim core."""
    boxes = [p.getAABB(obj.body_id, link, physicsClientId=sim.client) for link in range(-1, p.getNumJoints(obj.body_id))]
    lo = tuple(min(b[0][k] for b in boxes) for k in range(3))
    hi = tuple(max(b[1][k] for b in boxes) for k in range(3))
    return lo, hi


def assert_box_equal(actual, expected):
    assert tuple(actual[0]) == pytest.approx(expected[0], abs=1e-9)
    assert tuple(actual[1]) == pytest.approx(expected[1], abs=1e-9)


def pair(a, b):
    return tuple(sorted((a.object_id, b.object_id)))


class TestCachedBoxCoversEveryLink:
    def test_cached_box_reaches_the_end_effector(self, sim):
        robot = arm(sim)

        cached = sim._cached_aabbs_dict[robot.object_id]

        assert cached[1][2] == pytest.approx(0.81, abs=1e-3)
        assert_box_equal(cached, whole_body_aabb(sim, robot))

    def test_full_rebuild_covers_every_link(self, sim):
        robot = arm(sim)
        sim._aabb_cache_valid = False

        sim.filter_aabb_pairs()

        assert_box_equal(sim._cached_aabbs_dict[robot.object_id], whole_body_aabb(sim, robot))

    def test_get_aabbs_covers_every_link(self, sim):
        robot = arm(sim)

        (aabb,) = sim.get_aabbs()

        assert_box_equal(aabb, whole_body_aabb(sim, robot))

    def test_collision_through_the_end_effector_is_found(self, sim):
        robot = arm(sim)
        target = box(sim, 0.0, 0.0, 0.78)  # touches only the end-effector sphere

        sim.step_once()

        assert pair(robot, target) in sim.get_active_collision_pairs()

    def test_query_aabb_finds_the_arm_by_its_upper_link(self, sim):
        robot = arm(sim)
        sim.filter_aabb_pairs()  # build the spatial hash

        found = sim.query_aabb((-0.1, -0.1, 0.7), (0.1, 0.1, 0.8))

        assert robot in found


class TestBoxFollowsTheJoints:
    def test_set_joint_position_refreshes_the_box_at_once(self, sim):
        robot = arm(sim)

        robot.set_joint_position(ELBOW, BENT)

        cached = sim._cached_aabbs_dict[robot.object_id]
        assert cached[0][1] < -0.4  # the upper arm now points towards -y
        assert_box_equal(cached, whole_body_aabb(sim, robot))

    def test_set_joints_positions_refreshes_the_box_at_once(self, sim):
        robot = arm(sim)

        robot.set_joints_positions({ELBOW: BENT})

        assert_box_equal(sim._cached_aabbs_dict[robot.object_id], whole_body_aabb(sim, robot))

    def test_placed_joint_reaches_a_box_and_collides(self, sim):
        robot = arm(sim)
        target = box(sim, 0.0, -0.42, 0.35)
        sim.step_once()
        assert pair(robot, target) not in sim.get_active_collision_pairs()

        robot.set_joint_position(ELBOW, BENT)
        sim.step_once()

        assert pair(robot, target) in sim.get_active_collision_pairs()

    def test_query_aabb_finds_the_arm_where_a_placed_joint_put_it(self, sim):
        robot = arm(sim)
        sim.filter_aabb_pairs()

        robot.set_joint_position(ELBOW, BENT)

        assert robot in sim.query_aabb((-0.05, -0.45, 0.3), (0.05, -0.35, 0.4))

    def test_kinematic_joint_motion_refreshes_the_box(self, sim):
        robot = arm(sim)
        target = box(sim, 0.0, -0.42, 0.35)
        robot.set_joint_target_by_name(ELBOW, BENT)

        for _ in range(60):  # 1.57 rad at the URDF's 1.8 rad/s takes 44 steps
            sim.step_once()
            assert_box_equal(sim._cached_aabbs_dict[robot.object_id], whole_body_aabb(sim, robot))

        assert robot.get_joint_state_by_name(ELBOW)[0] == pytest.approx(BENT, abs=1e-3)
        assert pair(robot, target) in sim.get_active_collision_pairs()

    def test_joint_placed_from_a_step_callback_is_refreshed_by_the_step(self, sim):
        robot = arm(sim)
        target = box(sim, 0.0, -0.42, 0.35)
        sim.step_once()  # the first check rebuilds every AABB; later ones rely on the refresh
        sim.register_callback(lambda *a, **k: robot.set_joint_position(ELBOW, BENT), frequency=None)

        sim.step_once()

        assert_box_equal(sim._cached_aabbs_dict[robot.object_id], whole_body_aabb(sim, robot))
        assert pair(robot, target) in sim.get_active_collision_pairs()

    def test_query_aabb_inside_a_step_sees_a_joint_placed_in_that_step(self, sim):
        robot = arm(sim)
        sim.filter_aabb_pairs()
        seen = []

        def callback(*_a, **_k):
            robot.set_joint_position(ELBOW, BENT)
            seen.append(robot in sim.query_aabb((-0.05, -0.45, 0.3), (0.05, -0.35, 0.4)))

        sim.register_callback(callback, frequency=None)
        sim.step_once()

        assert seen == [True]

    def test_restoring_a_joint_checkpoint_refreshes_the_box(self, sim):
        robot = arm(sim)
        robot.set_joint_position(ELBOW, BENT)
        checkpoint = robot.capture_kinematic_joint_execution()
        robot.set_joint_position(ELBOW, 0.0)

        robot.restore_kinematic_joint_execution(checkpoint)

        cached = sim._cached_aabbs_dict[robot.object_id]
        assert cached[0][1] < -0.4
        assert_box_equal(cached, whole_body_aabb(sim, robot))

    def test_physics_joint_motion_refreshes_the_box(self, physics_sim):
        robot = arm(physics_sim, mass=None)
        assert robot.object_id in physics_sim._physics_objects
        robot.set_joint_target_by_name(ELBOW, BENT)

        for _ in range(100):
            physics_sim.step_once()

        cached = physics_sim._cached_aabbs_dict[robot.object_id]
        assert cached[0][1] < -0.3
        assert_box_equal(cached, whole_body_aabb(physics_sim, robot))


class TestSingleLinkCost:
    def test_one_get_aabb_call_per_update(self, sim, monkeypatch):
        cube = box(sim, 0.0, 0.0, 0.0, name="cube")
        calls = {"getAABB": 0, "getNumJoints": 0}
        real_get_aabb, real_num_joints = p.getAABB, p.getNumJoints

        def get_aabb(*args, **kwargs):
            calls["getAABB"] += 1
            return real_get_aabb(*args, **kwargs)

        def num_joints(*args, **kwargs):
            calls["getNumJoints"] += 1
            return real_num_joints(*args, **kwargs)

        monkeypatch.setattr(p, "getAABB", get_aabb)
        monkeypatch.setattr(p, "getNumJoints", num_joints)

        for i in range(1, 11):
            cube.set_pose(Pose.from_xyz(0.1 * i, 0.0, 0.0))

        assert calls == {"getAABB": 10, "getNumJoints": 0}
