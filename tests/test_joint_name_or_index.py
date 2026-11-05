"""The single-joint APIs take a joint by index or by name.

``set_joint_target``, ``get_joint_state`` and ``is_joint_at_target`` accept
``joint: Union[int, str]`` like ``set_joint_position`` and
``is_joint_moving`` do, so a joint is named the same way everywhere. The
``*_by_name`` variants remain as aliases. What each method does with a joint
the body does not have is unchanged and checked here for both spellings.
"""

import pybullet as p
import pytest

from pybullet_fleet import Agent, AgentSpawnParams, MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.tools import resolve_joint_index

ELEVATOR = "robots/elevator.urdf"  # one prismatic joint, "lift"
LIFT = "lift"
MANIPULATOR = "robots/mobile_manipulator.urdf"  # wheels, a fixed mount, and four arm joints
FIXED = "base_to_mount"
DT = 0.02


def _sim(physics=False):
    sim = MultiRobotSimulationCore(
        SimulationParams(gui=False, physics=physics, timestep=DT, monitor=False, enable_monitor_gui=False)
    )
    sim.initialize_simulation()
    return sim


@pytest.fixture
def sim_core():
    sim = _sim()
    yield sim
    try:
        p.disconnect(sim.client)
    except p.error:
        pass


def _lift(sim_core, **kwargs):
    return Agent.from_params(
        AgentSpawnParams(urdf_path=ELEVATOR, name="lift", use_fixed_base=True, **kwargs), sim_core=sim_core
    )


def _manipulator(sim_core):
    return Agent.from_params(AgentSpawnParams(urdf_path=MANIPULATOR, name="arm"), sim_core=sim_core)


def _run(sim_core, steps=200):
    for _ in range(steps):
        sim_core.step_once()


class TestResolveJoint:
    def test_a_name_resolves_to_its_index(self, sim_core):
        agent = _manipulator(sim_core)

        assert agent._resolve_joint("elbow_to_wrist") == agent._joint_index_by_name("elbow_to_wrist")

    def test_an_index_in_range_is_returned_as_is(self, sim_core):
        agent = _manipulator(sim_core)

        assert agent._resolve_joint(2) == 2

    def test_a_fixed_joint_resolves(self, sim_core):
        agent = _manipulator(sim_core)

        assert agent._resolve_joint(FIXED) == agent._joint_index_by_name(FIXED)

    @pytest.mark.parametrize("joint", ["no_such_joint", 99, -1])
    def test_an_unknown_joint_is_none_with_a_warning(self, sim_core, caplog, joint):
        agent = _manipulator(sim_core)

        with caplog.at_level("WARNING"):
            assert agent._resolve_joint(joint) is None

        assert str(joint) in caplog.text


class TestSetJointTarget:
    def test_a_name_moves_the_joint(self, sim_core):
        agent = _lift(sim_core)

        agent.set_joint_target(LIFT, 1.0)
        _run(sim_core)

        assert agent.get_joint_state(0)[0] == pytest.approx(1.0)
        assert agent.last_joint_targets == {0: 1.0}

    def test_an_index_still_moves_the_joint(self, sim_core):
        agent = _lift(sim_core)

        agent.set_joint_target(0, 1.0)

        assert agent.last_joint_targets == {0: 1.0}

    def test_a_name_drives_the_motor_with_physics(self):
        sim = _sim(physics=True)
        try:
            agent = _lift(sim, mass=1.0)
            assert not agent._use_kinematic_joints

            agent.set_joint_target(LIFT, 1.0)

            assert agent.last_joint_targets == {0: 1.0}
        finally:
            p.disconnect(sim.client)

    @pytest.mark.parametrize("joint", ["no_such_joint", 99, -1])
    def test_an_unknown_joint_is_ignored_with_a_warning(self, sim_core, caplog, joint):
        agent = _lift(sim_core)

        with caplog.at_level("WARNING"):
            assert agent.set_joint_target(joint, 1.0) is None

        assert agent.last_joint_targets == {}
        assert str(joint) in caplog.text

    @pytest.mark.parametrize("by_name", [True, False])
    def test_a_fixed_joint_is_skipped(self, sim_core, by_name):
        agent = _manipulator(sim_core)
        joint = FIXED if by_name else agent._joint_index_by_name(FIXED)

        agent.set_joint_target(joint, 0.5)

        assert agent._joint_index_by_name(FIXED) not in agent.last_joint_targets


class TestGetJointState:
    def test_a_name_reads_the_joint(self, sim_core):
        agent = _lift(sim_core)
        agent.set_joint_position(LIFT, 2.0)

        assert agent.get_joint_state(LIFT) == agent.get_joint_state(0) == pytest.approx((2.0, 0.0))

    def test_a_fixed_joint_can_be_read_by_name(self, sim_core):
        agent = _manipulator(sim_core)

        assert agent.get_joint_state(FIXED) == agent.get_joint_state(agent._joint_index_by_name(FIXED))

    def test_an_unknown_name_reads_zero_with_a_warning(self, sim_core, caplog):
        agent = _lift(sim_core)

        with caplog.at_level("WARNING"):
            assert agent.get_joint_state("no_such_joint") == (0.0, 0.0)

        assert "no_such_joint" in caplog.text

    @pytest.mark.parametrize("joint", [99, -1])
    def test_an_index_out_of_range_still_raises(self, sim_core, joint):
        agent = _lift(sim_core)

        with pytest.raises(p.error):
            agent.get_joint_state(joint)


class TestIsJointAtTarget:
    def test_a_name_checks_the_joint(self, sim_core):
        agent = _lift(sim_core)
        agent.set_joint_position(LIFT, 2.0)

        assert agent.is_joint_at_target(LIFT, 2.0)
        assert not agent.is_joint_at_target(LIFT, 1.0)

    def test_a_name_resolves_a_tolerance_keyed_by_name(self, sim_core):
        agent = _lift(sim_core)
        agent.set_joint_position(LIFT, 2.0)

        assert agent.is_joint_at_target(LIFT, 2.05, tolerance={LIFT: 0.1})
        assert not agent.is_joint_at_target(LIFT, 2.05, tolerance={LIFT: 0.01})

    @pytest.mark.parametrize("joint", ["no_such_joint", 99, -1])
    def test_an_unknown_joint_is_false_with_a_warning(self, sim_core, caplog, joint):
        agent = _lift(sim_core)

        with caplog.at_level("WARNING"):
            assert agent.is_joint_at_target(joint, 0.0) is False

        assert str(joint) in caplog.text


class TestByNameAliases:
    def test_set_joint_target_by_name(self, sim_core):
        agent = _lift(sim_core)

        agent.set_joint_target_by_name(LIFT, 1.0)

        assert agent.last_joint_targets == {0: 1.0}

    def test_set_joint_target_by_name_ignores_an_unknown_name(self, sim_core):
        agent = _lift(sim_core)

        assert agent.set_joint_target_by_name("no_such_joint", 1.0) is None

        assert agent.last_joint_targets == {}

    def test_get_joint_state_by_name(self, sim_core):
        agent = _lift(sim_core)
        agent.set_joint_position(LIFT, 2.0)

        assert agent.get_joint_state_by_name(LIFT) == pytest.approx((2.0, 0.0))
        assert agent.get_joint_state_by_name("no_such_joint") == (0.0, 0.0)

    def test_is_joint_at_target_by_name(self, sim_core):
        agent = _lift(sim_core)
        agent.set_joint_position(LIFT, 2.0)

        assert agent.is_joint_at_target_by_name(LIFT, 2.0)
        assert agent.is_joint_at_target_by_name("no_such_joint", 2.0) is False


class TestTheOldKeyword:
    """The first parameter used to be ``joint_index``; a call passing it by that keyword still works."""

    def test_set_joint_target(self, sim_core):
        agent = _lift(sim_core)
        agent.set_joint_target(joint_index=0, target_position=1.0)  # pyright: ignore[reportCallIssue]
        assert agent.last_joint_targets == {0: 1.0}

    def test_get_joint_state(self, sim_core):
        agent = _lift(sim_core)
        agent.set_joint_position(LIFT, 2.0)
        assert agent.get_joint_state(joint_index=0)[0] == pytest.approx(2.0)  # pyright: ignore[reportCallIssue]

    def test_is_joint_at_target(self, sim_core):
        agent = _lift(sim_core)
        agent.set_joint_position(LIFT, 2.0)
        assert agent.is_joint_at_target(joint_index=0, target=2.0)  # pyright: ignore[reportCallIssue]

    def test_the_new_keyword(self, sim_core):
        agent = _lift(sim_core)
        agent.set_joint_target(joint=LIFT, target_position=1.0)
        assert agent.last_joint_targets == {0: 1.0}

    @pytest.mark.parametrize(
        "call",
        [
            lambda agent: agent.set_joint_target(0, 1.0, joint_index=0),
            lambda agent: agent.set_joint_target(joint=0, joint_index=0, target_position=1.0),
            lambda agent: agent.get_joint_state(0, joint_index=0),
        ],
        ids=["positional", "keyword", "get_joint_state"],
    )
    def test_both_spellings_at_once_is_an_error(self, sim_core, call):
        agent = _lift(sim_core)
        with pytest.raises(TypeError, match="multiple values for argument 'joint'"):
            call(agent)


class TestANonDefaultClient:
    """A joint name resolves on the agent's own physics client.

    PyBullet's module-level calls default to the first client. With a second
    simulation running, a name looked up without the agent's client would be
    searched for on whatever body has the same id in the first one.
    """

    @pytest.fixture
    def two_sims(self):
        first = _sim()
        _manipulator(first)  # body 0 on the default client: no joint named "lift"
        second = _sim()
        assert second.client != first.client
        yield first, second
        for sim in (second, first):
            try:
                p.disconnect(sim.client)
            except p.error:
                pass

    def test_set_joint_target(self, two_sims):
        _, second = two_sims
        agent = _lift(second)

        agent.set_joint_target(LIFT, 1.0)

        assert agent.last_joint_targets == {0: 1.0}

    def test_get_joint_state(self, two_sims):
        _, second = two_sims
        agent = _lift(second)
        agent.set_joint_position(0, 2.0)

        assert agent.get_joint_state(LIFT)[0] == pytest.approx(2.0)

    def test_is_joint_at_target(self, two_sims):
        _, second = two_sims
        agent = _lift(second)
        agent.set_joint_position(0, 2.0)

        assert agent.is_joint_at_target(LIFT, 2.0)

    def test_set_joint_position(self, two_sims):
        _, second = two_sims
        agent = _lift(second)

        assert agent.set_joint_position(LIFT, 2.0)
        assert agent.get_joint_state(0)[0] == pytest.approx(2.0)


class TestMultiJointNamesOnANonDefaultClient:
    """The multi-joint ``*_by_name`` helpers and IK joint names resolve on the agent's own client.

    The first simulation holds an elevator at body 0, whose only joint is
    "lift": any arm joint name looked up there is missing.
    """

    ARM_JOINTS = ("mount_to_shoulder", "shoulder_to_elbow", "elbow_to_wrist", "wrist_to_end")

    @pytest.fixture
    def two_sims(self):
        first = _sim()
        _lift(first)  # body 0 on the default client: only a joint named "lift"
        second = _sim()
        assert second.client != first.client
        yield first, second
        for sim in (second, first):
            try:
                p.disconnect(sim.client)
            except p.error:
                pass

    def test_get_joints_state_by_name(self, two_sims):
        _, second = two_sims
        agent = _manipulator(second)
        agent.set_joint_position("elbow_to_wrist", 0.5)

        states = agent.get_joints_state_by_name(["elbow_to_wrist"])

        assert states["elbow_to_wrist"][0] == pytest.approx(0.5)

    def test_set_joints_targets_by_name(self, two_sims):
        _, second = two_sims
        agent = _manipulator(second)
        index = agent._joint_index_by_name("elbow_to_wrist")

        agent.set_joints_targets_by_name({"elbow_to_wrist": 0.5})

        assert agent.last_joint_targets == {index: 0.5}

    def test_are_joints_at_targets_by_name(self, two_sims):
        _, second = two_sims
        agent = _manipulator(second)
        agent.set_joint_position("elbow_to_wrist", 0.5)

        assert agent.are_joints_at_targets_by_name({"elbow_to_wrist": 0.5})

    def test_ik_joint_names(self, two_sims, caplog):
        from pybullet_fleet.agent import IKParams

        _, second = two_sims
        agent = Agent.from_params(
            AgentSpawnParams(urdf_path=MANIPULATOR, name="arm", ik_params=IKParams(ik_joint_names=self.ARM_JOINTS)),
            sim_core=second,
        )

        with caplog.at_level("WARNING"):
            agent._solve_ik([0.5, 0.0, 0.6], ee_link_index=agent._joint_index_by_name("wrist_to_end"))

        assert "not found" not in caplog.text


class TestResolveJointIndexClient:
    def test_looks_the_name_up_on_the_given_client(self):
        first = _sim()
        _lift(first)
        second = _sim()
        try:
            agent = _manipulator(second)

            index = resolve_joint_index(agent.body_id, "elbow_to_wrist", physics_client_id=second.client)

            assert index == agent._joint_index_by_name("elbow_to_wrist")
        finally:
            for sim in (second, first):
                try:
                    p.disconnect(sim.client)
                except p.error:
                    pass
