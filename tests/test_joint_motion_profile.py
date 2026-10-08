"""Kinematic joints can be given a speed and an acceleration ramp.

Without a profile a joint runs at the URDF's ``<limit velocity>`` and reaches
it instantly. The speed is therefore a property of the model file -- two
installations that differ only in how fast a cabin travels needed two URDFs,
and ``changeDynamics(maxJointVelocity=...)`` does not help because
``getJointInfo()`` keeps reporting the original. A real cabin, hoist or linear
axis also ramps rather than stepping to full speed.
"""

import math

import pytest

from pybullet_fleet import Agent, AgentSpawnParams, MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.action import JointAction
from pybullet_fleet.devices.elevator import Elevator, ElevatorParams

URDF = "robots/elevator.urdf"  # ships with pybullet_fleet: prismatic "lift", <limit velocity="2.0">
JOINT = "lift"
DT = 0.02
TRAVEL = 0.6


@pytest.fixture
def sim_core():
    import pybullet as p

    sim = MultiRobotSimulationCore(
        SimulationParams(gui=False, physics=False, timestep=DT, monitor=False, enable_monitor_gui=False)
    )
    sim.initialize_simulation()
    yield sim
    try:
        p.disconnect(sim.client)
    except p.error:
        pass


def _travel_time(sim_core, agent, distance=TRAVEL, limit=4000):
    agent.add_action(JointAction(target_joint_positions={JOINT: distance}))
    for step in range(limit):
        sim_core.step_once()
        if abs(agent.get_joint_state_by_name(JOINT)[0] - distance) < 1e-6:
            return (step + 1) * DT
    raise AssertionError("joint never reached its target")


class TestJointSpeedOverride:
    def test_without_a_profile_the_urdf_limit_applies(self, sim_core):
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)
        # 2.0 m/s from the URDF, so 0.6 m in 0.30 s, plus the step it lands on.
        assert _travel_time(sim_core, agent) == pytest.approx(0.30, abs=DT)

    def test_max_velocity_replaces_it(self, sim_core):
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)
        agent.set_joint_motion_profile(JOINT, max_velocity=0.4)
        assert _travel_time(sim_core, agent) == pytest.approx(TRAVEL / 0.4, abs=DT)

    def test_a_joint_can_be_named_or_indexed(self, sim_core):
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)
        agent.set_joint_motion_profile(JOINT, max_velocity=0.4)
        by_name = dict(agent._joint_motion_profiles)
        agent._joint_motion_profiles.clear()
        agent.set_joint_motion_profile(agent._joint_index_by_name(JOINT), max_velocity=0.4)
        assert agent._joint_motion_profiles == by_name

    def test_an_unknown_joint_is_refused(self, sim_core):
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)
        with pytest.raises(KeyError, match="no joint named"):
            agent.set_joint_motion_profile("nope", max_velocity=0.4)

    def test_non_positive_values_are_refused(self, sim_core):
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)
        for kwargs in ({"max_velocity": 0.0}, {"max_accel": -1.0}, {"max_decel": 0.0}):
            with pytest.raises(ValueError):
                agent.set_joint_motion_profile(JOINT, **kwargs)

    def test_non_finite_values_are_refused(self, sim_core):
        """NaN and inf both slip past `value <= 0.0`, and would then produce a
        nan or infinite step."""
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)
        for bad in (float("nan"), float("inf"), float("-inf")):
            for key in ("max_velocity", "max_accel", "max_decel"):
                with pytest.raises(ValueError, match="finite"):
                    agent.set_joint_motion_profile(JOINT, **{key: bad})


class TestAccelerationRamp:
    def test_a_ramp_takes_the_trapezoid_time(self, sim_core):
        """0.6 m at 0.4 m/s with 1.0 m/s^2 ramps: 0.4 s up, 0.4 s down over
        0.16 m, then 0.44 m of cruise -- 1.9 s against 1.5 s flat."""
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)
        agent.set_joint_motion_profile(JOINT, max_velocity=0.4, max_accel=1.0)
        assert _travel_time(sim_core, agent) == pytest.approx(1.9, abs=4 * DT)

    def test_a_short_travel_never_reaches_full_speed(self, sim_core):
        """Triangle, not trapezoid: 2 * sqrt(d / a) for d below the ramp
        distance."""
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)
        agent.set_joint_motion_profile(JOINT, max_velocity=2.0, max_accel=1.0)
        assert _travel_time(sim_core, agent, distance=0.1) == pytest.approx(2.0 * math.sqrt(0.1 / 1.0), abs=4 * DT)

    def test_an_asymmetric_ramp_is_slower_than_a_symmetric_one(self, sim_core):
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)
        agent.set_joint_motion_profile(JOINT, max_velocity=0.4, max_accel=1.0, max_decel=1.0)
        symmetric = _travel_time(sim_core, agent)

        agent.set_joint_motion_profile(JOINT, max_decel=0.25)
        agent.add_action(JointAction(target_joint_positions={JOINT: 0.0}))
        for _ in range(4000):
            sim_core.step_once()
            if abs(agent.get_joint_state_by_name(JOINT)[0]) < 1e-6:
                break
        asymmetric = _travel_time(sim_core, agent)
        assert asymmetric > symmetric

    def test_no_accel_keeps_the_old_instant_full_speed(self, sim_core):
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)
        agent.set_joint_motion_profile(JOINT, max_velocity=0.4)
        assert _travel_time(sim_core, agent) == pytest.approx(TRAVEL / 0.4, abs=DT)

    def test_a_target_replaced_mid_travel_still_arrives(self, sim_core):
        """The reason the trapezoid is integrated step by step rather than
        planned once: there is no plan to invalidate.

        The replacement goes through ``set_joint_target_by_name``, not a
        second ``JointAction``: ``add_action()`` appends to the queue, so a
        second action would wait for the first to finish rather than replace
        its target, and the test would not exercise this at all.
        """
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)
        agent.set_joint_motion_profile(JOINT, max_velocity=0.4, max_accel=1.0)
        index = agent._joint_index_by_name(JOINT)

        agent.add_action(JointAction(target_joint_positions={JOINT: 2.0}))
        for _ in range(40):
            sim_core.step_once()
        # Up to speed and still travelling, which is the state being replaced.
        assert agent.get_joint_state_by_name(JOINT)[0] > 0.0
        assert agent._joint_speeds.get(index, 0.0) > 0.0

        agent.clear_actions()
        agent.set_joint_target_by_name(JOINT, 0.3)
        carried = agent._joint_speeds.get(index, 0.0)
        assert abs(carried) > 0.0, "the speed is carried into the new travel, not reset"

        for _ in range(4000):
            sim_core.step_once()
            if abs(agent.get_joint_state_by_name(JOINT)[0] - 0.3) < 1e-6:
                break
        assert agent.get_joint_state_by_name(JOINT)[0] == pytest.approx(0.3, abs=1e-6)
        assert agent._joint_speeds.get(index) is None, "the speed is dropped on arrival"


class TestDirectionReversal:
    """Raised in review: an unsigned carried speed reversed instantaneously.

    With a magnitude alone, a target moved to the other side of the joint let
    the next step keep full speed and simply apply it the other way, skipping
    the deceleration the profile asks for.
    """

    def _moving_agent(self, sim_core):
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)
        agent.set_joint_motion_profile(JOINT, max_velocity=0.4, max_accel=1.0)
        agent.add_action(JointAction(target_joint_positions={JOINT: 2.0}))
        for _ in range(40):
            sim_core.step_once()
        return agent, agent._joint_index_by_name(JOINT)

    def test_the_carried_speed_is_signed(self, sim_core):
        agent, index = self._moving_agent(sim_core)
        assert agent._joint_speeds[index] > 0.0

        agent.clear_actions()
        agent.set_joint_target_by_name(JOINT, -1.0)
        sim_core.step_once()

        assert agent._joint_speeds[index] > 0.0, "still travelling the old way while braking"

    def test_it_brakes_through_zero_before_reversing(self, sim_core):
        agent, index = self._moving_agent(sim_core)
        before = agent.get_joint_state_by_name(JOINT)[0]
        speed = agent._joint_speeds[index]

        agent.clear_actions()
        agent.set_joint_target_by_name(JOINT, -1.0)

        # It must still be moving the old way for at least one step: braking
        # from `speed` at 1.0 m/s^2 takes `speed` seconds, many steps at DT.
        sim_core.step_once()
        assert agent.get_joint_state_by_name(JOINT)[0] > before

        peak = before
        steps_still_advancing = 0
        for _ in range(400):
            sim_core.step_once()
            position = agent.get_joint_state_by_name(JOINT)[0]
            if position > peak:
                peak, steps_still_advancing = position, steps_still_advancing + 1
            if agent._joint_speeds.get(index, 0.0) < 0.0:
                break

        # Braking from `speed` at 1.0 m/s^2 takes `speed` seconds, which is
        # many steps at DT -- so the joint travels well past where the new
        # target was issued before it turns around.
        assert peak > before
        assert steps_still_advancing > 1, "it reversed in a single step"
        assert speed / 1.0 > 2 * DT, "the brake really does span several steps"

    def test_it_still_reaches_the_new_target(self, sim_core):
        agent, index = self._moving_agent(sim_core)
        agent.clear_actions()
        agent.set_joint_target_by_name(JOINT, -1.0)

        # Run until the trajectory is finished rather than until the position
        # looks close: a tolerance on position can be met a step before the
        # trajectory ends, and then the carried speed is legitimately still
        # set.
        for _ in range(4000):
            sim_core.step_once()
            if agent._joint_speeds.get(index) is None:
                break

        assert agent.get_joint_state_by_name(JOINT)[0] == pytest.approx(-1.0, abs=1e-6)
        assert agent._joint_speeds.get(index) is None

    def test_a_reversal_takes_longer_than_the_same_move_from_rest(self, sim_core):
        """The braking distance is the difference, and it is what the old
        magnitude-only carry threw away."""
        agent, _ = self._moving_agent(sim_core)
        agent.clear_actions()
        agent.set_joint_target_by_name(JOINT, -1.0)
        moving = 0
        for _ in range(4000):
            sim_core.step_once()
            moving += 1
            if abs(agent.get_joint_state_by_name(JOINT)[0] + 1.0) < 1e-6:
                break

        rested = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="b", use_fixed_base=True), sim_core=sim_core)
        rested.set_joint_motion_profile(JOINT, max_velocity=0.4, max_accel=1.0)
        start = agent.get_joint_state_by_name(JOINT)[0]
        del start
        rested.set_joint_target_by_name(JOINT, -1.0)
        from_rest = 0
        for _ in range(4000):
            sim_core.step_once()
            from_rest += 1
            if abs(rested.get_joint_state_by_name(JOINT)[0] + 1.0) < 1e-6:
                break
        assert moving > from_rest


class TestCheckpointCarriesTheRampSpeed:
    """Raised in review: ramp speed is execution state, so a checkpoint taken
    mid-travel has to carry it or the restored agent takes a different path to
    the same target."""

    def _ramping_agent(self, sim_core):
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)
        agent.set_joint_motion_profile(JOINT, max_velocity=0.4, max_accel=1.0)
        agent.add_action(JointAction(target_joint_positions={JOINT: 2.0}))
        for _ in range(40):
            sim_core.step_once()
        return agent

    def test_capture_records_it(self, sim_core):
        agent = self._ramping_agent(sim_core)
        index = agent._joint_index_by_name(JOINT)
        captured = agent.capture_kinematic_joint_execution()
        assert captured[index]["speed"] == pytest.approx(agent._joint_speeds[index])

    def test_restore_puts_it_back(self, sim_core):
        agent = self._ramping_agent(sim_core)
        index = agent._joint_index_by_name(JOINT)
        captured = agent.capture_kinematic_joint_execution()
        speed = agent._joint_speeds[index]

        agent._joint_speeds.clear()
        agent.restore_kinematic_joint_execution(captured)

        assert agent._joint_speeds[index] == pytest.approx(speed)

    def test_a_checkpoint_without_the_field_still_restores(self, sim_core):
        """Written before ramps existed: that joint just starts from rest."""
        agent = self._ramping_agent(sim_core)
        captured = agent.capture_kinematic_joint_execution()
        for entry in captured:
            entry.pop("speed")

        agent.restore_kinematic_joint_execution(captured)

        assert agent._joint_speeds == {}

    def test_a_nonfinite_speed_is_refused(self, sim_core):
        agent = self._ramping_agent(sim_core)
        captured = agent.capture_kinematic_joint_execution()
        captured[agent._joint_index_by_name(JOINT)]["speed"] = float("nan")
        with pytest.raises(ValueError, match="nonfinite"):
            agent.restore_kinematic_joint_execution(captured)


class TestElevatorParamsMotion:
    def _elevator(self, sim_core, **motion):
        return Elevator.from_params(
            ElevatorParams(
                urdf_path=URDF,
                name="lift1",
                use_fixed_base=True,
                floors={"0": 0.0, "1": TRAVEL, "2": 2 * TRAVEL},
                initial_floor="0",
                **motion,
            ),
            sim_core=sim_core,
        )

    def _ride_time(self, sim_core, car, floor="1", limit=4000):
        car.request_floor(floor)
        for step in range(limit):
            sim_core.step_once()
            if car.current_floor == floor and not car.is_moving:
                return (step + 1) * DT
        raise AssertionError("the cabin never arrived")

    def test_without_motion_params_the_urdf_still_decides(self, sim_core):
        car = self._elevator(sim_core)
        assert self._ride_time(sim_core, car) == pytest.approx(TRAVEL / 2.0, abs=3 * DT)

    def test_max_speed_sets_the_cabin_speed(self, sim_core):
        car = self._elevator(sim_core, max_speed=0.4)
        assert self._ride_time(sim_core, car) == pytest.approx(TRAVEL / 0.4, abs=3 * DT)

    def test_max_accel_makes_the_ride_a_trapezoid(self, sim_core):
        """A ramped ride is measurably longer than a flat one.

        The arrival test here is the elevator's, which settles once the joint
        is inside JointAction's tolerance rather than exactly on target, so
        the ride reads a little short of the 1.9 s closed form -- which is why
        this compares the two rides rather than pinning a number.
        """
        flat = self._ride_time(sim_core, self._elevator(sim_core, max_speed=0.4))
        ramped = self._ride_time(sim_core, self._elevator(sim_core, max_speed=0.4, max_accel=1.0))
        assert flat == pytest.approx(1.5, abs=3 * DT)
        assert ramped > flat + 0.2
        assert ramped == pytest.approx(1.9, abs=0.15)

    def test_a_zero_is_refused_rather_than_ignored(self, sim_core):
        """Truthiness here would skip the profile for 0.0, silently falling
        back to the URDF instead of rejecting an invalid value."""
        for field in ("max_speed", "max_accel", "max_decel"):
            with pytest.raises(ValueError):
                self._elevator(sim_core, **{field: 0.0})

    def test_from_dict_carries_the_motion_fields(self):
        params = ElevatorParams.from_dict(
            {
                "name": "lift1",
                "urdf_path": URDF,
                "floors": {"0": 0.0, "1": TRAVEL},
                "initial_floor": "0",
                "max_speed": 0.4,
                "max_accel": 1.0,
                "max_decel": 0.5,
            }
        )
        assert (params.max_speed, params.max_accel, params.max_decel) == (0.4, 1.0, 0.5)
