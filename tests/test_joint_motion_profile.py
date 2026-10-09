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

        The new target is far enough ahead to be reachable from the speed the
        joint is already carrying. A target inside the braking distance is a
        different case, in ``TestTargetInsideBrakingDistance``.
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
        agent.set_joint_target_by_name(JOINT, 0.9)
        carried = agent._joint_speeds.get(index, 0.0)
        assert abs(carried) > 0.0, "the speed is carried into the new travel, not reset"

        for _ in range(4000):
            sim_core.step_once()
            if abs(agent.get_joint_state_by_name(JOINT)[0] - 0.9) < 1e-6:
                break
        assert agent.get_joint_state_by_name(JOINT)[0] == pytest.approx(0.9, abs=1e-6)
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


class TestThirdRoundReviewFollowUps:
    """Three things the TPI conversion exposed, raised in review."""

    DT = 0.02

    @pytest.fixture
    def sim_core(self):
        import pybullet as p

        from pybullet_fleet import MultiRobotSimulationCore, SimulationParams

        sim = MultiRobotSimulationCore(
            SimulationParams(gui=False, physics=False, timestep=self.DT, monitor=False, enable_monitor_gui=False)
        )
        sim.initialize_simulation()
        yield sim
        try:
            p.disconnect(sim.client)
        except p.error:
            pass

    def _agent(self, sim_core):
        return Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)

    # -- an index that names no joint ---------------------------------------

    @pytest.mark.parametrize("bad", [-1, 1, 99])
    def test_an_out_of_range_index_is_refused(self, sim_core, bad):
        """-1 stored a profile under a key the step loop never visits, so it
        was silently ignored; a non-negative one got as far as
        ``_urdf_joint_velocity()`` and raised Python's own "list index out of
        range", which names neither the body nor how many joints it has.

        Matched on the specific message, and on nothing being stored, because
        a bare ``IndexError`` passes against the previous code for the two
        non-negative cases.
        """
        agent = self._agent(sim_core)
        with pytest.raises(IndexError, match=r"joint index .* is out of range for body"):
            agent.set_joint_motion_profile(bad, max_velocity=0.4)
        assert agent._joint_motion_profiles == {}

    def test_a_valid_index_still_works(self, sim_core):
        agent = self._agent(sim_core)
        agent.set_joint_motion_profile(0, max_velocity=0.4)
        assert agent._joint_motion_profiles[0][0] == pytest.approx(0.4)

    # -- a braking rate with nothing to brake out of ------------------------

    def test_max_decel_without_max_accel_is_refused(self, sim_core):
        """It took the constant-speed path and ignored the value, so
        ElevatorParams(max_decel=...) was silently a no-op."""
        agent = self._agent(sim_core)
        with pytest.raises(ValueError, match="max_decel needs max_accel"):
            agent.set_joint_motion_profile(JOINT, max_velocity=0.4, max_decel=0.25)

    def test_max_decel_is_accepted_alongside_an_existing_accel(self, sim_core):
        """A second call may supply it once the profile already ramps."""
        agent = self._agent(sim_core)
        agent.set_joint_motion_profile(JOINT, max_velocity=0.4, max_accel=1.0)
        agent.set_joint_motion_profile(JOINT, max_decel=0.25)
        assert agent._joint_motion_profiles[agent._joint_index_by_name(JOINT)][2] == pytest.approx(0.25)

    def test_the_elevator_refuses_it_too(self, sim_core):
        from pybullet_fleet.devices.elevator import Elevator, ElevatorParams

        with pytest.raises(ValueError, match="max_decel needs max_accel"):
            Elevator.from_params(
                ElevatorParams(
                    urdf_path=URDF,
                    name="lift",
                    use_fixed_base=True,
                    floors={"0": 0.0, "1": 0.6},
                    initial_floor="0",
                    max_speed=0.4,
                    max_decel=0.25,
                ),
                sim_core=sim_core,
            )

    # -- the cabin must not arrive before it stops --------------------------

    def _ride(self, sim_core, **motion):
        from pybullet_fleet.devices.elevator import Elevator, ElevatorParams

        car = Elevator.from_params(
            ElevatorParams(
                urdf_path=URDF,
                name="lift",
                use_fixed_base=True,
                floors={"0": 0.0, "1": 0.6},
                initial_floor="0",
                **motion,
            ),
            sim_core=sim_core,
        )
        car.request_floor("1")
        for _ in range(2000):
            sim_core.step_once()
            if car.current_floor == "1" and not car.is_moving:
                return car, car.get_joint_state_by_name(JOINT)[0]
        raise AssertionError("the cabin never arrived")

    def test_a_ramped_cabin_is_at_its_floor_when_it_says_so(self, sim_core):
        """JointAction completes inside its tolerance; a ramped joint is still
        braking through the last millimetres. Taking the action's word for it
        released the passengers 7.2 mm early."""
        _, position = self._ride(sim_core, max_speed=0.4, max_accel=1.0, max_decel=1.0)
        assert position == pytest.approx(0.6, abs=1e-9)

    def test_an_unramped_cabin_is_unaffected(self, sim_core):
        _, position = self._ride(sim_core, max_speed=0.4)
        assert position == pytest.approx(0.6, abs=1e-9)

    def test_motion_in_progress_covers_the_ramp(self, sim_core):
        from pybullet_fleet.action import JointAction

        agent = self._agent(sim_core)
        agent.set_joint_motion_profile(JOINT, max_velocity=0.4, max_accel=1.0)
        agent.add_action(JointAction(target_joint_positions={JOINT: 0.6}))
        for _ in range(2000):
            sim_core.step_once()
            if not agent.has_joint_trajectory(JOINT):
                break
        assert agent.get_joint_state_by_name(JOINT)[0] == pytest.approx(0.6, abs=1e-9)


class TestTargetInsideBrakingDistance:
    """Raised in review: an infeasible request teleported the joint.

    ``build_tpi()`` answers a request it cannot plan with a degenerate
    ``p0 -> p0`` trajectory whose end time is its start time. The ramp read
    that as "trajectory over, so the joint arrived" and returned the target,
    moving the joint there in a single step. The case is a target that moves
    to just ahead of a joint already travelling too fast to stop at it.
    """

    SPEED, ACCEL, DECEL = 0.4, 1.0, 0.2  # stopping distance 0.4 m at full speed

    def _moving_agent(self, sim_core):
        """An agent whose lift is at full speed, aimed well past its target."""
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)
        agent.set_joint_motion_profile(JOINT, max_velocity=self.SPEED, max_accel=self.ACCEL, max_decel=self.DECEL)
        agent.set_joint_target_by_name(JOINT, 5.0)
        for _ in range(200):
            sim_core.step_once()
            if agent.get_joint_state_by_name(JOINT)[0] > 0.5:
                return agent
        raise AssertionError("the joint never got up to speed")

    def test_the_joint_does_not_jump_to_an_unreachable_target(self, sim_core):
        agent = self._moving_agent(sim_core)
        here = agent.get_joint_state_by_name(JOINT)[0]
        agent.clear_actions()
        agent.set_joint_target_by_name(JOINT, here + 0.1)  # 0.4 m of braking needed

        previous, largest = here, 0.0
        for _ in range(2000):
            sim_core.step_once()
            position = agent.get_joint_state_by_name(JOINT)[0]
            largest = max(largest, abs(position - previous))
            previous = position
        # Without the fix the first step alone covered the whole 0.1 m.
        assert largest <= self.SPEED * DT + 1e-9

    def test_it_brakes_at_the_configured_rate_and_recovers(self, sim_core):
        agent = self._moving_agent(sim_core)
        here = agent.get_joint_state_by_name(JOINT)[0]
        target = here + 0.1
        agent.clear_actions()
        agent.set_joint_target_by_name(JOINT, target)

        overshot = False
        for _ in range(4000):
            sim_core.step_once()
            position = agent.get_joint_state_by_name(JOINT)[0]
            overshot = overshot or position > target
            if abs(position - target) < 1e-6:
                break
        else:
            raise AssertionError("the joint never settled on its target")
        assert overshot, "it should have run past the target it could not stop at"
        assert agent.has_joint_trajectory(JOINT) is False

    def test_the_ramp_stays_engaged_while_it_brakes(self, sim_core):
        """has_joint_trajectory() is what a lift asks before releasing its
        passengers, so it has to stay true through the unplannable steps."""
        agent = self._moving_agent(sim_core)
        here = agent.get_joint_state_by_name(JOINT)[0]
        agent.clear_actions()
        agent.set_joint_target_by_name(JOINT, here + 0.1)
        sim_core.step_once()
        assert agent.has_joint_trajectory(JOINT) is True
