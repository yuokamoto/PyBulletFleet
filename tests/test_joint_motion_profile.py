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
        assert _travel_time(sim_core, agent, distance=0.1) == pytest.approx(
            2.0 * math.sqrt(0.1 / 1.0), abs=4 * DT
        )

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
        planned once: there is no plan to invalidate."""
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="a", use_fixed_base=True), sim_core=sim_core)
        agent.set_joint_motion_profile(JOINT, max_velocity=0.4, max_accel=1.0)
        agent.add_action(JointAction(target_joint_positions={JOINT: 2.0}))
        for _ in range(40):
            sim_core.step_once()
        assert agent.get_joint_state_by_name(JOINT)[0] > 0.0

        agent.add_action(JointAction(target_joint_positions={JOINT: 0.3}))
        for _ in range(4000):
            sim_core.step_once()
            if abs(agent.get_joint_state_by_name(JOINT)[0] - 0.3) < 1e-6:
                break
        assert agent.get_joint_state_by_name(JOINT)[0] == pytest.approx(0.3, abs=1e-6)


class TestElevatorParamsMotion:
    def _elevator(self, sim_core, **motion):
        return Elevator.from_params(
            ElevatorParams(
                urdf_path=URDF, name="lift1", use_fixed_base=True,
                floors={"0": 0.0, "1": TRAVEL, "2": 2 * TRAVEL}, initial_floor="0",
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

    def test_from_dict_carries_the_motion_fields(self):
        params = ElevatorParams.from_dict(
            {
                "name": "lift1", "urdf_path": URDF, "floors": {"0": 0.0, "1": TRAVEL},
                "initial_floor": "0", "max_speed": 0.4, "max_accel": 1.0, "max_decel": 0.5,
            }
        )
        assert (params.max_speed, params.max_accel, params.max_decel) == (0.4, 1.0, 0.5)
