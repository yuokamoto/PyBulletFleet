"""Asymmetric accel/decel support in the TPI layer and the per-agent controllers.

``build_tpi`` used to pin ``dec_max`` to ``accel``, so a vehicle that brakes
harder than it accelerates could not be modelled even though
``TwoPointInterpolation`` has supported ``dec_max`` all along. These tests pin
the new behaviour and the guard that keeps the batched controllers — which
integrate the decel phase with the accel scalar — from silently producing
wrong positions.
"""

import math

import pytest

from pybullet_fleet._tpi import build_tpi, extract_phase_params
from pybullet_fleet.controller_params import ControllerParams


class TestBuildTpiDecel:
    def test_decel_defaults_to_accel(self):
        """Omitting decel reproduces the symmetric profile of earlier releases."""
        tpi = build_tpi(p0=0.0, pe=5.0, vmax=2.0, accel=1.0, t0=0.0)
        assert tpi.amax_decel == pytest.approx(tpi.amax_accel)

    def test_decel_is_forwarded(self):
        tpi = build_tpi(p0=0.0, pe=5.0, vmax=2.0, accel=1.0, t0=0.0, decel=0.25)
        assert tpi.amax_accel == pytest.approx(1.0)
        assert tpi.amax_decel == pytest.approx(0.25)

    def test_softer_braking_takes_longer(self):
        """A weaker decel must stretch the trajectory, not leave it unchanged."""
        symmetric = build_tpi(p0=0.0, pe=5.0, vmax=2.0, accel=1.0, t0=0.0)
        soft_brake = build_tpi(p0=0.0, pe=5.0, vmax=2.0, accel=1.0, t0=0.0, decel=0.25)
        assert soft_brake.get_duration() > symmetric.get_duration()

    def test_endpoint_still_reached_exactly(self):
        tpi = build_tpi(p0=0.0, pe=5.0, vmax=2.0, accel=1.0, t0=0.0, decel=0.25)
        pos, vel, _ = tpi.get_point(tpi.get_end_time())
        assert pos == pytest.approx(5.0, abs=1e-6)
        assert vel == pytest.approx(0.0, abs=1e-6)

    def test_zero_distance_fallback_keeps_decel(self):
        """The degenerate p0 -> p0 fallback must not drop the decel argument."""
        tpi = build_tpi(p0=1.0, pe=1.0, vmax=2.0, accel=1.0, t0=0.0, decel=0.25)
        assert tpi.amax_decel == pytest.approx(0.25)


class TestExtractPhaseParamsGuard:
    def test_symmetric_profile_is_accepted(self):
        tpi = build_tpi(p0=0.0, pe=5.0, vmax=2.0, accel=1.0, t0=0.0)
        t_accel, t_const, t_total, accel = extract_phase_params(tpi)
        assert accel == pytest.approx(1.0)
        assert t_total > 0.0
        assert math.isclose(t_total, t_accel + t_const + (t_total - t_accel - t_const))

    def test_asymmetric_profile_is_rejected(self):
        """Batched controllers integrate decel with the accel scalar, so refuse."""
        tpi = build_tpi(p0=0.0, pe=5.0, vmax=2.0, accel=1.0, t0=0.0, decel=0.25)
        with pytest.raises(ValueError, match="symmetric accel/decel"):
            extract_phase_params(tpi)


class TestCheckpointCaptureGuard:
    def test_capture_rejects_an_asymmetric_trajectory(self):
        """The record holds one `accel`, so an asymmetric profile is not restorable."""
        from pybullet_fleet import Pose
        from pybullet_fleet.controller import OmniController
        from pybullet_fleet.types import ControllerMode, PosePhase

        controller = OmniController(ControllerParams(max_linear_accel=1.0, max_linear_decel=0.25))
        # Put the controller in the one state capture supports, then give it a
        # trajectory whose braking differs from its acceleration.
        controller._mode = ControllerMode.POSE
        controller._pose_phase = PosePhase.FORWARD
        controller._path = [Pose.from_xyz(1.0, 0.0, 0.0)]
        controller._current_waypoint_index = 0
        controller._goal_pose = Pose.from_xyz(1.0, 0.0, 0.0)
        controller._forward_start_pos = __import__("numpy").zeros(3)
        controller._tpi_forward = build_tpi(p0=0.0, pe=1.0, vmax=2.0, accel=1.0, t0=0.0, decel=0.25)

        with pytest.raises(ValueError, match="Unsupported omni navigation state"):
            controller.capture_straight_navigation()


class TestControllerParamsDecel:
    def test_unset_decel_mirrors_accel(self):
        p = ControllerParams(max_linear_accel=3.0)
        assert p.max_linear_decel is None
        assert p._eff_linear_decel() == 3.0
        assert p.scalar_max_linear_decel() == pytest.approx(3.0)

    def test_unset_decel_mirrors_framework_default_accel(self):
        p = ControllerParams()
        assert p._eff_linear_decel() == p._eff_linear_accel()

    def test_explicit_decel_is_independent(self):
        p = ControllerParams(max_linear_accel=3.0, max_linear_decel=0.5)
        assert p.scalar_max_linear_accel() == pytest.approx(3.0)
        assert p.scalar_max_linear_decel() == pytest.approx(0.5)

    def test_per_axis_decel_projects_like_accel(self):
        """Per-axis decel uses the same direction projection as per-axis accel."""
        import numpy as np

        p = ControllerParams(max_linear_accel=[1.0, 4.0, 0.0], max_linear_decel=[2.0, 8.0, 0.0])
        x_axis = np.array([1.0, 0.0, 0.0])
        y_axis = np.array([0.0, 1.0, 0.0])
        assert p.linear_accel_along_direction(x_axis) == pytest.approx(1.0)
        assert p.linear_decel_along_direction(x_axis) == pytest.approx(2.0)
        assert p.linear_accel_along_direction(y_axis) == pytest.approx(4.0)
        assert p.linear_decel_along_direction(y_axis) == pytest.approx(8.0)

    def test_from_dict_accepts_decel(self):
        p = ControllerParams.from_dict({"max_linear_accel": 2.0, "max_linear_decel": 0.5})
        assert p.max_linear_decel == 0.5


class TestPerAgentControllersUseTheDecelLimit:
    """Raised in review: the controller wiring was not exercised at all.

    The tests above cover ``build_tpi()`` and inject ``_tpi_forward`` by hand,
    so a regression in which limit ``_init_linear_tpi()`` selects for ``dmax``
    would have passed the suite. These drive a real agent through
    ``set_path()`` and measure the trajectory it produces.
    """

    DISTANCE = 8.0
    LIMITS = dict(max_linear_vel=[2.5, 2.5, 0.0], max_linear_accel=[1.5, 1.5, 0.0], navigation_2d=True)
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

    def _agent(self, sim_core, name, controller_cls, **extra):
        from pybullet_fleet import Agent, AgentSpawnParams

        return Agent.from_params(
            AgentSpawnParams(
                urdf_path="cube_small.urdf",
                name=name,
                controller=controller_cls(ControllerParams(**{**self.LIMITS, **extra})),
            ),
            sim_core=sim_core,
        )

    def _travel_time(self, sim_core, agent, limit=4000):
        from pybullet_fleet.geometry import Pose

        agent.set_path([Pose.from_xyz(self.DISTANCE, 0.0, 0.0)], auto_approach=False)
        for step in range(limit):
            sim_core.step_once()
            if not agent.is_moving:
                return (step + 1) * self.DT
        raise AssertionError("the agent never arrived")

    def test_a_softer_decel_makes_an_omni_trajectory_longer(self, sim_core):
        from pybullet_fleet import OmniController

        symmetric = self._travel_time(sim_core, self._agent(sim_core, "sym", OmniController))
        softer = self._travel_time(sim_core, self._agent(sim_core, "soft", OmniController, max_linear_decel=[0.5, 0.5, 0.0]))
        assert softer > symmetric + 0.5

    def test_a_harder_decel_makes_it_shorter(self, sim_core):
        from pybullet_fleet import OmniController

        symmetric = self._travel_time(sim_core, self._agent(sim_core, "sym", OmniController))
        harder = self._travel_time(sim_core, self._agent(sim_core, "hard", OmniController, max_linear_decel=[6.0, 6.0, 0.0]))
        assert harder < symmetric - 0.2

    def test_the_omni_duration_matches_the_closed_form(self, sim_core):
        """v/a + v/d + (s - v^2/2a - v^2/2d)/v, with v capped by the ramps."""
        from pybullet_fleet import OmniController

        v, a, d = 2.5, 1.5, 0.5
        expected = v / a + v / d + (self.DISTANCE - v * v / (2 * a) - v * v / (2 * d)) / v
        measured = self._travel_time(sim_core, self._agent(sim_core, "closed", OmniController, max_linear_decel=[d, d, 0.0]))
        assert measured == pytest.approx(expected, abs=4 * self.DT)

    def test_a_differential_trajectory_uses_it_too(self, sim_core):
        from pybullet_fleet import DifferentialController

        symmetric = self._travel_time(sim_core, self._agent(sim_core, "dsym", DifferentialController))
        softer = self._travel_time(
            sim_core, self._agent(sim_core, "dsoft", DifferentialController, max_linear_decel=[0.5, 0.5, 0.0])
        )
        assert softer > symmetric + 0.5


class TestBatchedControllersRefuseAtThePathBoundary:
    """Raised in review: the rejection arrived partway through set_path(), and
    for a differential path needing an initial rotation it arrived later still
    -- from inside a simulation step."""

    LIMITS = dict(
        max_linear_vel=[2.5, 2.5, 0.0],
        max_linear_accel=[1.5, 1.5, 0.0],
        max_linear_decel=[0.5, 0.5, 0.0],
        navigation_2d=True,
    )

    @pytest.fixture
    def sim_core(self):
        import pybullet as p

        from pybullet_fleet import MultiRobotSimulationCore, SimulationParams

        sim = MultiRobotSimulationCore(
            SimulationParams(gui=False, physics=False, timestep=0.02, monitor=False, enable_monitor_gui=False)
        )
        sim.initialize_simulation()
        yield sim
        try:
            p.disconnect(sim.client)
        except p.error:
            pass

    def _managed_agent(self, sim_core, mode):
        from pybullet_fleet import Agent, AgentManager, AgentSpawnParams, OmniController

        manager = AgentManager(sim_core=sim_core)
        manager.enable_batch(mode)
        agent = Agent.from_params(
            AgentSpawnParams(
                urdf_path="cube_small.urdf",
                name="bot",
                controller=OmniController(ControllerParams(**self.LIMITS)),
            ),
            sim_core=sim_core,
        )
        manager.add_object(agent)
        return agent

    @pytest.mark.parametrize("mode", ["batch_omni", "batch_differential"])
    def test_set_path_refuses_an_asymmetric_profile(self, sim_core, mode):
        from pybullet_fleet.geometry import Pose

        agent = self._managed_agent(sim_core, mode)
        with pytest.raises(ValueError, match="asymmetric"):
            agent.set_path([Pose.from_xyz(5.0, 0.0, 0.0)], auto_approach=False)

    @pytest.mark.parametrize("mode", ["batch_omni", "batch_differential"])
    def test_a_refused_path_leaves_the_agent_idle(self, sim_core, mode):
        """It used to report is_moving with no trajectory behind it."""
        from pybullet_fleet.geometry import Pose

        agent = self._managed_agent(sim_core, mode)
        with pytest.raises(ValueError):
            agent.set_path([Pose.from_xyz(5.0, 0.0, 0.0)], auto_approach=False)
        assert agent.is_moving is False

    def test_a_differential_path_needing_a_turn_still_fails_at_set_path(self, sim_core):
        """The forward TPI was built later, from batch_advance(), so a path
        requiring an initial rotation raised from inside the step instead."""
        from pybullet_fleet.geometry import Pose

        agent = self._managed_agent(sim_core, "batch_differential")
        with pytest.raises(ValueError, match="asymmetric"):
            agent.set_path([Pose.from_xyz(0.0, 5.0, 0.0)], auto_approach=False)

        # And the step that follows is uneventful, rather than raising.
        sim_core.step_once()

    @pytest.mark.parametrize("mode", ["batch_omni", "batch_differential"])
    def test_a_symmetric_profile_is_still_accepted(self, sim_core, mode):
        from pybullet_fleet import Agent, AgentManager, AgentSpawnParams, OmniController
        from pybullet_fleet.geometry import Pose

        manager = AgentManager(sim_core=sim_core)
        manager.enable_batch(mode)
        symmetric = {k: v for k, v in self.LIMITS.items() if k != "max_linear_decel"}
        agent = Agent.from_params(
            AgentSpawnParams(
                urdf_path="cube_small.urdf",
                name="ok",
                controller=OmniController(ControllerParams(**symmetric)),
            ),
            sim_core=sim_core,
        )
        manager.add_object(agent)

        agent.set_path([Pose.from_xyz(5.0, 0.0, 0.0)], auto_approach=False)
        assert agent.is_moving is True


class TestSecondRoundReviewFollowUps:
    """Two holes the first round of fixes left, raised in review."""

    LIMITS = dict(max_linear_vel=[2.5, 2.5, 0.0], navigation_2d=True)

    @pytest.fixture
    def sim_core(self):
        import pybullet as p

        from pybullet_fleet import MultiRobotSimulationCore, SimulationParams

        sim = MultiRobotSimulationCore(
            SimulationParams(gui=False, physics=False, timestep=0.02, monitor=False, enable_monitor_gui=False)
        )
        sim.initialize_simulation()
        yield sim
        try:
            p.disconnect(sim.client)
        except p.error:
            pass

    def _agent(self, sim_core, name, **extra):
        from pybullet_fleet import Agent, AgentSpawnParams, OmniController

        return Agent.from_params(
            AgentSpawnParams(
                urdf_path="cube_small.urdf",
                name=name,
                controller=OmniController(ControllerParams(**{**self.LIMITS, **extra})),
            ),
            sim_core=sim_core,
        )

    def test_a_per_axis_profile_symmetric_only_on_x_is_refused(self, sim_core):
        """The forward scalar agreed while y diverged, so the check passed and
        a y-directed path raised after the path state had been committed."""
        from pybullet_fleet import AgentManager
        from pybullet_fleet.geometry import Pose

        manager = AgentManager(sim_core=sim_core)
        manager.enable_batch("batch_omni")
        agent = self._agent(
            sim_core,
            "peraxis",
            max_linear_accel=[1.0, 4.0, 0.0],
            max_linear_decel=[1.0, 8.0, 0.0],
        )
        manager.add_object(agent)

        controller = manager.batch_controller
        idx = controller._agent_index[id(agent)]

        with pytest.raises(ValueError, match="asymmetric"):
            agent.set_path([Pose.from_xyz(0.0, 5.0, 0.0)], auto_approach=False)

        # The point of the guard: nothing committed. Before this fix the
        # forward-scalar check passed, set_path() wrote the path and waypoint
        # index, and only then did _begin_waypoint() build a y-directed TPI
        # and raise.
        assert controller._paths[idx] == []
        assert agent.is_moving is False

    def test_the_generic_capture_path_refuses_it_too(self, sim_core):
        """capture_navigation_state() serializes a single `accel`, and
        restore_navigation_state() rebuilds the forward TPI from it with
        symmetric braking -- so capturing an asymmetric trajectory would
        resume it at the wrong rate, silently."""
        from pybullet_fleet.geometry import Pose

        agent = self._agent(
            sim_core,
            "capture",
            max_linear_accel=[1.5, 1.5, 0.0],
            max_linear_decel=[0.5, 0.5, 0.0],
        )
        agent.set_path([Pose.from_xyz(6.0, 0.0, 0.0)], auto_approach=False)
        for _ in range(20):
            sim_core.step_once()

        with pytest.raises(ValueError, match="asymmetric"):
            agent._controllers[0].capture_navigation_state()

    def test_a_symmetric_trajectory_still_captures(self, sim_core):
        from pybullet_fleet.geometry import Pose

        agent = self._agent(sim_core, "sym_capture", max_linear_accel=[1.5, 1.5, 0.0])
        agent.set_path([Pose.from_xyz(6.0, 0.0, 0.0)], auto_approach=False)
        for _ in range(20):
            sim_core.step_once()

        state = agent._controllers[0].capture_navigation_state()
        assert state["forward"] is not None
        assert state["forward"]["accel"] == pytest.approx(1.5)


class TestIsMovingIsSetUnderTheBatchLock:
    """Raised in review: setting the flag after the lock released races a
    concurrent step_once() that completes a zero-distance path and clears it,
    leaving an idle row reporting is_moving."""

    @pytest.fixture
    def sim_core(self):
        import pybullet as p

        from pybullet_fleet import MultiRobotSimulationCore, SimulationParams

        sim = MultiRobotSimulationCore(
            SimulationParams(gui=False, physics=False, timestep=0.02, monitor=False, enable_monitor_gui=False)
        )
        sim.initialize_simulation()
        yield sim
        try:
            p.disconnect(sim.client)
        except p.error:
            pass

    def _managed(self, sim_core):
        from pybullet_fleet import Agent, AgentManager, AgentSpawnParams, OmniController

        manager = AgentManager(sim_core=sim_core)
        manager.enable_batch("batch_omni")
        agent = Agent.from_params(
            AgentSpawnParams(
                urdf_path="cube_small.urdf",
                name="bot",
                controller=OmniController(
                    ControllerParams(max_linear_vel=[2.5, 2.5, 0.0], max_linear_accel=[1.5, 1.5, 0.0], navigation_2d=True)
                ),
            ),
            sim_core=sim_core,
        )
        manager.add_object(agent)
        return manager, agent

    def test_a_step_landing_at_lock_release_is_not_overridden(self, sim_core):
        """The race, made deterministic.

        ``synchronized_batch_advance()`` takes the same lock, so a step can
        only interleave once ``synchronized_set_path()`` has released it. This
        wraps that method to run exactly one step at that moment, with a
        zero-distance path the step completes immediately.

        With the flag set inside the lock, the step's clear is the last word
        and the agent is correctly idle. With it set by the caller afterwards,
        the assignment lands after the step and revives an idle row.
        """
        from pybullet_fleet.geometry import Pose

        manager, agent = self._managed(sim_core)
        controller = manager.batch_controller
        original = controller.synchronized_set_path

        def step_at_lock_release(a, path, **kwargs):
            original(a, path, **kwargs)
            sim_core.step_once()

        controller.synchronized_set_path = step_at_lock_release

        here = agent.get_pose().position
        agent.set_path([Pose.from_xyz(here[0], here[1], here[2])], auto_approach=False)

        assert agent.is_moving is False

    def test_a_step_completing_the_path_leaves_it_idle(self, sim_core):
        """A zero-distance path is finished by the very next step; the flag
        must not be revived afterwards."""
        from pybullet_fleet.geometry import Pose

        _, agent = self._managed(sim_core)
        here = agent.get_pose().position

        agent.set_path([Pose.from_xyz(here[0], here[1], here[2])], auto_approach=False)
        for _ in range(5):
            sim_core.step_once()

        assert agent.is_moving is False

    def test_a_refused_path_still_leaves_it_idle(self, sim_core):
        """The flag is inside the lock, so a refusal never reaches it."""
        from pybullet_fleet import Agent, AgentManager, AgentSpawnParams, OmniController
        from pybullet_fleet.geometry import Pose

        manager = AgentManager(sim_core=sim_core)
        manager.enable_batch("batch_omni")
        agent = Agent.from_params(
            AgentSpawnParams(
                urdf_path="cube_small.urdf",
                name="bad",
                controller=OmniController(
                    ControllerParams(
                        max_linear_vel=[2.5, 2.5, 0.0],
                        max_linear_accel=[1.5, 1.5, 0.0],
                        max_linear_decel=[0.5, 0.5, 0.0],
                        navigation_2d=True,
                    )
                ),
            ),
            sim_core=sim_core,
        )
        manager.add_object(agent)

        with pytest.raises(ValueError, match="asymmetric"):
            agent.set_path([Pose.from_xyz(3.0, 0.0, 0.0)], auto_approach=False)
        assert agent.is_moving is False
