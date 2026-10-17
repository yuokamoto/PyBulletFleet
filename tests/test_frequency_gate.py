"""Frequency-limited handlers run on a fixed grid of simulated time.

Simulated time accumulates one timestep at a time and carries float error, so
``now - last >= interval`` skipped beats (a 10 Hz callback on a 0.1 s timestep
ran at gaps of 0.1 s and 0.2 s) and resetting ``last`` to the run time drifted
when the interval was not a whole number of timesteps. Callbacks, plugins, the
collision check and the DataMonitor refresh all share :class:`FrequencyGate`.
"""

import pytest

from pybullet_fleet import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet._frequency import FrequencyGate
from pybullet_fleet.events import SimEvents
from pybullet_fleet.sim_plugin import SimPlugin


def _times(timestep, steps):
    """Step start times the way the core produces them: accumulated."""
    now, out = 0.0, []
    for _ in range(steps):
        out.append(now)
        now += timestep
    return out


def _fires(gate, timestep, steps):
    return [(now, dt) for now in _times(timestep, steps) for dt in [gate.due(now, timestep)] if dt is not None]


class TestFrequencyGate:
    def test_no_skipped_beats_at_the_step_rate(self):
        fires = _fires(FrequencyGate(10.0), 0.1, 101)
        assert len(fires) == 100  # 0.1 .. 10.0 s
        assert all(dt == pytest.approx(0.1) for _, dt in fires)

    def test_no_drift_when_the_interval_is_not_a_whole_number_of_steps(self):
        # 3 Hz on a 0.1 s timestep: due at 1/3, 2/3, 1 ... s, run on the first
        # step at or after each, so exactly 3 runs per simulated second.
        fires = _fires(FrequencyGate(3.0), 0.1, 301)
        assert len(fires) == 90
        assert [round(now, 6) for now, _ in fires[:6]] == [0.4, 0.7, 1.0, 1.4, 1.7, 2.0]

    def test_first_run_is_one_interval_in(self):
        fires = _fires(FrequencyGate(2.0), 0.1, 6)
        assert [round(now, 6) for now, _ in fires] == [0.5]
        assert fires[0][1] == pytest.approx(0.5)

    def test_a_gap_runs_once_not_once_per_missed_beat(self):
        gate = FrequencyGate(10.0)
        assert gate.due(0.0, 0.1) is None
        assert gate.due(0.1, 0.1) == pytest.approx(0.1)
        assert gate.due(5.0, 0.1) == pytest.approx(4.9)
        assert gate.due(5.0 + 0.05, 0.1) is None
        assert gate.due(5.1, 0.1) == pytest.approx(0.1)

    def test_faster_than_the_step_rate_runs_every_step(self):
        fires = _fires(FrequencyGate(100.0), 0.1, 11)
        assert len(fires) == 10
        assert all(dt == pytest.approx(0.1) for _, dt in fires)

    def test_every_step_without_a_frequency(self):
        fires = _fires(FrequencyGate(None), 0.1, 5)
        assert len(fires) == 5

    @pytest.mark.parametrize("frequency", [0.0, -1.0, float("inf"), float("nan")])
    def test_rejects_a_bad_frequency(self, frequency):
        with pytest.raises(ValueError):
            FrequencyGate(frequency)


@pytest.fixture
def sim():
    import pybullet as p

    sim = MultiRobotSimulationCore(
        SimulationParams(gui=False, monitor=False, timestep=0.1, target_rtf=0, monitor_log_interval=0)
    )
    sim.initialize_simulation()
    yield sim
    p.disconnect(sim.client)


class TestTheCoreUsesIt:
    def test_callback_runs_every_beat(self, sim):
        dts = []
        sim.register_callback(lambda sc, dt: dts.append(dt), frequency=10.0)
        for _ in range(101):
            sim.step_once()
        assert len(dts) == 100
        assert all(dt == pytest.approx(0.1) for dt in dts)

    def test_plugin_runs_every_beat(self, sim):
        class Counter(SimPlugin):
            def __init__(self, sim_core):
                super().__init__(sim_core)
                self.dts = []

            def on_step(self, dt):
                self.dts.append(dt)

        plugin = sim.register_plugin(Counter(sim), frequency=1.0)
        for _ in range(101):
            sim.step_once()
        # Accumulating 0.1 ten times is 0.9999999999999999: the old
        # accumulator missed every other beat.
        assert len(plugin.dts) == 10
        assert all(dt == pytest.approx(1.0) for dt in plugin.dts)

    def test_collision_check_runs_every_beat(self, sim, monkeypatch):
        checks = []
        real = sim.check_collisions
        monkeypatch.setattr(sim, "check_collisions", lambda **kw: (checks.append(sim.sim_time), real(**kw))[1])
        sim.set_collision_check_frequency(10.0)
        for _ in range(101):
            sim.step_once()
        assert len(checks) == 100


class TestPostStepPhase:
    def test_runs_after_the_post_step_event(self, sim):
        order = []
        sim.events.on(SimEvents.POST_STEP, lambda **_: order.append("event"))
        sim.register_callback(lambda sc, dt: order.append("step"))
        sim.register_callback(lambda sc, dt: order.append("post_step"), phase="post_step")
        sim.step_once()
        assert order == ["step", "event", "post_step"]

    def test_is_frequency_limited_like_the_rest(self, sim):
        dts = []
        sim.register_callback(lambda sc, dt: dts.append(dt), frequency=5.0, phase="post_step")
        for _ in range(21):
            sim.step_once()
        assert len(dts) == 10
        assert all(dt == pytest.approx(0.2) for dt in dts)

    def test_step_end_time_is_the_poses_time(self, sim):
        seen = []
        sim.register_callback(lambda sc, dt: seen.append((sc.sim_time, sc.step_end_time)), phase="post_step")
        sim.register_callback(lambda sc, dt: seen.append((sc.sim_time, sc.step_end_time)))
        sim.step_once()
        sim.step_once()
        assert [end - start for start, end in seen] == [pytest.approx(0.1)] * 4
        assert sim.step_end_time == pytest.approx(0.2)  # outside a step: the elapsed time

    def test_step_end_time_is_the_start_before_the_update(self, sim):
        """PRE_STEP runs before the objects move: its poses are the start's."""
        seen = []
        sim.events.on(SimEvents.PRE_STEP, lambda **_: seen.append((sim.sim_time, sim.step_end_time)))
        sim.step_once()
        sim.step_once()
        assert [end - start for start, end in seen] == [0.0, 0.0]

    def test_rejects_an_unknown_phase(self, sim):
        with pytest.raises(ValueError):
            sim.register_callback(lambda sc, dt: None, phase="later")
