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

    def test_a_first_call_on_a_due_time_runs_at_once(self):
        """After a restore at 10.0 s, a 10 Hz schedule runs at 10.0, not 10.1."""
        gate = FrequencyGate(10.0)
        assert gate.due(10.0, 0.1) == pytest.approx(0.1)
        assert gate.due(10.1, 0.1) == pytest.approx(0.1)

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
        for _ in range(100):  # steps ending at 0.1 .. 10.0 s: one beat each
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
        for _ in range(100):  # steps ending at 0.1 .. 10.0 s: one beat each
            sim.step_once()
        assert len(checks) == 100

    def test_plugin_back_from_every_step_starts_a_fresh_schedule(self, sim):
        class Counter(SimPlugin):
            def __init__(self, sim_core):
                super().__init__(sim_core)
                self.dts = []

            def on_step(self, dt):
                self.dts.append(dt)

        plugin = sim.register_plugin(Counter(sim), frequency=1.0)
        for _ in range(15):
            sim.step_once()
        plugin.frequency = None
        for _ in range(30):
            sim.step_once()
        plugin.frequency = 1.0
        plugin.dts.clear()
        sim.step_once()
        # Not an immediate run carrying the whole gap since the last one.
        assert plugin.dts == []
        for _ in range(10):
            sim.step_once()
        assert plugin.dts == [pytest.approx(1.0)]

    @pytest.mark.parametrize("frequency", [0, -1.0])
    def test_a_non_positive_collision_frequency_disables_checks(self, sim, monkeypatch, frequency):
        checks = []
        monkeypatch.setattr(sim, "check_collisions", lambda **kw: checks.append(1) or ([], None))
        sim.set_collision_check_frequency(frequency)
        for _ in range(5):
            sim.step_once()
        assert checks == []


class TestPostStepPhase:
    def test_runs_after_the_post_step_event(self, sim):
        order = []
        sim.events.on(SimEvents.POST_STEP, lambda **_: order.append("event"))
        sim.register_callback(lambda sc, dt: order.append("step"))
        sim.register_callback(lambda sc, dt: order.append("post_step"), phase="post_step")
        sim.step_once()
        assert order == ["step", "event", "post_step"]

    def test_is_frequency_limited_like_the_rest(self, sim):
        seen = []
        sim.register_callback(lambda sc, dt: seen.append((sc.sim_time, dt)), frequency=5.0, phase="post_step")
        for _ in range(20):
            sim.step_once()
        # Scheduled on the poses' time: runs when they reach 0.2, 0.4 ... 2.0 s.
        assert [round(t, 9) for t, _ in seen] == [round(0.2 * k, 9) for k in range(1, 11)]
        assert all(dt == pytest.approx(0.2) for _, dt in seen)

    def test_both_phases_see_the_time_of_the_state(self, sim):
        """A Phase-1 and a post-step callback in the same step see the same
        sim_time: the end of the step, the time of the poses."""
        seen = []
        sim.register_callback(lambda sc, dt: seen.append(("post", round(sc.sim_time, 9))), phase="post_step")
        sim.register_callback(lambda sc, dt: seen.append(("step", round(sc.sim_time, 9))))
        sim.step_once()
        sim.step_once()
        assert seen == [("step", 0.1), ("post", 0.1), ("step", 0.2), ("post", 0.2)]
        assert sim.sim_time == pytest.approx(0.2)  # outside a step: the completed time

    def test_a_phase_one_schedule_at_the_step_rate_runs_from_the_first_step(self, sim):
        seen = []
        sim.register_callback(lambda sc, dt: seen.append(round(sc.sim_time, 9)), frequency=10.0)
        for _ in range(3):
            sim.step_once()
        assert seen == [0.1, 0.2, 0.3]

    def test_a_post_step_schedule_at_the_step_rate_runs_from_the_first_step(self, sim):
        seen = []
        sim.register_callback(lambda sc, dt: seen.append(round(sc.sim_time, 9)), frequency=10.0, phase="post_step")
        for _ in range(3):
            sim.step_once()
        assert seen == [0.1, 0.2, 0.3]

    def test_rejects_an_unknown_phase(self, sim):
        with pytest.raises(ValueError):
            sim.register_callback(lambda sc, dt: None, phase="later")
