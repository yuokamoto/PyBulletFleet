"""Per-step hooks: ``register_callback()``, ``events.on()`` and plugin ``on_step()``.

The first group pins the observable behaviour each mechanism has always had --
order, schedule, arguments and exception handling -- so that driving all three
through the event bus changes none of it. The second group covers what the bus
adds: an event at the ``STEP`` point and frequency-limited subscriptions.
"""

import logging
import time

import pybullet as p
import pytest

from pybullet_fleet import Agent, AgentSpawnParams, CallbackPhase, MultiRobotSimulationCore, Pose, SimulationParams
from pybullet_fleet.events import EventBus, SimEvents
from pybullet_fleet.sim_plugin import SimPlugin


@pytest.fixture
def sim():
    sim = MultiRobotSimulationCore(
        SimulationParams(gui=False, monitor=False, timestep=0.1, target_rtf=0, monitor_log_interval=0)
    )
    sim.initialize_simulation()
    yield sim
    p.disconnect(sim.client)


class _Recorder(SimPlugin):
    """A plugin that appends ``(name, dt)`` to a shared list on each ``on_step``."""

    def __init__(self, sim_core, name="plugin", log=None, fail=False):
        super().__init__(sim_core)
        self.name = name
        self.log = log if log is not None else []
        self.fail = fail

    def on_step(self, dt):
        self.log.append((self.name, dt))
        if self.fail:
            raise RuntimeError("plugin boom")


def _names(log):
    return [entry[0] if isinstance(entry, tuple) else entry for entry in log]


# ---------------------------------------------------------------------------
# Behaviour that must not change
# ---------------------------------------------------------------------------


class TestOrder:
    def test_one_step_runs_the_hooks_in_a_fixed_order(self, sim):
        """pre_step event, STEP callbacks, plugins, post_step event, POST_STEP callbacks --
        regardless of the order in which they were registered."""
        log = []
        sim.register_callback(lambda sc, dt: log.append("post_cb"), phase=CallbackPhase.POST_STEP)
        sim.register_plugin(_Recorder(sim, "plugin", log))
        sim.events.on(SimEvents.POST_STEP, lambda **_: log.append("post_event"))
        sim.register_callback(lambda sc, dt: log.append("step_cb"))
        sim.events.on(SimEvents.PRE_STEP, lambda **_: log.append("pre_event"))
        sim.step_once()
        assert _names(log) == ["pre_event", "step_cb", "plugin", "post_event", "post_cb"]

    def test_callbacks_run_in_registration_order(self, sim):
        log = []
        for name in "abc":
            sim.register_callback(lambda sc, dt, name=name: log.append(name))
        sim.step_once()
        assert log == ["a", "b", "c"]

    def test_plugins_run_in_load_order_after_every_step_callback(self, sim):
        log = []
        sim.register_plugin(_Recorder(sim, "p1", log))
        sim.register_plugin(_Recorder(sim, "p2", log))
        sim.register_callback(lambda sc, dt: log.append("cb"))
        sim.step_once()
        assert _names(log) == ["cb", "p1", "p2"]

    def test_post_step_callbacks_follow_every_post_step_handler_whatever_its_priority(self, sim):
        log = []
        sim.register_callback(lambda sc, dt: log.append("cb"), phase="post_step")
        sim.events.on(SimEvents.POST_STEP, lambda **_: log.append("late"), priority=1000)
        sim.events.on(SimEvents.POST_STEP, lambda **_: log.append("early"), priority=-1000)
        sim.step_once()
        assert log == ["early", "late", "cb"]

    def test_a_callback_registered_during_a_step_first_runs_in_the_next(self, sim):
        log = []

        def adder(sc, dt):
            if not log:
                sc.register_callback(lambda sc, dt: log.append("added"))
            log.append("adder")

        sim.register_callback(adder)
        sim.step_once()
        assert log == ["adder"]
        sim.step_once()
        assert log == ["adder", "adder", "added"]

    def test_a_callback_unregistered_during_a_step_still_runs_in_it(self, sim):
        log = []

        def second(sc, dt):
            log.append("second")

        def first(sc, dt):
            log.append("first")
            sc.unregister_callback(second)

        sim.register_callback(first)
        sim.register_callback(second)
        sim.step_once()
        sim.step_once()
        assert log == ["first", "second", "first"]


class TestArguments:
    def test_a_callback_gets_the_core_and_the_time_since_it_last_ran(self, sim):
        seen = []
        sim.register_callback(lambda *args, **kwargs: seen.append((args, kwargs)), frequency=5.0)
        for _ in range(4):
            sim.step_once()
        assert len(seen) == 2
        for args, kwargs in seen:
            assert kwargs == {}
            assert args[0] is sim
            assert args[1] == pytest.approx(0.2)

    def test_a_plugin_gets_the_time_since_it_last_ran(self, sim):
        plugin = sim.register_plugin(_Recorder(sim), frequency=5.0)
        for _ in range(4):
            sim.step_once()
        assert [dt for _, dt in plugin.log] == [pytest.approx(0.2), pytest.approx(0.2)]

    def test_step_events_get_the_timestep_and_the_time_they_observe(self, sim):
        seen = []
        sim.events.on(SimEvents.PRE_STEP, lambda **kw: seen.append(("pre", kw)))
        sim.events.on(SimEvents.POST_STEP, lambda **kw: seen.append(("post", kw)))
        sim.step_once()
        assert [name for name, _ in seen] == ["pre", "post"]
        assert seen[0][1] == {"dt": pytest.approx(0.1), "sim_time": pytest.approx(0.0)}
        assert seen[1][1] == {"dt": pytest.approx(0.1), "sim_time": pytest.approx(0.1)}


class TestExceptions:
    def test_a_callback_exception_propagates_and_skips_the_rest_of_the_step(self, sim):
        log = []

        def boom(sc, dt):
            raise RuntimeError("callback boom")

        sim.register_callback(boom)
        sim.register_callback(lambda sc, dt: log.append("later_cb"))
        sim.register_plugin(_Recorder(sim, "plugin", log))
        sim.events.on(SimEvents.POST_STEP, lambda **_: log.append("post_event"))
        with pytest.raises(RuntimeError, match="callback boom"):
            sim.step_once()
        assert log == []
        assert sim.step_count == 1  # the clock had already advanced
        assert sim._in_step is False

    def test_a_post_step_callback_exception_propagates(self, sim):
        sim.register_callback(lambda sc, dt: 1 / 0, phase="post_step")
        with pytest.raises(ZeroDivisionError):
            sim.step_once()

    def test_a_plugin_exception_is_logged_and_the_step_goes_on(self, sim, caplog):
        log = []
        sim.register_plugin(_Recorder(sim, "bad", log, fail=True))
        sim.register_plugin(_Recorder(sim, "good", log))
        sim.events.on(SimEvents.POST_STEP, lambda **_: log.append("post_event"))
        with caplog.at_level(logging.ERROR):
            sim.step_once()
        assert _names(log) == ["bad", "good", "post_event"]
        assert "Plugin _Recorder.on_step() failed" in caplog.text

    def test_an_event_handler_exception_is_logged_and_the_step_goes_on(self, sim, caplog):
        log = []

        def boom(**_):
            raise RuntimeError("handler boom")

        sim.events.on(SimEvents.POST_STEP, boom)
        sim.events.on(SimEvents.POST_STEP, lambda **_: log.append("next"))
        sim.register_callback(lambda sc, dt: log.append("post_cb"), phase="post_step")
        with caplog.at_level(logging.ERROR):
            sim.step_once()
        assert log == ["next", "post_cb"]
        assert "EventBus handler error for event 'post_step'" in caplog.text


class TestRegistry:
    def test_unregister_removes_the_earliest_registration_of_a_function(self, sim):
        log = []

        def cb(sc, dt):
            log.append(sc.sim_time)

        sim.register_callback(cb, phase="post_step")
        sim.register_callback(cb)
        assert sim.unregister_callback(cb) is True
        sim.step_once()
        assert len(log) == 1  # the STEP one is left
        assert len(sim._callbacks) == 1 and sim._callbacks[0]["phase"] == CallbackPhase.STEP
        assert sim.unregister_callback(cb) is True
        assert sim.unregister_callback(cb) is False

    def test_the_registry_lists_function_phase_and_schedule(self, sim):
        def cb(sc, dt):
            pass

        sim.register_callback(cb, frequency=4.0, phase="post_step")
        (info,) = sim._callbacks
        assert info["func"] is cb
        assert info["phase"] == CallbackPhase.POST_STEP
        assert info["gate"].frequency == 4.0

    def test_frequency_zero_means_every_step(self, sim):
        log = []
        sim.register_callback(lambda sc, dt: log.append(dt), frequency=0)
        for _ in range(3):
            sim.step_once()
        assert len(log) == 3

    def test_a_negative_frequency_is_rejected(self, sim):
        with pytest.raises(ValueError):
            sim.register_callback(lambda sc, dt: None, frequency=-1.0)
        assert sim._callbacks == []

    def test_clearing_the_event_bus_keeps_callbacks_and_plugins(self, sim):
        log = []
        sim.register_callback(lambda sc, dt: log.append("cb"))
        sim.register_plugin(_Recorder(sim, "plugin", log))
        sim.events.on(SimEvents.POST_STEP, lambda **_: log.append("event"))
        sim.events.clear()
        sim.step_once()
        assert _names(log) == ["cb", "plugin"]


class TestProfiling:
    def test_plugin_time_is_not_counted_as_callback_time(self, sim):
        class Slow(SimPlugin):
            def on_step(self, dt):
                time.sleep(0.02)

        sim.register_plugin(Slow(sim))
        profile = sim.step_once(return_profiling=True)
        assert profile["callbacks"] < 20.0
        assert profile["phase1_update"] >= 20.0
