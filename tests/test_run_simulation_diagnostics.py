"""run_simulation() paces against the monotonic clock, so wall-clock jumps
(NTP / host suspend / WSL2 drift) no longer freeze the sim. These tests pin that
behaviour: a backward wall-clock jump is logged but never produces a long sleep,
and a steady clock stays silent.

The fake monotonic clock advances ONLY via the mocked time.sleep(), so the tests
are decoupled from how many times run_simulation() reads time.monotonic().
"""

import logging

import pybullet_fleet.core_simulation as cs
from pybullet_fleet import MultiRobotSimulationCore, SimulationParams


def _run(monkeypatch, wall_fn, *, target_rtf=3.0, duration=0.3, timestep=0.05, **params):
    """Run a short headless sim with mocked clocks; return (sleeps, sim).

    ``wall_fn(mono)`` maps the current monotonic time to a wall-clock reading,
    letting a test inject a wall-clock jump without touching pacing. Extra
    keyword args (e.g. max_sleep_frames) are forwarded to SimulationParams.
    """
    clock = {"mono": 0.0}
    sleeps = []

    def fake_mono():
        return clock["mono"]  # advances only via sleep → independent of read count

    def fake_sleep(seconds):
        sleeps.append(seconds)
        clock["mono"] += max(0.0, seconds)

    monkeypatch.setattr(cs.time, "monotonic", fake_mono)
    monkeypatch.setattr(cs.time, "sleep", fake_sleep)
    monkeypatch.setattr(cs.time, "time", lambda: wall_fn(clock["mono"]))

    sim = MultiRobotSimulationCore(
        SimulationParams(
            gui=False, monitor=False, physics=False, target_rtf=target_rtf, duration=duration, timestep=timestep, **params
        )
    )
    sim.run_simulation()
    return sleeps, sim


def test_backward_wall_clock_jump_does_not_freeze(monkeypatch, caplog):
    # Wall clock tracks monotonic but takes a one-time -30s backward jump — the
    # WSL2/NTP anomaly that used to balloon into a multi-second sleep.
    state = {"off": 0.0}

    def wall_fn(mono):
        if state["off"] == 0.0 and mono > 0.02:
            state["off"] = -30.0
        return 1000.0 + mono + state["off"]

    with caplog.at_level(logging.WARNING, logger="pybullet_fleet.core_simulation"):
        sleeps, sim = _run(monkeypatch, wall_fn)

    msgs = "\n".join(r.getMessage() for r in caplog.records)
    assert "wall clock jumped" in msgs, msgs  # detected + logged (informational)
    assert all(s < 1.0 for s in sleeps), f"unexpectedly long sleep: {max(sleeps, default=0)}"
    assert sim.step_count > 0  # made progress, did not hang/freeze


def test_rtf3_does_not_clamp_under_normal_pacing(monkeypatch, caplog):
    # At rtf=3 the ahead-sleep is abs(time_diff)/rtf. Without the /rtf conversion
    # it would over-sleep ~3x and trip the safety clamp; assert it does not.
    with caplog.at_level(logging.WARNING, logger="pybullet_fleet.core_simulation"):
        sleeps, _sim = _run(monkeypatch, lambda mono: 1000.0 + mono, target_rtf=3.0)

    msgs = "\n".join(r.getMessage() for r in caplog.records)
    assert "clamping sleep" not in msgs, msgs
    assert all(s < 1.0 for s in sleeps)


def test_max_sleep_frames_is_configurable(monkeypatch, caplog):
    # A tiny max_sleep_frames makes the clamp bite on normal sleeps (proves the
    # SimulationParams value is wired through); the default does not clamp.
    with caplog.at_level(logging.WARNING, logger="pybullet_fleet.core_simulation"):
        _run(monkeypatch, lambda mono: 1000.0 + mono, target_rtf=1.0, duration=0.15, max_sleep_frames=0.01)
    tiny_msgs = "\n".join(r.getMessage() for r in caplog.records)
    assert "clamping sleep" in tiny_msgs, tiny_msgs

    caplog.clear()
    with caplog.at_level(logging.WARNING, logger="pybullet_fleet.core_simulation"):
        _run(monkeypatch, lambda mono: 1000.0 + mono, target_rtf=1.0, duration=0.15, max_sleep_frames=4.0)
    default_msgs = "\n".join(r.getMessage() for r in caplog.records)
    assert "clamping sleep" not in default_msgs


def test_negative_max_sleep_frames_does_not_crash(monkeypatch):
    # A misconfigured negative max_sleep_frames clamps sleep_cap to 0, so the sim
    # never sleeps — time.sleep() must not receive a negative value (no ValueError).
    # Use a self-advancing monotonic clock so the loop still progresses with no sleep.
    clock = {"mono": 0.0}
    sleeps = []

    def fake_mono():
        clock["mono"] += 0.001  # advances on its own → loop progresses even with no sleep
        return clock["mono"]

    def fake_sleep(seconds):
        sleeps.append(seconds)
        clock["mono"] += max(0.0, seconds)

    monkeypatch.setattr(cs.time, "monotonic", fake_mono)
    monkeypatch.setattr(cs.time, "sleep", fake_sleep)
    monkeypatch.setattr(cs.time, "time", lambda: 1000.0 + clock["mono"])

    sim = MultiRobotSimulationCore(
        SimulationParams(
            gui=False, monitor=False, physics=False, target_rtf=1.0, duration=0.15, timestep=0.05, max_sleep_frames=-1.0
        )
    )
    sim.run_simulation()  # must not raise ValueError from time.sleep(negative)
    assert all(s >= 0.0 for s in sleeps)
    assert sim.step_count > 0


def test_steady_clock_no_warnings(monkeypatch, caplog):
    with caplog.at_level(logging.WARNING, logger="pybullet_fleet.core_simulation"):
        _sleeps, _sim = _run(monkeypatch, lambda mono: 1000.0 + mono, target_rtf=1.0, duration=0.15)

    msgs = "\n".join(r.getMessage() for r in caplog.records)
    assert "wall clock jumped" not in msgs
    assert "clamping sleep" not in msgs
