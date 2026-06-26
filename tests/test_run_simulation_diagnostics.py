"""run_simulation() paces against the monotonic clock, so wall-clock jumps
(NTP / host suspend / WSL2 drift) no longer freeze the sim. These tests pin that
behaviour: a backward wall-clock jump is logged but never produces a long sleep,
and a steady clock stays silent.
"""

import logging

import pybullet_fleet.core_simulation as cs
from pybullet_fleet import MultiRobotSimulationCore, SimulationParams


def _run_with_clocks(monkeypatch, *, wall_fn, mono_fn, target_rtf=3.0, duration=0.3, timestep=0.05):
    """Run a short headless sim with mocked clocks; return (sleeps, sim)."""
    sleeps = []
    monkeypatch.setattr(cs.time, "monotonic", mono_fn)
    monkeypatch.setattr(cs.time, "time", wall_fn)
    monkeypatch.setattr(cs.time, "sleep", lambda s: sleeps.append(s))  # never actually block
    sim = MultiRobotSimulationCore(
        SimulationParams(gui=False, monitor=False, physics=False, target_rtf=target_rtf, duration=duration, timestep=timestep)
    )
    sim.run_simulation()
    return sleeps, sim


def test_backward_wall_clock_jump_does_not_freeze(monkeypatch, caplog):
    # Monotonic advances steadily (drives pacing); the wall clock jumps backward
    # ~30s once — exactly the WSL2/NTP anomaly that used to cause a long sleep.
    mono = {"t": 0.0}
    wall = {"t": 1000.0, "jumped": False}

    def fake_mono():
        mono["t"] += 0.01
        return mono["t"]

    def fake_wall():
        wall["t"] += 0.01
        if not wall["jumped"] and mono["t"] > 0.1:
            wall["t"] -= 30.0  # backward jump (the old freeze trigger)
            wall["jumped"] = True
        return wall["t"]

    with caplog.at_level(logging.WARNING, logger="pybullet_fleet.core_simulation"):
        sleeps, sim = _run_with_clocks(monkeypatch, wall_fn=fake_wall, mono_fn=fake_mono)

    msgs = "\n".join(r.getMessage() for r in caplog.records)
    # The jump is detected and logged (informational)...
    assert "wall clock jumped" in msgs, msgs
    # ...but it must NOT freeze: no multi-second sleep, and the sim made progress.
    assert all(s < 1.0 for s in sleeps), f"unexpectedly long sleep: {max(sleeps, default=0)}"
    assert sim.step_count > 0


def test_rtf3_does_not_clamp_under_normal_pacing(monkeypatch, caplog):
    # At rtf=3 the ahead-sleep is abs(time_diff)/rtf. If the /rtf conversion were
    # missing it would over-sleep ~3x and trip the safety clamp; assert it doesn't.
    mono = {"t": 0.0}

    def fake_mono():
        mono["t"] += 0.02
        return mono["t"]

    wall = {"t": 1000.0}

    def fake_wall():
        wall["t"] += 0.02
        return wall["t"]

    with caplog.at_level(logging.WARNING, logger="pybullet_fleet.core_simulation"):
        sleeps, _sim = _run_with_clocks(monkeypatch, wall_fn=fake_wall, mono_fn=fake_mono, target_rtf=3.0)

    msgs = "\n".join(r.getMessage() for r in caplog.records)
    assert "clamping sleep" not in msgs, msgs
    assert all(s < 1.0 for s in sleeps)


def test_steady_clock_no_warnings(monkeypatch, caplog):
    t = {"wall": 1000.0, "mono": 0.0}

    def fake_mono():
        t["mono"] += 0.02
        return t["mono"]

    def fake_wall():
        t["wall"] += 0.02
        return t["wall"]

    with caplog.at_level(logging.WARNING, logger="pybullet_fleet.core_simulation"):
        _sleeps, _sim = _run_with_clocks(monkeypatch, wall_fn=fake_wall, mono_fn=fake_mono, target_rtf=1.0, duration=0.15)

    msgs = "\n".join(r.getMessage() for r in caplog.records)
    assert "wall clock jumped" not in msgs
    assert "clamping sleep" not in msgs
