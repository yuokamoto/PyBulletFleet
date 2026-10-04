"""Requirements-discovery scenario: lifecycle and completed-step observations."""

import math
import sys

import pytest
import pybullet as p

import pybullet_fleet.examples.manipulation_state_scenario as scenario


def test_manipulation_state_and_entity_lifecycle():
    result = scenario.run_scenario()
    samples = result["observations"]
    assert result["final_stage"] == "done"
    assert tuple(samples) == ("before_spawn", "CP1", "CP2", "CP3", "CP4", "after_delete")
    assert [samples[name]["step"] for name in samples] == [0, 3, 16, 21, 28, 29]
    assert all(sample["sim_time"] == pytest.approx(sample["step"] * 0.1) for sample in samples.values())

    assert not samples["before_spawn"]["box_present"]
    assert all(samples[name]["box_present"] for name in ("CP1", "CP2", "CP3", "CP4"))
    assert not samples["after_delete"]["box_present"]
    box_id = result["identity"]["box-001"]
    assert all(samples[name]["box_runtime_object_id"] == box_id for name in ("CP1", "CP2", "CP3", "CP4"))
    assert result["lifecycle"] == [
        {
            "operation": "spawn",
            "phase": "PRE_STEP",
            "counter_during_callback": 0,
            "completed_step": 0,
            "sim_time": 0.0,
            "object_id": box_id,
        },
        {
            "operation": "delete",
            "phase": "POST_STEP",
            "counter_during_callback": 28,
            "completed_step": 29,
            "sim_time": 2.9,
            "object_id": box_id,
        },
    ]

    assert samples["CP1"]["base_is_moving"]
    assert 0 < samples["CP1"]["base_position"][0] < samples["CP2"]["base_position"][0]
    assert samples["CP2"]["joint_position"] > samples["CP1"]["joint_position"]
    assert samples["CP3"]["joint_position"] < 0.8  # Carry motion is still in progress.
    assert samples["CP3"]["joint_position"] > samples["CP4"]["joint_position"]
    assert samples["CP2"]["joint_reported_velocity"] == 0.0  # Current kinematic API contract.
    assert samples["CP3"]["joint_reported_velocity"] == 0.0

    assert not samples["CP2"]["box_attached"]
    assert samples["CP3"]["box_attached"]
    assert samples["CP3"]["attached_children"] == [box_id]
    assert samples["CP3"]["box_position"] != samples["CP2"]["box_position"]
    assert not samples["CP4"]["box_attached"]
    assert samples["CP4"]["attached_children"] == []
    assert math.dist(samples["CP3"]["box_position"], samples["CP4"]["box_position"]) > 0
    assert samples["after_delete"]["attached_children"] == []
    assert samples["after_delete"]["box_runtime_object_id"] is None


def test_gui_options_reach_scenario_without_requiring_a_desktop(monkeypatch, capsys):
    called = []

    def fake_run(**kwargs):
        called.append(kwargs)
        return {"observations": {"CP1": {"step": 3, "sim_time": 0.3, "box_present": True, "box_attached": False}}}

    monkeypatch.setattr(scenario, "run_scenario", fake_run)
    monkeypatch.setattr(sys, "argv", ["scenario", "--gui", "--rtf", "2", "--hold-gui"])
    scenario.main()
    assert called == [{"gui": True, "view_rtf": 2.0, "hold_gui": True}]
    assert "CP1: step=3" in capsys.readouterr().out


def test_gui_hold_runs_before_core_disconnect_without_a_desktop(monkeypatch):
    real_core = scenario.MultiRobotSimulationCore
    held_clients = []

    def direct_core(params):
        params.gui = False
        params.target_rtf = 0
        return real_core(params)

    monkeypatch.setattr(scenario, "MultiRobotSimulationCore", direct_core)
    monkeypatch.setattr(scenario, "_hold_final_gui", lambda sim: held_clients.append(sim.client))
    result = scenario.run_scenario(gui=True, view_rtf=2, hold_gui=True)
    assert result["final_stage"] == "done"
    assert len(held_clients) == 1


def test_invalid_gui_rate_and_hold_mode():
    with pytest.raises(ValueError, match="view_rtf"):
        scenario.run_scenario(gui=True, view_rtf=0)
    with pytest.raises(ValueError, match="hold_gui"):
        scenario.run_scenario(hold_gui=True)


def test_setup_failure_disconnects_client(monkeypatch):
    real_core = scenario.MultiRobotSimulationCore
    clients = []

    def capture_core(params):
        sim = real_core(params)
        clients.append(sim.client)
        return sim

    def fail_spawn(*args, **kwargs):
        raise RuntimeError("setup failed")

    monkeypatch.setattr(scenario, "MultiRobotSimulationCore", capture_core)
    monkeypatch.setattr(scenario.Agent, "from_params", fail_spawn)
    with pytest.raises(RuntimeError, match="setup failed"):
        scenario.run_scenario()
    assert len(clients) == 1
    assert not p.isConnected(clients[0])
