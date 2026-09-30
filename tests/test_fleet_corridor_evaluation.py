"""Scenario-level evidence for the external corridor evaluator."""

import sys

import pybullet as p
import pytest

import pybullet_fleet.examples.fleet_corridor_evaluation as corridor
from pybullet_fleet.examples.fleet_corridor_evaluation import (
    CorridorConfig,
    _make_sim,
    _observe_collisions,
    compare_reports,
    run_policy,
)
from pybullet_fleet.geometry import Pose


@pytest.mark.parametrize("dt", [0.1, 0.05])
def test_geometric_margin_overlap_and_step_cadence(dt):
    config = CorridorConfig(timestep=dt, cutoff=1)
    sim, entities = _make_sim(config)
    robots = {name: obj for name, obj in entities.values() if name.startswith("a")}
    active = {}
    episodes = []
    try:
        robots["a00"].set_pose(Pose.from_xyz(0, 0, 0.1))
        robots["a01"].set_pose(Pose.from_xyz(0.11, 0, 0.1))
        sim.step_once()
        _observe_collisions(sim, entities, active, episodes, config.margin)
        pair = ("a00", "a01")
        assert active[pair]["category"] == "margin_only"
        assert active[pair]["start_step"] == 1

        robots["a01"].set_pose(Pose.from_xyz(0.09, 0, 0.1))
        sim.step_once()
        _observe_collisions(sim, entities, active, episodes, config.margin)
        assert active[pair]["category"] == "geometric_overlap"
        assert any(episode["category"] == "margin_only" for episode in episodes)

        robots["a01"].set_pose(Pose.from_xyz(0.30, 0, 0.1))
        sim.step_once()
        _observe_collisions(sim, entities, active, episodes, config.margin)
        assert pair not in active
        assert any(episode["category"] == "geometric_overlap" for episode in episodes)
        assert sim.collision_count >= 1
    finally:
        p.disconnect(sim.client)


def test_independent_policy_runs_and_fixed_workload():
    config = CorridorConfig(cutoff=30)
    uncontrolled = run_policy("uncontrolled", config)
    gated = run_policy("direction_gate", config)
    comparison = compare_reports(uncontrolled, gated)
    assert comparison["conditions"] == uncontrolled["conditions"] == gated["conditions"]
    assert len(uncontrolled["tasks"]) == len(gated["tasks"]) == 40
    assert {task["task_id"] for task in uncontrolled["tasks"]} == {task["task_id"] for task in gated["tasks"]}
    assert uncontrolled["run_id"] != gated["run_id"]
    assert uncontrolled["decisions"] != gated["decisions"]
    assert uncontrolled["metrics"]["corridor_geometric_overlap_episodes"] > 0
    assert gated["metrics"]["corridor_geometric_overlap_episodes"] == 0
    assert uncontrolled["crowding_intervals"]
    assert not gated["crowding_intervals"]
    assert all(interval["start_step"] <= interval["end_step"] for interval in uncontrolled["crowding_intervals"])
    for report in (uncontrolled, gated):
        metrics = report["metrics"]
        assert metrics["completed"] + metrics["rejected"] + metrics["unfinished"] == 40
        assert metrics["simulated_seconds"] == config.cutoff
        assert metrics["completed_per_sim_second"] == pytest.approx(metrics["completed"] / config.cutoff)
        assert all("command_id" in decision for decision in report["decisions"] if decision["decision"] == "navigate")
    assert uncontrolled["metrics"]["external_admission_delay_seconds"]["max"] == 0
    assert gated["metrics"]["external_admission_delay_seconds"]["max"] > 0


def test_unissued_tasks_remain_in_fixed_denominator():
    report = run_policy("direction_gate", CorridorConfig(cutoff=1))
    metrics = report["metrics"]
    assert metrics["completed"] == 0
    assert metrics["rejected"] == 0
    assert metrics["unfinished"] == 40
    assert metrics["released_but_unissued"] > 0
    assert metrics["command_to_arrival_seconds"]["count"] == 0
    assert metrics["core_margin_entry_count"] >= 0


def test_comparison_rejects_different_conditions():
    report = run_policy("uncontrolled", CorridorConfig(cutoff=1))
    different = {**report, "conditions": {**report["conditions"], "timestep": 0.2}}
    with pytest.raises(ValueError, match="equivalent"):
        compare_reports(report, different)
    changed_tasks = [dict(task) for task in report["tasks"]]
    changed_tasks[0]["destination"] = [99, 0]
    with pytest.raises(ValueError, match="workload"):
        compare_reports(report, {**report, "policy": "direction_gate", "tasks": changed_tasks})
    with pytest.raises(ValueError, match="different policies"):
        compare_reports(report, report)


def test_gui_observation_uses_same_scenario_without_desktop(monkeypatch):
    original_make_sim = corridor._make_sim

    def direct_client(config, *, gui=False):
        assert gui
        return original_make_sim(config, gui=False)

    monkeypatch.setattr(corridor, "_make_sim", direct_client)
    monkeypatch.setattr(corridor.time, "sleep", lambda _: None)
    report = run_policy("uncontrolled", CorridorConfig(cutoff=1), gui=True, view_rtf=3)
    assert report["execution"] == {"gui": True, "target_view_rtf": 3}
    assert report["metrics"]["simulated_seconds"] == 1
    assert len(report["tasks"]) == 40


def test_gui_requires_one_policy_and_positive_view_rate(monkeypatch, tmp_path):
    monkeypatch.setattr(sys, "argv", ["corridor", str(tmp_path / "unused"), "--gui"])
    with pytest.raises(SystemExit, match="2"):
        corridor.main()
    with pytest.raises(ValueError, match="view_rtf"):
        run_policy("uncontrolled", gui=True, view_rtf=0)


def test_single_policy_cli_writes_only_one_report(monkeypatch, tmp_path):
    output = tmp_path / "one-policy"
    monkeypatch.setattr(sys, "argv", ["corridor", str(output), "--policy", "uncontrolled", "--cutoff", "1"])
    corridor.main()
    assert sorted(path.name for path in output.iterdir()) == ["uncontrolled.json"]
