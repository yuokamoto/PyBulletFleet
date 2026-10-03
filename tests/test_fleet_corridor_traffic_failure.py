"""Behavioral checks for the one-sided, externally controlled traffic experiment."""

import sys

import pybullet as p
import pytest

import pybullet_fleet.examples.fleet_corridor_traffic_failure as traffic
from pybullet_fleet.examples.fleet_corridor_traffic_failure import TrafficConfig, _workload, run_policy


def test_four_robot_collision_response_recovers_after_minimum_stop():
    config = TrafficConfig(robots=4, cutoff=60)
    baseline = run_policy("pass_through", config)
    response = run_policy("collision_stop", config)

    assert baseline["tasks"] == response["tasks"] == _workload(config)
    assert (baseline["policy"], response["policy"]) == ("pass_through", "collision_stop")
    assert baseline["schema_version"] == response["schema_version"] == 2
    assert baseline["metrics"]["passed_count"] == response["metrics"]["passed_count"] == 4
    assert response["metrics"]["all_passed_at_seconds"] > baseline["metrics"]["all_passed_at_seconds"]
    assert response["metrics"]["stop_count"] == response["metrics"]["resume_count"] > 0
    assert response["metrics"]["robot_overlap_entries_by_zone"]["before_entrance"] > 0
    assert response["metrics"]["peak_blocked_before_entrance"] > 0
    assert response["metrics"]["peak_blocked_in_corridor"] == 0
    assert response["metrics"]["wall_overlap_samples"] == 0
    assert not response["metrics"]["deadlock_at_cutoff"]
    for report in (baseline, response):
        assert all(
            {
                decision["route_phase"]
                for decision in report["decisions"]
                if decision["robot_id"] == task["robot_id"] and "route_phase" in decision
            }
            == {"entrance", "exit", "destination"}
            for task in report["tasks"]
        )
    assert all(
        interval["end_step"] - interval["start_step"] >= config.block_steps for interval in response["blocked_intervals"]
    )
    assert all(step > 0 for step in response["first_corridor_entry_step"].values())


def test_twenty_robot_response_changes_all_pass_time_without_core_collision_response():
    config = TrafficConfig(robots=20, cutoff=60)
    baseline = run_policy("pass_through", config)
    response = run_policy("collision_stop", config)

    assert baseline["conditions"] == response["conditions"]
    assert baseline["metrics"]["passed_count"] == response["metrics"]["passed_count"] == 20
    assert response["metrics"]["all_passed_at_seconds"] > baseline["metrics"]["all_passed_at_seconds"]
    assert response["metrics"]["peak_blocked"] > 0
    assert response["metrics"]["peak_blocked_before_entrance"] > 0
    assert response["metrics"]["peak_blocked_in_corridor"] == 0
    assert response["metrics"]["robot_overlap_entries_by_zone"]["before_entrance"] > 0
    assert sum(response["metrics"]["robot_overlap_entries_by_zone"].values()) == response["metrics"]["robot_overlap_entries"]
    assert all(
        decision["step"] <= response["first_corridor_exit_step"][decision["robot_id"]]
        for decision in response["decisions"]
        if decision["action"] == "stop"
    )
    assert response["metrics"]["stop_count"] == response["metrics"]["resume_count"] > 0
    assert response["metrics"]["wall_overlap_samples"] == 0


def test_collision_after_corridor_exit_can_stop_robot_before_endpoint(monkeypatch):
    original_overlaps = traffic._robot_overlaps

    def with_post_exit_overlap(check, step, names, goals):
        overlaps = original_overlaps(check, step, names, goals)
        if step == 90:
            overlaps.add(("r00", "r01"))
        return overlaps

    monkeypatch.setattr(traffic, "_robot_overlaps", with_post_exit_overlap)
    report = run_policy("collision_stop", TrafficConfig(robots=4, cutoff=30))
    exited = report["first_corridor_exit_step"]
    arrived = report["endpoint_arrival_step"]
    assert any(
        decision["action"] == "stop" and exited[decision["robot_id"]] < decision["step"] < arrived[decision["robot_id"]]
        for decision in report["decisions"]
    )
    assert report["metrics"]["endpoint_unfinished_count"] == 0


def test_completed_endpoint_is_not_restarted_by_later_overlap(monkeypatch):
    original_overlaps = traffic._robot_overlaps

    def with_completed_robot_overlap(check, step, names, goals):
        overlaps = original_overlaps(check, step, names, goals)
        if step == 125:
            overlaps.add(("r00", "r01"))
        return overlaps

    monkeypatch.setattr(traffic, "_robot_overlaps", with_completed_robot_overlap)
    report = run_policy("collision_stop", TrafficConfig(robots=4, cutoff=30))
    assert report["endpoint_arrival_step"]["r01"] < 125
    assert report["endpoint_arrival_step"]["r00"] > 125
    assert not any(decision["action"] == "stop" and decision["step"] == 125 for decision in report["decisions"])


def test_unfinished_robots_are_not_reported_as_completed_at_cutoff():
    report = run_policy("collision_stop", TrafficConfig(robots=4, cutoff=1))
    assert report["metrics"]["passed_count"] == 0
    assert report["metrics"]["unpassed_count"] == 4
    assert report["metrics"]["all_passed_at_seconds"] is None
    assert report["metrics"]["deadlock_at_cutoff"]
    assert report["metrics"]["all_arrived_at_seconds"] is None
    assert report["metrics"]["endpoint_unfinished_count"] == 4


def test_corridor_passage_and_endpoint_arrival_have_separate_cutoff_results():
    report = run_policy("pass_through", TrafficConfig(robots=20, cutoff=9))
    assert report["metrics"]["passed_count"] == 20
    assert report["metrics"]["all_passed_at_seconds"] is not None
    assert report["metrics"]["endpoint_unfinished_count"] > 0
    assert report["metrics"]["all_arrived_at_seconds"] is None


@pytest.mark.parametrize("robots", [0, 1, 3])
def test_requires_even_robot_count(robots):
    with pytest.raises(ValueError, match="even"):
        TrafficConfig(robots=robots)


def test_gui_observation_uses_same_workload_without_desktop(monkeypatch):
    original_make_sim = traffic._make_sim
    observed_connected = []

    def direct_client(config, tasks, *, gui=False, monitor_gui=False):
        assert gui
        assert not monitor_gui
        return original_make_sim(config, tasks, gui=False)

    monkeypatch.setattr(traffic, "_make_sim", direct_client)
    monkeypatch.setattr(traffic, "_hold_final_gui", lambda sim: observed_connected.append(bool(p.isConnected(sim.client))))
    monkeypatch.setattr(traffic.time, "sleep", lambda _: None)
    report = run_policy("collision_stop", TrafficConfig(robots=4, cutoff=1), gui=True, view_rtf=3, hold_gui=True)
    assert observed_connected == [True]
    assert report["execution"] == {"gui": True, "monitor_gui": False, "target_view_rtf": 3}
    assert report["tasks"] == _workload(TrafficConfig(robots=4, cutoff=1))


def test_gui_requires_one_policy_and_positive_view_rate(monkeypatch, tmp_path):
    monkeypatch.setattr(sys, "argv", ["traffic", str(tmp_path / "unused"), "--gui"])
    with pytest.raises(SystemExit, match="2"):
        traffic.main()
    with pytest.raises(ValueError, match="view_rtf"):
        run_policy("collision_stop", gui=True, view_rtf=0)
    monkeypatch.setattr(sys, "argv", ["traffic", "--monitor"])
    with pytest.raises(SystemExit, match="2"):
        traffic.main()


def test_single_policy_cli_writes_only_one_report(monkeypatch, tmp_path):
    output = tmp_path / "one-policy"
    monkeypatch.setattr(sys, "argv", ["traffic", str(output), "--policy", "collision_stop", "--robots", "4", "--cutoff", "1"])
    traffic.main()
    assert sorted(path.name for path in output.iterdir()) == ["collision_stop.json"]


def test_omitted_output_creates_unique_directories(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(traffic.tempfile, "gettempdir", lambda: str(tmp_path))
    monkeypatch.setattr(sys, "argv", ["traffic", "--policy", "collision_stop", "--robots", "4", "--cutoff", "1"])
    traffic.main()
    traffic.main()
    runs = sorted(path for path in tmp_path.iterdir() if path.is_dir())
    assert len(runs) == 2
    assert all((path / "collision_stop.json").exists() for path in runs)
    output = capsys.readouterr().out
    assert all(f"Saved reports in {path}" in output for path in runs)
