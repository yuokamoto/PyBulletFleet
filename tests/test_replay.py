"""Contract and independent execution tests for the navigation replay profile."""

import json

import pytest

from pybullet_fleet.replay import ReplayError, ReplaySession, ReplayArtifact, ReplayInput, compare, reexecute


def definition(controller="omni"):
    return {
        "world": {"entities": [{"entity_id": "r1", "name": "robot", "position": [0, 0, 0.1]}]},
        "pbf": {"controller": controller, "timestep": 0.1},
    }


def test_unknown_profile_fields_rejected_before_creating_artifact(tmp_path):
    initial = definition()
    initial["pbf"]["physics"] = True
    with pytest.raises(ReplayError, match="unsupported"):
        ReplaySession.create(initial, output=tmp_path / "run")
    assert not (tmp_path / "run").exists()


@pytest.mark.parametrize("controller", ["omni", "batch_omni"])
def test_fresh_reexecution_and_step_order(tmp_path, controller):
    path = tmp_path / "original"
    with ReplaySession.create(definition(controller), output=path) as session:
        session.step([ReplayInput.navigate("robot", (2, 0), command_id="nav", source="ros")])
        for _ in range(4):
            session.step()
        session.step([ReplayInput.stop(["robot"], command_id="stop")])
        for _ in range(4):
            session.step()
    original = ReplayArtifact.open(path)
    assert original.initial["world"]["entities"][0]["entity_id"] == "r1"
    for i in range(3):
        replay = reexecute(original, tmp_path / f"replay-{i}")
        assert compare(original, replay).status == "matched"


def test_writer_failure_cannot_finalize(tmp_path, monkeypatch):
    path = tmp_path / "run"
    session = ReplaySession.create(definition(), output=path)

    def fail(*args):
        raise OSError("disk full")

    monkeypatch.setattr(session._writer, "append", fail)
    with pytest.raises(ReplayError, match="incomplete"):
        session.step([ReplayInput.stop(["robot"])])
    session.close()
    assert not (path / "completion.json").exists()
    with pytest.raises(ReplayError) as error:
        ReplayArtifact.open(path)
    assert error.value.status == "incomplete"


def test_corrupt_artifact(tmp_path):
    path = tmp_path / "run"
    with ReplaySession.create(definition(), output=path) as session:
        session.step()
    with (path / "initial_state.json").open("a") as stream:
        stream.write(json.dumps({"unexpected": True}))
    with pytest.raises(ReplayError) as error:
        ReplayArtifact.open(path)
    assert error.value.status == "invalid"


def rewrite(path, filename, transform):
    """Re-sign a malformed artifact to exercise semantic validation, not just hashes."""
    import hashlib

    target = path / filename
    value = (
        json.loads(target.read_text())
        if filename.endswith(".json")
        else [json.loads(line) for line in target.read_text().splitlines()]
    )
    transformed = transform(value)
    target.write_text(
        json.dumps(transformed) + "\n" if filename.endswith(".json") else "".join(json.dumps(r) + "\n" for r in transformed)
    )
    completion_path = path / "completion.json"
    completion = json.loads(completion_path.read_text())
    completion["sha256"][filename] = hashlib.sha256(target.read_bytes()).hexdigest()
    completion_path.write_text(json.dumps(completion))


@pytest.mark.parametrize("controller", ["omni", "batch_omni"])
def test_rejections_same_step_stop_order_and_outcomes(tmp_path, controller):
    path = tmp_path / "run"
    with ReplaySession.create(definition(controller), output=path, observation_interval=3) as session:
        (ack,) = session.step(
            [
                ReplayInput(
                    "navigate",
                    {"goals": [{"name": "robot", "position": [1, 0]}, {"name": "missing", "position": [0, 0]}]},
                    command_id="repeat",
                )
            ]
        )
        assert ack.accepted_names == ("robot",)
        assert ack.rejected == {"missing": "unknown robot"}
        session.step(
            [ReplayInput.navigate("robot", (1, 0), command_id="repeat"), ReplayInput.stop(["robot"], command_id="repeat")]
        )
        session.step([ReplayInput.stop(["robot"]), ReplayInput.navigate("robot", (0.4, 0), yaw=0.3)])
        for _ in range(30):
            session.step()
    artifact = ReplayArtifact.open(path)
    events = [r["event"] for r in artifact.journal() if r["record_type"] == "event"]
    assert {e.get("outcome") for e in events} >= {"stopped", "arrived"}
    assert [r["state_step"] for r in artifact.observations()] == list(range(0, 34, 3))
    assert compare(artifact, reexecute(artifact, tmp_path / "copy")).status == "matched"


def test_order_changes_result(tmp_path):
    nav = ReplayInput.navigate("robot", (1, 0), command_id="navigate")
    stop = ReplayInput.stop(["robot"], command_id="stop")
    for label, inputs in (("a", [nav, stop]), ("b", [stop, nav])):
        with ReplaySession.create(definition(), output=tmp_path / label) as session:
            session.step(inputs)
            session.step()
    result = compare(tmp_path / "a", tmp_path / "b")
    assert result.status == "different"
    assert result.first_difference is not None
    assert result.first_difference["phase"] == "input"
    assert result.first_difference["step"] == 0


def test_variant_config_reports_conditions_and_first_observed_difference(tmp_path):
    original = tmp_path / "original"
    with ReplaySession.create(definition(), output=original) as session:
        session.step([ReplayInput.navigate("robot", (10, 0))])
        for _ in range(20):
            session.step()
    variant = reexecute(original, tmp_path / "variant", pbf_overrides={"limits": {"max_linear_vel": 0.2}})
    result = compare(original, variant)
    assert result.status == "different"
    assert "initial" in result.conditions
    assert result.first_difference is not None
    assert result.first_difference["phase"] == "observation"


def test_fresh_process_three_reexecutions(tmp_path):
    import subprocess
    import sys

    original = tmp_path / "original"
    with ReplaySession.create(definition("batch_omni"), output=original) as session:
        session.step([ReplayInput.navigate("robot", (2, 0), source="rmf")])
        for _ in range(8):
            session.step()
        session.step([ReplayInput.stop(["robot"])])
    for index in range(3):
        destination = tmp_path / str(index)
        result = subprocess.run(
            [
                sys.executable,
                "-c",
                "from pybullet_fleet.replay import reexecute; import sys; reexecute(sys.argv[1], sys.argv[2])",
                str(original),
                str(destination),
            ],
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stderr
        assert compare(original, destination).status == "matched"


def test_direct_core_command_is_not_recorded_as_session_input(tmp_path):
    from pybullet_fleet.fleet_api import FleetCommandDispatcher

    path = tmp_path / "run"
    with ReplaySession.create(definition(), output=path) as session:
        session.step([ReplayInput.navigate("robot", (4, 0), command_id="go")])
        assert FleetCommandDispatcher(session._sim).stop(["robot"]).accepted_names == ("robot",)
        session.step()
    artifact = ReplayArtifact.open(path)
    assert [record["input"]["command_type"] for record in artifact.journal() if record["record_type"] == "command_intent"] == [
        "navigate"
    ]
    repeated = reexecute(path, tmp_path / "repeated")
    assert compare(path, repeated).status == "different"


@pytest.mark.parametrize("field", ["entity_id", "name"])
def test_duplicate_identity_rejected(field):
    initial = definition()
    second = dict(initial["world"]["entities"][0], entity_id="r2", name="second")
    second[field] = initial["world"]["entities"][0][field]
    initial["world"]["entities"].append(second)
    with pytest.raises(ReplayError, match="duplicate"):
        ReplaySession.create(initial)


def test_generated_identity_persisted(tmp_path):
    initial = definition()
    del initial["world"]["entities"][0]["entity_id"]
    with ReplaySession.create(initial, output=tmp_path / "run"):
        pass
    artifact = ReplayArtifact.open(tmp_path / "run")
    copied = reexecute(artifact, tmp_path / "copy")
    assert artifact.initial == copied.initial
    assert compare(artifact, copied).status == "matched"


def test_profile_version_and_integrity_are_distinct(tmp_path):
    path = tmp_path / "run"
    with ReplaySession.create(definition(), output=path):
        pass
    rewrite(path, "manifest.json", lambda m: {**m, "profile": {"id": "pbf.kinematic_navigation", "version": 999}})
    assert compare(path, path).status == "unsupported"


def test_missing_asset_and_environment_are_not_result_differences(tmp_path):
    path = tmp_path / "run"
    with ReplaySession.create(definition(), output=path):
        pass
    rewrite(path, "manifest.json", lambda m: {**m, "environment": {"other": "environment"}})
    with pytest.raises(ReplayError, match="environment mismatch"):
        reexecute(path, tmp_path / "refused")
    variant = reexecute(path, tmp_path / "variant", allow_environment_change=True)
    assert compare(path, variant).conditions["environment"]
    rewrite(path, "manifest.json", lambda m: {**m, "assets": {}})
    with pytest.raises(ReplayError, match="asset"):
        reexecute(path, tmp_path / "asset-refused", allow_environment_change=True)


def test_semantic_order_validation_even_with_valid_digest(tmp_path):
    path = tmp_path / "run"
    with ReplaySession.create(definition(), output=path) as session:
        session.step([ReplayInput.stop(["robot"])])

    def corrupt(rows):
        rows[0]["order"] = 9
        return rows

    rewrite(path, "journal.jsonl", corrupt)
    assert compare(path, path).status == "invalid"


def test_unfinished_command_is_incomplete(tmp_path):
    path = tmp_path / "run"
    with ReplaySession.create(definition(), output=path) as session:
        session.step([ReplayInput.stop(["robot"])])
    rewrite(path, "journal.jsonl", lambda rows: rows[:1])
    assert compare(path, path).status == "incomplete"


def test_static_geometry_collision_events_use_stable_ids(tmp_path):
    initial = definition()
    initial["world"]["entities"].append(
        {
            "entity_id": "wall",
            "name": "wall",
            "kind": "static_box",
            "position": [0.5, 0, 0.1],
            "half_extents": [0.05, 0.5, 0.5],
        }
    )
    path = tmp_path / "run"
    with ReplaySession.create(initial, output=path, observation_interval=7) as session:
        session.step([ReplayInput.navigate("robot", (2, 0))])
        for _ in range(19):
            session.step()
    artifact = ReplayArtifact.open(path)
    collisions = [r for r in artifact.journal() if r["record_type"] == "event" and r["event"]["type"].startswith("collision")]
    assert {r["event"]["type"] for r in collisions} == {"collision_started", "collision_ended"}
    assert all(r["event"]["entities"] == ["r1", "wall"] for r in collisions)
    assert [r["state_step"] for r in artifact.observations()] == [0, 7, 14, 20]
    assert compare(artifact, reexecute(artifact, tmp_path / "copy")).status == "matched"


def test_no_observation_construction_without_writer(monkeypatch):
    def forbidden(*args):
        raise AssertionError("observation constructed without recording")

    monkeypatch.setattr(ReplaySession, "_observe", forbidden)
    with ReplaySession.create(definition()) as session:
        session.step([ReplayInput.stop(["robot"])])


def test_unknown_input_stops_before_mutation(tmp_path):
    path = tmp_path / "run"
    with ReplaySession.create(definition(), output=path) as session:
        with pytest.raises(ReplayError, match="unsupported"):
            session.step([ReplayInput("execute_action", {})])
        assert session.step_count == 0
    assert not (path / "completion.json").exists()


def test_initial_definition_is_not_a_mutation_channel(tmp_path):
    with ReplaySession.create(definition(), output=tmp_path / "run") as session:
        changed = session.initial
        changed["pbf"]["timestep"] = 9.0
        session.step()
    artifact = ReplayArtifact.open(tmp_path / "run")
    assert artifact.initial["pbf"]["timestep"] == 0.1


def test_controller_parameter_mutation_is_detected(tmp_path):
    session = ReplaySession.create(definition(), output=tmp_path / "run")
    session._agents["r1"].controller_params.max_linear_vel = 0.1
    with pytest.raises(ReplayError, match="runtime"):
        session.step()
    session.close()
    assert compare(tmp_path / "run", tmp_path / "run").status == "incomplete"


@pytest.mark.parametrize("mutation", ["controller_chain", "collision_frequency"])
def test_unmanaged_runtime_mutation_is_detected_before_step(tmp_path, mutation):
    path = tmp_path / "run"
    session = ReplaySession.create(definition(), output=path)
    if mutation == "controller_chain":
        session._agents["r1"]._controllers.append(object())
    else:
        session._sim._collision_check_frequency = 0
    with pytest.raises(ReplayError, match="runtime"):
        session.step()
    session.close()
    assert compare(path, path).status == "incomplete"


def test_ack_write_failure_leaves_applied_intent_incomplete(tmp_path, monkeypatch):
    path = tmp_path / "run"
    session = ReplaySession.create(definition(), output=path)
    append = session._writer.append

    def fail_result(kind, record):
        if record.get("record_type") == "command_result":
            raise OSError("ack write failed")
        append(kind, record)

    monkeypatch.setattr(session._writer, "append", fail_result)
    with pytest.raises(ReplayError, match="incomplete"):
        session.step([ReplayInput.navigate("robot", (10, 0))])
    assert session._agents["r1"].goal_pose is not None
    session.close()
    records = [json.loads(line) for line in (path / "journal.jsonl").read_text().splitlines()]
    assert [r["record_type"] for r in records] == ["command_intent"]
    assert compare(path, path).status == "incomplete"


def test_finalize_failure_does_not_publish_completion(tmp_path, monkeypatch):
    path = tmp_path / "run"
    session = ReplaySession.create(definition(), output=path)
    session.step()

    def fail(step):
        raise OSError("flush failed")

    monkeypatch.setattr(session._writer, "finish", fail)
    with pytest.raises(ReplayError, match="finalization"):
        session.close()
    assert not (path / "completion.json").exists()
    session.close()


def test_exception_in_context_is_not_a_successful_empty_run(tmp_path):
    with pytest.raises(RuntimeError):
        with ReplaySession.create(definition(), output=tmp_path / "run"):
            raise RuntimeError("application failed")
    assert compare(tmp_path / "run", tmp_path / "run").status == "incomplete"


@pytest.mark.parametrize(
    "malformation", ["duplicate_key", "nan", "boolean_step", "missing_entity", "entities_list", "bad_quaternion"]
)
def test_observation_semantics_are_validated(tmp_path, malformation):
    path = tmp_path / "run"
    with ReplaySession.create(definition(), output=path):
        pass

    def corrupt(rows):
        if malformation == "boolean_step":
            rows[0]["state_step"] = False
        elif malformation == "missing_entity":
            rows[0]["entities"] = {}
        elif malformation == "entities_list":
            rows[0]["entities"] = ["r1"]
        elif malformation == "bad_quaternion":
            rows[0]["entities"]["r1"]["orientation"] = [0, 0, 0, 0]
        elif malformation == "nan":
            rows[0]["entities"]["r1"]["position"][0] = float("nan")
        return rows

    rewrite(path, "observations.jsonl", corrupt)
    if malformation == "duplicate_key":
        import hashlib

        target = path / "observations.jsonl"
        target.write_text(target.read_text().replace('"state_step": 0', '"state_step": 0, "state_step": 0'))
        completion = json.loads((path / "completion.json").read_text())
        completion["sha256"]["observations.jsonl"] = hashlib.sha256(target.read_bytes()).hexdigest()
        (path / "completion.json").write_text(json.dumps(completion))
    with pytest.raises(ReplayError) as error:
        ReplayArtifact.open(path)
    assert error.value.status == "invalid"
    assert compare(path, path).status == "invalid"


def test_target_allowlist_and_duplicate_requests_roundtrip(tmp_path):
    initial = definition()
    initial["world"]["entities"].append({"entity_id": "r2", "name": "second", "position": [0, 2, 0.1]})
    path = tmp_path / "run"
    with ReplaySession.create(initial, output=path) as session:
        (ack,) = session.step([ReplayInput.stop(["robot", "second"], allowed_names=("robot",))])
        assert ack.accepted_names == ("robot",)
        assert ack.rejected == {"second": "not managed by this interface"}
        (ack,) = session.step([ReplayInput.stop(["robot", "robot"])])
        assert ack.rejected == {"robot": "duplicate target"}
    assert compare(path, reexecute(path, tmp_path / "copy")).status == "matched"


def test_representative_example(tmp_path):
    from pybullet_fleet.examples.replay.navigation_reexecution import run

    results = run(tmp_path / "demo")
    assert results["repeated"].status == "matched"
    assert results["variant"].status == "different"
