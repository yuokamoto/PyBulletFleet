"""Streaming recording and strict, non-executable artifact validation."""

from __future__ import annotations

import hashlib
import importlib.metadata
import json
import math
import platform
from pathlib import Path
from typing import Any, Iterator

from .schema import (
    PROFILE,
    SCHEMA_VERSION,
    ReplayError,
    ReplayInput,
    digest,
    encode,
    keys,
    name,
    normalize_initial,
    number,
    positive_int,
    vector,
)

FILES = ("manifest.json", "initial_state.json", "journal.jsonl", "observations.jsonl")
PACKAGE = Path(__file__).resolve().parents[1]


def environment() -> dict:
    """Content fingerprint works for wheels and dirty trees without running git."""
    source = hashlib.sha256()
    for path in sorted(PACKAGE.rglob("*.py")):
        source.update(str(path.relative_to(PACKAGE)).encode())
        source.update(path.read_bytes())
    versions = {}
    for package in ("pybullet-fleet", "pybullet", "numpy", "scipy", "two-point-interpolation"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = "unknown"
    return {
        "python": platform.python_version(),
        "system": platform.system(),
        "machine": platform.machine(),
        "packages": versions,
        "pbf_source_sha256": source.hexdigest(),
    }


def assets() -> dict:
    return {
        "simple_cube": {
            "locator": "package:pybullet_fleet/robots/simple_cube.urdf",
            "sha256": digest(PACKAGE / "robots/simple_cube.urdf"),
        }
    }


def decode(text: str) -> Any:
    def reject_constant(value):
        raise ValueError(f"nonfinite JSON constant: {value}")

    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result

    return json.loads(text, parse_constant=reject_constant, object_pairs_hook=unique)


def read_json(path: Path) -> Any:
    try:
        return decode(path.read_text())
    except (OSError, ValueError) as exc:
        raise ReplayError("invalid", f"{path.name}: {exc}") from exc


def records(path: Path) -> Iterator[dict]:
    try:
        with path.open() as stream:
            for line in stream:
                if not line.endswith("\n"):
                    raise ReplayError("incomplete", f"truncated {path.name}")
                value = decode(line)
                # Also reject NaN/infinity accepted by Python's decoder.
                encode(value)
                if not isinstance(value, dict):
                    raise ValueError("record must be an object")
                yield value
    except (OSError, ValueError, TypeError) as exc:
        if isinstance(exc, ReplayError):
            raise
        raise ReplayError("invalid", f"{path.name}: {exc}") from exc


class ArtifactWriter:
    """A synchronous bounded-buffer writer. No successful footer on error."""

    def __init__(self, path: Path, manifest: dict, initial: dict):
        path.mkdir(parents=True, exist_ok=False)
        self.path = path
        self.streams: dict[str, Any] = {}
        self.counts = {"journal": 0, "observations": 0}
        try:
            (path / "manifest.json").write_text(encode(manifest) + "\n")
            (path / "initial_state.json").write_text(encode(initial) + "\n")
            for kind in self.counts:
                self.streams[kind] = (path / f"{kind}.jsonl").open("w")
        except Exception:
            self.abort()
            raise

    def append(self, kind: str, value: dict) -> None:
        record = {**value, "record_seq": self.counts[kind]}
        text = encode(record) + "\n"
        if self.streams[kind].write(text) != len(text):
            raise OSError("short artifact write")
        self.counts[kind] += 1

    def finish(self, final_step: int) -> None:
        for stream in self.streams.values():
            stream.flush()
            stream.close()
        completion = {
            "final_step": final_step,
            "counts": self.counts,
            "sha256": {filename: digest(self.path / filename) for filename in FILES},
        }
        temporary = self.path / "completion.tmp"
        temporary.write_text(encode(completion) + "\n")
        temporary.replace(self.path / "completion.json")

    def abort(self) -> None:
        for stream in self.streams.values():
            try:
                stream.close()
            except OSError:
                pass


class ReplayArtifact:
    """Validated recording. Validation never spawns an engine or executes code."""

    def __init__(self, path: Path, manifest: dict, initial: dict, completion: dict):
        self.path = path
        self.manifest = manifest
        self.initial = initial
        self.completion = completion

    @classmethod
    def open(cls, path: str | Path) -> ReplayArtifact:
        path = Path(path)
        if not (path / "completion.json").is_file():
            raise ReplayError("incomplete", "successful completion marker is missing")
        try:
            manifest = read_json(path / "manifest.json")
            keys(
                manifest,
                {
                    "schema_version",
                    "profile",
                    "run_id",
                    "environment",
                    "assets",
                    "provenance",
                    "observation_interval",
                    "velocity_semantics",
                    "outcome_semantics",
                },
                "manifest",
            )
            if (
                type(manifest["schema_version"]) is not int
                or manifest["schema_version"] != SCHEMA_VERSION
                or manifest["profile"] != PROFILE
                or type(manifest["profile"]["version"]) is not int
            ):
                raise ReplayError("unsupported", "unknown required schema/profile version")
            name(manifest["run_id"], "run_id")
            for field in ("environment", "assets", "provenance"):
                if not isinstance(manifest[field], dict):
                    raise ReplayError("invalid", f"{field} must be an object")
            completion = read_json(path / "completion.json")
            keys(completion, {"final_step", "counts", "sha256"}, "completion")
            keys(completion["counts"], {"journal", "observations"}, "counts")
            for filename in FILES:
                if not (path / filename).is_file():
                    raise ReplayError("incomplete", f"missing {filename}")
                if digest(path / filename) != completion["sha256"][filename]:
                    raise ReplayError("invalid", f"integrity mismatch: {filename}")
            initial = read_json(path / "initial_state.json")
            if normalize_initial(initial) != initial:
                raise ReplayError("invalid", "initial definition must include all resolved v1 fields")
            artifact = cls(path, manifest, initial, completion)
            artifact._validate_records()
            return artifact
        except ReplayError:
            raise
        except (KeyError, TypeError, ValueError, OSError) as exc:
            raise ReplayError("invalid", f"malformed artifact: {exc}") from exc

    def journal(self) -> Iterator[dict]:
        return records(self.path / "journal.jsonl")

    def observations(self) -> Iterator[dict]:
        return records(self.path / "observations.jsonl")

    def _validate_records(self) -> None:
        final_step = self.completion["final_step"]
        if type(final_step) is not int or final_step < 0:
            raise ReplayError("invalid", "final_step must be nonnegative integer")
        interval = positive_int(self.manifest["observation_interval"], "observation_interval")
        entity_ids = {e["entity_id"] for e in self.initial["world"]["entities"]}
        names = {e["name"] for e in self.initial["world"]["entities"] if e["kind"] == "robot"}
        dt = self.initial["pbf"]["timestep"]
        count, pending, last_step, order = 0, None, -1, 0
        result_seen = False
        latest: dict[str, dict] = {}
        ids_by_name = {e["name"]: e["entity_id"] for e in self.initial["world"]["entities"]}
        active_pairs: set[tuple[str, ...]] = set()
        for field in ("journal", "observations"):
            total = self.completion["counts"][field]
            if type(total) is not int or total < 0:
                raise ReplayError("invalid", "record counts must be nonnegative integers")
        for record in self.journal():
            if type(record["record_seq"]) is not int:
                raise ReplayError("invalid", "record_seq must be integer")
            if record["record_seq"] != count or record["run_id"] != self.manifest["run_id"]:
                raise ReplayError("invalid", "journal sequence/run mismatch")
            count += 1
            step = record["step"]
            if type(step) is not int or not 0 <= step < final_step or step < last_step:
                raise ReplayError("invalid", "invalid journal step")
            if number(record["sim_time"], "sim_time") != step * dt:
                raise ReplayError("invalid", "journal time/step mismatch")
            if step != last_step:
                order, result_seen = 0, False
            last_step = step
            kind = record["record_type"]
            common = {"record_seq", "run_id", "record_type", "step", "sim_time", "phase"}
            if kind == "command_intent":
                if type(record["order"]) is not int:
                    raise ReplayError("invalid", "input order must be integer")
                keys(record, common | {"order", "input"}, "intent")
                if pending is not None or result_seen or record["phase"] != "input" or record["order"] != order:
                    raise ReplayError("invalid", "command order/phase mismatch")
                if ReplayInput.from_record(record["input"]).to_record() != record["input"]:
                    raise ReplayError("invalid", "noncanonical input")
                pending = record
            elif kind == "command_result":
                if type(record["order"]) is not int:
                    raise ReplayError("invalid", "result order must be integer")
                keys(record, common | {"order", "ack"}, "result")
                if pending is None or record["order"] != order or record["phase"] != "input" or step != pending["step"]:
                    raise ReplayError("invalid", "result without corresponding intent")
                ack = record["ack"]
                keys(ack, {"command_id", "source", "sim_time", "accepted_names", "rejected"}, "ack")
                accepted, rejected = ack["accepted_names"], ack["rejected"]
                if not isinstance(accepted, list) or not isinstance(rejected, dict):
                    raise ReplayError("invalid", "invalid ack target containers")
                for target in accepted:
                    name(target, "accepted name")
                for target, reason in rejected.items():
                    name(target, "rejected name")
                    name(reason, "rejection reason")
                payload = pending["input"]["payload"]
                targets = set(payload["names"] if "names" in payload else [g["name"] for g in payload["goals"]])
                if (
                    len(set(accepted)) != len(accepted)
                    or set(accepted) & set(rejected)
                    or set(accepted) | set(rejected) != targets
                ):
                    raise ReplayError("invalid", "ack does not partition input targets")
                if ack["command_id"] != pending["input"]["command_id"] or ack["source"] != pending["input"]["source"]:
                    raise ReplayError("invalid", "ack correlation mismatch")
                if number(ack["sim_time"], "ack.sim_time") != step * dt or not set(ack["accepted_names"]) <= names:
                    raise ReplayError("invalid", "ack time/targets mismatch")
                for target in accepted:
                    latest[ids_by_name[target]] = {
                        "input_step": step,
                        "input_order": order,
                        "command_id": ack["command_id"],
                        "outcome": "arrived" if pending["input"]["command_type"] == "navigate" else "stopped",
                    }
                pending, order = None, order + 1
            elif kind == "event":
                keys(record, common | {"event"}, "event record")
                if pending is not None or record["phase"] != "after_update":
                    raise ReplayError("invalid", "event phase mismatch")
                result_seen = True
                event = record["event"]
                if event["type"] in ("collision_started", "collision_ended"):
                    keys(event, {"type", "entities"}, "collision")
                    pair = event["entities"]
                    if (
                        not isinstance(pair, list)
                        or len(pair) != 2
                        or not set(pair) <= entity_ids
                        or pair != sorted(set(pair))
                    ):
                        raise ReplayError("invalid", "invalid collision identity")
                    pair_key = tuple(pair)
                    if event["type"] == "collision_started":
                        if pair_key in active_pairs:
                            raise ReplayError("invalid", "duplicate collision start")
                        active_pairs.add(pair_key)
                    else:
                        if pair_key not in active_pairs:
                            raise ReplayError("invalid", "collision end without start")
                        active_pairs.remove(pair_key)
                elif event["type"] == "outcome":
                    keys(event, {"type", "entity_id", "outcome", "command_id", "input_step", "input_order"}, "outcome")
                    if event["entity_id"] not in entity_ids or event["outcome"] not in ("arrived", "stopped"):
                        raise ReplayError("invalid", "invalid outcome")
                    request = latest.pop(event["entity_id"], None)
                    if request is None or any(event[key] != value for key, value in request.items()):
                        raise ReplayError("invalid", "outcome does not refer to the latest accepted input")
                else:
                    raise ReplayError("unsupported", "unknown event type")
            else:
                raise ReplayError("unsupported", f"unknown journal record {kind}")
        if pending is not None:
            raise ReplayError("incomplete", "command application has no result")
        if count != self.completion["counts"]["journal"]:
            raise ReplayError("invalid", "journal count mismatch")
        count, expected_step = 0, 0
        for observation in self.observations():
            keys(observation, {"record_seq", "run_id", "state_step", "sim_time", "entities"}, "observation")
            step = observation["state_step"]
            if type(step) is not int or type(observation["record_seq"]) is not int:
                raise ReplayError("invalid", "observation step/sequence must be integer")
            if observation["record_seq"] != count or observation["run_id"] != self.manifest["run_id"]:
                raise ReplayError("invalid", "observation sequence/run mismatch")
            if step != expected_step or number(observation["sim_time"], "observation.sim_time") != step * dt:
                raise ReplayError("invalid", "observation step/time mismatch")
            values = observation["entities"]
            if set(values) != entity_ids:
                raise ReplayError("invalid", "full observation entity set mismatch")
            for value in values.values():
                keys(value, {"position", "orientation", "linear_velocity", "angular_velocity", "is_moving"}, "entity state")
                vector(value["position"], 3, "position")
                quaternion = vector(value["orientation"], 4, "orientation")
                if not math.isclose(sum(v * v for v in quaternion), 1.0, rel_tol=0, abs_tol=1e-6):
                    raise ReplayError("invalid", "orientation must be a unit quaternion")
                vector(value["linear_velocity"], 3, "linear_velocity")
                vector(value["angular_velocity"], 3, "angular_velocity")
                if type(value["is_moving"]) is not bool:
                    raise ReplayError("invalid", "is_moving must be boolean")
            count += 1
            expected_step = min(step + interval, final_step) if step < final_step else final_step + 1
        expected_count = 1 + final_step // interval + int(final_step % interval != 0)
        if count != expected_count or count != self.completion["counts"]["observations"]:
            raise ReplayError("incomplete", "observation coverage/count mismatch")
