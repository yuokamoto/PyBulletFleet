# Navigation replay implementation plan and decisions

**Usage status:** Development validation profile; not ready for general or
production snapshot/replay use.

Scope and architecture approved by the Human under
`docs/AI_DEVELOPMENT_WORKFLOW.md`; implementation does not authorize merge.

## Approved decisions

- ReplaySession owns a fresh, restricted kinematic navigation instance.
- Record effective input after producer translation, before controller update.
- Stable artifact entity IDs are distinct from Fleet API names and runtime IDs.
- PBF-owned versioned artifact separates initial/common state, execution journal,
  observations, provenance and completeness; no shared USO dependency.
- Full observations only; no checkpoint/restore or delta mechanism.
- Fail fast when recording completeness cannot be guaranteed.
- Compare same-environment supported execution and explicitly changed variants.
- A representative navigation/stop scenario is sufficient; a historical failure
  is preferred when available but is not required.
- Measure 100/1000-agent overhead; 5%/20% are not acceptance gates.

## Implementation sequence and locations

1. `replay/schema.py`: restricted initial/input profile, defaults and validation.
2. `replay/session.py`: initialization, stable-ID mapping, supported entities.
3. `ReplaySession.step()` applies effective inputs before an ordinary core step
   and records outcomes afterward; no replay-specific core/API mutation guards.
4. `replay/artifact.py`: synchronous JSON/JSONL writer, completion and reader.
5. `replay/runner.py`: fresh re-execution and streaming result comparison.
6. `tests/test_replay.py`, representative example, replay benchmark and docs.

Session recording is not an EventBus subscriber. Normal dispatch remains
synchronous; recording a command before its mutation does not equate acceptance
with completion. Required I/O failure cannot disappear in EventBus exception
isolation. Following Human review, the implementation moved replay execution
control outside core simulation; direct core calls are outside the recording
contract and cannot be exhaustively detected.

## Verification

- Schema, identity, allowed-name/rejection and order tests.
- Fresh independent-process re-execution, both controller profiles, outcomes and
  collision-step equality; numeric observations within stated tolerance.
- Explicit variant comparison with first observed difference.
- Unsupported input/detected persistent runtime mutation, corruption, unknown versions, missing
  assets, incomplete intent/results, write/finalization failure.
- No observation construction without a writer; normal core regression tests.
- Repository `make verify`, documentation build and representative example.
- Matched before/after disabled, session-only, 1 sim-Hz and every-step full
  recording measurements at 100 and 1000 agents, with setup/finalization/storage
  and memory separately reported.

No ROS surface is changed. Live ROS ingress migration, rosbag, cross-backend
conformance, generic assets, async I/O, delta tracking and checkpointing remain
outside scope. The artifact carries producer-independent source/command/run
identifiers for future correlation.

## Review handoff

See [evidence](evidence.md) for measured results, deviations and the acceptance
criteria mapping. Independent review and Human Final Review follow implementation;
no automatic commit, push, merge or USO repository modification is authorized.
