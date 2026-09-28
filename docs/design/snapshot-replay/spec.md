# Snapshot / Replay — Approved v1 Scope

**Status:** Implemented; independent review and Human Final Review pending.

The first conceptual change is re-execution and result comparison of effective
Fleet API commands from a known initial state, for a fixed kinematic navigation
profile. Scope and architecture were explicitly approved under workflow v0.1.

## Contract

- Session-owned fresh simulation; physics off, fixed timestep/entities.
- Planar navigate and stop, omni/batch_omni, bundled simple_cube/static boxes.
- Stable artifact entity ID separate from API name and runtime IDs.
- Input phase `(step=k, order=n)` before controller update; observations of S_(k+1).
- PBF-owned versioned JSON/JSONL: initial state, execution journal, full
  observations, provenance, successful completion/integrity.
- Same-environment supported-profile comparison, not bitwise/cross-platform determinism.
- Fail-fast when recording completeness is lost; valid differences remain distinct.

See [user contract](../../how-to/replay.md), [implementation plan](plan.md),
and [evidence](evidence.md).

## Explicit non-goals

Execution checkpoint/restore/resume, general result playback, deltas, arbitrary
Python/plugin serialization, BT/device/action/joint/attachment replay, dynamic
worlds, ROS/DDS deterministic replay, rosbag integration, generic asset management,
shared USO runtime/schema package, GUI editor and cloud storage.

## Relationship to the 2026-04-05 draft

The original draft proposed USO-shaped full/delta serialization followed by
physics-off pose playback, then a USO adapter. The later roadmap also proposed
input re-execution and checkpoint/restore. Those are distinct capabilities;
neither draft supplied the sufficient execution-state contract for arbitrary
restore. The approved v1 deliberately implements initial-state re-execution.

The original draft's single-string `connected_to`, collision-oriented
`_moved_this_step` delta strategy, and assumed private lookup fields must not be
used as current contracts. USO itself now describes logical connections as lists
and leaves joint/version/extension details open. No persisted old format was
identified, so there is no legacy-reader migration. Git history retains the old
draft. Existing movie recording and ordinary Fleet API behavior remain separate.

## USO evolution

This PBF artifact validates concrete concepts for future USO refinement. It does
not permanently choose a competing schema. Findings will feed into USO, then be
validated in another backend before any shared abstraction is extracted. The USO
repository is not modified by this change.
