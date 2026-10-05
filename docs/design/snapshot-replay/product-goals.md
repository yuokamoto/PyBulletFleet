# Snapshot / Replay — product goals and capability boundary

**Status:** Direction for future scope reviews, not an approved implementation
plan or a claim that these capabilities already work.

Read this document before scoping further snapshot, restart, playback or trace
work. Each change should state which user operation it advances, what it makes
observable, and which supported profiles remain excluded. The [v1 scope](spec.md)
and [single-omni evidence](checkpoint-evidence.md) describe narrower delivered
capabilities.

The approved end-to-end feature theme is specified in the
[Snapshot / Restore / Replay / Playback V1 plan](feature-v1-spec-plan.md).
Its implementation is in progress; the plan is not a claim of delivered V1 support.

## Intended user workflow

1. **Record:** A user opts in when running an ordinary simulation (conceptually
   `save=on`). PBF records enough time-indexed simulation results for observation
   and, for declared supported profiles, enough execution state and effective
   inputs to restart. Recording must identify its coverage and completeness;
   merely saving poses does not create a resumable checkpoint.

   The application chooses a goal or attachment operation, but once PBF
   accepts it, the effective command and its simulation step/phase/order and
   parameters must be available for recording. This includes movement and,
   for future supported profiles, attach/detach, spawn/delete and permitted
   runtime configuration changes. PBF must expose trustworthy execution facts;
   an external recorder may own the durable journal. Current v1 journaling is
   limited to its supported navigate/stop boundary, and a current active goal
   in a checkpoint does not recover the full earlier input history.
2. **Restart:** A user selects a recorded time (conceptually
   `pbf-restart <artifact> --time <t>`). The tool selects the latest usable
   checkpoint **at or before** `t`, never one after `t`, so the requested
   interval is not skipped. Whether it then advances from that checkpoint to
   exactly `t` using recorded inputs, or starts at the selected checkpoint,
   remains a later design decision. After restore, normal PBF execution runs.
   An external application can then try another policy; it must restore or
   reconstruct any state it owns. PBF settings may be changed at runtime only
   where the ordinary API permits that change. A changed policy/configuration
   is a variant, not an identical reproduction.
3. **Playback:** A user observes recorded results without rerunning the original
   policy or physics (conceptually `pbf-playback <artifact> --rtf <rate>`).
   Playback pacing is a display control, distinct from simulation RTF and from
   restarting an executable simulation. Sampling and seek behavior must be
   defined so short events are not silently claimed as observed.
4. **Debug:** A user inspects a time-aligned trace of relevant inputs, events,
   their start/end times, outcomes and effective parameters to understand what
   happened. The trace must state what is measured versus inferred and what
   crosses the PBF/external-application boundary. It complements snapshots;
   snapshots alone cannot explain causal decisions.

The desired result is to reproduce a failure, watch it, restart before it to
try a changed policy or permitted runtime parameter, and compare outcomes.
CLI names, configuration names, artifact layout, checkpoint cadence and exact
`--time` semantics are provisional. Do not treat all recorded samples as
execution checkpoints or assume that arbitrary PBF/controller/plugin/ROS state
is serializable. The checklist below tracks known checkpoint profile limits; it is not exhaustive.

## Current position

- V1 implements restricted initial-state + recorded-input re-execution and
  comparison, not recorded-result playback or intermediate restore.
- The separate single-omni proof restores one fixed physics-off straight
  navigation profile at a completed step in a fresh process and continues
  with `run_simulation(resume=True)`. It does not provide opt-in recording of
  arbitrary simulations, selectable-time restart or the proposed CLI. Its
  reference mode uses `run_simulation()` with `POST_STEP` observations, while
  source mode calls `step_once()` to stop at one predetermined completed-step
  boundary. Ordinary `run_simulation()` with opt-in checkpoint capture remains
  a distinct TODO.
- Broader controller, entity lifecycle, Action, device, physics, external app
  and ROS/RMF state, plus a general trace and playback UI, remain separate
  scope decisions. Implementation should expand from concrete supported
  profiles and evidence rather than claiming whole-simulation capture.

## Checkpoint state capability checklist

"Observed" means a value can be read or supplied in one run; it does **not**
mean that it can be persisted, loaded or resumed. **Done means proven only for
the named profile**, not for every robot or for the manipulation scenario that
motivated this inventory. This checklist tracks execution-checkpoint state,
not playback or trace coverage. It is a capability inventory, not an approved
implementation sequence or a guarantee that every future plugin or controller
state has already been identified.

| State or operation | Checkpoint status | Remaining boundary |
| --- | --- | --- |
| Completed step/time and base pose, velocity and moving flag | **Done: single-omni proof only.** A completed-step clock and `Agent` motion fields are restored in a fresh process. | Other world/controller profiles must verify their own state and step semantics; CP1–CP4 of the manipulation scenario are observations, not restorable checkpoints. |
| Active straight omni goal/trajectory | **Done: single-omni proof only.** One built-in controller resumes one forward waypoint and reaches its destination. | The proof disables auto-approach and final-orientation alignment. `Agent.restore_motion_state()` alone does not restore controller execution. |
| Omni rotation phase | **TODO: not captured or restored.** | Capture angular interpolation and rotation-phase state; prove continuation from a checkpoint taken during rotation. |
| Omni multi-waypoint path and auto-approach | **TODO: not captured or restored.** | Preserve the effective path, current waypoint/progress and any generated approach segment; test continuation across a waypoint transition. |
| Omni final-orientation alignment | **TODO: not captured or restored.** | Capture final rotation target, alignment phase and completion state; test an in-progress final turn. |
| Batch omni controller state | **TODO: not captured or restored.** | Inventory controller-owned shared/batch state and prove a separate supported multi-robot continuation profile. |
| Active differential-drive goal/trajectory | **TODO: not captured or restored.** | Inventory turn/heading phase, angular trajectory and speed, goal/path progress and completion flags; prove a mid-turn fresh-process continuation against an uninterrupted run. |
| World construction, configuration and assets | **Partial: fixed single-omni proof only.** The proof stores and validates fixed conditions, but `restore` still calls the same hard-coded `_make_sim()` to build the world and controller. | **TODO:** persist enough effective construction data to reconstruct a supported simulation from the artifact without an example-specific `_make_sim()` definition. Validate available assets and compatible settings; user-provided assets are not bundled by this proof. |
| Collision active pairs and check cadence | **TODO: excluded from the proof.** Collision checks are disabled in the omni run. | Inventory active-pair/event continuity, margins and sampling schedule only for a collision-relevant restore profile. |
| Joint position and active target/interpolation | **TODO: observable only in the manipulation scenario.** Position is public, but reported kinematic velocity remains `0.0` during movement. | Define joint execution-state read/restore for a manipulation profile; resolve velocity semantics separately. |
| Attachment parent link and relative transform | **TODO: presence observable only.** The driver knows its requested link and offset. | Define a complete supported attachment-state read/restore contract; do not infer arbitrary existing attachments from driver knowledge. |
| Live entity roster and construction data | **TODO: observable only.** The driver owns the box's spawn parameters. | Capture effective construction data to recreate live entities at restore, then prove detach/delete after continuation. |
| Durable entity identity and ordered spawn/delete | **TODO: driver-local label only.** PBF IDs are run-local. | Persist stable mapping and, when exact re-execution is required, ordered effective lifecycle inputs; sampled rosters miss changes between samples. |
| External-driver stage and later inputs | **TODO: held only by the manipulation scenario.** | Save/restore driver state if it must resume; record and apply later inputs for identical re-execution. |
| Effective PBF input history and timing | **Partial: v1 navigate/stop only.** The single-omni proof captures its active goal but no command history; direct movement and attach/detach are not generally journaled. | Expose/record accepted state-changing operations with effective parameters and step/phase/order for each supported profile. Keep external policy decisions and durable artifact orchestration outside the simulation core. |
| Randomness, external clocks and integration state | **TODO: excluded from the proof.** | Identify seeds/generator state, timing and ROS/RMF or other external inputs when a concrete continuation depends on them. |
| Action queue and in-progress Action | **TODO: not exercised in the manipulation scenario.** Other examples can queue Actions. | Investigate queue/order, phase, sub-action progress, targets, timers and outcomes before mid-Action restore. |
| Devices/plugins and physics-engine state | **TODO: excluded from the proof.** | Inventory only for a concrete profile; Agent/controller restore does not imply arbitrary Python or engine-state serialization. |

Progress should be marked against a concrete supported profile and a passing
fresh-process restore test, rather than marking a row complete merely because
its current value can be observed. Recorded-result playback is a separate
capability and is not tracked by this checklist.
