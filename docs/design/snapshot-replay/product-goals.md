# Snapshot / Replay — product goals and capability boundary

**Status:** Direction for future scope reviews, not an approved implementation
plan or a claim that these capabilities already work.

The existing replay and checkpoint implementations are development validation
profiles. General recording, restart and playback of ordinary simulations are
not ready for general or production use.

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

   The Python API should make the normal supported case approximately a
   single load operation followed by ordinary `run_simulation()` (conceptually
   `sim = load(dir); sim.run_simulation()`). The exact method name and whether
   it constructs a new core or initializes an existing fresh core remain to be
   designed. The caller should not have to load a manifest, construct a
   profile, select a checkpoint and pass raw state dictionaries separately.
   Keep lower-level operations available for inspection and custom workflows.
   Internal schema/profile/handler boundaries are implementation details of
   this simple user operation, not separate user steps or automatic reasons to
   split the feature into further slices.
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

- PR #50 implements restricted initial-state + recorded-input re-execution and
  comparison. That API remains separate from the supported manipulation
  recording artifact.
- The separate single-omni proof restores one fixed physics-off straight
  navigation profile at a completed step in a fresh process and continues
  with `run_simulation(resume=True)`. It does not provide opt-in recording of
  arbitrary simulations, selectable-time restart or the proposed CLI. Its
  reference mode uses `run_simulation()` with `POST_STEP` observations, while
  source mode calls `step_once()` to stop at one predetermined completed-step
  boundary. Ordinary `run_simulation()` with opt-in checkpoint capture remains
  a distinct TODO in that earlier proof.
- The example-side physics-off omni manipulation profile now records every
  completed step of an ordinary run, restores a selected checkpoint in a
  fresh process, resumes the external scenario stage and reads result frames
  for playback. It drives one named joint but captures every joint's position
  and configured target on the supported robot, plus one box; it does not imply
  arbitrary-example save/load or general recorded-input re-execution.
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
| Recording profile and owner-state extension | **Partial.** PBF accepts a versioned `RecordingProfile` and stores typed/versioned `sim`, `agents` (including controller state) and `objects` data. The manipulation profile lives in the validation example; an independent empty-world profile verifies that the recorder is not tied to it. | Only declared profiles can validate and restore their state. Reusable capture/restore contracts for other controllers, entities and custom `SimObject` state, including restore order and completeness, remain to be designed and proven. A versioned envelope alone does not make arbitrary state restorable. |
| Single-call supported restore entry point | **TODO.** The validation application currently loads the manifest, builds a profile, selects a checkpoint, restores the core and resumes its own stage through several calls. | Provide one supported load operation that resolves the declared profile/handlers, checks the artifact, selects the requested checkpoint and returns a ready-to-run simulation; keep the detailed APIs for advanced callers. State clearly which external application state still needs caller participation and reject unsupported execution state without claiming an arbitrary-world restore. |
| Separate state schemas, handlers and profiles | **TODO.** The validation profile currently combines concrete state shape, validation, capture and restore for one mobile manipulator and box. | Define versioned schemas with explicit meanings for reusable PBF-owned mobile base, arm/joints, mobile manipulator composition and payload/attachment state; separate their capture/validate/restore handlers from a profile that declares the supported combination and restore order. Let capture iterate declared live agents/objects and dispatch to explicit handlers, reporting missing support rather than serializing arbitrary object attributes. Compare a typed-model conversion library only for mapping declared state to/from stored data; it cannot discover execution state. Extract only state actually owned and supported by PBF. Prove the components by restoring a second, differently composed supported simulation in a fresh process; shortening the existing profile alone is insufficient. |
| Checkpoint validation responsibility and cadence | **TODO.** The recorder runs common-envelope and profile-specific validation after every completed step; the validation profile also rechecks invariant manifest fields and the robot asset hash. Loading and restoring validate again. | Separate construction/manifest compatibility checks at recording start and artifact load from changing-state checks per step and full restore checks. Compare declarative validation (for example, strict Pydantic models or JSON Schema) for both the common manifest/checkpoint envelope and concrete owner state before selecting one approach; check exact JSON types, versioning, added dependencies and per-step cost. Keep cross-field restore invariants and file hashes explicit where a schema alone is insufficient. Retain profile-specific support limits without placing scenario expected outcomes in reusable schemas. Measure cost before changing validation cadence, and ensure a failed capture is reported as an incomplete recording rather than silently trusted. Reuse state invariants in tests where useful; test assertions for trajectories, timing and final outcomes remain separate. |
| Duplicate frame/checkpoint storage and long recordings | **TODO.** V1 writes a result frame and a full checkpoint at every completed step, duplicating much of the agent/object state; the one-robot measurement does not establish long-run or fleet-scale cost. | Prefer investigating full per-step checkpoints as the single stored state source, with playback frames projected from them without advancing controllers or physics. Preserve step/seek, lifecycle visibility, recording integrity and the ability to distinguish display data from restorable execution state. Compare artifact size, per-step write cost/RTF, playback seek/read cost and memory against the current format over longer runs and representative fleets before changing the format or checkpoint cadence. Playback may pay more read cost, but recording should remain practical. |
| User-defined state extension | **TODO.** Callers can supply a whole `RecordingProfile` and additional `DataRecord` values, but no reusable registration/composition contract exists for custom execution-state handlers. | Let user code declare a versioned state schema and capture/validate/restore handler, then register it in a profile without modifying PBF core classes. A `DataRecord` alone is sufficient for inspectable data, but cannot imply resumable custom behavior. Define how unsupported or missing handlers are reported and keep callback code outside the artifact. |
| Caller-defined data records | **Partial.** `DataRecord` callbacks receive a completed-step context and contribute named, versioned JSON data to results/checkpoints. The manipulation scenario records its stage through a required `scenario` record. | Callers must re-register callback code when continuing; its execution is not serialized. PBF does not infer how arbitrary record values affect future behavior. Callback failures still need the best-effort policy below. |
| Completed step/time and base pose, velocity and moving flag | **Done: single-omni proof and supported manipulation profile.** A completed-step clock and `Agent` motion fields are restored in a fresh process. | Other world/controller profiles must verify their own state and step semantics. |
| Active omni pose goal/trajectory | **Done for the supported manipulation profile.** The ordinary `set_goal_pose()` path, generated approach waypoint and current waypoint are captured and resumed. | Omni velocity-mode commands are not part of this profile. `Agent.restore_motion_state()` alone does not restore controller execution. |
| Omni final-orientation rotation phase | **Done for the supported pose-navigation profile.** The final turn's slerp/TPI state resumes from an in-progress checkpoint and reaches completion. | Other controller modes and custom controller implementations need separate profiles. |
| Omni multi-waypoint path and auto-approach | **Done for the profile's generated approach + destination path.** The saved path and current waypoint survive fresh-process restore. | General multi-goal input commands are not exposed by this scenario's Fleet API surface. |
| Omni velocity-mode execution | **TODO: not captured or restored.** | Capture body-frame linear/angular commands and their watchdog/mode state when a supported scenario needs it. |
| Batch omni controller state | **TODO: not captured or restored.** | Inventory controller-owned shared/batch state and prove a separate supported multi-robot continuation profile. |
| Active differential-drive goal/trajectory | **TODO: not captured or restored.** | Inventory turn/heading phase, angular trajectory and speed, goal/path progress and completion flags; prove a mid-turn fresh-process continuation against an uninterrupted run. |
| World construction, configuration and assets | **Done for the supported manipulation profile.** The example-owned profile uses PBF recording/restore tooling to reconstruct its robot and live box from artifact construction/state and verifies the robot asset hash. | The earlier proof still uses hard-coded `_make_sim()`; arbitrary worlds, user assets and other settings are not covered. |
| Collision active pairs and check cadence | **TODO: excluded from the proof.** Collision checks are disabled in the omni run. | Inventory active-pair/event continuity, margins and sampling schedule only for a collision-relevant restore profile. |
| Joint position and active target/interpolation | **Done for all joints on the supported kinematic robot.** Every URDF joint position and configured target is captured and restored; the scenario exercises one moving joint across CP3. | Reported public joint velocity remains `0.0` during motion; other robots and physics motors are not covered. |
| Attachment parent link and relative transform | **Done for the supported kinematic box.** Parent name, link name and offset are restored at CP3. | Physics constraints and arbitrary attachment graphs are not covered. |
| Live entity roster and construction data | **Done for one supported box.** A checkpoint reconstructs its shape, pose and presence; continuation detaches and deletes it. | Other entity types or multiple/dynamic assets need their own declared construction contract. |
| Common entity construction capture and subclass extension | **TODO.** The supported profile records its box through `SimObject.from_params()`; the new versioned profile envelope does not capture construction data from other factories. Runtime spawn encoding is currently an optional profile method, `spawn_record`, discovered with `getattr`; profiles without it reject a recorded spawn. | Design a common construction-state hook reached by public factory paths (`from_params`, `from_mesh`, `from_sdf`, Agent factories) after they provide the effective creation parameters. Define an explicit contract or handler registration for optional spawn support, including how unsupported spawns are reported, instead of leaving the extension implicit in `getattr`. Allow child/custom classes to add explicitly declared, versioned state through a natural override/extension point; do not imply arbitrary Python object serialization. |
| Durable entity identity and ordered spawn/delete | **Done for this artifact profile.** Unique stable names map to new runtime IDs; ordered spawn/delete inputs are recorded. | General cross-run identity and exact re-execution of arbitrary lifecycle histories remain open. |
| External-driver stage and later inputs | **Partial.** The example saves/restores its stage and observation flags through a required `scenario` data record, so its existing callbacks continue after CP1/CP3. | Arbitrary application state and replay of later recorded inputs under an absent application are not supported. |
| Effective PBF input history and timing | **Partial.** The supported Fleet API commands and box spawn/remove path are recorded with phase, step and order. | Direct unobserved mutators, other APIs and general manipulation input re-execution are not covered. |
| Recording failure isolation / best-effort capture | **TODO.** Runtime spawn encoding, effective-input capture, profile/data-record capture, validation and durable-write errors can propagate through normal PBF operations or `step_once()` and interrupt the simulation. | Reject invalid recording configuration at setup, but isolate recording failures discovered during an otherwise valid run. An unsupported new object or one failed provider must not fail the spawn/step or silently make later checkpoints appear restorable. Report per-owner/per-step gaps and retain usable observations where possible; if durable output cannot continue, disable recording with an explicit incomplete status while simulation continues. Define what can still be read or restored after each failure class and test these paths. |
| Randomness, external clocks and integration state | **TODO: excluded from the proof.** | Identify seeds/generator state, timing and ROS/RMF or other external inputs when a concrete continuation depends on them. |
| Action queue and in-progress Action | **TODO: not exercised in the manipulation scenario.** Other examples can queue Actions. | Investigate queue/order, phase, sub-action progress, targets, timers and outcomes before mid-Action restore. |
| Devices/plugins and physics-engine state | **TODO: excluded from the proof.** | Inventory only for a concrete profile; Agent/controller restore does not imply arbitrary Python or engine-state serialization. Evaluate elevator and conveyor execution-state handlers when those devices have concrete behavior and continuation tests. |

Progress should be marked against a concrete supported profile and a passing
fresh-process restore test, rather than marking a row complete merely because
its current value can be observed. Recorded-result playback is a separate
capability and is not tracked by this checklist.

For failure isolation, distinguish an invalid profile or output setting found
before recording starts from a new unsupported state encountered during a run.
The latter includes a runtime spawn outside the declared profile: the object
operation should retain its normal PBF outcome, while recording reports the
unsupported interval and does not advertise an incomplete checkpoint as
usable. A single provider failure should not automatically discard unrelated
valid records or stop simulation. This is the desired behavior, not a claim
about the current implementation.
