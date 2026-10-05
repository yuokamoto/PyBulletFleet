# Snapshot / Restore / Replay / Playback V1 — feature specification and plan

**Status:** Scope and architecture direction approved for implementation.
Material changes to the supported profile or state ownership return for Human
Review. This applies to the whole feature theme; controller and entity-state
tests are not separate feature themes.

Read the [product goals and capability checklist](product-goals.md) before
changing this scope. The goal here is a usable path through those goals, not a
claim that every checklist row will be complete in V1.

## Product goal and user operation

**Today:** PR #50 can re-execute a restricted initial state and recorded
navigate/stop inputs. PR #55 can observe a manipulation/lifecycle sequence.
PR #56 can resume one fixed straight-moving omni robot from a predetermined
checkpoint in a fresh process. A user still cannot opt into recording an
ordinary supported run, choose its completed-step checkpoint, restart the
manipulation sequence from a saved artifact, or inspect its recorded results
without running the simulation again.

**V1 outcome:** A user runs one declared supported PBF scenario normally with
recording enabled, obtains time-indexed results and selectable usable
execution checkpoints, terminates the source process, restores a selected
checkpoint in a fresh process, resumes normal simulation execution, and opens
the recorded results for step/seek playback. The artifact reports exactly
which inputs, state owners and steps it covers. A changed external policy or
runtime setting is a new continuation, not proof of identical replay.

The intended user-facing shape is below. Names are proposals, not existing
APIs or commands:

```python
sim = build_supported_simulation()  # ordinary PBF setup, reused by the scenario
sim.configure_state_recording(
    enabled=True, output="run-dir", checkpoint_every_steps=1,
    profile="kinematic_manipulation_v1",
)
register_scenario_callbacks(sim, scenario_state=ScenarioState())
sim.run_simulation()  # PBF records completed steps; no user step loop

# Separate process, after the source has exited:
restored = load_supported_checkpoint("run-dir", at_or_before=2.1)
sim, scenario_state = restored.create_simulation_and_scenario()
sim.run_simulation(resume=True)

with ResultPlayback.open("run-dir") as playback:
    playback.seek_step(21)
    print(playback.state, playback.events)
    playback.step()
```

`configure_state_recording()` is a proposed simple setting, not an existing
method. `record=True` in normal configuration could be equally appropriate;
an output location can be auto-named when omitted. A context manager may be
offered for callers that want explicit file lifetime, but it should not be
required for ordinary runs. `create_simulation_and_scenario()` represents
PBF-provided reconstruction tools plus a supported scenario entrypoint and
an explicit external-state restore hook. V1 must **not** promise that an
artifact can construct arbitrary user Python applications. A narrow example
CLI should expose the same record, restart and playback operations, with an
optional GUI for visual inspection. Exact API/CLI spelling belongs to the
implementation review. The existing video `SimulationRecorder` and
`start_recording()` are for GIF/MP4 capture; state recording needs a distinct
name.

## Capability and responsibility boundary

| Capability | V1 meaning |
| --- | --- |
| Observation | Read current PBF facts in memory; this alone cannot restart a run. |
| Recording | Write a time-indexed history of supported inputs, results, events and checkpoints. It is an operation over time, not a smaller snapshot. |
| Snapshot / checkpoint | One profile-qualified execution state sufficient to restore at its completed-step boundary. A result frame can have fewer fields; a recording can contain many checkpoints. |
| Restore | Validate the artifact, reconstruct the supported world in a fresh process, then apply its execution state. |
| Resume | Continue from the restored boundary through the ordinary `run_simulation(resume=True)` path. |
| Re-execution | Apply recorded effective inputs to an executable initial/checkpoint state. #50 already provides this for its restricted fixed-world profile; a general V1 manipulation re-execution claim requires its own coverage proof. |
| Playback | Read recorded results in time order, step or seek, without advancing PBF controllers or physics. Playback pacing is display pacing, separate from simulation RTF. |

PBF core, Agent, controller and SimObject may expose their owned state and
supported restore operations. **PBF's recording facility** should own capture
scheduling, artifact I/O, validation, checkpoint selection, restoration tools
and playback reading. These belong in the PBF package, but the simulation
core need not directly manage files or launch processes. The caller owns when
to start/stop its application, the policy it runs and any application-only
state. It should be able to restore using PBF's supported APIs/tools without
hand-writing the state reconstruction sequence. PBF must expose accepted
operations and effective parameters where V1 needs them, but it does not own
the external application's decisions.

The default proposal for this small V1 profile is a **full checkpoint after
every completed step** (`checkpoint_every_steps=1`), plus each step's result
frame and ordered events. The time at which a failure will matter is usually
unknown during recording. If every-step checkpoints prove too expensive,
measure the cost and return for a frequency/retention decision rather than
silently weakening selectable-time restart. A lower frequency is a storage
and performance option, but selecting the latest earlier checkpoint then
advancing to an arbitrary requested step requires complete intervening input
capture and re-execution; V1 does not assume that mechanism is already proven.

## Proposed supported V1 profile

Use the existing [manipulation state scenario](../../how-to/manipulation-state-scenario.md)
as the **acceptance sequence**, adapting rather than duplicating its setup:
spawn a box; navigate while the base is moving (CP1); move one named joint;
attach the box to a named robot link; move the attached joint while it is in
motion (CP3); detach; then remove the box. CP1 and CP3 are mandatory restore
tests, not the only checkpoint times. CP2 and CP4 remain useful observation
and regression points; under every-step recording they also have checkpoints.
The external scenario stage must resume after CP1 and CP3, or those checkpoints are not
usable for this operation. A reference run and both restored continuations
must exercise the same supported sequence.

The profile is one mobile manipulator, one supported built-in **omni**
controller, one named kinematic joint, one constructed box, direct supported
PBF/Fleet API operations, physics off, fixed timestep and packaged/available
unchanged assets. This retains #56's controller state path while extending
one user operation across joint, attachment and entity lifecycle. PR #55's
example currently uses a differential controller. The first implementation
check must confirm that the same mobile manipulator and route work with omni;
if they do not, return with evidence and a profile choice rather than quietly
dropping manipulation or adding differential support. Separate differential
navigation, including its turning phase, is the next controller profile after
V1.

Existing `name` values are not unique and runtime PBF `object_id`/PyBullet
body IDs are run-local. V1 therefore uses a recording-local stable entity key,
validated for uniqueness within the artifact, mapped to runtime objects in
each process. This is an artifact identity map, not a new global PBF identity
system. User-provided assets remain the user's responsibility; the artifact
records locators and a compatibility fingerprint and fails visibly if the
needed asset differs or is missing.

## Non-goals and explicit later profiles

V1 does not support arbitrary PBF examples; arbitrary controller/Action/BT
state; differential or batch navigation; arbitrary plugins, callbacks,
devices or custom SimObjects; physics-engine state; ROS/RMF/DDS state;
multi-robot execution; universal schema; USO runtime unification; delta
checkpoints; GUI editing; or warehouse evaluation. The selected scenario's
registered callback code is recreated by the application, and its mutable
stage is saved through an explicit application-state hook. An undeclared
stateful callback or plugin makes a checkpoint unsupported; V1 must fail
closed instead of silently omitting it.

The extension contract must remain possible: a future owner can declare a
state key, schema version, capture/restore operation, and restore-order
dependencies for a controller, Action, BT, plugin, callback or custom
SimObject. Restoring code itself is not implied. The V1 implementation should
define the completeness and unsupported-state rule and one concrete external
scenario-state participant; it should not introduce an unused generic
`SimObject.save/load` interface or generic plugin serializer. A later profile
can test and refine that participant contract before claiming arbitrary
extension support. Physics-off lets PyBullet engine-state restoration wait.

## End-to-end acceptance demonstration

1. Reuse the PR #55 sequence under the declared V1 profile in an ordinary
   `run_simulation()` call. Opt into state recording and persist ordered
   input/lifecycle records, every completed-step result frame and a full
   checkpoint at every completed boundary. CP1 and CP3 are selected later for
   restore tests. Show a finished manifest with coverage, assets, version,
   step/time convention and no incomplete checkpoint presented as usable.
2. Exit the source process. In a genuinely fresh process, select CP1, rebuild
   the supported world from artifact construction data plus the supported
   application entrypoint, restore the completed-step clock, live entity
   roster, robot/controller/joint state and external scenario stage, then
   continue with `run_simulation(resume=True)` through attach, detach and
   delete. Repeat from CP3 while the box is attached and the joint is moving.
3. Compare each restored boundary *before* continuation with the source and
   compare every subsequent completed-step robot pose, joint position, box
   presence/pose/attachment, step/time, terminal navigation state and ordered
   lifecycle outcomes with an uninterrupted reference run. Use declared
   numeric tolerances on one supported environment; do not claim bitwise or
   cross-platform identity. Assert box absence before spawn, attachment at
   CP3, detachment before removal and no stale live attachment after removal.
4. Open the source artifact for playback in another process. Seek CP1 and CP3,
   step through attach/detach/delete, inspect frames and ordered events, and
   prove that no controller/physics `step_once()` or application policy
   execution is used. Optional GUI display must use the same recorded data;
   `--rtf` controls display rate, not simulation execution. Playback must
   identify missing frames or intervals rather than invent intermediate state.
5. Reject incompatible profile/version, changed `dt` or asset fingerprint,
   missing required state, duplicate stable keys, unsupported active
   callbacks/plugins/Actions, and corrupt/incomplete artifacts before
   claiming restore. A failed durable write must fail the recording operation
   visibly. Document the exact coverage limits in the user-facing guide.

One V1 example command/entrypoint must let a normal user run the sequence,
record it, restart from CP1 or CP3 and play it back. Tests may use narrower
state fixtures, but cannot replace this complete demonstration.

## State and input inventory for implementation

| State/fact | Owner and current evidence | V1 capture/restore or record requirement |
| --- | --- | --- |
| Fixed timestep, physics mode, controller/robot configuration, asset identity | Core/world construction; #56 hard-codes `_make_sim()` | Persist effective supported construction and validate compatibility. Reconstruct without requiring the old process or its runtime IDs. |
| Completed step and elapsed simulation time | Core; `POST_STEP` occurs before counters advance | Capture only after a completed boundary. On restore, the next step evaluates from that boundary; distinguish callback phase/order for inputs issued inside a step. |
| Base pose/motion, active destination and omni trajectory phase | Agent/controller; #56 proves one straight-forward phase | Reuse #56 capture/restore where valid. Reject unimplemented omni phases rather than silently reissuing a goal; either constrain the V1 route to the proven phase or extend state for any phase the acceptance run actually reaches. |
| Joint position, target and interpolation progress | Agent; #55 shows public kinematic velocity is `0.0` while moving | Capture private execution facts through a supported Agent operation; restore target/progress and prove subsequent positions. Do not relabel reported `0.0` as measured velocity. |
| Live box construction, pose, membership and stable key | SimObject/core; #55 driver owns spawn parameters | Persist effective construction and checkpoint roster. Map artifact key to fresh runtime object on restore. |
| Parent, named link and relative attachment transform | SimObject/Agent; #55 public attachment view is incomplete | Provide a narrow supported capture/restore operation and rebuild relation after both entities exist; do not infer it from current world pose. |
| Accepted navigate/joint/attach/detach and spawn/delete operations | Fleet API plus core lifecycle paths | Record effective parameters, result/ack where available, stable targets, step, phase and order. A checkpoint's active goal does not replace the input history; no claim of complete arbitrary-input capture. |
| Scenario stage, next intended operation and any scenario timer | External application | Save via an explicit application-state participant, restore after PBF state and before resuming callbacks. This is not a PBF core variable. |
| Per-step poses, joint/attachment/lifecycle results and events | PBF observations plus scenario-owned outcomes | Store immutable result frames separately from execution checkpoints and ordered events separately from sampled roster changes. |

## Architecture and implementation sequence

1. **Confirm the vertical path.** Run the PR #55 sequence with the proposed
   omni profile and inspect its actual controller phases. Inventory all active
   mutable owners at CP1/CP3 and label each as PBF-owned or application-owned.
   This check blocks V1 because the existing manipulation example and #56
   use different controllers; it is not a separate feature proof.
2. **Establish one supported capture contract.** Add only the core/Agent/
   controller/SimObject state operations needed by the inventory, with
   profile-qualified validation. Expose a failure-visible completed-step
   capture point **after** `step_once()` advances its counters, usable while
   ordinary `run_simulation()` runs. `POST_STEP` and `EventBus.emit()` alone
   cannot provide this contract: POST_STEP observes old counters and EventBus
   logs handler exceptions instead of propagating durable write failures.
   Keep recording scheduling and file I/O in a PBF recording component outside
   the simulation core. Prefer a narrow synchronous completed-boundary
   observer or an equivalent explicit API;
   finalize its public shape during Architecture Review.
3. **Use one versioned artifact with distinct parts.** Proposed logical parts
   are a manifest/coverage declaration, supported construction, ordered
   effective-input and lifecycle journal, per-completed-step result frames,
   and full per-step checkpoints with external scenario-state sections. Reuse
   #50's streaming writer, checksums and validation ideas where they fit;
   its `simple_cube`/`static_box` initial schema cannot be relabeled as a
   mobile-manipulator checkpoint. Write checkpoints atomically and publish
   them as usable only after successful validation. A full checkpoint is
   proposed for one robot/box; delta encoding is not required. Measure
   per-step write cost and artifact size before confirming this default.
4. **Restore in a fresh process.** Validate profile, artifact integrity,
   assets and dependencies first. Construct the core and static world;
   construct live entities with new runtime IDs; initialize once; restore
   joint/base/controller execution state, then attachment relation and core
   clock; restore scenario stage and re-register callback code; compare the
   boundary; finally call `run_simulation(resume=True)`. Test the exact order
   against CP1 and CP3. Do not replay an attach or navigate command merely
   to synthesize the checkpoint state.
5. **Playback the result stream.** Provide a read-only step/seek cursor over
   recorded frames and ordered events. A small CLI/example may render those
   frames in GUI mode, but it must never execute the original controller or
   application. Playback can share artifact validation with restore while
   keeping a distinct capability claim.
6. **Verify and simplify.** Run the fresh-process comparison and playback
   demonstrations, focused state/negative tests, repository verification for
   the final source diff, docs build, and self-review against the product
   operation. Measure artifact size and enabled-vs-disabled step/RTF cost on
   the supported scenario; report measured cost without inventing a scale
   target. Replace redundant proof code only after its user/test coverage is
   retained. Do not add another evaluation scenario.

The candidate PR boundary is (1) ordinary recording plus CP1/CP3
checkpoint/restore/resume, which yields a useful restart operation, and (2)
recorded-result playback using the same artifact, which yields an independent
viewing operation. They remain one V1 theme and one acceptance demonstration;
each PR must say what remains. Combine them if implementation is reviewable.
Do not split by omni, joint, attachment or lifecycle simply because tests
cover them separately. A material change to the approved state ownership or
unsupported-state policy returns to Human Architecture Review.

## Relationship to existing work

- **#50:** Keep its restricted initial-state/input re-execution working while
  V1 is built. Reuse writer/validation mechanisms selectively; do not force
  the new checkpoint profile into its fixed-world schema. Its public API is a
  deprecation candidate **only after** a replacement reproduces its concrete
  user operation, a migration path is documented, and Human Architecture
  Review approves the compatibility change. General manipulation
  re-execution is not claimed merely because the V1 journal exists.
- **#55:** Reuse the scenario sequence, URDF, lifecycle and CP inventory.
  Adapt the example or share small setup helpers only where this reduces
  duplication without creating a general manipulation framework.
- **#56:** Reuse the completed-step clock restore, `run_simulation(resume=True)`
  and supported omni trajectory state. Replace its hard-coded world
  reconstruction for V1 rather than adding another isolated proof.
- **SimulationRecorder:** Keep the existing video recorder distinct from
  execution-state recording. The two may run together but have separate
  completeness claims.

## Human decisions required before implementation

1. **Scope:** Approve the single-manipulator, direct-command, physics-off
   sequence with CP1 and CP3 as the V1 acceptance profile. Confirm that full
   manipulation re-execution is later, while #50's restricted re-execution
   remains available during V1.
2. **Controller contingency:** Approve omni as the initial controller and a
   mandatory early compatibility check against the PR #55 route. If it fails,
   choose between adapting the route and bringing differential into V1; do
   not silently narrow the acceptance sequence.
3. **Architecture:** Approve PBF-owned state recording, artifact handling and
   restore/playback tools, implemented outside the simulation core, plus a
   narrow failure-visible completed-step capture point and profile-specific
   PBF state operations. The application owns its process and policy. Decide
   whether the synchronous observer is a public hook or internal to the
   supported recorder attachment API.
4. **Extension boundary:** Approve explicit completeness declarations and
   fail-closed handling of undeclared stateful plugins/callbacks/custom
   objects. V1 supports one external scenario-state hook and defines a
   future participant contract without claiming arbitrary serialization.
5. **User surface:** Approve a Python API plus one scenario-specific CLI with
   optional GUI for record/restart/playback. Prefer a simple recording config
   (`record=True` plus optional output/frequency) over mandatory `with` usage.
   Exact names, directory naming, checkpoint selection syntax and display
   pacing can be settled during implementation without changing the
   capability boundary.
6. **Checkpoint cadence:** Approve full checkpoints every completed step for
   the small V1 profile, subject to measured storage and step-time cost.
   Changing that default or relying on re-execution between sparse
   checkpoints would require explicit review of the user-visible time
   selection behavior.

`--time` selecting the latest checkpoint at or before the requested time is
the product direction. Whether V1 continues from that checkpoint directly or
re-executes recorded inputs to exactly the requested time remains open; the
CP1/CP3 acceptance demonstration tests two representative choices from the
per-step checkpoint sequence. The former is the proposed V1 behavior, and any
stronger claim needs a separate test of the intervening input coverage.
