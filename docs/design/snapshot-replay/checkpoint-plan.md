# Single-omni checkpoint/restore proof — implementation plan

**Status:** Scope and architecture approved; implemented for Human Review.
See [implementation evidence](checkpoint-evidence.md) for actual results.

## Question and supported profile

Prove the minimum state needed to stop a process while one built-in per-agent
`OmniController` is navigating, reconstruct the same physics-off, fixed-step,
fixed-world simulation in a fresh process, and continue the same movement.
Use one packaged robot, one straight planar destination, no auto-approach
waypoint or final orientation change, no callbacks/plugins/Actions/batch
controller, and a completed-step checkpoint during acceleration. A headless
reference run and a headless fresh-process run use the same effective initial
construction/configuration. Optional GUI viewing is not part of the proof.

The four claims are distinct: **observable** values can be read;
**serializable** values can cross the process boundary; **restorable** values
can be applied to a new supported instance; **resumable** means subsequent
ordinary steps match the reference within the declared tolerance. Current
replay observations establish only the first claim. No row below is currently
claimed to satisfy all four.

## Repository findings that constrain the proof

`Agent.set_goal_pose()` delegates to `set_path()`, which may add an
auto-approach waypoint for a goal more than 0.5 m away. Keep the first profile
to exactly one effective waypoint and verify it after issuing the goal. The
controller's `POSE`/`FORWARD` path state determines completion, while its
forward `TwoPointInterpolation` is evaluated at `sim.sim_time` from the
**original** trajectory start. Reissuing the goal from the checkpoint pose
would create a different acceleration profile.

`step_once()` sets `sim.sim_time` from `_elapsed_sim_time` before object updates;
it increments `_elapsed_sim_time` and `_step_count` only after `POST_STEP`.
Thus after completed step `S_k`, the canonical elapsed time and next evaluation
time are `k * dt`, although the public `sim_time` field still reflects the
evaluation time of that last step. Capture only after `step_once()` returns,
not inside `POST_STEP`; restore a coherent clock so the next step evaluates at
`k * dt`. Compare the completed-step index and canonical elapsed time, not a
`POST_STEP` callback's stale counter. `run_simulation()` currently calls
`initialize_simulation()` on entry, resetting the restored counters, and
disconnects on exit. The restore proof therefore needs an explicit way to
enter its existing run loop **without** reinitializing the loaded state; the
user should not have to own a `step_once()` loop after loading. In this
profile, a narrow `run_simulation(resume=True)` option is the recommended
contract. A plain `run_simulation()` should retain its current fresh-run
behavior; making it silently detect restored state would require a broader
core lifecycle contract.

## Proposed minimum state inventory and ownership

| State | Owner | Proposed capture/restore requirement |
| --- | --- | --- |
| Effective robot construction, asset locator, built-in controller type and limits, physics-off setting, fixed `dt` | Profile metadata / world-entity construction | Recreate the same supported instance before loading mutable state. Validate profile, asset and configuration compatibility; runtime PyBullet/PBF IDs need not match. |
| Completed step `k` and elapsed time | Simulation/core | Restore the canonical completed-step clock. For fixed `dt`, validate elapsed time against `k * dt`; make the next `step_once()` evaluate at `k * dt`. Do not restore wall-clock pacing as simulation state. |
| Robot pose, orientation, reported linear/angular velocity and moving flag | Agent/world entity | Restore pose and PBF motion fields so `S_k` matches before advancing. Reported velocity is observable/comparison state; it does not replace the controller trajectory. |
| Active `POSE`/`FORWARD` mode, one effective goal/path and completion-relevant flags | Controller execution and command/goal state | Preserve the active destination and enough path state for the existing completion path to reach `IDLE` and clear `is_moving` exactly once. Reject other modes, phases, extra waypoints or final rotation in this profile. |
| Forward trajectory origin, direction/distance, start time and effective speed/acceleration definition | Controller execution state | Reconstruct an equivalent TPI and cached forward vectors without using the checkpoint pose as the new origin. Determine the smallest explicit parameters by testing; do not pickle the third-party TPI object or copy all controller private fields. |
| Profile ID/version and fixed-configuration identity | Profile-specific checkpoint metadata | Reject missing, incompatible or nonfinite data before applying any state. Keep this checkpoint distinct from initial definitions, input journals and sampled observations. |

The current controller has related internal fields such as `_path`,
`_current_waypoint_index`, `_goal_pose`, `_mode`, `_pose_phase`,
`_forward_start_pos`, `_forward_direction_3d`, `_forward_direction_unit`,
`_forward_total_distance_3d`, and `_tpi_forward`. This is a **dependency
inventory**, not a request to serialize every field literally. For example,
direction and distance may be derived from a validated trajectory origin and
goal; effective TPI limits and start time must still reconstruct the same
curve. During implementation, prove which fields are derivable and reject a
record if required state cannot be reconstructed. No generic `SimObject`
save/load interface is proposed.

## Capture, persistence and restore order

1. Build the reference and source runs independently from equivalent effective
   initial conditions. Issue one navigation goal and verify the effective
   controller path has one waypoint and no rotation/final-alignment phase.
2. In the source process, finish `step_once()` for `S_k` while acceleration is
   active; capture the completed-step clock, agent/world state and validated
   controller/goal state. Serialize a strict, versioned, profile-specific JSON
   checkpoint with the effective initial construction/configuration needed by
   the fresh process. Write it completely before ending the source process.
3. In a genuinely fresh process, validate the artifact and profile first.
   Construct the fixed world and robot, initialize the core once, then restore
   clock, pose/motion and controller state in a documented order. Check
   cross-field invariants before mutation where possible, and fail rather than
   continue with an incomplete or unsupported checkpoint. Do not emit a new
   navigation command during restore.
4. Inspect the restored `S_k` state before another step. Call the normal
   simulation run loop with explicit resume intent (proposed
   `sim.run_simulation(resume=True)`) so it continues from `S_{k+1}` through
   arrival without resetting the loaded clock. The user does not manually
   step the restored simulation. External code owns process orchestration,
   file I/O and comparison; PBF owns its ordinary run loop and narrow
   supported state operations.

The single-file JSON is a proof artifact, not a universal snapshot schema or
an extension of v1 `observations.jsonl`. It may reuse existing PBF validation
helpers where appropriate, but must not imply that the v1 re-execution profile
supports intermediate restore. The artifact records a stable robot identity
for matching across processes, not a promise that runtime object IDs coincide.

## Comparison and verification

Use an uninterrupted reference run captured from the same initial conditions.
At `S_k`, compare pose/orientation, reported velocity, moving status, active
goal, step and elapsed time with both the source checkpoint and restored state.
Then compare **every** resumed completed-step pose and velocity through arrival,
the step of `is_moving` becoming false, the final pose, and absence of an
extra goal command. Compare the same step index rather than wall time or RTF.
The proposed run-loop option must preserve the restored clock and interpret
`duration` as the same absolute simulation-time cutoff used today; an already
passed cutoff must not restart the run. The first proof uses `target_rtf=0`,
so wall-clock pacing is outside its correctness claim. If nonzero RTF is later
supported, pacing must be rebased at resume rather than waiting for elapsed
time since zero.
Use a documented floating-point tolerance suitable for one machine/profile;
bitwise or cross-platform identity is not required. Select `S_k` strictly
inside acceleration, and include a negative control that reissues the goal
from the checkpoint pose and demonstrably diverges from the reference suffix.

Tests should also reject wrong profile/version, changed `dt`/controller limits,
incomplete or nonfinite trajectory data, wrong goal/path shape, and a capture
attempt from within an incomplete step. Verify source process termination
before creating the restoration process. A scenario-level test should assert
the complete observable → serializable → restorable → resumable chain, not
just equality of the final destination. Run focused tests, `make verify`, docs
build and self-review after implementation.

## Architecture decisions for Human review

1. **Narrow state interface versus external private-field access.** Recommend
   profile-specific capture/restore operations owned by the core clock,
   Agent and built-in controller, with strict validation. The external
   coordinator should never reconstruct a third-party TPI by mutating private
   fields. This adds a small supported state contract, not a generic
   `SimObject` serializer. Approve this boundary and the fact that its exact
   method names/record types are implementation details to review later.
2. **Execution entry after restore.** Recommend a narrow explicit resume mode
   on `run_simulation()` that skips fresh-run initialization after a validated
   checkpoint load and uses the existing loop. The normal call remains a fresh
   run. This is a small public core API change; it is justified by the user
   operation `load checkpoint → run simulation`, while checkpoint file I/O and
   restore sequencing stay external. Human review should confirm the explicit
   flag rather than an implicit restored-state switch. The capture-side proof
   may step externally to stop exactly at `S_k`; the resumed user workflow
   should not require a manual step loop.
3. **Artifact scope.** Recommend one strict JSON checkpoint for the single
   profile, separate from replay v1. It carries only supported effective
   construction, clock, agent and controller state. No shared PBF/USO format
   or generic per-object save/load contract is approved by this choice.

Human Architecture Approval is needed for these three boundaries before
implementation. If inspection during implementation shows that the proposed
profile cannot restore and run without a larger core responsibility or a
broader public API, stop and return for review. Actions/BTs, attachments,
entity lifecycle, multiple robots, physics, arbitrary plugins/devices,
ROS/RMF, playback and arbitrary-time checkpoints remain out of scope.
