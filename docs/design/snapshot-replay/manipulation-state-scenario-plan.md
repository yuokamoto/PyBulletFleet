# Manipulation and entity-lifecycle state scenario — implementation plan

**Status:** Scenario implemented locally after Human Scope and Plan Approval;
pending Human Review. No checkpoint/restore architecture approval is implied.

## Purpose and boundary

Build one small, physics-off scenario to discover the state needed to inspect
and eventually continue a mobile manipulator run. This change exercises existing
PBF capabilities; it does not implement checkpoint/restore, state recording,
playback, or a reusable manipulation framework. The scenario driver owns its
stage transitions. PBF owns the robot, object, movement, joint and attachment
facts. Keep the existing `mobile_manipulator_demo.py` intact for this slice.

## Existing capability and proposed sequence

The existing mobile-manipulator example supplies a combined mobile-base/arm
model and demonstrates kinematic joint movement and link attachments. Use its
model and a simple pickable box, but a shorter, explicitly staged driver using
ordinary PBF APIs:

1. Start with the mobile manipulator and no box. At a defined completed-step
   boundary, spawn one box through `SimObject.from_params()` or `from_mesh()`.
2. Navigate the base toward the box. Observe CP1 while navigation is in
   progress, after the box exists.
3. Set an arm joint target. Observe CP2 while the joints are moving.
4. Attach the box to a named robot link with an explicit relative transform.
   Move the arm with the box attached and observe CP3. A short base movement
   while attached is optional only if the simpler sequence does not expose
   base/attachment interaction adequately.
5. Detach the box and observe CP4 before deletion. At a later defined step,
   call `sim.remove_object(box)` and confirm the box is absent. This final
   absence observation is an additional lifecycle boundary, not a checkpoint
   implementation. Do not delete an attached object in this first scenario.

Use one fixed timestep, fixed asset paths and joint/link names, and explicit
command/operation steps. Direct Fleet API navigation, joint and attach/detach
commands are preferred where supported; direct object creation and removal use
the existing PBF object/core APIs because Fleet API has no spawn/delete command.
Do not use IK, `PickAction`, a generic action queue, plugins, or ROS. If the
actual model or command semantics make any stage unreliable, report the gap
before adding a core abstraction.

## State questions at the observation points

| Point | World observation | State needed for a future supported continuation | External driver state |
| --- | --- | --- | --- |
| CP1: base moving, box present | Base pose/motion, box identity/pose, joint values, completed step/time | Base controller target, trajectory and timing; current entity roster and construction data | Next stage and its trigger |
| CP2: arm moving | CP1 facts plus changing joint positions and reported velocities | Joint targets, interpolation/motor state and any active base control | When to attach |
| CP3: box attached and arm moving | Parent/child identities, parent link, relative transform, box world pose, joint values | Link-following/constraint state plus continuing controller state | When to detach |
| CP4: box detached | Independent box pose and absent attachment relation | Remaining controller state; any later delete is a separate input | Delete trigger |
| After deletion | Box absent from live roster; surviving entities unchanged | No stale live attachment/lookup references | Completion stage |

The present `get_joint_state()` reports zero velocity for kinematically
interpolated joints; do not mistake that for measured zero motion. Public
`get_attached_objects()` / `is_attached()` do not expose the full parent-link and
relative-transform relation from an arbitrary child. The scenario can assert
the relation it commanded, but this is not yet a general checkpoint read API.
Record these as findings, not as automatic API additions.

## Identity and lifecycle interpretation

PBF already gives each live `SimObject` an `object_id` distinct from PyBullet's
`body_id`. It is unique within a core run and resets with the simulation; it
is not a durable cross-run ID. The v1 replay artifact already uses a separate
stable `entity_id` and maps it to the runtime object ID. For this scenario, use
one driver-owned stable ID such as `box-001` and an explicit mapping to the
created object. Do not add another core ID or promise that either runtime ID
will match in a second process. Names are descriptive and not globally unique.

A full observation at a step can list the entities then alive. Comparing two
such lists reveals the **net** additions and removals between samples, if IDs
remain stable. It cannot recover a spawn and delete that both occurred between
samples, their order relative to other inputs, or the effective constructor
arguments. Exact initial-state re-execution therefore needs ordered lifecycle
inputs; a checkpoint at CP1–CP4 needs the current roster and sufficient
construction state; post-run state playback needs sampled rosters at the
chosen cadence. None of these is implemented by this scenario. Include both
spawn and delete in the scenario so these requirements are observable.

USO's draft full snapshot contains an asset map; its illustrative delta has
`new_assets` and `removed_assets`. This supports the existence-change concept,
but does not settle PBF command ordering, runtime-ID mapping, link-level
attachment transforms, or execution-controller state. No schema adoption or
USO repository change is proposed.

## Verification and evidence

- A deterministic, headless run completes the staged sequence using existing
  APIs, with tests checking the expected pose/joint changes, attach-follow
  behavior, and object presence before spawn, after spawn, after detach, and
  after delete. Assertions identify the completed simulation step and phase.
- Confirm that creation/removal events and the live roster agree for this one
  object. Test that a removed box is not reported as still attached or live;
  if current behavior fails, report the missing capability rather than
  silently adding a generalized lifecycle mechanism.
- Inspect CP1–CP4 in memory and produce a short state inventory: obtainable
  through public APIs, only obtainable through private state, and owned by the
  external driver. This is investigation evidence, not a persisted snapshot or
  replay artifact.
- Run focused tests and repository verification. Measure ordinary scenario
  runtime only to catch an obvious regression; no scale/performance claim is
  part of this one-robot proof.

## Future relationship to existing demos and CLI

This scenario may eventually replace the state-discovery portion of
`mobile_manipulator_demo.py`, or that demo may reuse its stages. Replacing the
whole demo is not a goal: it currently demonstrates richer `PickAction`/IK
behavior. A future inspect/record/load CLI can operate across examples only
when their effective inputs, construction data, required controller/action
state and external driver state are covered by an approved execution profile.
The CLI alone cannot make arbitrary callbacks, plugins or examples restorable.

## Human architecture review

No material core architecture or public API change is planned for this
scenario, so no Architecture Gate is required before implementing the approved
scenario with existing APIs. If the first implementation needs a new public
attachment/lifecycle-state API, or changes `remove_object()` semantics, stop
and return for Human Architecture Review. Future checkpoint format, lifecycle
journal, stable-ID ownership and CLI scope require separate decisions.
