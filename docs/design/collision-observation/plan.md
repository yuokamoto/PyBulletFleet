# Collision observation — implementation plan

**Status:** Scope and architecture approved; implementation complete, awaiting review.

## Findings

`check_collisions()` first obtains candidate pairs from `filter_aabb_pairs()`.
It checks candidates plus active pairs involving moved objects, then stores only
the active pair set. For `CLOSEST_POINTS`, PyBullet already returns closest-point
tuples at the configured margin. `HYBRID` uses the same query for two kinematic
objects and `getContactPoints()` when either object uses physics.
`CONTACT_POINTS` uses `getContactPoints()` throughout. The returned tuples are
currently reduced to a boolean; no second geometry query is needed to retain
the minimum `point[8]` for a closest-point pair. Contact-point records must be
marked as contact observations, without claiming the configured margin was
used as their threshold.

`step_once()` performs the collision check before `POST_STEP`, then increments
`step_count` and elapsed simulation time after the callback. Thus a check made
during a step represents the state at the end of that step even though the
current counters have not yet advanced. `collision_check_frequency=None` means
every step, a positive rate may skip steps, and zero disables automatic checks.
Direct calls to `check_collisions()` can also occur between steps. These paths
must share one explicit observation timestamp rule.

The corridor example reads every eligible pair through private body IDs after
each step. The proposed API would replace that query only for pairs accepted
by PBF's collision pipeline. It would preserve the external episode tracking,
geometry-specific filtering, task metrics and policy logic. Since the core may
ignore static pairs or disabled objects, the API must not promise an all-pairs
distance survey.

## Recommended public contract

Add `get_collision_observation()` on `MultiRobotSimulationCore`, returning a
read-only value containing the most recent **completed check** and its active
qualifying pairs. The check value should contain `step`, `sim_time`, effective
check method, configured margin, and immutable pair records. Each pair record
should have sorted PBF object IDs, the branch actually used (`closest_points`
or `contact_points`), and `signed_distance: float | None`. For a closest-point
pair, use the minimum finite `point[8]`; nonpositive values mean geometric
touch/overlap and positive values mean within the configured clearance. For a
contact-point pair, return `None` unless its distance semantics are confirmed
in focused tests and documented separately. The top-level method is useful for
interpreting hybrid configuration; the per-pair branch is authoritative. Name
these fields `configured_method` on the check and `detection_method` on each
pair so their different meanings are visible in the API.

No completed check yet should return `None`. A completed check with no active
pairs should return an observation with an empty pair collection. A skipped
step should leave the last observation and its timestamp unchanged; callers
can compare its `step` with the current simulation step to detect staleness.
This is a sampled state, not a history or an event stream. An explicit manual
check publishes an observation at the currently completed step/time. During
`step_once()`, publish it for the step being completed, before `POST_STEP` so
that subscribers see it there. Do not infer that a contact persisted during
the interval between checks.

Use immutable plain values (for example, frozen dataclasses and a tuple of
pair records) so callers cannot change core state. Preserve the existing
`check_collisions()` return shape, `get_active_collision_pairs()`, collision
count and started/ended events. No Fleet API wrapper is needed for this first
Python-library use; the scenario already holds the simulation-core instance.

Keep `_active_collision_pairs` as the set used for membership and enter/exit
events. Store observation metadata in a separate dictionary keyed by the same
sorted pair and retain only the last check's step/time/configuration. Construct
an immutable view when the getter is called. This leaves the existing
transition logic and public pair-list API intact, without retaining two copies
of the pair collection.
An object-to-active-pairs reverse index keeps object removal and disable
proportional to that object's active neighbors rather than scanning all active
pairs; update it only when a pair enters or exits, and clear it with the active
set on reset or filter changes.

## Implementation sequence

1. Define the narrow public record types and getter, document their sampling
   and mode semantics. Store only the latest completed check, not a timeline.
2. In each existing narrow-phase branch, retain the minimum signed distance
   from its existing closest-point result for qualifying pairs. Populate the
   latest active-pair records and clear records for resolved or removed pairs.
   Preserve a record for an unchanged active pair until it is rechecked, but
   keep the check timestamp honest: if a pair was not rechecked in a completed
   check, its distance must carry its own `sample_step/time` or be marked stale.
   Prefer per-pair sample timestamps to fabricating a new measurement.
3. Set the check timestamp consistently for automatic and direct checks.
   Reset/invalidate the cached observation on simulation reset, object
   removal/disable and other lifecycle paths that already clear active pairs.
4. Replace the corridor example's private `getClosestPoints` loop with the
   public observation for its current closest-point profile. Keep the same
   pair category, corridor geometry and episode accounting. Explicitly note
   that the source changes from all-pairs sampling to PBF-qualified pairs and
   compare both outputs before claiming metric equivalence.
5. Update collision documentation, the corridor guide/evidence, and the
   `[Unreleased]` changelog entry for the public API and measurement change.

## Verification

- Focused tests: positive gap within margin, geometric overlap, zero margin,
  pair entry/persistence/exit, object removal/disable, empty/no-check state,
  manual check, check-frequency skips, and `POST_STEP` timestamp visibility.
- Run closest-point, contact-point and hybrid fixtures. Verify per-pair branch
  and `None` distance in contact mode; include normal 2D and 3D cases so PR
  #52's mode behavior stays intact.
- Re-run the 20-robot corridor scenario for both policies with the old and
  proposed collectors under the same fixed conditions. Compare qualifying
  pair categories and episode counts. Investigate differences caused by
  static filtering or incremental checks instead of silently changing the
  published result.
- Benchmark 20-robot scenario and representative 100/1000-object collision
  checks at every-step and reduced cadences. Report check/step wall time,
  active-pair count and retained observation memory against the current code.
  Use the existing collision benchmark carefully: its explicit moved-object
  marking is needed to measure a full check rather than an empty incremental
  pass. Do not assert a performance limit before baseline measurements.
- Run focused tests, `make verify` and `make docs`; self-review the new public
  contract, sampling limits and any changed scenario measurements.

## Architecture decision for Human review

The meaningful decision is whether to retain the latest per-pair observation
on every collision check or require an opt-in setting. **Recommendation:**
retain it by default, bounded to active qualifying pairs and last-check
metadata. It makes the read-only API usable from callbacks without changing the
check workflow or adding a second all-pairs pass. The added allocation and memory
cost must be measured as above; if material, return with an opt-in alternative
before implementation expands. Approval should also confirm the proposed
sampled-state contract: stale pairs carry their own sample timestamp, and
contact-mode distance may be absent. No generalized collision architecture is
proposed.
