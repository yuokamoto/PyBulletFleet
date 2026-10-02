# AI Workflow Learnings

Reusable observations from completed development work. These are candidates for
future workflow improvement, not new requirements or an activity log.

## Distinguish the final capability from the incremental slice

**Occurrences:** 1 (#50, snapshot/replay v1)

**Evidence:** The approved v1 implemented initial-state input re-execution, but
review discussions had to repeatedly distinguish it from recorded-result
playback and intermediate checkpoint restore/resume. The final user goals and
the limited role of v1 were then made explicit in the specification, user guide,
and roadmap.

**Reusable check:** For an incremental v1, describe both the final intended
capability and the capability delivered by this slice as user actions and
observable results. In replay scopes, name playback, re-execution, and
restore/resume separately before deriving required state and inputs.

**Candidate destination:** Snapshot/replay design template or repo-local Skill,
if the distinction causes confusion again. **Status:** watch; no promotion yet.

## Separate simulator capabilities from replay orchestration

**Occurrences:** 1 (#50, snapshot/replay v1)

**Evidence:** The first implementation put replay-session-dependent behavior in
simulation and model paths. Human review clarified that simulator internals may
need to expose or restore required state and inputs, while recording, replay,
restore, and resume workflows should be controlled externally. The implementation
was refactored so `ReplaySession` applies inputs and advances ordinary steps.

**Reusable check:** During future scope and architecture review, distinguish
simulator-internal save/load or input-access capabilities from external
orchestration before adding hooks to core simulation paths. Define what direct,
out-of-session mutations mean for the recording contract.

**Candidate destination:** Snapshot/replay architecture guidance, if another
slice needs the same correction. **Status:** watch; no promotion yet.

## Preserve existing mode semantics during a localized correctness fix

**Occurrences:** 1 (#52, collision-margin broadphase)

**Evidence:** The margin fix went through several broader implementations before
review narrowed it to the candidate AABB envelope. Independent review then
found that the initial fix changed the 2D mode's Z-overlap behavior. A focused
regression restored that existing mode contract.

**Reusable check:** State the invariant being repaired and the behavior of each
existing mode before changing shared candidate extraction. Keep the fix at the
smallest layer that enforces the invariant, and test a counterexample plus mode
boundaries.

**Candidate destination:** Collision/performance development guidance if this
recurs. **Status:** watch; no promotion yet.
