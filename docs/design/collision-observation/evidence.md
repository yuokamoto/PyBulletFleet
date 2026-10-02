# Collision observation — implementation evidence

**Status:** Implementation complete; awaiting independent and Human Final Review.

The core retains `_active_collision_pairs` as the state used for enter/exit
transitions. A separate pair-keyed cache holds the last narrow-phase method,
signed closest-point distance where available, and the pair's sample step/time.
An object-to-pairs reverse index limits removal/disable cleanup to the removed
object's active neighbors; it is maintained on pair entry/exit and reset.
It adds references to each active pair under both object IDs. This trades
memory proportional to active pairs for removal work proportional to the
removed object's active neighbors, without scanning unrelated pairs.
In a synthetic 100,001-pair check where the removed ID had one pair, removal
took about 0.045 ms; the former-style full-set filter took about 40.8 ms.
This isolated comparison illustrates the complexity difference, not typical
fleet runtime.
The public `get_collision_observation()` constructs immutable values from that
cache and the last completed check's metadata when called. No second all-pairs
geometry query is performed by that API or by the
corridor evaluator. A skipped check leaves its check timestamp unchanged; a
pair not rechecked during a completed check retains its earlier sample time.
The evaluator carries such an active episode without adding a new observed
step or splitting it.

## Scenario comparison

At the 300 s headless cutoff, the old all-pair collector and the new PBF
observation collector produced the same episode counts:

| Policy | Collector | Margin-only | Geometric overlap | Corridor overlap | Core margin entries |
| --- | --- | ---: | ---: | ---: | ---: |
| Uncontrolled | Old / new | 44 / 44 | 180 / 180 | 100 / 100 | 180 / 180 |
| Direction gate | Old / new | 46 / 46 | 116 / 116 | 0 / 0 | 116 / 116 |

The policies and task logic were unchanged. These counts demonstrate parity
for this scenario; they do not make the public observation an all-pairs survey.
The original collector queried every robot/robot and robot/wall pair and can
still be appropriate for an explicit all-pairs study.

## Verification and cost

- Focused collision and corridor tests: 92 passed. They cover distance sign,
  margin zero, persistence and clearing, hybrid and contact methods, skipped
  checks, and object disable.
- Post-index `make verify`: 1,795 passed, 12 skipped; 81.44% coverage.
  Focused collision, core-simulation and corridor tests: 258 passed.
- `PRE_COMMIT_HOME=/tmp/pbf-pre-commit make lint`: passed. Documentation build
  also passed with `LC_ALL=C LANG=C make docs`; the locale override is needed
  by the local shell.
- In a serial, dense-scene microbenchmark with all objects marked moved,
  100 objects produced 342 active pairs. The pre-change collision-check median
  was 21.0–21.5 ms across two runs; the current check median was 21.78 ms and
  the on-demand getter median was 0.15 ms. At 1,000 objects and 3,811 active
  pairs, pre-change check medians were 648–657 ms; the current check median
  was 654.32 ms and the getter median was 2.32 ms. Each run discarded one
  warmup and used 20 measured checks at 100 objects or five at 1,000.
  These short measurements show no clear material regression; they are not a
  release performance guarantee. The cache holds at most one record per active
  qualifying pair plus last-check metadata. The published tuple is assembled
  only when read; it is not retained by the core.

## Review limits

Contact-point records intentionally have no signed distance. The API is a
sampled state and does not report collisions that begin and end between checks.
PBF's candidate filters and disabled/static-object rules still define which
pairs qualify. The example's episode durations count newly sampled steps, so
they need not equal continuous contact duration.

The evaluator reconciles episodes with `check.pairs` rather than relying only
on `COLLISION_ENDED`. That event is emitted when a pair resolves during a
collision check, but object removal/disable and a broadphase filter reset can
clear active pairs without emitting it. An event-only collector would still
need a state reconciliation path. `check.pairs` supplies that current state;
the evaluator's pair-keyed mapping associates PBF object IDs with scenario
pair names.

## Retrospective follow-up: step/time contract

`step_once()` advances state and checks collisions before it increments
`step_count` and elapsed simulation time. `POST_STEP` therefore sees the new
state with the old core counters; collision observation currently labels that
state as `step_count + 1` and `elapsed_sim_time + dt`. A manual collision check
inside `PRE_STEP` can also receive the next-step label because `_in_step` is
already true. This is a simulator-wide timing contract question, not a reason
to change step order within the collision-observation slice.

For next-work prioritization, compare this boundary with playback, checkpoint,
trace and other simulator gaps. If selected, investigate all `PRE_STEP` and
`POST_STEP` consumers, replay artifacts and metric timestamps, then propose a
single rule: pre-step refers to the interval start and post-step to its end.
Do not move the counter update in this change without that compatibility
review. This is a technical follow-up candidate, not an AI workflow learning.
