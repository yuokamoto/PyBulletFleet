# Fleet corridor evaluation — implementation plan

**Status:** Plan for Human Architecture Approval. Scope approved for a synthetic
20-robot corridor scenario and an external evaluator/measurement contract;
implementation has not begun. See [scope draft](spec.md).

## Repository findings and constraints

- `MultiRobotSimulationCore.step_once()` commits kinematic poses, refreshes
  collision geometry, checks collisions, then emits `POST_STEP`. With
  `collision_check_frequency=None`, it checks once per completed step; a
  positive Hz value may skip steps, and zero disables checks.
- In kinematic mode, `CLOSEST_POINTS` uses `p.getClosestPoints(...,
  distance=collision_margin)`. The core maintains one set of active pairs and
  emits `COLLISION_STARTED`/`COLLISION_ENDED` when that threshold is crossed.
  It does not retain separate near-miss and geometric-overlap event streams.
  Its public cumulative `collision_count` counts new threshold-crossing pairs,
  including positive-distance margin intrusions; it is not an overlap count.
  `CONTACT_POINTS` reads a physics contact cache and is unsuitable as the
  primary kinematic measurement without a collision-detection pass.
- The `getClosestPoints` result exposes a signed contact distance. A focused
  geometry test should verify whether the scenario can classify the same
  candidate pair as **safety-margin intrusion** (`0 < distance <= margin`) or
  **geometric touch/overlap** (`distance <= 0`) after each completed step.
  These are geometric observations, not solver contact forces or continuous
  collision detection. Do not label the latter a physical impact.
- `FleetCommandDispatcher.navigate/stop` returns acceptance/rejection acks;
  task completion must be observed after stepping. `FleetStateProvider`, Agent
  pose/motion state, core collision pairs and lifecycle events are available.
  Existing `MoveAction` timing is not automatically the proposed Fleet API
  movement-task timing. `MultiRobotSimulationCore` already exposes `sim_time`,
  `step_count`, `collision_count` and `get_active_collision_pairs()`, but it
  does not expose movement-command completion-time or completion-rate metrics
  as a reusable public contract.
- PR #50's `ReplaySession` owns a restricted world and replays saved effective
  navigate/stop commands. It cannot record arbitrary state-dependent policy
  decisions in an existing simulation, and replaying Policy A's commands under
  Policy B would freeze A's decisions. The comparison therefore needs two
  independent runs from the same scenario/workload definition, each executing
  its own policy. Artifact replay integration is out of scope.

## Architecture and data flow

Keep the first slice in an example/scenario package and tests, with no replay
hooks or task-evaluation logic added to `MultiRobotSimulationCore`. Add a
small opt-in, public PBF evaluation component for movement-task metric inputs
and summaries where existing PBF access is insufficient:

```text
fixed scene + immutable workload + policy parameters
                 |
       external scenario driver
       |        |          |
 Fleet API    step_once   post-step state/collision query
       |        |          |
       +---- PBF metric records/summary, driven by external collector
                    |
          per-run records + comparison report
```

The scenario owns stable robot/task IDs, deterministic request order, fixed
spawn/route geometry, and policy state. It is an example of an external fleet
management application: it may live in the PBF repository for verification,
but uses PBF as a library rather than becoming part of the simulation core.
The external collector observes Fleet API commands and their outcomes and feeds
a PBF-provided movement-metric surface. That surface should expose raw
per-command records and aggregate values
to callers, rather than only printing them or hiding them inside an example
file. PBF computes command-to-arrival durations and completion rates from
PBF-visible facts. The external app owns any time before its API call; PBF
does not report that as waiting time. Keep this a small, opt-in Python API,
not a general task engine or a claim that PBF judges algorithm quality. Use
existing public core/Fleet APIs for state and collision facts; add a narrow
read-only accessor only if feasibility shows a concrete missing observation.
Start with direct Python Fleet API calls, no ROS/DDS.

The proposed public surface is a small `pybullet_fleet.evaluation` module:
immutable movement-command lifecycle records and a pure summarizer accepting
accepted/rejected commands, terminal outcomes, observed collision
episodes and simulated cutoff. It returns typed per-command results plus a run
summary. Scenario code is responsible for identifying endpoint arrival and
supplying that fact. It reports admission delay separately in its own
comparison report, without passing pre-command release time to PBF metrics.
This keeps the metric formulas reusable and testable without putting task state
or policy decisions into the simulation core. Exact class/function names can
be settled during detailed
design; adding broad evaluator registration or plugin machinery is outside
scope.

Each individual PBF metric record should retain `run_id`, stable robot ID,
`command_id`, and simulation step/time for acceptance and terminal outcome.
The external app retains `task_id` and links it to the command ID. These are
correlation fields for a future operation trace, not a new trace API or a
requirement that all metric records become spans. The metric source and
aggregates must remain complete if a later trace exporter samples or fails.
Do not add a new global `operation_id` contract in this slice; a later tracing
design can map the existing IDs and define cross-process propagation.

**Common behavior for both policies:** define a local stop/hold rule for an
occupied path or safety zone, if needed to turn bidirectional conflict into
observable waiting. Without such a rule, the kinematic robots can simply
overlap and proceed; collision counts alone would not demonstrate congestion.
Keep this local rule identical in both runs. Policy A has no corridor admission
control; Policy B grants entry in one direction at a time, with a documented
fairness/timeout rule. Record each policy's admission, hold and release
decisions in a scenario-owned event stream. These records explain this
scenario only; they are not a generalized PBF trace API.

Use an explicit corridor rectangle and entrance/exit lines. A robot is in the
corridor according to its reference point under a documented geometry rule;
The external app may measure admission delay from its task release until it
issues `navigate`; this is not a PBF waiting metric. Once `navigate` is
accepted, the controller may start moving in the next step. Distinguish an
external admission delay from a robot stopped
inside or near the corridor. Track possible deadlock/stall as an outcome, not
an infinite simulation loop.

## Workload and provisional cutoff

Start with 20 kinematic `simple_cube` omni robots, ten staged on each side,
and two predeclared A-to-B/B-to-A movement requests per robot (40 tasks).
Issue the second request only after the first reaches its destination. Fix
poses, route waypoints, robot dimensions, collision mode, motion limits,
the timestep and dispatch order in the scenario configuration. Default to
`dt=0.1 s`, but allow a positive configured value that divides the 300 s
cutoff into an integer number of steps. The bundled
`simple_cube` collision box is 0.1 m wide; a provisional 0.25–0.35 m corridor
would admit roughly two or three such robots abreast. Use nonoverlapping
staging points, a roughly 6 m corridor and roughly 10 m endpoint route. Exact
dimensions and speed must be calibrated in the first feasibility run and then
frozen for both policies.

Propose a **300 s simulated-time cutoff**. For a roughly 10 m endpoint route
and 1 m/s nominal speed, a free traversal is on the order of 10–15 s. If a
robot occupies the proposed 6 m corridor for roughly 6 s, forty traversals
through an approximately two-robot-capacity corridor require at least about
120 s of corridor occupancy, before acceleration, direction switches and
waiting. This is only a rough capacity bound, not a throughput prediction.
Thus 300 s gives room for congestion while still exposing stalled or unfinished
work; it is not a promise that all tasks finish.
Before freezing the benchmark, pilot both policies and report the actual
uncontended traversal time and capacity bound. If the geometry makes the 300 s
window uninformative, revise the fixed cutoff in the plan for Human review
before using results for a comparative claim. Do not tune the cutoff separately
per policy or extend stalled runs until they look successful.

## Collision and sampling contract

Set kinematic `CLOSEST_POINTS`, `NORMAL_2D` robots, an explicit positive
`collision_margin`, and `collision_check_frequency=None`. Observe after every
completed step at the configured timestep (default `dt=0.1 s`). For each step,
map active object-ID pairs to stable
scenario robot IDs, query signed closest-point distance for those pairs and
emit categorized episodes. Keep the core's threshold transitions as evidence
of **margin intrusion**; an external collector can additionally classify
`distance <= 0` as **geometric touch/overlap** if the focused geometry test
confirms this query path. Preserve the numeric distance and threshold used.
Also sample the endpoints of episodes, so a short event seen for one step does
not disappear from a lower-rate summary.

At any configured step cadence, a collision occurring and clearing between two
completed steps is still unobservable. A smaller timestep improves temporal
resolution but does not provide continuous collision detection. Avoid language
such as “all collisions detected.”
Report counts of observed episodes and observed active-step durations, with
time resolution and any interval-censoring stated. If geometric classification
cannot be validated with existing APIs, retain only the margin-threshold
metric and report contact/overlap as unsupported for this slice; do not add a
generalized collision system.

## Task and metric contract

A task is one movement request from an origin staging endpoint to the opposite
destination endpoint. Its stable ID links the external release/admission record
with Fleet API ack, first movement, endpoint arrival, rejection and cutoff. A task completes
when the robot reaches the commanded destination pose within a fixed position
tolerance **and** is no longer moving, at a completed step. Command ack is not
completion. If the destination is repeatedly re-commanded, the task remains
one task; record every command ID. A rejected request is terminally rejected;
unissued second-leg tasks at cutoff are unfinished in the **external workload**
denominator, but are not PBF movement-command records.

Report: completed/40, rejected/failed/unfinished counts for the external
workload; completed movement commands per simulated second (optionally
normalized to commands/hour); PBF-visible command-to-arrival distribution;
observed margin-intrusion and
geometric-overlap episodes, and total simulated duration. Give censored
unfinished tasks separately; do not compute their nonexistent completion
times. Report wall duration, step time and RTF separately as experiment cost.
The external app also reports release-to-command admission delay and
release-to-arrival task time, clearly labeled as external-policy metrics.
PBF waiting time before an API call is zero/not applicable, never inferred
from pose. No single composite score or built-in “better policy” verdict.

Expose these measurements through the public PBF evaluation component so
another Python caller can obtain the same task records and summaries without
running this example. Existing core collision counters remain available, but
the per-run report should retain categorized pair/step evidence because a
single cumulative count does not distinguish margin intrusion from overlap.
The report must include a final collision count after task completion or the
fixed cutoff; it must not stop observing when the last task arrives.

For future general use, make the scenario report self-describing and versioned:
record run/scenario ID, conditions, units, timebase, robot/task IDs, metric
definitions, completion/censoring rules and references to detailed records.
Keep the corridor geometry and policy decisions in named scenario-specific
sections. A later general artifact can map or import these records; do not
extend PR #50's replay schema or freeze a universal artifact contract here.

## Implementation sequence

1. **Feasibility calibration:** in a disposable local script, use existing
   spawn, Fleet API and step APIs to confirm two-sided traffic, staging,
   post-step collision classification and a finite 300 s run. Record actual
   geometry, no-conflict traversal time and whether the common hold rule
   produces measurable congestion. If it cannot, return with evidence and a
   revised scenario proposal rather than implementing an unrelated feature.
2. **Scenario driver:** add a deterministic, reusable example module under
   `pybullet_fleet/examples/` (or `benchmark/experiments/` if its primary
   audience is evaluation), with immutable workload and two policy strategies.
   Keep policy decisions outside the simulation core. Decide the exact home
   before coding; do not create a public general policy API.
3. **Public metric surface and external collector:** add the smallest opt-in
   PBF Python metric records/summary needed for movement-command completion,
   rate and observed collision categories. Let the scenario-local collector
   supply task and policy context and format the result.
   Produce a machine-readable per-run result containing conditions, task
   lifecycle records, decision records, observed collision episodes and
   aggregate metrics through the public PBF metric surface. A small command-line
   entry point runs either policy and writes its own result; a comparison
   command/run mode reads both reports.
4. **Verification:** add focused tests for task lifecycle and censoring,
   the API-call timing boundary, collision category and cadence, stable IDs and
   same-workload policy independence. Run the full scenario twice under each
   policy to establish repeatability and show at least one conflict/stall and
   one observed safety event. Treat outcomes as evidence rather than hard-code
   an assumed ranking of policies.
5. **Performance and docs:** measure disabled baseline versus collector-enabled
   20-robot runs under the same conditions, keeping simulation outcomes and
   wall/RTF costs separate. Document how to run the example, interpret its
   metrics, and understand sampling limits. Update the `[Unreleased]`
   changelog for the user-visible example/documentation.

The optional behavior “stop a robot when collision is observed, then resume
only after a user-requested reset/teleport” is a **follow-up candidate**, not
part of this measurement slice. Existing `Agent.stop()` and `SimObject.set_pose()`
do not by themselves define a safe reset transaction: controller goal/trajectory,
active collision pairs, task status and replay inputs would need explicit
semantics. The current pass-through behavior remains measurable; collect the
final collision count after task completion or cutoff.

No new replay schema, checkpoint, playback UI, generic trace API, ROS bridge
change, shared USO abstraction, or general collision framework is planned.
If a safe implementation requires one, return for scope/architecture review.

## Verification gates

- `make verify` and `make docs`, plus targeted scenario/collision tests.
- Two independent policy runs from identical workload/configuration, with
  their own decision records. A diagnostic check should demonstrate that
  merely re-executing A's recorded commands cannot count as a B run.
- Collision classification fixture with separated, within-margin and
  penetrating geometric cases; verify `COLLISION_STARTED`/`ENDED` timing at
  every-step cadence for at least two configured timesteps and state the
  missed-between-steps limit.
- Result arithmetic: every task accounted for, throughput denominator fixed,
  incomplete tasks censored, no ack mistaken for arrival.
- A public API test obtains per-command and aggregate metrics without importing
  the example; the scenario report includes the final collision count even
  when all movement tasks have finished.
- Per-command records preserve run/robot/command IDs and simulation step/time;
  aggregate counts remain correct with no trace exporter attached.
- Repeatability and 20-robot runtime cost recorded with hardware/configuration
  and no unsupported deterministic or performance guarantee.

## Architecture decisions for Human review

1. **Collision semantics and cadence:** `closest_points` is approved. Confirm
   the names “safety-margin intrusion” and “geometric touch/overlap” if the
   latter is validated; neither means physics impact. Approve configurable
   timestep, every-step checking and its explicit between-step blind spot.
2. **Policy/metric ownership:** independent external policy runs are approved.
   The remaining clarification is the minimal public PBF metric API: it can
   calculate and expose durations/rates from PBF-visible command and outcome
   times; pre-command admission delay belongs only to the external app.
   Scenario policy and value judgments remain outside the core.
3. **Traffic rule:** a shared local hold rule is approved if feasibility shows
   otherwise the kinematic agents pass through each other. This rule must be
   identical in A and B; only corridor admission differs.
4. **Workload/cutoff:** two movements per robot (40 tasks) and the
   provisional 300 s simulated-time cutoff, subject to the feasibility
   calibration and a documented return to Human review if it must change.
5. **Artifact scope:** the separate scenario report is approved with its
   versioned, self-describing structure so later use cases can reuse or map
   common fields, without declaring a general PBF/USO/replay schema now.
