# Fleet corridor failure scenario — scope draft

**Status:** First-slice scope and architecture approved; implementation in progress.

## Problem and intended outcome

PyBulletFleet is a fleet-scale simulator, not the authority that judges whether
a fleet algorithm is good. A user should be able to create a repeatable traffic
failure, inspect what happened, and supply the observations, events, inputs and
timing needed by an external evaluator. Existing benchmarks primarily measure
simulation/transport performance; they do not establish how congestion and
collisions affect fleet task completion.

The motivating scene has areas A and B joined by a corridor that admits about
two or three robots abreast. Twenty robots receive repeated A-to-B/B-to-A
movement requests. Compare an uncontrolled, bidirectional entry policy with a
policy that grants corridor entry to one direction at a time. The final use case
may grow to 20–100 robots carrying approximately three items per robot from A
to B, but item creation, pick/drop and warehouse-wide throughput are not part
of the first slice.

## Responsibility and capability boundaries

PBF supplies simulation state, collision facts, command acknowledgements and
available outcome events. The external scenario defines tasks, corridor
occupancy and entry policy; its evaluator computes congestion and fleet-level
metrics and compares outcomes. PBF does not declare a policy successful. A
command acknowledgement is not movement or task completion.

The five desired uses are separate capabilities:

| Use | Current capability | Gap to validate in this scenario |
| --- | --- | --- |
| A. Initial-state + input re-execution | PR #50 records supported navigate/stop inputs and re-executes a restricted kinematic world. | A live entry policy may make decisions from state each step; fixed recorded inputs reproduce the original policy's actions, not a changed policy. Broader input/state capture requires its own scope. |
| B. Recorded-result playback | Full observations can be recorded in the v1 profile. | No timeline player or seek/display contract exists. Sparse sampling may miss the onset of a failure. |
| C. Checkpoint/restore/resume | No intermediate execution checkpoint exists. | Controller state and any pending policy/task state must be captured before a changed-policy continuation can be claimed. |
| Trace | Supported request, ack, outcome and collision transitions are recorded in the v1 journal. | Corridor-entry decisions, waiting reasons and task causality are not generally traced across arbitrary policy code or ROS/RMF. |
| Metrics | Step/RTF benchmarks and state/event access exist. | The external evaluator must derive movement completion and rate from PBF facts; waiting before a Fleet API call belongs entirely to the external policy. |

For future checkpoint work, simulator/controller code may expose the state and
save/load operations needed for a supported profile. External orchestration
controls when capture, restore, policy substitution and continuation happen.
Do not put that workflow into the simulation core without a demonstrated need.

## Proposed first slice: scenario and measurement contract

Define one small, physics-off corridor scene and a finite workload. The
measurement default is headless; optional GUI observation runs one policy at
a time and does not replace the independent headless comparison.
Use a fixed timestep, explicit initial poses, robot dimensions, corridor width,
route waypoints, movement limits, input schedule and run cutoff. Start with 20
robots and two entry policies; 100 robots is a later scale check. The scenario
should use ordinary PBF APIs, with an external driver/evaluator. How the driver
records dynamically chosen inputs is an explicit design question, not assumed
to be solved by the current ReplaySession.

The *uncontrolled* policy allows both directions to request entry. The
*controlled* policy permits only one direction to enter at a time and releases
waiting robots under a stated fairness rule. Neither policy is assumed to avoid
all collisions; the experiment must first show that the chosen geometry and
motion actually produce meaningful conflict. Do not infer congestion from
collision count alone.

For each movement task, the external app records release/admission decisions;
PBF-visible movement begins with a Fleet API command. Record a stable task ID,
robot ID, command/ack time, terminal time and terminal reason. Report
at least:

- completed tasks per simulated hour, completion fraction at the fixed cutoff,
  and per-task completion-time distribution; count unfinished tasks explicitly;
- PBF-visible command-to-arrival duration and completed commands per simulated
  time. The external app separately reports its release-to-command admission
  delay and release-to-arrival duration; those are not PBF waiting metrics;
- collision-enter events by robot pair, collision duration or observed active
  steps, and the detection settings used.

The scenario timestep is configurable; a result must state its actual value and
collision-check cadence. Keep simulated time for fleet outcomes separate from wall time and RTF, which
describe the cost of running the experiment. Collision detection identifies
proximity/contact according to configured mode and margin; it does not itself
prevent collisions or explain policy decisions. A low-frequency collision pass
can miss short contacts, so the detection cadence is part of the conditions.

The evaluator's individual records retain run, robot, task and command IDs plus
simulation step/time, allowing later correlation with an operation trace.
Metric collection and aggregate counts must remain complete even if a future
trace exporter samples or drops spans; end-to-end trace propagation is a
separate follow-up.

## Acceptance criteria draft for the first slice

1. The scene, workload, policies, cutoff and random inputs (if any) are fixed
   or recorded so two runs can be compared under stated conditions.
2. The external evaluator accounts for every issued task as completed, failed,
   rejected or unfinished and reports the metrics above without confusing ack
   with completion.
3. A run produces inspectable evidence for a congestion interval and any
   collision episode: entity pair, simulation step/time, state and relevant
   request/policy decision. If the proposed geometry does not produce both
   phenomena reliably, the result is reported and the scenario is revised
   before claiming coverage.
4. The two policies are compared on the same workload. Report both safety and
   task outcomes; do not assert that the controlled policy is better merely
   because collision count is lower.
5. Report exactly which of replay, playback, checkpoint and trace can be
   demonstrated with existing capabilities. Missing capabilities remain
   separately scoped follow-ups, not implicit acceptance criteria for this
   first scenario/measurement slice.
6. Measure the added scenario instrumentation cost separately from normal
   step time/RTF before using the result at larger fleet sizes.

## Non-goals and follow-ups

No warehouse output model, item handling, ROS/DDS replay, universal congestion
detector, universal task engine, shared PBF/USO runtime, GUI editor, or all-five-
capabilities implementation in one change. Follow-ups may add recorded-result
playback, a profile-specific execution checkpoint, broader policy input
capture, operation tracing, item transport and a 100-robot scale run when the
scenario demonstrates their value. Keep the PBF → findings → other designs →
USO refinement → another backend sequence before extracting a shared schema.

## Scope decisions and remaining architecture review

The Human approved this synthetic scenario, 20 robots, endpoint arrival as
movement-task completion, and separate follow-ups for playback, checkpoint,
broader replay and tracing. The architecture approval keeps all task,
congestion, policy-comparison and fleet-level metric semantics in the external
evaluator. PBF exposes only the narrow facts it owns. A
collision-stop/reset-and-teleport mode is a follow-up; it is not required to
count collisions in this slice.
