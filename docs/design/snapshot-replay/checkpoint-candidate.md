# Checkpoint / restore / resume: next-slice candidate

**Status:** Investigation and proposed direction for review after the current
navigation replay PR. This is not an approved implementation scope or plan.

## Product goal and current boundary

The user goals are (1) playback of recorded simulation data, (2) restore an
intermediate state and resume, and (3) change an algorithm after restore to
reproduce a failure or compare outcomes. The working model is **state + ordered
inputs + execution**. Schema, journal, identity, trace and USO mapping are means
to those workflows, not independent product goals.

V1 already reconstructs a supported initial world, journals effective
navigate/stop inputs by step/order, re-executes those inputs, compares results,
and validates artifact completeness. Its observations contain pose, reported
velocity and `is_moving`; they do **not** contain a resumable controller state.
V1 cannot restore an intermediate step. Keep the approved v1 scope and its
[implementation plan](plan.md) separate from this candidate.

## Smallest checkpoint proof to review next

Use one per-agent omni robot, physics off, fixed timestep, and no obstacles or
callbacks. Checkpoint at a completed step `S_k` during straight-line navigation.
Terminate the source simulation, reconstruct a fresh simulation in another
process, load the checkpoint, and advance without another command. Compare each
post-`S_k` pose and the arrival result with a separately captured uninterrupted
reference run. This tests a real intermediate restore, not prefix re-execution
from `S_0` or reissuing the goal from the checkpoint pose.

The proposed minimum state for this **specific** profile is:

| State | Why it is needed |
| --- | --- |
| Initial object construction and fixed execution configuration | Recreate the same robot and controller in the new process; reuse the v1 initial definition where it suffices. |
| Completed `state_step` and simulation time | The next step and trajectory lookup use simulation time; `MultiRobotSimulationCore` maintains both step count and elapsed time. |
| Current pose and reported motion state | Reconstruct the state at `S_k`, including `Agent._is_moving` and reported velocity used in state comparison. |
| Active omni navigation state | `KinematicController` has POSE mode, path/goal progress and completion/alignment flags. These decide how motion ends. |
| Active forward trajectory definition | `OmniController` evaluates a time-based TPI from the original start position, direction, distance, start time and motion constraints. Pose plus goal alone would generate a new acceleration profile. |

These are evidence-based candidates, not a commitment to serialize every private
field. The implementation investigation should determine the smallest explicit
trajectory data that reconstructs the same TPI and prove sufficiency with a
mid-acceleration checkpoint. The single straight-line test can exclude rotation,
multi-waypoint navigation, batch state and collision bookkeeping. Later input
reapplication requires inputs **after** `S_k`; it is not necessary to prove the
first restore. If a future run must preserve journal outcome events as well as
movement, the session's active-command correlation also needs restoration.

## Proposed implementation boundary

Reuse v1's supported initial-world construction, identity mapping, input timing
rules and comparison concepts. Add a small, profile-specific checkpoint record
and a fresh-instance restore path at a completed step boundary. Do not relabel
`observations.jsonl` as execution checkpoints or add a generic Python-object
serializer. The first validation compares the uninterrupted and resumed suffix,
not just the final position. On unsupported or incomplete state, fail explicitly.
The preferred responsibility split is for PBF simulation/controller code to
expose only the supported execution-state save/load needed for this profile;
an external coordinator owns checkpoint files, input replay, stepping and
comparison. Confirm the smallest useful save/load surface during design rather
than putting replay orchestration into `MultiRobotSimulationCore`.

Keep event ownership equally narrow. Collision transitions are simulation
facts: a future core event/query boundary could expose them to an external
recorder without making the core own a durable event log. V1 instead derives
transitions in `ReplaySession` by comparing the core's active collision pairs
across steps. `arrived` and `stopped` are different: they are replay-profile
outcomes tied to a recorded request, its ID and saved tolerances, so they belong
in the external coordinator rather than a generic core event log.

Observation and execution checkpoint data will overlap, especially for pose.
For a supported profile, an observation could eventually be projected from a
complete checkpoint state. A checkpoint additionally needs the execution state
required to resume; an observation selects fields for playback and comparison.
Do not assume the current observation schema is already a literal subset of a
future checkpoint format or redesign it before the minimal restore proof.

PyBullet provides in-memory `saveState()` and file-based `saveBullet()` /
`restoreState(fileName=...)` in its
[official example](https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/saveRestoreState.py).
For this physics-off kinematic case, PBF computes motion in its controller and
skips `stepSimulation()`. An engine snapshot alone cannot restore PBF's goal,
trajectory or simulation counters. Assess native state saving only if it
simplifies a concrete requirement; do not make physics-state portability a
prerequisite for this slice.

## Proposed acceptance criteria and decisions

- A checkpoint taken during acceleration resumes in a fresh process after the
  original simulation has terminated.
- `S_k`, every resumed pose through arrival, and the arrival result match the
  uninterrupted reference within a stated numeric tolerance.
- A test distinguishes true trajectory continuation from reissuing the goal at
  `S_k`; unsupported or incomplete checkpoints fail instead of silently running.
- The checkpoint is identified separately from initial definitions,
  observations and the input journal.

Human Scope / Architecture Approval is needed before implementation: confirm the
single per-agent omni translation case as the first slice and a separate,
profile-specific checkpoint contract. Algorithm replacement after restore,
recorded-data playback and post-checkpoint input replay remain later decisions.

Out of scope for this proof: arbitrary Python/plugin serialization, batch or
differential control, actions, behavior trees, attachments, devices, physics,
ROS/RMF or distributed state, USO runtime/schema adoption, cross-simulator
portability, generic event sourcing and operation tracing. Findings can later
be classified as simulator-independent, PBF-specific, PyBullet-specific or
uncertain before feeding them back into USO. The USO repository is unchanged.
