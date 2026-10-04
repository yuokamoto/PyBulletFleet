# Checkpoint / restore / resume: next-slice candidate

For the overall recording, restart, playback and debugging destination, read
the [product goals](product-goals.md) before choosing another checkpoint slice.

**Status:** The single-omni proof was implemented after Scope and Architecture
Approval. See the [implementation plan](checkpoint-plan.md) and
[evidence](checkpoint-evidence.md). Broader checkpoint support is future work.

## Follow-up profile inventory (not yet approved)

The completed proof covers only one straight-moving, per-agent omni robot.
`Agent.restore_motion_state()` now restores Agent-owned fields without assuming
a controller type; it does **not** restore a controller's goal or execution
state. Each additional controller/profile needs its own state inventory and a
fresh-process comparison against an uninterrupted run before claiming resume.
The [checkpoint state capability checklist](product-goals.md#checkpoint-state-capability-checklist)
tracks the supported omni subset and the remaining state categories together.

| Candidate | Status | State or behavior to investigate |
| --- | --- | --- |
| Per-agent straight omni navigation | **Done: fixed single-robot profile only** | Completed-step clock, Agent motion and one active straight trajectory were restored and compared in a fresh process. |
| Per-agent differential navigation | TODO; plausible next narrow profile, subject to scope review | Heading/turn phase, angular trajectory and velocity, active goal/path, completion flags, and their timing across a completed-step restore. Include a mid-turn checkpoint and compare subsequent pose, orientation and arrival. |
| Broader omni and batch controllers | TODO, separate profiles | Multi-waypoint/final-orientation phases and batch-owned execution state; the straight-path proof does not cover them. |
| Manipulation, attachment and entity lifecycle | TODO, separate profiles | Joint interpolation, parent link/relative transform, spawn/delete identity and reconstruction, and external driver progress; see the manipulation-state scenario evidence. |
| Actions/BT, devices/plugins, physics and external inputs | TODO, only with concrete use cases | Queues, timers, device or engine state, and post-checkpoint input order may be required. Do not infer coverage from pose restoration. |

This inventory tracks omissions; it is not a commitment to implement every row
or to introduce a generic controller serializer. The external coordinator
continues to own checkpoint workflow and artifact persistence.

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

Human Scope and Architecture Approval selected the single per-agent omni
translation case as the first slice, which is now implemented for review.
Algorithm replacement after restore,
recorded-data playback and post-checkpoint input replay remain later decisions.

Out of scope for this proof: arbitrary Python/plugin serialization, batch or
differential control, actions, behavior trees, attachments, devices, physics,
ROS/RMF or distributed state, USO runtime/schema adoption, cross-simulator
portability, generic event sourcing and operation tracing. Findings can later
be classified as simulator-independent, PBF-specific, PyBullet-specific or
uncertain before feeding them back into USO. The USO repository is unchanged.
