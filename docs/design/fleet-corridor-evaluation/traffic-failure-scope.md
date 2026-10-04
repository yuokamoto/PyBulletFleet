# Corridor traffic failure — scope investigation

**Status:** Human selected a revised external response rule after the initial
4-robot pilot, then extended its stop/restart scope through endpoint arrival.
The route merges at the corridor entrance; the revised 4- and 20-robot runs
completed. See
`traffic-failure-pilot-evidence.md` for both results.

## Two evaluation meanings

**Safety evaluation (current behavior).** Kinematic robots continue after a
sampled safety-margin violation or geometric overlap. The external evaluator
counts episodes and task outcomes. Overlap is not physical impact, and there
is no collision penalty in task timing.

**Traffic-failure evaluation.** Spawn robots only in area A and give them
simultaneous A-to-B movement tasks that converge before the narrow corridor.
On a sampled robot–robot geometric overlap, an external app lets the robot
nearest the B-side corridor exit continue and stops the others for one
simulated second. If that winner is already stopped, it resumes immediately;
otherwise, among all stopped robots, only the one nearest that exit is eligible
to resume on a completed step after its cooldown. It may resume
despite a persisting overlap; repeated stop/restart is recorded. Delay and
blocking are consequences of this explicit external
response rule, not PyBullet contact dynamics or a core congestion state.
This is a test of whether PBF can support a collision-induced traffic failure;
it is not an evaluation of an optimal congestion-avoidance policy.

## What the existing corridor example implies

The current example spawns ten robots on each side, alternates two y lanes,
and offsets both starting x and destination x by robot index. Robots in the
same direction have the same speed limits and largely preserve their spacing.
Merely moving all 20 spawns to area A while preserving parallel lanes and
goals may therefore produce no meaningful same-direction conflicts. A pilot
must deliberately create a merge: distinct, non-overlapping A-side starts and
approaches that converge on a shared corridor lane or waypoint. It must first
show collisions under the unchanged pass-through kinematic behavior; if not,
revise the initial conditions rather than claim a traffic failure.

The current mixed-direction report cannot be the comparison baseline for a
one-sided workload. Run pass-through and collision-response variants from the
same one-sided starts, endpoint tasks, timestep and cutoff. Keep the original
two-policy example intact during the pilot.

## Existing PBF capabilities and limits

The normal `run_simulation()` loop already supports external `PRE_STEP`
decisions and `POST_STEP` measurements. `FleetStateProvider.get_states_2d()`
supplies positions and movement state. `get_collision_observation()` supplies
active qualifying pairs, signed closest-point distance and check/sample
timestamps in this profile; the app can distinguish `distance <= 0` from a
positive-gap margin event. It must use fresh samples when making decisions.
`FleetCommandDispatcher.stop()` stops a named robot immediately and clears its
goal/path. A later `navigate()` starts a new trajectory from its current pose.
The app must retain the original task destination, acknowledge each command,
and distinguish a stopped task from an endpoint-completed task.

An overlap observed in `POST_STEP` already happened; stopping there prevents
further movement but cannot prevent that overlap. The 0.02 m margin is much
smaller than the possible movement in a 0.1 s step at the configured 1 m/s
speed. Neither margin observation nor every-step checks guarantee that a
fast, short conflict is caught before overlap. This is acceptable for a
traffic-response experiment but not a safety or nonpenetration guarantee.
Stopped kinematic robots do not physically block a moving robot, which can
pass through them unless the external rule also stops it.

## Experiment and decision rule

1. Pilot with a few same-side robots using the existing corridor walls and
   robot model. Pick non-overlapping starts and deterministic approach/goal
   geometry that causes a reproducible merge. Issue all A-to-B tasks at the
   same simulated time. Confirm a pass-through run produces sampled robot–robot
   overlap near the corridor. Then scale the same rule to 20 robots if it works.
2. In a second run with identical initial conditions, build the graph of
   freshly observed overlapping robot–robot pairs after each completed step.
   For each connected conflict group, choose the robot nearest the B-side
   corridor exit plane; break ties by stable robot ID. Keep that
   winner moving toward its current route waypoint. Stop the others through Fleet API
   and mark them blocked for **1.0 simulated second**, except when a stopped
   robot becomes the selected winner and must resume immediately. Robot–wall
   contacts are excluded from this experiment's trigger: they indicate a
   separate path-planning failure.
3. At or after the one-second deadline, release only the blocked robot nearest
   the exit and reissue `navigate()` to its retained waypoint. Other blocked
   robots remain stopped. This release may happen while an overlap persists,
   because waiting for strict clearance deadlocked the initial pilot. A
   released robot may collide again; record any repeated stop/restart. Do not
   use teleport as silent recovery.
   A stopped robot selected as a conflict-group winner resumes immediately
   before the other members are stopped, even if its deadline has not elapsed.
4. Make **time until every robot passes the corridor** the corridor outcome.
   Record each robot's first entry into the corridor and first exit through
   the B-side boundary, using the same reference-point geometry as the
   existing evaluator. Compare the last exit time between the pass-through
   and collision-response runs. Separately compare the last endpoint-arrival
   time to capture stopping after the exit. Also report blocked robots over simulated
   time, blocked-time distribution, spatial queue near the corridor, endpoint
   task completion, and stop/reissue decisions. Keep wall time/RTF separate.
   A queue claim requires both waiting robots and a stated spatial predicate.

Both runs use the same **300 s simulated-time cutoff**. If any robot has not
passed through the corridor by then, report its count and classify that run
as `deadlock_at_cutoff` for this experiment. This is an operational deadline
definition, not proof that motion could never resume. Report the all-pass time
only for runs in which every robot exits; do not substitute 300 s as a
completed time.

The initial 4-robot pilot produced robot–robot conflicts but did not recover by
the 300 s cutoff. Under the revised exit-priority release rule, both 4 and 20
robots passed before cutoff. One second remains a trial parameter, not a
validated safety interval. The observed stopping is not physical
nonpenetration; resumed robots may still pass through each other.

The first exit-priority route still merged in the corridor because each robot
navigated directly from a narrow two-lane start to a central goal. The current
route has four feeder lanes and explicit entrance/exit waypoints. Collision
observations remain global. Following Human review, the stop rule applies to
robots after the B-side corridor exit until endpoint arrival. Completed robots
are excluded from the response, while their overlaps remain measured. A
post-exit stop can affect endpoint-arrival time without changing that robot's
already recorded corridor-exit time; do not count it as entrance blocking.

## Missing capabilities and Human decisions

No missing PBF API has been identified for this bounded external experiment.
The app can own the blocked state, cooldown, conflict graph, task ledger and
recovery decisions. PBF does not provide physical nonpenetration, a guaranteed
pre-overlap trigger, automatic collision stop, or continuation of a cancelled
trajectory. If the pilot shows that the moving winner repeatedly passes
through stopped robots or cannot clear an overlap, that is a failure of this
proposed traffic model; return for scope review rather than hiding it with a
core behavior change.

Human has confirmed the collision-triggered, policy-imposed failure goal,
excluded robot–wall contacts as separate planner failures, approved the
all-corridor-exit time and 300 s cutoff, and approved a few-robot calibration
pilot before the one-sided 20-robot comparison.

The implemented pilot uses geometric overlap (`signed_distance <= 0`) as the
stop trigger; positive-gap margin observations do not stop robots. The
exit-priority rule permits recovery in this geometry, but is not a promise
that every workload will clear by 300 s.

No generic traffic framework, automatic core response, playback,
checkpoint/restore or generalized tracing is proposed here.
