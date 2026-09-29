# Corridor evaluation — implementation evidence

**Status:** Implemented on the draft PR branch; awaiting independent and Human Final Review.

## Delivered boundary

`pybullet_fleet/examples/fleet_corridor_evaluation.py` is an external fleet
management example. It creates a fresh PBF simulation for each of two policies,
issues ordinary Fleet API commands, samples state and geometry, and writes a
scenario-owned report. It adds no simulator task, congestion, policy or replay
orchestration API. The user guide is [here](../../how-to/fleet-corridor-evaluation.md).

The fixed workload has 20 robots, 40 A-to-B/B-to-A endpoint movement tasks,
two 0.1 m-wide cube lanes in a 0.34 m-wide corridor, 0.1 s default timestep,
0.02 m proximity margin and 300 s simulated-time cutoff. The external gate
allows two concurrent tasks in one direction and alternates direction between
batches. The uncontrolled policy dispatches all ready tasks.

## Observed outcomes

Local macOS 14.6.1 x86_64, Python 3.12.5, PyBullet build 2026-09-26.
Two independent runs per policy produced the same simulated-time metrics.
These figures describe this synthetic configuration, not a policy ranking:

| Measurement | Uncontrolled | Direction gate |
| --- | ---: | ---: |
| Completed at 300 s / workload | 40 / 40 | 40 / 40 |
| Completed by 60 s | 40 | 9 |
| All tasks arrived by simulated second | 28.4 | 245.1 |
| Median external admission delay, s | 0 | 49.95 |
| Peak robot references in corridor | 20 | 2 |
| Corridor steps with more than three robots | 188 | 0 |
| Observed corridor geometric-overlap episodes | 100 | 0 |
| Observed total geometric-overlap episodes | 180 | 116 |
| Core detected threshold-entry count | 180 | 116 |

Both policies have the same fixed-window completion rate, 40/300 tasks per
simulated second. That single rate hides the substantial latency difference.
The gate reduces corridor overlap but still permits overlap outside the
corridor, and it imposes long external admission delays. Uncontrolled robots
pass through one another; the observed crowding is not physical blocking.
This scenario therefore demonstrates density, overlap and policy-induced queue
measurements, not a realistic traffic-jam dynamics model.

Collision episodes are sampled once per completed step. A focused fixture at
0.1 s and 0.05 s distinguishes positive-gap margin proximity, geometric
overlap and separation. The core's AABB broadphase omits some positive-gap
near misses, as already documented by an xfail in
`tests/test_collision_comprehensive.py`. The scenario queries every
robot/robot and robot/wall pair directly for complete *sampled* near-miss
classification. It cannot see contact between samples. Geometric overlap in
kinematic mode does not mean a physical impact.

The all-pair collector has visible cost: approximate 300 s runs took 3.2–3.6 s
wall time with episode collection, versus 0.8–1.0 s without it; core step time
remained about 0.3–0.4 s. RTF was about 85–93 with collection and 315–370
without. These are local observations, not a release performance guarantee.

## Follow-up signals

- **Playback:** The per-step scenario observations are not stored as a
  continuous result timeline. Episode start positions and task/decision records
  permit limited inspection, but not rosbag-like playback or seeking.
- **Checkpoint/restore:** A changed-policy continuation just before overlap
  would be useful. The simulator would need profile-specific state save/load;
  external orchestration should own capture, substitution and resume.
- **Replay/input capture:** The policies choose actions dynamically. PR #50
  recorded-command re-execution would reproduce the original policy's commands,
  not let a changed policy decide. A broader live input boundary is separate.
- **Trace:** Stable run/robot/task/command IDs and step/time support later
  correlation. There is no generalized causal trace across controller, policy
  and collision episodes.
- **Traffic realism:** A common occupancy/stop rule or physical interactions
  could model blocking, but that would change scenario semantics and requires
  its own review. Item transport and 100-robot scaling remain deferred.

## Verification

Focused scenario tests cover two cadences, collision categories and clearing,
equivalent independent policy workloads, accounting and cutoff censoring.
`make verify` passed: 1754 passed, 12 skipped, one expected failure and 81.39%
coverage. After the final crowding-interval and comparison checks were added,
the focused tests (5 passed) and all changed-file pre-commit hooks passed.
`make docs` passed with `LC_ALL=C LANG=C` because the local shell locale was
not available to Sphinx. The CLI produced all three JSON reports in a smoke
run.
