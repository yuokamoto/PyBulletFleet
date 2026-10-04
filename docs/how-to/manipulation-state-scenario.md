# Inspect the manipulation state scenario

This small example is for discovering future checkpoint requirements. It does
not save or load a snapshot and does not provide recorded-result playback.
It uses existing PBF APIs to create a box during a run, move a mobile
manipulator, attach the box to its end-effector link, move a joint while
carrying it, detach the box, and delete it. Attachment uses an explicit
link-relative offset; it is not a contact-validated grasp. Collision checks
are disabled in this state-discovery scenario.
The existing `mobile_manipulator_demo.py` uses an Action chain for a more
natural application-level sequence. This example deliberately issues direct
commands and keeps its stage in the external driver, so the two ownership
boundaries can be compared. An external driver could also queue Actions; an
action-based state-discovery counterpart is a follow-up, not part of this
example.

Run the deterministic headless scenario:

```bash
python -m pybullet_fleet.examples.validation.manipulation_state_scenario
```

To watch it in PyBullet GUI at normal simulated speed:

```bash
python -m pybullet_fleet.examples.validation.manipulation_state_scenario --gui --hold-gui
```

`--rtf 2` requests 2× viewing speed. `--hold-gui` keeps the final frame open
until you close the window or press Ctrl+C. The program prints the completed
step and simulation time for CP1 (base moving), CP2 (joint moving), CP3
(attached box and joint moving), CP4 (detached box), and the before-spawn and
after-delete states. These are in-memory observations; the example writes no
snapshot, recording or lifecycle journal.

The box has a driver-owned stable label `box-001`. PBF assigns a runtime
`object_id` and PyBullet assigns a `body_id`; neither numeric ID is promised
to match another process. `PRE_STEP` runs before the next motion step. In
`POST_STEP`, the physical state already reflects the completed step, while
the core's public step/time counters advance **after** callbacks return. The
example labels observations with the completed step and `step × 0.1 s`.

Kinematic joint state currently reports velocity `0.0` even while the joint
position changes. `get_attached_objects()` and `is_attached()` show the
relationship exists, but public APIs do not reconstruct the full parent-link
and relative-transform state from an arbitrary child. The driver knows the
link and offset it requested; that knowledge is not a checkpoint API. See
`docs/design/snapshot-replay/manipulation-state-scenario-evidence.md` in the
repository for the observed values and limits.
