# Inspect the manipulation state scenario

```{warning}
Development validation only. The recording, checkpoint/restore and playback
commands below exercise one declared scenario and profile. They are not ready
for general use with ordinary PBF examples or custom simulations; do not rely
on their artifacts for production runs.
```

This example began as a checkpoint-state inventory. Its default invocation
still runs that observation-only scenario. With `--record`, it now exercises
the supported Snapshot / Restore / Playback V1 profile.
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
after-delete states. Without `--record`, these are in-memory observations.

## Record, restart and play back the supported profile

Use the ordinary simulation loop with opt-in recording. Omitting a directory
creates a unique path under `recordings/` and prints it:

```bash
python -m pybullet_fleet.examples.validation.manipulation_state_scenario --record
```

You can also choose a directory and watch the run:

```bash
python -m pybullet_fleet.examples.validation.manipulation_state_scenario --record my-run --gui
```

After that process has exited, restart from the latest completed checkpoint
at or before the requested simulation time. CP1 is step 3 / 0.3 s and CP3 is
step 25 / 2.5 s in this omni profile:

```bash
python -m pybullet_fleet.examples.validation.manipulation_state_scenario --restore my-run --time 2.5 --gui
```

Playback reads saved result frames without stepping the controller or physics:

```bash
python -m pybullet_fleet.examples.validation.manipulation_state_scenario --playback my-run
python -m pybullet_fleet.examples.validation.manipulation_state_scenario --playback my-run --gui --rtf 0.5
```

The recording stores one full checkpoint and one result frame after each
completed step, plus ordered supported inputs and lifecycle operations. The
example registers a named, versioned data callback that saves its stage and
observations in `records.scenario`. It reads the value and re-registers its
callback code in a fresh process. Runtime PBF object IDs are remapped; the
stable names in this artifact are validated as unique. The robot asset must
remain available and unchanged. `--rtf` during playback is display pacing.

The recorder accepts a profile with a stable ID/version and operations for
construction, capture, validation and restore. It also accepts `DataRecord`
entries: each supplies a name, version and callback taking the completed
simulation, step and elapsed time. The callback returns finite JSON data.
Profiles define supported `sim`, `agents` and `objects` state; each entity and
controller state includes its type and version. The example's
`KinematicManipulationProfile` lives in `examples/validation/` and fills these
sections only for this Omni mobile manipulator and box. Custom data is saved
as values; callback code is re-registered by the caller after restore.

This V1 profile uses the ordinary Omni pose navigation path, including its
generated approach waypoint and final orientation turn. It is one built-in
mobile manipulator. Checkpoints include all of its kinematic joint positions
and configured targets, plus one box, with fixed timestep and physics off.
This scenario actively drives one named arm joint. It does not save arbitrary
callbacks, plugins, Actions, custom entities, physics state or ROS/RMF state.
The default differential-controller example is not a supported checkpoint
profile. A recorded result frame is useful for viewing; only a complete
checkpoint can resume execution. Input history here covers the supported
Fleet API commands and box spawn/remove path, not arbitrary direct mutators.

The box has a scenario-owned stable label `box-001`. PBF assigns a runtime
`object_id` and PyBullet assigns a `body_id`; neither numeric ID is promised
to match another process. `PRE_STEP` runs before the next motion step. In
`POST_STEP`, the physical state already reflects the completed step, while
the core's public step/time counters advance **after** callbacks return. The
example labels observations with the completed step and `step × 0.1 s`.

Kinematic joint state currently reports velocity `0.0` even while the joint
position changes. `get_attached_objects()` and `is_attached()` show the
relationship exists, but public APIs do not reconstruct the full parent-link
and relative-transform state from an arbitrary child. The driver knows the
link and offset it requested; the V1 profile now captures those fields for
this supported kinematic attachment. See
`docs/design/snapshot-replay/manipulation-state-scenario-evidence.md` in the
repository for the observed values and limits.
