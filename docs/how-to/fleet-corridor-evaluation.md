# Run the synthetic fleet corridor evaluation

This example runs two independent 20-robot simulations through a narrow
corridor. It compares unrestricted bidirectional commands with an external
direction gate. It is a synthetic movement workload, not a warehouse delivery
benchmark or a reproduction of a field incident.

```bash
python -m pybullet_fleet.examples.fleet_corridor_evaluation /tmp/pbf-corridor-run
```

The output directory must not already exist. `--dt 0.05` changes the timestep;
`--cutoff 300` sets the fixed simulated-time window. The cutoff must be an
integer number of steps. The command writes `uncontrolled.json`,
`direction_gate.json`, and `comparison.json`. Each policy starts in a fresh
simulation with the same initial poses, 40 two-leg movement tasks, geometry,
motion limits and cutoff. The comparison does not use recorded-command replay.

## Watch a policy in the GUI

Select one policy at a time. The GUI defaults to 1× simulated time and keeps
the final view open until you close the window or press Ctrl+C. The JSON report
is written when the window closes. Blue robots start in area A; orange robots
start in area B.

```bash
python -m pybullet_fleet.examples.fleet_corridor_evaluation /tmp/pbf-corridor-watch-a \
  --gui --policy uncontrolled --cutoff 30
python -m pybullet_fleet.examples.fleet_corridor_evaluation /tmp/pbf-corridor-watch-b \
  --gui --policy direction_gate --rtf 3
```

The 30-second run is for a quick look and censors unfinished tasks. For the
defined 300-second comparison, omit `--cutoff`; direction gate reaches its
last endpoint at about 245 simulated seconds in the tested configuration.
`--rtf` changes live GUI viewing speed, not the simulation timestep. This is a
live simulation run, not recorded-result playback. Manual pause
or single-step interaction may change command timing, so use the default
headless two-policy command for repeatable measurement. GUI mode writes only
the selected policy's report, not `comparison.json`.

On macOS, native PyBullet GUI can be used from Terminal. If the native window
does not work well, the launcher added by [PR #48](https://github.com/yuokamoto/PyBulletFleet/pull/48)
can show the Linux GUI in a browser (requires Colima and Docker CLI):

```bash
scripts/run_linux_gui_macos.sh fleet_corridor_evaluation.py \
  /tmp/pbf-corridor-browser --gui --policy uncontrolled --cutoff 30
```

That launcher runs in a temporary container; use the native Python command
above when you need to retain the JSON report on the Mac.

The JSON includes versioned conditions, measurement definitions, task release,
command acknowledgement, first movement and arrival times, external policy
decisions, observed proximity/overlap episodes, and aggregate metrics. A task
is complete only when the robot arrives at its endpoint and stops. An accepted
command is not a completed task. Unissued second legs remain in the workload
denominator and are marked unfinished at cutoff.

`external_admission_delay_seconds` measures time from task release to the
external policy's `navigate` call. PBF cannot observe a request it has not
received, so this is not simulator waiting time. `command_to_arrival_seconds`
starts when the command is issued. `completed_per_sim_second`, completion
fraction, the 60-second completion count, and all-tasks-completed time describe
fleet task outcomes in simulated time; wall seconds and RTF describe execution
cost. Fixed-window rate can be identical even when task latency differs.

The external evaluator marks corridor crowding when more than three robot
reference points occupy x=[-3,3], y=[-0.17,0.17]. This is a transparent
scenario definition, not a PBF congestion verdict or a physical queue.
The report retains each observed crowding interval with start/end step and
peak occupancy, as well as the aggregate count of over-capacity steps.
Kinematic robots pass through one another. The direction gate grants up to two
active movement tasks in one direction and alternates direction after a batch
finishes. A gate slot is held until endpoint arrival, including travel outside
the corridor. It can create substantial admission delay while reducing observed
overlap in the corridor; overlap elsewhere may remain.

Collision episodes are sampled after every completed step. `margin_only`
means signed closest-point distance is positive and at most the configured
0.02 m margin. `geometric_overlap` means distance is zero or negative; it is
not a physics impact. The example queries robot/robot and robot/wall pairs
directly because the core's AABB broadphase can omit positive-gap near misses.
The separate `core_margin_entry_count` is the core's detected threshold-entry
count and is not an overlap count or a complete near-miss count. Each episode
records pair IDs, step/time, start positions, active task IDs, minimum sampled
distance and observed duration. Events that begin and end between samples can
still be missed, so smaller `--dt` improves resolution without guaranteeing
continuous detection.

The report is a scenario-owned artifact. Its IDs and step/time fields permit
later trace correlation, but it is not a replay artifact or a general PBF
metrics API. Playback, checkpoint/restore, broader input capture and general
trace propagation remain separate future capabilities.
