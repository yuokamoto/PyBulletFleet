# Run the one-sided corridor traffic experiment

This is an **evaluation example**, not a traffic-control algorithm supplied
by PBF. It shows how an external application can use PBF's collision
observations and command APIs to measure the consequences of its own response
rule. The stop/restart rule below is deliberately illustrative; designing or
judging a production algorithm is outside this repository's scope.

This synthetic example starts robots in area A and sends them through a narrow
corridor toward B. It compares ordinary kinematic pass-through with a
scenario-owned collision response. It does not simulate physical blocking or
reproduce a particular field incident.

```bash
python -m pybullet_fleet.examples.fleet_corridor_traffic_failure
```

With no output argument, each run creates a unique `pbf-traffic-*` directory
under the operating system's temporary directory and prints its full path.
On macOS, that directory may be under `/var/folders/.../T` rather than `/tmp`.
To choose a lasting location, pass that
directory as the positional argument; an explicitly named directory must not
already exist. The default runs 20 robots for 300 simulated seconds at a
0.1 s timestep and writes `pass_through.json` and `collision_stop.json`.
Use `--robots 4` for a smaller pilot. `--dt` and
`--cutoff` change the timestep and fixed simulated-time window; the cutoff
must contain an integer number of steps.

To watch the collision-stop variant at normal speed, run one policy in the
PyBullet GUI:

```bash
python -m pybullet_fleet.examples.fleet_corridor_traffic_failure \
  --gui --monitor --policy collision_stop --cutoff 60
```

Use `--policy pass_through` to watch the baseline; it gets a different
automatically named output directory. The initial view is closer to the corridor. While the simulation
runs, use the arrow keys or right-drag to pan, `=` / `-` or the mouse wheel to
zoom, and left-drag to rotate. `--rtf 3` requests 3× real-time viewing speed.
`--monitor` opens the DataMonitor beside the PyBullet GUI. Its `Collisions`
field shows active PBF-qualified pairs and `Tot Collis.` counts new pair
entries across the entire scene, including margin-only and wall pairs; neither
is the example's robot-overlap entry count. The display refreshes about every
0.5 wall-clock seconds.
The final frame stays open until you close the GUI or press Ctrl+C; the JSON report is saved
afterward. GUI pause or single-step interaction may change command timing, so
use the default headless command for repeatable measurement. This shows a
live run, not recorded-result playback.

On macOS, if the native PyBullet window is unavailable, the existing Colima
launcher can display the same example in a browser:

```bash
scripts/run_linux_gui_macos.sh fleet_corridor_traffic_failure.py \
  --gui --policy collision_stop --cutoff 60
```

The launcher runs in a temporary container. Use the native Python command
above when the JSON report must remain on the Mac.

Both variants use the same initial poses, route and motion limits in fresh
simulations. Robots start in four A-side feeder lanes, converge on an entrance
waypoint at x=-1.2, cross to an exit waypoint at x=1.2, then disperse to
separate B-side endpoints. The app issues each waypoint through the ordinary
Fleet API; it does not add route planning or traffic response to PBF core.
In report schema version 2, each navigation decision names its `route_phase`
as `entrance`, `exit` or `destination` rather than using a numeric index.
The report identifies the selected external algorithm with `policy`, matching
the `--policy` option and the bidirectional evaluation example.
The response application reads PBF's collision observation
after each completed step. A fresh robot–robot signed distance of zero or
less forms a conflict; robot–wall observations are counted but do not trigger
stops. Within a connected conflict group, the robot nearest the B-side exit
plane at x=0.75 continues and other moving members receive `stop` commands.
If that winner is already stopped, the app immediately reissues its navigation
command before stopping the others. Otherwise, among all stopped robots, at
most one per step is eligible to resume after one simulated second: the one
nearest the exit. The app then reissues
its original `navigate` endpoint. It may still overlap another robot and be
stopped again. PBF core itself does not stop robots on collision. The response
can also stop a robot after it has passed the B-side corridor exit, until it
arrives at its destination. Completed robots are excluded from further
stop/restart decisions. All robot overlaps remain measured, including pairs
involving completed robots; those completed pairs do not trigger a response.

The primary corridor outcome is the time when every robot has passed the B-side
boundary. `all_arrived_at_seconds` separately measures when every robot reaches
its destination, including any delay after the exit. If a robot has not passed
by the 300 s cutoff, the report gives no all-pass time and marks
`deadlock_at_cutoff`; this is a deadline label, not proof of permanent deadlock.
If any destination remains unfinished, `all_arrived_at_seconds` is null and
`endpoint_unfinished_count` reports how many. The report also contains
per-robot entry, exit and endpoint-arrival steps, stop/restart decisions, blocked intervals and
overlap entry counts by location. `before_entrance` means both reference points
are before x=-0.75; `near_destinations` means both are at x>=4. The remaining
zones cover the corridor/boundary and the immediate exit area. These are
scenario geometry labels, not PBF collision types. Simulated-time outcomes
are separate from wall time.
An accepted command is not arrival or passage.

With the shortened x=[-0.75,0.75] corridor, all 20 robots passed the exit at
8.6 simulated seconds for pass-through and 35.9 seconds for collision-stop;
all reached their destinations at 16.1 and 43.4 seconds, respectively.
In the response run, 170 robot-overlap entries were observed before x=-0.75,
and the peak of 18 blocked robots was also before the entrance; no blocked
robots were observed inside the corridor. No post-exit stop happened in this
fixed run, though the response rule now permits one before endpoint arrival.
Because kinematic
robots can pass through each other, the example does not establish impact
severity or collision avoidance. The repository's
`docs/design/fleet-corridor-evaluation/traffic-failure-pilot-evidence.md`
records the initial strict-clearance deadlock and the revised rule.
