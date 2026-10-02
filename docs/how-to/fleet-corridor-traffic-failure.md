# Run the one-sided corridor traffic experiment

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
  --gui --policy collision_stop --cutoff 60
```

Use `--policy pass_through` to watch the baseline; it gets a different
automatically named output directory. The initial view is closer to the corridor. While the simulation
runs, use the arrow keys or right-drag to pan, `=` / `-` or the mouse wheel to
zoom, and left-drag to rotate. `--rtf 3` requests 3× real-time viewing speed.
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
waypoint at x=-3.45, cross to an exit waypoint at x=3.45, then disperse to
separate B-side endpoints. The app issues each waypoint through the ordinary
Fleet API; it does not add route planning or traffic response to PBF core.
The response application reads PBF's collision observation
after each completed step. A fresh robot–robot signed distance of zero or
less forms a conflict; robot–wall observations are counted but do not trigger
stops. Within a connected conflict group, the robot nearest the B-side exit
plane at x=3 continues and other moving members receive `stop` commands.
Among all stopped robots, at most one per step is eligible to resume after at
least one simulated second: the one nearest the exit. The app then reissues
its original `navigate` endpoint. It may still overlap another robot and be
stopped again. PBF core itself does not stop robots on collision. The response
does not stop a robot after it has passed the B-side corridor exit. Robot
overlaps beyond the exit are still measured; they are not a traffic-stop
trigger in this example.

The primary outcome is the time when every robot has passed the B-side
corridor boundary. If any robot remains at the 300 s cutoff, the report gives
no all-pass time and marks `deadlock_at_cutoff`. This is a deadline label, not
proof of permanent deadlock. The report also contains per-robot entry, exit
and endpoint-arrival steps, stop/restart decisions, blocked intervals and
overlap entry counts by location. `before_entrance` means both reference points
are before x=-3; `near_destinations` means both are at x>=4. The remaining
zones cover the corridor/boundary and the immediate exit area. These are
scenario geometry labels, not PBF collision types. Simulated-time outcomes
are separate from wall time.
An accepted command is not arrival or passage.

In the fixed 20-robot run, ordinary pass-through finished at 10.9 simulated
seconds and collision-stop finished at 37.3 seconds. In the response run, all
154 pre-entrance robot-overlap entries were observed before x=-3, and the peak
of 19 blocked robots was also before the entrance; no blocked robots were
observed inside the corridor. One overlap entry remained near the dispersed
destinations and was measured without triggering a stop. Because kinematic
robots can pass through each other, the example does not establish impact
severity or collision avoidance. The repository's
`docs/design/fleet-corridor-evaluation/traffic-failure-pilot-evidence.md`
records the initial strict-clearance deadlock and the revised rule.
