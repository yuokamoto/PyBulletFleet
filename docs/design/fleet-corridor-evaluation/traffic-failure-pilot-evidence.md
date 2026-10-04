# One-sided corridor traffic failure — pilot evidence

**Status:** Initial strict-clearance pilot, exit-priority pilot, and current
entrance-merge route recorded. This is a synthetic scenario result, not a
field-failure claim. Earlier tables document superseded route/rule variants.

The external example is `pybullet_fleet/examples/evaluation/fleet_corridor_traffic_failure.py`.
Run the approved 20-robot workload with:

```bash
python -m pybullet_fleet.examples.evaluation.fleet_corridor_traffic_failure --robots 20 --cutoff 300
```

The current variants use the same A-side starts, B-side endpoints, 0.1 s timestep,
1 m/s speed limit, 0.02 m collision margin and 300 s simulated-time cutoff.
The response uses fresh robot–robot observations with signed distance <= 0;
wall observations do not trigger stops. `pass_through` retains ordinary
kinematic movement. The **initial, superseded** `collision_stop` pilot chose the member
nearest its endpoint in each overlap group, stopped the others for at least
1 s, and required fresh clearance before resuming. This version deadlocked.

| Outcome | Pass-through | Collision-stop |
| --- | ---: | ---: |
| Robots past the B-side corridor exit by 300 s | 4/4 | 1/4 |
| Time when all passed | 8.6 s | Not reached |
| Robot overlap entries | 6 | 3 |
| Wall overlap samples | 0 | 0 |
| Stop / resume commands | 0 / 0 | 3 / 0 |
| Unfinished at cutoff | 0 | 3 |

The response run is `deadlock_at_cutoff` under the agreed operational
definition. This does not prove permanent physical deadlock. At step 64,
`r01` stopped and `r00` was selected to continue. At steps 66 and 67, the
already stopped `r01` was selected to continue against `r03` and `r02`, so
the latter two also stopped. `r00` exited at step 84. The remaining three
never met the release condition by step 3000 because their sampled overlap
pairs did not clear. This confirms that the proposed winner/release rule can
select a stopped winner and can create a group with no moving member.

The experiment shows that current PBF state, collision observations, and
stop/navigate APIs suffice to express *conflict → blocking → reduced passage
count* externally. The three stopped robots were inside the corridor; this
pilot does not yet establish a queue behind its entrance. PBF does not supply
the missing recovery rule;
that decision belongs to the scenario application. The present result cannot
demonstrate recovery or a finite all-pass delay, so the planned 20-robot run
would only amplify an unresolved policy problem. No simulation-core change is
indicated by this pilot.

Human then proposed releasing the stopped robot closest to the corridor exit
while leaving the others stopped. The revised response uses the B-side exit
plane at x=3 as the priority target. After each completed step with a fresh
collision observation, at most one blocked robot may resume: the one nearest
that plane, if its 1 s cooldown has elapsed. A persisting overlap does not
prevent release. The next collision check may stop it again. Each overlap
group also keeps the member nearest the exit moving and stops other moving
members. This remains an external policy; PBF core does not stop robots
automatically.

| Exit-priority rule on the superseded straight route | 4 pass-through | 4 collision-stop | 20 pass-through | 20 collision-stop |
| --- | ---: | ---: | ---: | ---: |
| Robots past B-side exit by 300 s | 4/4 | 4/4 | 20/20 | 20/20 |
| Time when all passed | 8.6 s | 11.1 s | 10.6 s | 25.1 s |
| Robot overlap entries | 6 | 9 | 190 | 214 |
| Stop / resume commands | 0 / 0 | 3 / 3 | 0 / 0 | 26 / 26 |
| Peak simultaneously blocked | 0 | 2 | 0 | 13 |
| Wall overlap samples | 0 | 0 | 0 | 0 |

That route allowed all 20 robots to cross, but the direct start-to-goal
commands caused the two feeder lanes to converge around the middle of the
corridor. The stopped robots were inside it. This did not demonstrate the
intended entrance-side traffic failure.

## Original entrance-merge route (6 m corridor)

Four A-side feeder lanes converge on a waypoint at x=-3.45, before the
corridor boundary x=-3. Each robot then navigates to x=3.45 on the centerline
and finally to its own dispersed B-side endpoint. Both variants receive the
same route waypoints through ordinary Fleet API calls. Collision stopping
remains external and uses the exit-priority, minimum-1-second release rule.

| Outcome at 300 s cutoff | 4 pass-through | 4 collision-stop | 20 pass-through | 20 collision-stop |
| --- | ---: | ---: | ---: | ---: |
| Robots past the B-side exit | 4/4 | 4/4 | 20/20 | 20/20 |
| Time when all passed | 9.7 s | 13.6 s | 10.9 s | 37.3 s |
| Overlap entries before entrance | 6 | 6 | 138 | 154 |
| Overlap entries in corridor/boundary | 0 | 0 | 20 | 0 |
| Overlap entries near destinations (both x>=4) | 0 | 0 | 1 | 1 |
| Stop / resume commands | 0 / 0 | 6 / 6 | 0 / 0 | 57 / 57 |
| Peak blocked before entrance | 0 | 3 | 0 | 19 |
| Peak blocked inside corridor | 0 | 0 | 0 | 0 |
| Wall overlap samples | 0 | 0 | 0 | 0 |

The 20-robot response run now demonstrates overlap and blocking before the
entrance, followed by a 26.4 s increase in all-pass time. It does not claim
physical nonpenetration: kinematic robots can still overlap. One overlap
entry remained near the dispersed destinations in both 20-robot variants.
Those pairs are included in the overlap metrics but, because both robots had
already passed the B-side exit, they do not trigger `stop`. Post-exit safety
response would be a separate policy question. The result is specific to this
synthetic geometry and workload.

## First shortened entrance-merge route (3 m corridor)

The wall now spans x=[-1.5,1.5]. The route uses waypoints x=-1.95 and
x=1.95; the four feeders, destination layout and external response rule are
unchanged. A 20-robot run with a 60 s cutoff produced:

| Outcome | Pass-through | Collision-stop |
| --- | ---: | ---: |
| Robots past the B-side exit | 20/20 | 20/20 |
| Time when all passed | 9.4 s | 36.2 s |
| Overlap entries before entrance | 134 | 177 |
| Overlap entries in corridor/boundary | 0 | 0 |
| Overlap entries after exit | 124 | 1 |
| Stop / resume commands | 0 / 0 | 52 / 52 |
| Peak blocked before entrance | 0 | 18 |
| Peak blocked inside corridor | 0 | 0 |

The shortened route still produces an entrance-side queue and delayed passage.
It changes the location and number of post-exit overlap observations, so the
original 6 m counts above remain historical evidence rather than current
expected results.

## Current entrance-merge route (1.5 m corridor)

The wall spans x=[-0.75,0.75], with route waypoints x=-1.2 and x=1.2.
The same 20-robot, 60 s check produced:

| Outcome | Pass-through | Collision-stop |
| --- | ---: | ---: |
| Robots past the B-side exit | 20/20 | 20/20 |
| Time when all passed | 8.6 s | 35.9 s |
| Time when all reached endpoints | 16.1 s | 43.4 s |
| Overlap entries before entrance | 136 | 170 |
| Overlap entries in corridor/boundary | 0 | 0 |
| Overlap entries after exit | 118 | 0 |
| Stop / resume commands | 0 / 0 | 75 / 75 |
| Peak blocked before entrance | 0 | 18 |
| Peak blocked inside corridor | 0 | 0 |

The entrance-side blocking and all-pass delay remain observable. Zone counts
and stop counts changed with the shorter path, so prior tables remain historical
measurements for their stated geometries.
Following Human review, the response was extended from corridor exit through
endpoint arrival. A 20-robot rerun at the 300 s cutoff retained the passage
and overlap counts above; all endpoints were reached at 16.1 s for pass-through
and 43.4 s for collision-stop. A later review fix immediately resumed a stopped
robot selected as a conflict-group winner, rather than stopping every other
moving member while it waited for its cooldown. In this run, 24 of the 75
blocked intervals ended early for that reason. Earlier measurements of 38.3 s
passage and 45.8 s endpoint arrival apply to the pre-fix response rule.
This particular workload produced no post-exit
stop under the extended rule. A focused injected-observation test confirms
that a fresh overlap between unfinished, post-exit robots can trigger a stop,
while a completed endpoint task is excluded. Thus the endpoint outcome is
reported separately; the measured 20-robot delay here remains attributable
to the entrance-side conflicts rather than a claimed post-exit jam.

Pre-entry admission would avoid some overlap, but it changes the experiment
from collision-induced failure toward traffic prevention and is not proposed
as the default. Automatic collision response in PBF core was not needed.
