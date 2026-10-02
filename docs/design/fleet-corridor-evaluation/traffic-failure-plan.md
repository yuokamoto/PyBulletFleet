# One-sided corridor traffic failure — implementation plan

**Status:** Entrance-merge route and revised external response rule tested with
4 and 20 robots. See
`traffic-failure-pilot-evidence.md`.

Keep the existing two-sided corridor example and simulation core unchanged.
Add one separate external example that runs identical one-sided workloads in
pass-through and collision-response modes. The example owns its policy,
task/crossing ledger, and JSON evidence. PBF continues to own motion,
commands and sampled collision facts. No new public or core API is planned,
so no material Architecture Gate is required.

1. Calibrate a few-robot pilot with the existing corridor walls and robot
   model. Use non-overlapping A-side starts in four feeder lanes and external
   Fleet API waypoints before the entrance, just beyond the exit, and at
   separated B-side endpoints. Verify a pass-through run gives a fresh
   robot–robot geometric overlap before the entrance; verify wall contacts are not used as stop
   triggers. If no useful merge occurs, revise only scenario geometry.
2. Implement the external response: after each completed check, form connected
   groups of fresh overlapping robot–robot pairs. Within each group, keep the
   robot nearest the B-side corridor exit moving, with stable-ID ties; stop the
   others once and hold them for at least 1 simulated second. On later checks,
   reissue navigation for at most one stopped robot per step: the stopped robot
   nearest the exit, once its cooldown has elapsed. It can resume despite a
   persisting overlap; record repeated stops, stop/reissue acknowledgements
   and unresolved blocks. Do not use teleport or infer an automatic core
   collision response.
3. Compare the modes at the same timestep, start poses, routes and 300 s
   cutoff. Record first corridor entry and B-side exit of each robot, primary
   all-pass time (absent if unfinished), operational `deadlock_at_cutoff`,
   blocked-time distribution, blocked-robot count/queue geometry, endpoint
   completion, overlap entries by pre-entrance/corridor/post-exit zone and
   wall/RTF execution cost. Run 20 robots
   only after the small pilot demonstrates the intended chain or report why
   it cannot.
4. Add focused tests for workload equivalence, exit-priority winner selection,
   minimum cooldown, repeated conflict, crossing accounting
   and cutoff censoring. Run the pilot and 20-robot cases headless, then
   `make verify` and `make docs`. Self-review scenario semantics, accidental
   core coupling and performance before asking for Human Final Review.

The initial strict-clearance pilot failed because stopped robots could be
selected as winners yet never resume. Human proposed exit-priority release;
the revised pilot passed at 4 and 20 robots without core changes. If another
geometry reveals a nonrecoverable group, return to Human Scope Review rather
than adding core collision physics or a generic traffic policy.
