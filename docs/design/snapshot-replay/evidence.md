# Navigation replay: implementation evidence and review handoff

**Status:** Implementation verification complete; independent review and Human
Final Review are still required. No commit, push or merge is part of this change.

## Verification and performance

Replay tests: 49 passed, including three independent fresh-process re-executions
and regression cases for all three Copilot mutation-path findings.
`make verify` passed: 1761 passed, 12 skipped, 1 xfailed, 81.43% coverage
(75% required). The new Python files were also passed explicitly to pre-commit,
since they are not tracked yet. The representative example completed with a
matched repeated run and an observed velocity difference for the variant.

The matched benchmark used macOS 14.6.1/x86_64, Python 3.12.5, local temporary
storage, the parent source revision (`d7c290b`) for `baseline`, 120 measured
steps after 10 warmup steps, fixed `dt=0.1`, navigation inputs for all agents
every 20 steps, and three separate processes per case. Values below are medians
of each process's mean step time; RTF uses measured loop time. `disabled` is the
current core with no session. `session` owns a simulation but writes nothing.
`record_1hz` writes observations every 10 steps; `record_every_step` writes all.
Run with `python benchmark/replay_benchmark.py --baseline-root
/tmp/pbf-replay-before-d7c290b --output /tmp/pbf-replay-results-review.json`.
These results were refreshed after the Copilot mutation-path fixes; do not
combine their absolute times with the earlier measurement session.

| Agents | Controller | Baseline ms | Disabled ms | Session ms | 1 Hz record ms | Every step ms | 1 Hz RTF | Every step RTF |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 100 | omni | 4.78 | 4.45 | 5.31 | 5.37 | 6.34 | 18.60 | 15.76 |
| 100 | batch_omni | 4.78 | 4.79 | 5.17 | 5.42 | 6.38 | 18.44 | 15.66 |
| 1000 | omni | 47.44 | 47.68 | 56.25 | 58.36 | 68.36 | 1.71 | 1.46 |
| 1000 | batch_omni | 50.21 | 49.21 | 55.55 | 57.15 | 70.34 | 1.75 | 1.42 |

At 1000 agents, 1 Hz artifact sizes were 2.83 MiB (omni) and 3.00 MiB
(batch_omni) for the 13 simulated seconds, versus 21.23 and 22.51 MiB for
every-step observations. Sampled peak RSS rose from 87.2 to 90.9 MiB for
omni and 91.3 to 95.2 MiB for batch_omni between disabled and 1 Hz recording.
Setup medians for 1000 agents were 0.92/1.07 s disabled and 1.00/1.15 s at
1 Hz; finalization was approximately 0.03 s at 1 Hz and 0.09/0.10 s every step.
The isolated process runs vary, so these figures characterize this workload,
machine and storage only. Neither 5% nor 20% overhead is an approval gate.
The added cost is material at 1000 agents and should inform later optimization.

`make docs` passed with the repository virtual environment and `LC_ALL=C`.

## Approved acceptance criteria and evidence

| Criterion | Evidence |
| --- | --- |
| Fresh supported initial world without ROS/USO/GUI | `test_generated_identity_persisted`, fresh-session tests |
| Complete supported input payload, step/order and rejection | `test_rejections_same_step_stop_order_and_outcomes`, order test |
| Independent repeatability | Both omni/batch_omni repeated executions; `test_fresh_process_three_reexecutions` |
| Pose/velocity and discrete comparisons | `compare`, every-step test artifacts, variant test |
| Explicit changed-condition comparison | `test_variant_config_reports_conditions_and_first_observed_difference` |
| Input → ack → outcome traceability | Outcome input_step/input_order/command_id; representative example |
| Unsupported/invalid/incomplete distinct | Version, identity, asset/environment, integrity and semantic validation tests |
| No silent required record loss | Writer failure, unsupported-input and unfinished-command tests |
| No unnecessary disabled capture | `test_no_observation_construction_without_writer`; core hooks inactive in ordinary runs |
| Representative navigation/stop value | `examples/replay/navigation_reexecution.py`; explicitly synthetic |
| Existing behavior / CI checks | Repository verification results below |

## Self-review findings

- The ordinary dispatcher keeps synchronous ack semantics and existing
  CommandEvent emission. Durable records use complete inputs and returned acks.
- Required I/O failures use direct session calls, not EventBus subscribers.
- Runtime IDs never become artifact identities; record comparison uses stable IDs.
- Full observations cannot claim to restore controller/plugin/engine state.
- Recorder memory is bounded by current state/input batch and normal file buffers;
  session dispatchers disable the old unbounded diagnostic command-event list.
- Core/user mutation guards are scoped to active owned sessions. They are not an
  arbitrary-Python sandbox; private/raw engine mutation remains outside contract.
- Per-agent angular motion can report speed rather than signed yaw rate. The
  artifact and guide preserve this distinction instead of asserting a universal
  engine velocity meaning.
- Initial profiling found repeated dataclass deep copies in profile validation;
  see performance evidence for the measured simplification and final results.

## Deviations / implementation-level decisions

- The initial bundled-model allowlist is `simple_cube`; static box geometry is
  supported. No generalized external model/asset packaging was added.
- Provenance records a PBF source-content hash rather than depending on git
  revision/dirty metadata. It works in wheels and fingerprints dirty source trees.
- Supported collision transitions are derived from the core's active pair set
  at the direct end-of-step hook. No new general persisted SimEvent API is added.
- The profile uses canonical `step * dt` time; ordinary simulation timestamps
  are unchanged. Comparison reports transition step versus completed state step.
- No public universal snapshot/restore API, ROS adapter, shared lazy-capture
  framework, delta mechanism, or asynchronous pipeline was introduced.

## PBF → USO findings

### Likely simulator-independent concepts supported by this implementation

- Durable entity identity separate from API address and engine handle.
- Explicit units/frame and state-time semantics.
- Integer state/transition steps in addition to floating simulation time.
- Initial state versus observations versus execution checkpoints.
- Schema/profile version, source/assets/configuration provenance and completeness.
- Fullness relative to a declared schema/profile, including static-data references.

These are candidates validated in one application, not a canonical contract.

### PBF-specific concepts

- Fleet API navigate/stop payloads, allowed-name validation and ack semantics.
- Input phase relative to PBF PRE_STEP and two-phase pose commit.
- Omni/batch_omni parameters and reported-velocity behavior.
- Collision-check opportunity timing and profile arrival/stop predicates.
- Bundled PyBullet model construction and package-source fingerprint.

### Assumptions needing another backend

- Whether the same common pose/velocity fields can have identical meanings.
- Whether step/phase/order maps cleanly onto another engine's scheduling.
- Which initial configuration is sufficient without runtime checkpoints.
- What shared identity/model mapping survives simplified versus articulated assets.
- Whether completeness/version rules fit another producer and reader.

### Concrete USO design changes to consider through Human review

1. Distinguish initial definition, full observation, and resumable execution state.
2. Specify timestamp epoch/units plus integer state_step; keep application phase/order
   in execution records instead of overloading world-state timestamps.
3. Separate asset identity from name/address and transient engine handle.
4. Define frame and velocity semantics; avoid assuming controller-reported values
   equal physics-solver values across implementations.
5. Define the reference closure of a full snapshot (static definitions/assets).
6. Keep input journals/profile requirements outside rendering reproduction_info.
7. Version state schemas and execution capabilities explicitly; reject unknown
   required versions instead of silently best-effort restoring.
8. Add a completeness/integrity contract distinct from behavioral equality.

The USO repository was not modified. A shared runtime package or compatibility
claim still requires explicit design and cross-backend validation.

## Remaining limitations and follow-ups

Only the approved v1 profile is implemented. Live ROS ingress/rosbag correlation,
external assets, differential/joint/action/device/BT profiles, playback, delta
storage and checkpoints require separate use cases and scope decisions. Sparse
observations locate only the first recorded difference. The source/environment
fingerprint is conservative and cannot prove full hardware equivalence. File
integrity is not authentication or a power-loss transaction guarantee.

The workflow's existing approval gates were sufficient; no workflow changes or
new agent roles were introduced for this first functional trial.
