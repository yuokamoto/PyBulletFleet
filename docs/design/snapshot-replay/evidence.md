# Navigation replay: implementation evidence and review handoff

**Usage status:** Evidence for a restricted development profile, not a general
or production snapshot/replay capability.

**Status:** Implementation verification complete; Copilot review findings
addressed and PR #50 merged on 2026-09-29.

## Verification and performance

Replay tests: 36 passed, including three independent fresh-process re-executions,
a case showing that a direct core command is not captured as session input,
and malformed observation container validation.
`make verify` passed: 1748 passed, 12 skipped, 1 xfailed, 81.38% coverage
(75% required). The new Python files were also passed explicitly to pre-commit
before their first commit. The representative example completed with a
matched repeated run and an observed velocity difference for the variant.

The matched benchmark used macOS 14.6.1/x86_64, Python 3.12.5, local temporary
storage, the parent source revision (`d7c290b`) for `baseline`, 120 measured
steps after 10 warmup steps, fixed `dt=0.1`, navigation inputs for all agents
every 20 steps, and three separate processes per case. Values below are medians
of each process's mean step time; RTF uses measured loop time. `disabled` is the
current core with no session. `session` owns a simulation but writes nothing.
`record_1hz` writes observations every 10 steps; `record_every_step` writes all.
Run with `python benchmark/replay_benchmark.py --baseline-root
/tmp/pbf-replay-before-d7c290b --output /tmp/pbf-replay-results-external-session.json`.
These results were refreshed after replay control moved outside the core;
do not combine their absolute times with the earlier measurement session.

| Agents | Controller | Baseline ms | Disabled ms | Session ms | 1 Hz record ms | Every step ms | 1 Hz RTF | Every step RTF |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 100 | omni | 2.77 | 2.78 | 2.99 | 3.06 | 3.64 | 32.69 | 27.42 |
| 100 | batch_omni | 2.94 | 2.89 | 3.25 | 3.30 | 3.95 | 30.30 | 25.32 |
| 1000 | omni | 31.24 | 31.74 | 36.63 | 37.24 | 44.97 | 2.69 | 2.22 |
| 1000 | batch_omni | 31.73 | 31.28 | 36.27 | 37.03 | 44.88 | 2.70 | 2.23 |

At 1000 agents, 1 Hz artifact sizes were 2.86 MiB (omni) and 3.03 MiB
(batch_omni) for the 13 simulated seconds, versus 21.64 and 22.87 MiB for
every-step observations. Sampled peak RSS rose from 87.3 to 91.0 MiB for
omni and 91.0 to 94.8 MiB for batch_omni between disabled and 1 Hz recording.
Setup medians for 1000 agents were 0.60/0.67 s disabled and 0.65/0.71 s at
1 Hz; finalization was approximately 0.02 s at 1 Hz and 0.06 s every step.
The isolated process runs vary, so these figures characterize this workload,
machine and storage only. Power and thermal state were not captured, so the
absolute times should not be attributed to a specific machine condition.
Neither 5% nor 20% overhead is an approval gate.
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
| No silent loss of session-managed records | Writer failure, unsupported-input and unfinished-command tests; direct core calls are outside the contract |
| No unnecessary disabled capture | `test_no_observation_construction_without_writer`; ordinary core has no replay hooks |
| Representative navigation/stop value | `examples/replay/navigation_reexecution.py`; explicitly synthetic |
| Existing behavior / CI checks | Repository verification results below |

## Self-review findings

- The ordinary dispatcher keeps synchronous ack semantics and existing
  CommandEvent emission. Durable records use complete session inputs and
  returned acks; direct commands through the private core are not captured.
- Required I/O failures use direct session calls, not EventBus subscribers.
- Runtime IDs never become artifact identities; record comparison uses stable IDs.
- Full observations cannot claim to restore controller/plugin/engine state.
- Recorder memory is bounded by current state/input batch and normal file buffers;
  session dispatchers disable the old unbounded diagnostic command-event list.
- Replay execution control now lives in `ReplaySession.step()`, outside the
  ordinary core/API paths. The session checks some persistent runtime changes,
  but direct core commands are unrecorded and need not be rejected. They remain
  outside the replay contract, not within an arbitrary-Python sandbox.
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
