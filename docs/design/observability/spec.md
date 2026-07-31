# Observability and End-to-End Operation Tracing

## Goal

Make one fleet operation diagnosable from its ingress to its outcome.  A
developer should be able to follow an RMF task or ROS/fleet command through
the simulation action and any workcell/plugin work, then correlate that trace
with structured logs and event records.  The same vocabulary should remain
usable when a Robot Proxy connects real robot applications.

This is operation-level observability.  It must not create a tracing span for
every simulation step.

## Correlation model

Each external or local operation carries:

- an OpenTelemetry-compatible trace context when the caller provides one;
- a stable `operation_id` when it does not;
- optional `fleet_id`, `robot_id`, `task_id`, `request_id`, and `command_id`.

The bridge, `FleetCommandDispatcher`, action system, and plugins propagate
these fields without deriving ownership from an agent type.  Child work, such
as an action started by a workcell operation, creates a child span or span link
while retaining the same operation ID.

## Span boundaries

Create spans for meaningful operation transitions only:

1. RMF task or ROS/fleet command accepted or rejected.
2. Command resolved to an agent and an action enqueued.
3. Action started and completed, failed, or cancelled.
4. Workcell/dispenser/ingestor work started and completed when applicable.

Simulation-step, pose-update, and per-message spans are prohibited.  Exporters
are opt-in and use asynchronous/batched processors so disabled observability
has no hot-path cost.

## Logs, events, and state

Structured logs and persisted events use the same correlation fields as spans.
`sim_time` and `step` are added where a simulation context exists.  Fleet state
is a current-state publication, not a trace stream; it may carry an operation
reference only when that is useful and bounded.

## Acceptance criteria

- A failed command can be located by `operation_id` and followed through every
  implemented boundary to its terminal action status.
- ROS 2, fleet API, and simulator-local commands produce the same core span
  attributes.
- A trace from a future Robot Proxy can be compared with a simulator-only
  trace without schema translation.
- Sampling cannot remove the structured terminal outcome of an operation.
