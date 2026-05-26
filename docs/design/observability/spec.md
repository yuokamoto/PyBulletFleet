# Observability — Design Spec

**Status:** Draft (not yet implemented)
**Date:** 2026-05-14
**Related:** [eventbus/spec.md](../eventbus/spec.md), [snapshot-replay/spec.md](../snapshot-replay/spec.md)
**Scope:** Structured logging, metrics (Prometheus/OTLP), distributed tracing (OpenTelemetry)

## Motivation

Current logging is plain text via `LazyLogger` / `NamedLazyLogger`. Long-running benchmarks (100+ agents, hours) and production deployments need:

- **Filterable / queryable logs** — Loki + Grafana, not `grep` on stderr
- **Time-series metrics** — FPS, action throughput, collision counts as scrapable gauges/counters
- **Causal traces** — "which Pick action took 12s and why" — Action-level spans in Grafana Tempo

This spec covers the operational observability layer. **It is intentionally separate from Replay** (covered by [snapshot-replay/spec.md](../snapshot-replay/spec.md)) because:

- Trace is **sampled** (cannot be replay source of truth)
- Trace is **coarse-grained** (Action-level, not per-step state)
- Trace **drops non-deterministic inputs** (RNG seed, callback returns)
- Replay needs lossless Event log + periodic Snapshot; Trace needs neither

However both layers **share the EventBus as their single emission point**.

## Layer Relationship

```
                    ┌──────────────┐
  Action transition │              │
  Spawn / Remove ──►│   EventBus   │  (single source of truth)
  Collision         │              │
                    └──────┬───────┘
                           │
        ┌──────────────────┼─────────────────────┐
        ▼                  ▼                     ▼
  events.jsonl       snapshots.jsonl       OTel exporter
  (lossless)         (low frequency)       (sampled)
        │                  │                     │
        └──── Replay ──────┘              Tempo / Grafana
                                          (this spec)
```

| Aspect | Replay | Observability (this spec) |
|---|---|---|
| Source | EventBus + periodic snapshot | EventBus (subset) |
| Lossless? | Required | Sampling allowed |
| Granularity | Per state-change | Action / Step / aggregated |
| Output | JSONL files (local or USO) | Loki / Tempo / Prometheus |
| Failure tolerance | Must not lose data | Best-effort; drop on backpressure |

## Components

### 1. Structured Logging

**Change:** `NamedLazyLogger` prefix string → `extra={...}` dict.

Before:
```python
self._log = NamedLazyLogger(__name__, prefix=f"[Agent:{self.object_id}:{self.name}] ")
self._log.info("Path complete")
# => [Agent:3:robot_A] Path complete
```

After:
```python
self._log = NamedLazyLogger(__name__, context={"agent_id": self.object_id, "name": self.name})
self._log.info("path_complete")
# => {"msg": "path_complete", "agent_id": 3, "name": "robot_A", "sim_time": 12.34, "step": 2961}
```

JSON formatter via `python-json-logger` (lightweight, stdlib-compatible).
Backward compatibility: keep `prefix=` kwarg as deprecated alias.

### 2. `SimContextFilter`

`logging.Filter` registered by `MultiRobotSimulationCore`. Auto-attaches:

- `sim_time` (float) — current physics time
- `step` (int) — tick number
- `run_id` (str) — UUID per `run_simulation()` invocation

Removes need for `extra={}` boilerplate on every log call inside the sim loop.

### 3. Metrics Exporter

`prometheus_client` based; opt-in via `SimulationParams(metrics_port=9090)`.

| Metric | Type | Labels | Source |
|---|---|---|---|
| `pbf_step_duration_seconds` | Histogram | — | `step_once()` timing |
| `pbf_steps_total` | Counter | — | step counter |
| `pbf_active_agents` | Gauge | — | `len(_agents)` |
| `pbf_active_objects` | Gauge | — | `len(_sim_objects)` |
| `pbf_actions_total` | Counter | `type`, `status` | EventBus `action.complete` |
| `pbf_action_duration_seconds` | Histogram | `type` | EventBus action span |
| `pbf_collision_pairs` | Gauge | — | collision detection result |
| `pbf_callback_duration_seconds` | Histogram | `name` | callback invocation |

OTLP exporter as alternative (push to OTel Collector).

### 4. OpenTelemetry Trace Exporter

EventBus subscriber that converts event pairs into spans.

**Span hierarchy:**

```
simulation.run               (root, lifetime of run_simulation())
├── agent.{id}.action.MoveAction       (action.start → action.complete)
├── agent.{id}.action.PickAction
└── agent.{id}.action.DropAction
```

**Span attributes:**

- `agent.id`, `agent.name`
- `action.type`, `action.status` (final)
- `sim_time.start`, `sim_time.end`, `sim_duration`
- `path.length` (for MoveAction)
- `causation_id` = OTel `span_id` (cross-link to Loki events)

**Forbidden:**

- Per-step spans (240 Hz × 100 agents = 24k spans/s; exporter cannot keep up)
- Storing full `Path` waypoints as attributes (use `path.length` summary)

**Exporter:** `BatchSpanProcessor` with OTLP gRPC. Sampling `ParentBasedTraceIdRatioBased(0.1)` default.

### 5. Hot-Path Safety

| Rule | Why |
|---|---|
| All exporters opt-in via `SimulationParams` | Default zero overhead |
| `LazyLogger.isEnabledFor()` preserved | f-string evaluation cost |
| Batch / async processors only | Sync export blocks sim loop |
| Action-level span granularity max | Per-step spans saturate exporters |
| Metrics scraped, not pushed per step | Pull model decouples from sim FPS |

## Output Path Recommendations

| Scenario | Logs | Metrics | Traces |
|---|---|---|---|
| **Local dev** | stdout JSON → eyes / `jq` | disabled | disabled |
| **CI / benchmark** | `logs/run-<ts>.jsonl` file | scrape via `--metrics-port` | disabled |
| **Long bench / prod** | OTel Collector → Loki | OTel Collector → Prometheus | OTel Collector → Tempo (sampled) |

## Configuration

```python
SimulationParams(
    log_format="json",            # "text" | "json"
    metrics_port=9090,            # None to disable
    otel_endpoint="localhost:4317",  # None to disable
    otel_sample_rate=0.1,
)
```

## Common Header Schema (shared with Replay)

Every JSONL line (events, snapshots, log records) and every OTel span attribute set MUST include:

```json
{"sim_time": 12.345, "step": 2963, "wall_time": 1715000000.123, "run_id": "uuid"}
```

This is the contract that lets Grafana correlate Loki logs ↔ Tempo traces ↔ Prometheus metrics ↔ Replay events on a single time axis.

## Implementation Order

1. JSON formatter + `extra={}` migration in `NamedLazyLogger`
2. `SimContextFilter` registered by `MultiRobotSimulationCore`
3. EventBus integration (depends on [eventbus/spec.md](../eventbus/spec.md) being implemented)
4. Prometheus exporter (independent, low risk)
5. OTel trace exporter (after EventBus)

Steps 1–2 are independent and can ship first.

## Out of Scope

- Replay engine (see [snapshot-replay/spec.md](../snapshot-replay/spec.md))
- USO integration (uses Event log directly, not OTel)
- Visualization replacement (`p.GUI` → Rerun) — separate concern
- Distributed tracing across ROS 2 / gRPC bridges — depends on those interfaces landing first
