# Co-Simulation — Distributed Robot Proxy Architecture

**Status:** Draft (not yet implemented)
**Date:** 2026-05-28
**Related:** [snapshot-replay/spec.md](../snapshot-replay/spec.md), [ros2-bridge](../../../ros2_bridge/README.md), [eventbus/spec.md](../eventbus/spec.md)

## Motivation

100 体以上のロボットを実機ソフトウェア（Nav2, task assigner, BT, FSM 等）と組み合わせて
シミュレーションする際、**シミュレータ本体ではなく Robot App 層がボトルネック**になる。

- Sim core: vectorized / batched 済み (per-agent ~µs)
- Robot App: シングルロボット前提で書かれている (per-tick ~10ms)
- 100 体 × 10ms = 1s/tick → real-time 崩壊

→ Robot App を **per-robot プロセス**で分散させる必要がある。
ただしシミュレータ自体は集中のままが速い (shared state, batch 最適化が効く)。

## Architecture: 3-Layer Co-Simulation

```
┌─────────────────────────────────────────────────────────┐
│  Layer 3: Robot App  (existing, unmodified, per-robot)  │
│  Nav2 / planner / task executor / perception            │
│       ▲ /robot_N/odom, /battery_state, /scan            │
│       │ /robot_N/cmd_vel, /goal_pose          ▼         │
├─────────────────────────────────────────────────────────┤
│  Layer 2: Robot Proxy  (NEW, per-robot, distributed)    │
│  - ROS pub/sub ⇄ sim state translation                  │
│  - use_sim_time clock sync                              │
│  - Sensor synthesis (lidar/odom from sim pose)          │
│       ▲ state read              │ cmd write             │
│       │ (shared mem / gRPC)     ▼                       │
├─────────────────────────────────────────────────────────┤
│  Layer 1: Sim Central  (existing core_simulation.py)    │
│  - World state (poses, batteries, collisions)           │
│  - Kinematic integration / collision broadphase         │
│  - Clock master                                         │
└─────────────────────────────────────────────────────────┘
```

| Layer | Cost | Centralized/Distributed | Modify? |
|-------|------|------------------------|---------|
| 3. Robot App | Heavy (5–20 ms/tick) | Already distributed via ROS | ❌ No modification |
| 2. Robot Proxy | Light (<100 µs/tick) | **Distributed** (1 process/robot) | ✅ New component |
| 1. Sim Central | Medium (batched) | Centralized | Existing, unchanged |

## Design Principles

### 1. Narrow Boundary, Not RPC-ification

**Anti-pattern (rejected):** Expose `agent.set_velocity()`, `agent.get_pose()`,
`sim.get_collisions()` etc. over network.

| Problem | Impact |
|---------|--------|
| Chatty interface | 1000 RPC/tick × 100Hz = 100k RPC/sec |
| N+1 problem | `for agent in agents: agent.get_pose()` → 100 round-trips |
| Latency death | 200 µs × 1000 RPCs = 200 ms/tick |
| Tight coupling | Method signature changes break all proxies |

**Correct pattern:** Publish a **fixed-schema batched snapshot**; collect commands
in a fixed-schema buffer. Only 3 things cross the boundary:

| Direction | Form | Frequency |
|-----------|------|-----------|
| **State snapshot** (Central → Proxy) | `ndarray[N, K]`, broadcast | Every step |
| **Command buffer** (Proxy → Central) | `ndarray[N, M]`, collected | Every step |
| **Events** (Central → Proxy) | `(robot_id, type, payload)` | Sparse, on occurrence |

### 2. Dual API Layers

In-process API (existing) is preserved unchanged for tests, plugins, and lightweight
inline callbacks. Network API is a **new thin facade** that exposes only the
boundary contract above.

| Layer A (in-process) | Layer B (network) |
|---------------------|-------------------|
| `agent.add_action()`, `sim.spawn()`, etc. | `StateSnapshot`, `CommandBuffer`, `Event` |
| Full Python objects | numpy arrays + primitives |
| Free to refactor | Stable ABI |
| Tests, plugins, light brain | Robot Proxy, external tools |

### 3. Per-Robot Process for Layer 2 (Even Though Lightweight)

Aggregating all proxies into one process is tempting (proxy work is light), but
should be avoided because:

1. **Topology must match real fleet** — Robot App expects 1 robot = 1 namespace
2. **Fault isolation** — One proxy crash should not kill others (matches real hardware)
3. **Deployment flexibility** — Robot App can run on different hosts / k8s pods
4. **DDS discovery scale** — Single-process DDS participant bloat hurts at 100+
5. **`use_sim_time` propagation** — Proxy + App as deployment unit

The whole point of co-simulation is that the Robot App **cannot tell it's in sim**.

## Schema Definitions

### StateSnapshot (Central → Proxy)

Published every sim step. Single contiguous buffer; proxy N reads row N.

```python
@dataclass
class StateSnapshot:
    sim_time: float                  # scalar
    step_count: int                  # scalar
    poses: np.ndarray                # [N, 7]  x,y,z,qx,qy,qz,qw
    velocities: np.ndarray           # [N, 6]  vx,vy,vz,wx,wy,wz
    batteries: np.ndarray            # [N]     0.0–1.0
    statuses: np.ndarray             # [N]     enum int (idle/moving/charging/...)
    # Optional per-robot extensions registered at init time
```

### CommandBuffer (Proxy → Central)

Proxy writes its own row; Central drains at the head of each step.

```python
@dataclass
class CommandBuffer:
    target_velocities: np.ndarray    # [N, 6]   NaN = not set
    target_poses: np.ndarray         # [N, 7]   NaN = not set
    flags: np.ndarray                # [N]      bitmask: HAS_VEL_CMD | HAS_POSE_CMD | ...
```

### Events (Central → Proxy)

Sparse, structured. Goes through the existing EventBus (see [eventbus/spec.md](../eventbus/spec.md)).

| Event | Payload | Subscribers |
|-------|---------|-------------|
| `collision.enter` / `collision.exit` | `(id_a, id_b, point)` | Proxy emits `/contact` topic |
| `agent.pick` / `agent.drop` | `(robot_id, object_id)` | Proxy emits `/gripper_state` |
| `agent.spawn` / `agent.remove` | `(robot_id, urdf_path)` | Proxy starts/stops ROS node |
| `sim.pause` / `sim.resume` | — | Proxy pauses publishing |

## Transport Options

| Transport | Latency | When to use |
|-----------|---------|-------------|
| **`multiprocessing.shared_memory` + ndarray view** | ~10 µs | Same-host (default, dev, benchmark) |
| **gRPC streaming** | ~200 µs | Cross-host, k8s deployments |
| **DDS direct** | ~500 µs | Full ROS integration, accepts heavier latency |
| **ZeroMQ** | ~50 µs | Polyglot (C++ proxy talking to Python core) |

**Recommended hybrid:** shared memory for the dense per-step ndarrays + ROS topics
for sparse structured commands and events.

## Sync Modes

### Lockstep (deterministic, benchmark)

```
central:  step(dt) → publish snapshot → wait for all N acks → step(dt) ...
proxy:    recv snapshot → tick App → submit cmd → ack
```

- Required for regression tests and replay
- Critical path = slowest brain
- All proxies must complete within step deadline

### Async / Real-Time (operational)

- Central advances on wall clock
- Proxies do best-effort; late commands apply on next tick
- Matches real hardware behaviour
- Default for `ros2_bridge` integration

Mode selection: `SimulationParams.brain_sync_mode: Literal["lockstep", "async"]`

## Relationship to Snapshot/Replay

**This is the key architectural synergy.** The IPC snapshot and the replay snapshot
share the same schema; same source feeds both sinks.

```
                    ┌─────────────────────────┐
                    │  StateSnapshot (single  │
                    │  schema, single emit)   │
                    └────────────┬────────────┘
                                 │
              ┌──────────────────┼──────────────────┐
              ▼                  ▼                  ▼
        ┌──────────┐      ┌──────────┐      ┌──────────┐
        │ Live IPC │      │ Replay   │      │ Observ-  │
        │ (proxy)  │      │ log      │      │ ability  │
        └──────────┘      └──────────┘      └──────────┘
        ring buffer       append-only        OTel spans
        shared mem        JSONL file         (sampled)
        every step        configurable Hz    Action-level
        lossy ok          lossless           lossy ok
```

| Aspect | Live IPC (Co-sim) | Replay Log |
|--------|-------------------|------------|
| Storage | Ring buffer in shared memory | Append-only file (JSONL / binary) |
| Frequency | Every step (must match sim rate) | Configurable (delta + periodic full) |
| Completeness | Current frame only | Full history |
| Loss tolerance | Acceptable (next frame coming) | Must be lossless |
| Schema | `StateSnapshot` (defined above) | Same `StateSnapshot` + headers |
| Needs commands? | Yes (Layer 2 → Layer 1) | **Yes, for reproducible replay** |

### Design Win: Define Schema Once

Both systems consume the **same dataclass**. Implementation:

1. `step_once()` produces a `StateSnapshot` object once per step
2. Subscribers fan out:
   - `IpcPublisher` → writes to shared memory ring buffer (for proxies)
   - `ReplayLogger` → appends to JSONL (existing `snapshot-replay` spec)
   - `MetricsExporter` → samples for Grafana (existing `observability` spec)

### Missing Piece in Current Replay Design

The existing [snapshot-replay/spec.md](../snapshot-replay/spec.md) records **output state**
but not **input commands** that came from external proxies. For true distributed replay,
the `CommandBuffer` must also be logged each step. Add:

```
run_<timestamp>/
  snapshots.jsonl     # State output (existing)
  events.jsonl        # Causal events (existing)
  commands.jsonl      # NEW: Input commands from proxies
```

Then `replay.py` can re-run the sim deterministically by:
1. Restoring from latest snapshot
2. Replaying recorded commands instead of querying live proxies
3. Verifying state matches recorded snapshots

## Implementation Plan

Six phases, mostly additive to existing code.

### Phase 1: Snapshot Schema & In-Process Producer
- Define `StateSnapshot` and `CommandBuffer` dataclasses
- Add `core_simulation.step_once()` hook to populate `StateSnapshot` at end of step
- Add `CommandBuffer.drain_into_agents()` hook at head of step
- **No IPC yet** — verify schema via in-process subscriber

### Phase 2: Shared Memory Transport
- `AgentStateBuffer` — `multiprocessing.shared_memory` ndarray wrapper
- `AgentCommandBuffer` — same, reverse direction
- Lifecycle: create at spawn, destroy at remove
- Benchmark vs in-process baseline (target: <5% overhead)

### Phase 3: Robot Proxy Node Template
- `RobotProxyNode` base class
- `Ros2RobotProxy` subclass (extends existing `ros2_bridge`)
- `run_proxies.py` supervisor: spawns N proxy processes
- Mapping: `robot_id` → shared memory row N

### Phase 4: Lockstep Synchronization
- Barrier primitive (e.g. `multiprocessing.Barrier`)
- `SimulationParams.brain_sync_mode`
- Per-step deadline + timeout handling
- Tests: deterministic output across runs

### Phase 5: Event Channel
- Wire EventBus to a multiprocess-safe queue per proxy
- Implement `collision`, `pick`/`drop`, `spawn`/`remove` topics

### Phase 6: Unified Snapshot Sink (Replay Integration)
- Refactor existing `ReplayLogger` (from snapshot-replay spec) to consume
  the same `StateSnapshot` produced in Phase 1
- Add `CommandBuffer` recording for input replay
- Verify replay matches live execution bit-for-bit (lockstep mode)

## Out of Scope (Explicit Non-Goals)

- **Sensor simulation** — Lidar/camera synthesis is a separate concern.
  Proxy may include trivial pose-based odom; rich sensors go through dedicated subsystems.
- **Cross-process sim sharding** — Sim Central stays single-process. Multi-process
  sim (e.g. one world per warehouse) is a separate roadmap item.
- **Authentication / security** — Assumes trusted environment (local cluster or VPN).
- **Heterogeneous robot types in one buffer** — Different robot classes get different
  shared memory regions. Mixing is supported via multiple `StateSnapshot` instances.

## Open Questions

1. **Buffer reallocation on spawn/remove** — Pre-allocate worst-case N, or
   grow dynamically? Pre-allocation is simpler and faster but caps fleet size.
2. **Per-robot extension fields** — How do users register custom state fields
   (e.g. tool state, payload mass) without breaking the fixed-schema contract?
3. **Backpressure in async mode** — If a proxy falls behind, do we drop its
   commands or queue them? Real hardware would drop; queueing risks instability.
4. **Cross-host shared memory** — gRPC fallback is clear, but is there a hybrid
   (RDMA, shared memory over network) worth supporting for tight clusters?
