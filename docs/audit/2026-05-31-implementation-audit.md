# Implementation Audit — 2026-05-31

> **Superseded by [2026-06-20](2026-06-20-implementation-audit.md)** — kept as a point-in-time record. Several items here (velocity API, cleaning, cart-delivery connection, diagnostics) have since been implemented.

Hands-on reference. Reflects actual code state as of this date.  
Companion to [docs/roadmap.md](../roadmap.md) (design intent) — this document records *what is true now*, not *what we want*.

---

## Confirmed Implemented

Items that design docs describe as planned, verified as working in code.

| Feature | Key files | Notes |
|---------|-----------|-------|
| **Two-Phase Step** (Phase 1 compute / Phase 2 flush) | `core_simulation.py:3238-3315` | `phase1_update` / `phase2_pose_flush` profiling stats present; `_batch_controllers` driven each step |
| **BatchKinematicController** (base) | `controllers/batch_base.py` | NumPy `_pos_buf`, `_orn_buf`, `_moved_mask` arrays; agent row-compaction on unregister |
| **BatchOmniController** (vectorized) | `controllers/batch_omni.py` | Multi-waypoint TPI + slerp, all-NumPy; equivalence with `OmniController` within ~1e-6 |
| **BatchDifferentialController** (vectorized) | `controllers/batch_differential.py` | ROTATE → FORWARD phases, `_PHASE_FINAL_ROTATE`; all-NumPy except one scipy import (see gaps) |
| **`_tpi.py` TPI primitives** | `pybullet_fleet/_tpi.py` | `build_tpi()`, `extract_phase_params()`, `trapezoid_distance()` shared by all controllers |
| **EventBus** (global + per-entity) | `pybullet_fleet/events.py` | 12+ `SimEvents` constants; lazy per-entity allocation; priority ordering; `COLLISION_STARTED/ENDED` with enter/exit diff |
| **SimPlugin ABC** | `pybullet_fleet/sim_plugin.py` | `on_init/on_step/on_reset/on_shutdown`; Hz-based throttling; YAML `plugins:` section; dynamic `importlib` load |
| **`SimObject.from_sdf()`** | `pybullet_fleet/sim_object.py:747-842` | Wraps `p.loadSDF()`, auto-registers to sim_core; within PyBullet's SDF constraints |
| **IKPoseAction** | `pybullet_fleet/action.py` | IK goal → action queue integration |
| **CameraController** | `pybullet_fleet/camera_controller.py` | Right-drag pan, +/- zoom, `o` top-down; `p.DIRECT` graceful disable |
| **Navigation 2D flag** | `BatchKinematicController.__init__` | `navigation_2d: bool` parameter propagated to all subclasses |
| **Centralized defaults** | `pybullet_fleet/_defaults.py` | `PBF_*` env var overrides; `.env` file support; YAML > env > defaults priority |
| **Controller registry** | `pybullet_fleet/controller.py` | `@register_controller` decorator; `batch_omni`, `batch_differential` registered |

---

## Confirmed Unimplemented (design docs exist)

Evidence: class/function names searched, not found in codebase.

### P0 — Practical blockers

**1. Snapshot / Replay**  
Spec: [docs/design/snapshot-replay/spec.md](../design/snapshot-replay/spec.md)  
Missing: `StateSnapshot`, `SnapshotLogger`, `ReplayController`  
Only existing: `SimulationRecorder` (pixel video only, no state serialization)  
Note: EventBus steps 1–2 (from roadmap implementation order) are complete — snapshot is the natural next step.

**2. Velocity API on Agent/SimObject**  
No design doc exists for this, but it is a *prerequisite* for Snapshot.  
Missing: `agent.linear_velocity`, `agent.angular_velocity` properties (wrapping `p.getBaseVelocity()`)

### P1 — Scale and visibility

**3. Observability** (structured logging → Prometheus → OpenTelemetry)  
Spec: [docs/design/observability/spec.md](../design/observability/spec.md)  
What exists: `LazyLogger` (prefix-string, not structured `extra={}`), `DataMonitor` GUI  
Missing: `logging.Filter` for `sim_time`/`step` injection, Prometheus scraper, OTel EventBus subscriber

**4. Co-Simulation** (distributed RobotProxy layer)  
Spec: [docs/design/co-simulation/spec.md](../design/co-simulation/spec.md)  
Missing: `RobotProxy`, `CommandBuffer`, IPC transport layer  
Note: shares `StateSnapshot` schema with Snapshot/Replay — implement those first

### P2 — Architecture quality

**5. `MultiRobotSimulationCore` subsystem extraction**  
Target: `CollisionSystem`, `VisualizerController`, `SimProfiler` (see [roadmap.md](../roadmap.md#refactoring))  
Current: ~3200-line monolith with all three subsystems inline

**6. scipy removal from `batch_differential.py`**  
`controllers/batch_differential.py:31` still imports `from scipy.spatial.transform import Rotation as R`  
`BatchOmniController` manages without scipy — `batch_differential` should match

**7. Manual quaternion helpers in `geometry.py`**  
`quat_rotate_vector`, `quat_multiply`, `quat_from_rotvec` not yet added  
(prerequisite for full scipy removal)

**8. SimBackend ABC + NumpyBackend**  
Spec: [roadmap.md — Long-Term Phase 1](../roadmap.md#long-term-backend-abstraction--beyond-pybullet)  
No `SimBackend` ABC exists; core still imports `pybullet as p` directly  
Note: Two-Phase Step buffered-pose pattern is the natural stepping stone toward this

---

## Code Gaps — No Design Doc

Issues found in code that are not covered by any existing spec.

### Correctness

**Velocity API absent** (`agent.py`, `sim_object.py`)  
`p.getBaseVelocity()` must be called directly. Needed for deterministic replay.  
Fix: add `linear_velocity` / `angular_velocity` properties to `Agent` and `SimObject`.

**Silent failures at boundaries** (`core_simulation.py`)  
- `remove_object()` silently `pass`es on `KeyError`  
- `set_poses()` batch API does not validate that `object_ids` belong to the sim  
These should raise (or at minimum warn) to surface bugs in large simulations.

**`batch_differential` scipy import** (`controllers/batch_differential.py:31`)  
Functional gap: `BatchOmniController` is fully scipy-free; `BatchDifferentialController` is not.  
Inconsistency implies the rotation math path is different, which it shouldn't be.

**Collision Enter/Exit stale state**  
If `collision_check_frequency > 1`, collision events are only emitted on checked steps.  
An agent that appears and disappears between checks will never trigger `COLLISION_ENDED`.  
No test covers this edge case.

### API consistency

**`Agent.set_pose()` vs `SimObject.set_pose()` signature mismatch**  
`Agent` has `preserve_velocity`; `SimObject` does not. Both are pose-settable entities — document why or unify.

**`agent.stop()` vs `agent.cancel_path()`**  
Two methods with overlapping semantics exist with no clear distinction in the public API.

### Performance (not in roadmap)

**`_moved_this_step` set rebuilt from scratch each step**  
Pattern: `self._moved_this_step = set()` at step start, then populated per-object.  
A dirty-flag per agent (cleared at flush) would eliminate set allocation overhead at 1000+ agents.

**Action queue is a plain `list`**  
`queue.pop(0)` is O(N). `collections.deque` would make front-pop O(1). Low impact at typical queue depth, but free win.

### Type safety

`geometry.py`, `controller_params.py`, `action.py` contain multiple `# type: ignore` comments.  
These are not tracked anywhere — worth a sweep to either fix or document with a reason.

---

## ros2_bridge ROADMAP Audit

Source: [ros2_bridge/README.md — Roadmap section](../../ros2_bridge/README.md#roadmap)

### Confirmed Implemented (in ros2_bridge)

| Feature | Notes |
|---------|-------|
| Per-robot topics (odom, joint_states, cmd_vel, plan, diagnostics) | Working |
| Action servers (NavigateToPose, FollowPath, FollowJointTrajectory, ExecuteAction) | Working |
| Simulation services (/sim/spawn, /sim/delete, /sim/step, /sim/reset, etc.) | Working |
| toggle_attach / attach_object services | Working |
| Open-RMF fleet adapter (patrol + delivery tasks) | Working |
| DoorHandler / LiftHandler / WorkcellHandler | Working |
| Demo scenarios (office, hotel, airport via Docker) | Working |

### Near-Term (未実装)

**Batch API ROS Wrapper（Pattern 2）**  
`/fleet/states` (N台分を1メッセージ) / `/fleet/navigate` batch serviceが未実装。  
依存：`pybullet_fleet_msgs` に `FleetStates.msg` 定義が必要。  
優先度：**中** — 100体以上をROS2経由で動かすユースケースが出たときに必要。

### Mid-Term (未実装)

**Direct Python Connection（Pattern 3/4）**  
fleet_adapter を ROS2 topic をバイパスして EventBus 経由で直接接続。  
優先度：**中〜高** — CI/headless テストで ROS2 環境なしに fleet_adapter ロジックを検証できる。

**Handler Decomposition**  
`RobotHandler` → `OdometryHandler`, `JointStateHandler`, `CmdVelHandler`, `NavigationHandler`, `DiagnosticsHandler` に分解。  
優先度：**低** — 機能的な問題はなく保守性の改善。

**`cargo_relative_pose` on Agent**  
`WorkcellPlugin._attach_z_offset` を `Agent` / `AgentSpawnParams` の `cargo_relative_pose: Pose` に移動。  
優先度：**低〜中** — ロボットごとに取り付け位置が異なる場合（フォークリフト/AMR/グリッパ）に必要。

**Multi-Floor Visualization**  
フロア表示 ON/OFF キー切替 (1/2/3キー)。  
優先度：**低** — GUI全体の方針（Rerun移行まで保留）と同じ扱い。

### Low Priority (未実装 or 部分実装)

**Cart Delivery（toggle_attach方式）**  
`toggle_attach` / `attach_object` サービス自体は実装済み。  
残り：fleet_adapter の `execute_action("delivery_pickup/dropoff")` → `toggle_attach` の接続。  
優先度：**中** — 残り工数が小さい割に delivery タスク完成度が上がる。

**Cleaning Simulation**  
現在 `execute_action("clean")` は即 `finished()` で返すだけ。  
優先度：**低** — airport_terminal デモのみ使用。

**Device Enhancements**（Elevator door, Double-hinge door, Xacro テンプレート）  
未実装。デモの視覚的完成度に影響するが機能ブロッカーではない。  
優先度：**低**。

### Documentation TODO (未作成)

- Standard Delivery フロー図（dispatch_delivery → RMF → WorkcellHandler → PickAction → DropAction）
- Cart Delivery フロー図（dispatch_cart_delivery → compose → toggle_attach）
- 通信手段の違い比較（Pattern 1/2/3/4）

優先度：**中** — ROS2ブリッジは onboarding コストが高く、フロー図があるとデバッグ効率が大きく上がる。

### ros2_bridge 固有のコードギャップ（設計doc外）

**BatchController との統合例がない**  
500体を ROS2 経由で動かす場合 `BatchOmniController` を使うべきだが、`RobotHandler` は今も per-agent `OmniController` を生成している。スケール時の推奨構成がドキュメントにも実装例にも存在しない。

**`charge` タスク未対応が見つけにくい**  
README 本文に「`finishing_request` を `"nothing"` or `"park"` にしないとデッドロック」とあるが目立たない。  
RMF ユーザーがハマりやすいポイントなので、より目立つ場所（Known Limitations セクション等）に移すべき。

---

## Priority Order (as of 2026-05-31)

### Core (pybullet_fleet)

```
P0  Snapshot/Replay
      └─ prerequisite: velocity API (linear_velocity / angular_velocity properties)

P1  Observability (structured logging → sim_time filter → Prometheus)
    scipy removal from batch_differential + geometry.py helpers
    Silent-failure fixes (remove_object, set_poses validation)

P2  Core subsystem extraction (CollisionSystem, VisualizerController, SimProfiler)
    Co-Simulation (after Snapshot schema is stable)

P3  SimBackend ABC + NumpyBackend
    C++ extensions (TPI, slerp, spatial hash)
    GUI inspector panel
```

### ros2_bridge

```
中   Cart Delivery 完成（toggle_attach → fleet_adapter 接続、残り工数小）
中   Direct Python Connection（CI/headless テスト効果大）
中   フロー図ドキュメント3本
低   Batch API ROS Wrapper（100体以上 ROS2 ユースケース次第）
低   Handler Decomposition / cargo_relative_pose / Device Enhancements
保留 Multi-Floor Visualization（Rerun 移行まで）
```
