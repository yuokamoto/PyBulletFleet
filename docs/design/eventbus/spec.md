# EventBus — Design Spec

**Date:** 2026-04-08
**Status:** Draft
**Depends on:** Plugin Architecture Phase 1–2 (implemented), Controller ABC (implemented)
**Parent spec:** [plugin-architecture/spec.md](../plugin-architecture/spec.md) Phase 3

## Context

PyBulletFleetのcallbackシステムは現在 `register_callback(fn, frequency)` のみ。
ROS bridgeやUSO snapshotなど、**外部統合レイヤーが「Agent追加」「ステップ前後」「衝突」などのライフサイクルイベントに反応する必要がある**が、手動ポーリングやpost-hookループで対応している状態。

Plugin Architecture spec (Phase 3) で設計済みの EventBus を実装し、`core_simulation.py` にbus systemとして統合する。

## Decision

**Global EventBus + Lazy per-entity EventBus** の2層構成を採用する。

選定理由:
- **Global bus**: sim全体のイベント (pre/post step, pause/resume) を一括管理
- **Per-entity bus**: Agent/SimObjectサブクラスが `self.events.on()` で自身のイベントに直接hookできる
- OOPエンジン (UE5, Unity) の per-entity delegate/callback パターンに倣う
- Per-entity bus は **lazy 生成** — 使わない entity はメモリ/CPU ゼロコスト

collision モデルは **Enter/Exit** を採用 (UE5 `OnBeginOverlap`/`OnEndOverlap`、Unity `OnCollisionEnter`/`OnCollisionExit` に準拠)。全主要エンジンの調査で pre/post collision は採用例なし。

### Engine Survey Summary

| 設計判断 | 根拠 |
|---------|------|
| collision Enter/Exit (not pre/post) | UE5, Unity, Gazebo, Bevy: **全エンジン** が Enter/Exit or post-only。pre/post collision の採用例なし |
| spawn/remove に pre なし (post-only) | UE5 `OnActorSpawned`, Unity `OnDestroy`: 全エンジンが post only |
| Per-entity bus | UE5 (per-component delegate), Unity (per-MonoBehaviour): OOP エンジンの標準パターン |
| Global + Per-entity 両方 | UE5 は World level (`OnActorSpawned`) + Actor level (`OnDestroyed`) の2層 |
| pre/post は step のみ | Gazebo (`PreUpdate`/`PostUpdate`), Bevy (schedule `PreUpdate`/`PostUpdate`): step 以外に pre/post なし |

## Requirements

### EventBus クラス

- `on(event, handler, priority=0)` — ハンドラ登録。priority低い順に実行。同一priorityはFIFO
- `off(event, handler)` — ハンドラ解除
- `emit(event, **kwargs)` — イベント発火。ハンドラをpriority順に実行
- `clear(event=None)` — 全ハンドラまたは特定イベントのハンドラをクリア
- `has_handlers(event)` — ハンドラ存在チェック
- ハンドラ内の例外はログ出力して続行（他ハンドラに影響しない）

### 2層構成

| 層 | 所有者 | 用途 |
|---|---|---|
| **Global bus** | `MultiRobotSimulationCore.events` | sim全体のイベント。外部統合向け |
| **Per-entity bus** | `SimObject.events` (Agent 継承) | 個別 entity のイベント。サブクラス hook 向け |

**Per-entity bus の lazy 生成:**
```python
class SimObject:
    def __init__(self):
        self._events: Optional[EventBus] = None  # lazy

    @property
    def events(self) -> EventBus:
        if self._events is None:
            self._events = EventBus()
        return self._events

    def _has_entity_events(self) -> bool:
        return self._events is not None
```

**emit フロー:**
1. Global bus に emit (常に)
2. Per-entity bus に emit (bus が存在する entity のみ — `_has_entity_events()` で O(1) チェック)

### イベントタイプ (12種)

#### Simulation Step (Global only)

| Event | kwargs | Emit Location |
|-------|--------|---------------|
| `pre_step` | `dt: float, sim_time: float` | `step_once()` 冒頭 |
| `post_step` | `dt: float, sim_time: float` | `step_once()` 末尾 |

#### Entity Lifecycle (Global + Per-entity)

| Event | kwargs | Global | Per-entity |
|-------|--------|--------|------------|
| `agent_spawned` | `agent: Agent` | ✓ | `agent.events.emit("agent_spawned")` |
| `agent_removed` | `agent: Agent` | ✓ | `agent.events.emit("agent_removed")` |
| `object_spawned` | `obj: SimObject` | ✓ | `obj.events.emit("object_spawned")` |
| `object_removed` | `obj: SimObject` | ✓ | `obj.events.emit("object_removed")` |

#### Collision — Enter/Exit Model (Global + Per-entity)

| Event | kwargs (Global) | kwargs (Per-entity) | 条件 |
|-------|-----------------|---------------------|------|
| `collision_started` | `obj_a, obj_b` | `other: SimObject` | ペアが**今回初めて**検出 |
| `collision_ended` | `obj_a, obj_b` | `other: SimObject` | ペアが**前回にはあったが今回消失** |

実装: `_prev_collision_pairs: Set[FrozenSet[int]]` を保持。毎collision check後に差分計算:
- `started = current_pairs - prev_pairs`
- `ended = prev_pairs - current_pairs`

#### Action (Global + Per-entity)

| Event | kwargs (Global) | kwargs (Per-entity) |
|-------|-----------------|---------------------|
| `action_started` | `agent, action` | `action: Action` |
| `action_completed` | `agent, action, status` | `action, status` |

#### Simulation Control (Global only)

| Event | kwargs |
|-------|--------|
| `paused` | (なし) |
| `resumed` | (なし) |

### core_simulation.py 統合

- `__init__()` で `self.events = EventBus()` と `self._prev_collision_pairs: Set[FrozenSet[int]] = set()` を初期化
- `step_once()` の構造:
  ```
  emit("pre_step")
  → agent updates (既存)
  → callbacks (既存、register_callback() 後方互換)
  → physics step (既存)
  → collision detection (既存)
  → collision Enter/Exit 差分計算 + emit (新規)
  → monitor update (既存)
  emit("post_step")
  ```
- `add_object()` で `object_spawned` + (Agent なら) `agent_spawned` を emit (global + per-entity)
- `remove_object()` で (Agent なら) `agent_removed` + `object_removed` を emit (global + per-entity)
- `pause()` / `resume()` で `paused` / `resumed` を emit (global only)
- `reset_simulation()` で `events.clear()` は**呼ばない** — ハンドラは登録したコードの責任

### Profiling 統合

`step_once()` の既存 `_profiling_stats` に EventBus 専用フィールドを追加。`enable_time_profiling: true` で自動計測:

| Profiling field | 計測内容 |
|---|---|
| `events_pre_step` | `pre_step` emit + 全ハンドラ実行時間 |
| `events_post_step` | `post_step` emit + 全ハンドラ実行時間 |
| `events_collision` | Enter/Exit diff 計算 + `collision_started`/`ended` emit (global + per-entity) |

これらは `_print_profiling_summary()` に自動で表示される:
```
[PROFILING] Last 100 steps: agent_update=5.20ms (40%), events_pre_step=0.01ms (0.1%),
  events_collision=0.03ms (0.2%), events_post_step=0.01ms (0.1%), ...
```

`action_started`/`action_completed` は低頻度のため `agent_update` に含まれる。必要なら `record_profiling()` でユーザーが自身のハンドラ時間を計測可能。

### register_callback() 後方互換

- 既存の `register_callback()` / `unregister_callback()` / `_callbacks` リストは**変更しない**
- EventBus は `register_callback()` の**代替ではなく拡張**

### action.py 統合

- Action 開始時に `action_started` emit (global + per-entity)
- Action 完了/失敗/キャンセル時に `action_completed` emit (global + per-entity)
- Action は `agent._sim_core.events` で global bus にアクセス、`agent.events` で per-entity bus にアクセス

### __init__.py

- `EventBus` をpublic APIとしてexport

## Constraints

- Global `emit()` は 100台で < 10μs / call
- Per-entity `_has_entity_events()` check は < 50ns / entity (attribute None check)
- Per-entity bus 未使用 entity は `_events is None` — メモリゼロ
- `EventBus` はグローバルステートを持たない（`sim_core` インスタンスごと + entity ごと）
- テスト間の分離は `sim_core` インスタンスの再生成で自動的に確保
- collision Enter/Exit は `has_handlers` チェック付き — ハンドラなしならループスキップ
- `_profiling_stats` に `events_pre_step`, `events_post_step`, `events_collision` を追加
- 既存テストは**修正なし**で全て通ること

## Out of Scope

- `register_callback()` の EventBus ベースへの書き換え
- typed event classes（文字列キーで十分）
- async ハンドラ
- `collision_stay` (毎フレーム持続中の衝突通知) — 100台スケールでは高コスト
- `pose_changed` per-entity イベント — 毎ステップ100回 emit は使用頻度に対してコスト高

## Implementation Approach

1. `pybullet_fleet/events.py` を新規作成
2. `pybullet_fleet/sim_object.py` に lazy `events` プロパティ追加
3. `core_simulation.py` に Global EventBus + collision Enter/Exit 差分 + emit 統合
4. `action.py` に `action_started` / `action_completed` emit 追加
5. `pybullet_fleet/__init__.py` に export 追加
6. `tests/test_events.py` を新規作成
7. 既存テスト全て通ることを確認

## Success Criteria

- [ ] Global: `sim.events.on("pre_step", handler)` で毎ステップ呼ばれる
- [ ] Global: `sim.events.on("collision_started", handler)` で衝突開始時に呼ばれる
- [ ] Global: `sim.events.on("collision_ended", handler)` で衝突終了時に呼ばれる
- [ ] Global: `sim.events.on("agent_spawned", handler)` で Agent 追加時に呼ばれる
- [ ] Per-entity: `agent.events.on("collision_started", handler)` で自身の衝突時のみ呼ばれる
- [ ] Per-entity: `agent.events.on("action_completed", handler)` で自身の Action 完了時のみ呼ばれる
- [ ] Lazy: bus 未使用 entity は `_events is None`
- [ ] Enter/Exit: 持続衝突中は `collision_started` が再発火しない
- [ ] Enter/Exit: 衝突解消時に `collision_ended` が発火する
- [ ] `register_callback()` が既存通り動作する
- [ ] priority 順でハンドラが実行される
- [ ] ハンドラ内例外が他ハンドラに影響しない
- [ ] 全既存テスト修正なしで通る
- [ ] `make verify` 通過

## References

- [plugin-architecture/spec.md](../plugin-architecture/spec.md) — Phase 3 EventBus original design
- [plugin-architecture/agent.spec.md](../plugin-architecture/agent.spec.md) — EventBus code patterns
- [ros2-bridge/spec.md](../ros2-bridge/spec.md) — EventBus integration example
- UE5: `OnBeginOverlap`/`OnEndOverlap`, `OnActorSpawned`, `PreInitializeComponents`/`PostInitializeComponents`
- Unity: `OnCollisionEnter`/`OnCollisionExit`, `Awake`/`OnDestroy`, `FixedUpdate`/`LateUpdate`
- Gazebo Ignition: `ISystemPreUpdate`/`ISystemPostUpdate`, `EachNew<>`/`EachRemoved<>`
- Bevy ECS: `PreUpdate`/`PostUpdate` schedules, `Event<T>`, `Added<T>`/`RemovedComponents<T>`
