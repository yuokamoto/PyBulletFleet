# Open-RMF Integration v2 — Roadmap

## v1 Status (Completed)

Office demo fully operational with Gazebo-equivalent functionality:

| Gazebo Plugin | PyBulletFleet Replacement | Status |
|---------------|--------------------------|--------|
| Gazebo sim | `bridge_node` (PyBullet) | ✅ Done |
| `rmf_demos_fleet_adapter` | `fleet_adapter` (EasyFullControl) | ✅ Done |
| Door plugin | `door_adapter` (instant open/close) | ✅ Done |
| `TeleportDispenser` | `workcell_adapter` (instant SUCCESS) | ✅ Done |
| `TeleportIngestor` | `workcell_adapter` (instant SUCCESS) | ✅ Done |

Verified: patrol tasks, delivery tasks, door traversal, rmf-web dashboard.

---

## v2 Action Plan

### Phase 1 — Parallel (no dependencies)

#### 1. Multi Fleet Support [P0]

Launch で複数の fleet_adapter インスタンスを起動。rmf_demos と同方式。

**Approach:** launch file で fleet ごとに fleet_adapter Node を追加。
fleet_adapter.py のコード変更なし。bridge_node は EventBus で動的にハンドラを追加するため対応済み。

**Unlocks:** airport_terminal (5 fleets), hotel (3 fleets), clinic

**Status:** ✅ Done — `airport_terminal_pybullet.launch.py` で 4 fleet 対応確認済み

---

#### 2. bridge_node EventBus Refactor [P1]

現在の bridge_node を EventBus + `register_callback()` ベースの observer plugin に変更。

**Current:**
```
bridge_node (owns sim_core)
    ├── ROS timer → sim.step_once()      # bridge がメインループ制御
    ├── direct access: agent.get_pose()   # sim_core に直接アクセス
    └── EventBus: agent_spawned/removed   # 一部のみ EventBus
```

**Target:**
```
sim_core (owns main loop)
    ├── events.on(POST_STEP, bridge.publish_states)
    ├── events.on(AGENT_SPAWNED, bridge.register_handler)
    └── register_callback(bridge.publish_clock, frequency=100.0)

bridge_node = ROSBridgePlugin (read-only observer)
```

**Unlocks:** Pattern 3/4, Device plugins, ExternalAgent plugin — 全ての plugin アーキテクチャの基盤。

**Status:** ✅ Done — bridge_node は PRE_STEP/POST_STEP/AGENT_SPAWNED/AGENT_REMOVED イベントフック。sim.run_simulation() が独自スレッドでメインループ制御

---

#### 3. Custom Message Package [P2]

**第一候補: `rmf_internal_msgs` の既存メッセージを利用。**

`rmf_internal_msgs` リポジトリ (Apache-2.0) は RMF のコアメッセージ群の親リポジトリ。
"Developers extending RMF will need to use these messages" と明記されており、利用に問題なし。
rmf_demos Docker イメージに colcon ビルド済みで全て含まれている。

**既存メッセージのマッピング:**
| 用途 | 既存メッセージ (rmf_internal_msgs) |
|------|-----------------------------------|
| batch state publishing | `rmf_fleet_msgs/FleetState` + `RobotState` |
| delivery simulation | `rmf_dispenser_msgs/DispenserRequest` + `DispenserResult` |
| elevator 制御 | `rmf_lift_msgs/LiftRequest` + `LiftState` |

**Fallback:** 既存メッセージで不足する場合のみカスタムメッセージを定義。

**カスタムメッセージの配置方法 (未決定 — 実装時に選択):**

| 方法 | メリット | デメリット |
|------|---------|------------|
| **A. `pybullet_fleet_ros` を `ament_cmake_python` に変更** | 1パッケージで完結。ユーザーは `pybullet_fleet_ros` だけ依存すれば msg も使える | `setup.py` → `CMakeLists.txt` 移行。ビルド設定が複雑化 |
| **B. `pybullet_fleet_msgs` を `ament_cmake` で別パッケージ追加** | `pybullet_fleet_ros` は変更なし。ROS 2 慣習に沿う (`nav2_msgs` 等) | 2パッケージ管理。依存追加が必要 |

**Note:** 現在の `pybullet_fleet_ros` は `ament_python` のため `.msg/.srv/.action` 定義不可。
Item 10 (ROS 2 Action Server) で `ExecuteAction.action` が必要になるため、いずれかの方法を選択する必要がある。

**Location (方法 B の場合):** `ros2_bridge/pybullet_fleet_msgs/`

**Status:** � Partial — 方法 B 採用。`pybullet_fleet_msgs` スキャフォールド + `ExecuteAction.action` 定義済み。FleetStates.msg 等は未定義

---

#### 4. Camera Control GUI [P3]

大規模マップのデバッグ支援。

- Top-down orthographic ビュー切替 (キー)
- Shift+drag でカメラ平行移動 (`p.getMouseEvents()`)
- `p.resetDebugVisualizerCamera()` でプログラム制御
- (将来) フロア切替ボタン (マルチフロア対応時)

**Location:** `pybullet_fleet/camera_controller.py`

**Status:** ✅ Done — right-drag パン、+/- ズーム、o キー top-down ビュー実装済み

---

#### 5. sim_time Acceleration [Cross-cutting]

Launch arg で全ノード `use_sim_time=true` に切替。`target_rtf=5.0` で全体5倍速。

```
bridge_node: target_rtf=5.0 → /clock runs 5× faster
All RMF nodes: use_sim_time=true → task planning at 5× speed
fleet_adapter: -sim → robot state updates at 5× speed
dispatch_patrol --use_sim_time → task timestamps from /clock
```

| Component | Change | Status |
|-----------|--------|--------|
| `common.launch.xml` | Pass `use_sim_time:=true` | 🔲 Launch arg |
| `fleet_adapter` | Pass `-sim` flag | 🔲 Launch arg |
| `door_adapter` | Add `use_sim_time` parameter | 🔲 |
| `workcell_adapter` | Add `use_sim_time` parameter | 🔲 |
| `bridge_node` | Already publishes `/clock` | ✅ Done |
| `dispatch_*` tools | Already support `--use_sim_time` | ✅ Done (upstream) |

**Why wall-clock is the current default:**
`/clock` starts from epoch 0. If only some nodes use sim_time while others use
wall-clock, timestamps differ by decades → task planner drops requests.
All-or-nothing: either all nodes use sim_time, or none do.

**Status:** ✅ Done — `pybullet_common.launch.py` で `use_sim_time` を全ノード (door_adapter, workcell_adapter, bridge_node, fleet_adapter) に伝搬

---

### Phase 2 — Device / Controller / Demos

**依存関係:** Phase 1 の items 1, 2, 4, 5 は完了。item 3 は部分完了 (ExecuteAction.action のみ)。
Phase 2 は items 6, 7 が即着手可能。items 8, 10 は item 3 の FleetStates.msg 完成待ち。

---

#### 6. DoorDevice / ElevatorDevice — Agent サブクラス + URDF ジョイント [P1]

Door/Elevator を Agent のサブクラスとして実装。URDF の revolute/prismatic ジョイントで
開閉・昇降をシミュレーションし、既存の JointAction / ジョイント制御をフル活用。

**Design decision:** Device を Agent サブクラスにする理由:
- JointAction のライフサイクル (NOT_STARTED → IN_PROGRESS → COMPLETED) がデバイス状態遷移に直結
- Agent のジョイント interpolation が自動でアニメーション処理
- fleet_adapter は fleet config YAML の `known_robots` のみ管理するため、
  Device を `_agents` に追加しても fleet_adapter への影響はゼロ

**基盤変更: SimObject.update() + _needs_update:**
```python
class SimObject:
    _needs_update: bool = False  # 静的オブジェクト → スキップ
    def update(self, dt: float) -> bool:
        return False  # デフォルト no-op

class Agent(SimObject):
    _needs_update = True  # 毎ステップ更新
```
`step_once()` を `_sim_objects` 統一ループに変更。`_needs_update=True` のオブジェクトのみ `update(dt)` 呼出。

**DoorDevice(Agent):**
```python
class DoorDevice(Agent):
    """URDF revolute/prismatic ジョイントで開閉。JointAction で制御。"""
    def request_open(self):
        self.add_action(JointAction(target_joint_positions=self._open_positions))
    def request_close(self):
        self.add_action(JointAction(target_joint_positions=self._close_positions))
    @property
    def door_state(self) -> str:  # "closed" | "opening" | "open" | "closing"
        # JointAction 状態 + ジョイント位置から導出
```

**ElevatorDevice(Agent):**
```python
class ElevatorDevice(Agent):
    """Prismatic Z ジョイントでフロア間移動。auto_attach で乗客自動追従。"""
    floors: dict[str, float]  # {"L1": 0.0, "L2": 8.0, "L3": 16.0}
    auto_attach: bool = True  # プラットフォーム上オブジェクトを自動 attach/detach
    def request_floor(self, floor_name: str):
        if self.auto_attach: self._attach_platform_objects()
        self.add_action(JointAction(target={self._joint_name: target_z}))
```

author_attach 方式: ハイブリッド (auto_attach=True でプラットフォーム上自動検出、手動 attach も可能)

**WorkcellDevice(SimObject):** 静的位置マーカー。`_needs_update = False`。PickAction の target_position として使用。

**Location:**
```
pybullet_fleet/devices/
├── __init__.py
├── door.py            # DoorDevice(Agent)
├── elevator.py        # ElevatorDevice(Agent)
└── workcell.py        # WorkcellDevice(SimObject)
robots/
├── door_hinge.urdf    # revolute joint ヒンジドア
├── door_slide.urdf    # prismatic joint スライドドア
└── elevator.urdf      # prismatic Z joint エレベータ

ros2_bridge/pybullet_fleet_ros/
├── door_adapter.py        # 変更: DoorDevice.request_open/close 連携
├── lift_adapter.py        # 新規: LiftRequest → ElevatorDevice.request_floor
└── workcell_adapter.py    # 変更: WorkcellDevice 位置参照
```

**Unlocks:** hotel (2 lifts, 3 floors), clinic (2 active lifts, 2 floors)

**Status:** 🔲 Not started

---

#### 7. Controller チェーン + ExternalAgent [P1]

Agent に複数 Controller のスタックを導入。高レベル Controller (PatrolController 等) が
`set_goal_pose()` を設定し、低レベル Controller (KinematicController) が実移動を処理。

専用の ExternalAgentPlugin は不要 — Controller チェーンが YAML で宣言的に設定されるため。

**Controller chain:**
```python
class Agent:
    _controllers: list[Controller]  # [KinematicCtrl, PatrolCtrl, ...]
    # 実行順: reversed() — 高レベル → 低レベル
    # PatrolController.compute() → set_goal_pose(next_wp)
    # KinematicController.compute() → 実移動

    def add_controller(self, ctrl):  # チェーン末尾に追加
    def set_controller(self, ctrl):  # 後方互換: 基底 Controller 差替
```

**PatrolController:**
ウェイポイント巡回。`set_goal_pose()` を呼ぶだけ。loop/wait_time 対応。

**RandomWalkController:**
ランダム近傍歩行。Gazebo `crowd_simulator` の**簡易版**として使用。
rmf_demos の crowd_simulator は nav graph 上のゴールセットベースだが、
Phase 2 では radius ベースのランダム歩行で代替する。

**YAML 設定 (external_agents セクション不要):**
```yaml
robots:
  - name: caddy_0
    type: agent
    motion_mode: differential
    controllers:
      - type: patrol
        waypoints: [[20,5,0.1], [30,5,0.1], [30,15,0.1]]
        wait_time: 3.0
        loop: true
  - name: pedestrian_0
    type: agent
    motion_mode: omnidirectional
    controllers:
      - type: random_walk
        radius: 5.0
```

**ROS handler:** `ExternalAgentHandler` — Odometry + TF のみ publish。
NavigateToPose action server は提供しない。
`handler_map` で `"caddy_*"`, `"pedestrian_*"` 等にマッチ。

**Location:**
```
pybullet_fleet/controllers/
├── __init__.py
├── patrol_controller.py
└── random_walk_controller.py
ros2_bridge/pybullet_fleet_ros/
└── external_agent_handler.py
```

**Unlocks:** airport caddy fleet, 歩行者シミュレーション (簡易版)

**Status:** 🔲 Not started

---

#### 11. Demo Launch Files & Configs [P1]

Phase 2 で追加される機能を使い、各 rmf_demos ワールドの launch file + bridge config を作成。
各デモが `docker compose` で起動・動作確認できる状態にする。

**デモ別スコープ:**

| Demo | Launch file | Bridge config | Fleet configs | Nav graphs | 追加要件 |
|------|-------------|---------------|---------------|------------|----------|
| **office** | ✅ 既存 | ✅ 既存 | ✅ 既存 | ✅ 既存 | — |
| **battle_royale** | 新規 | 新規 | 新規 (1 fleet × 4 robots) | rmf_demos から | config のみ |
| **airport_terminal** | ✅ 既存 | 更新 (Door + caddy) | 更新 (5 fleets) | rmf_demos から | DoorDevice ×5, caddy (PatrolController) |
| **hotel** | 新規 | 新規 | 新規 (3 fleets) | rmf_demos から | DoorDevice ×12, ElevatorDevice ×2 |
| **clinic** | 新規 | 新規 | 新規 (2 fleets) | rmf_demos から | DoorDevice ~10, ElevatorDevice ×2 |

**成果物:**
```
ros2_bridge/pybullet_fleet_ros/
├── config/
│   ├── bridge_office.yaml          # ✅ 既存
│   ├── bridge_battle_royale.yaml   # 新規
│   ├── bridge_airport.yaml         # 更新: DoorDevice + caddy
│   ├── bridge_hotel.yaml           # 新規: 3 floors, 2 lifts, 12 doors
│   └── bridge_clinic.yaml          # 新規: 2 floors, 2 lifts, ~10 doors
├── launch/
│   ├── office_pybullet.launch.py           # ✅ 既存
│   ├── battle_royale_pybullet.launch.py    # 新規
│   ├── airport_terminal_pybullet.launch.py # ✅ 既存 (更新)
│   ├── hotel_pybullet.launch.py            # 新規
│   └── clinic_pybullet.launch.py           # 新規
docker/
└── .env                                    # DEMO_WORLD で切替
```

**docker compose 起動:**
```bash
# office (既存)
DEMO_WORLD=office docker compose -f docker-compose.rmf.yaml up

# hotel (新規)
DEMO_WORLD=hotel docker compose -f docker-compose.rmf.yaml up

# battle_royale (新規)
DEMO_WORLD=battle_royale docker compose -f docker-compose.rmf.yaml up
```

**Note:**
- Multi-floor visualization (item 9) は Phase 3。hotel/clinic は機能的に動作するが
  3 フロアが重なって表示される。フロア透明化切替は後回し。
- crowd_simulator は RandomWalkController で簡易代替。nav graph ベースのパス計画は将来対応。

**Status:** 🔲 Not started

---

#### 8. Batch API ROS Wrapper (Pattern 2) [P2]

`AgentManager` の既存 batch API に numpy flat array 一括取得を追加し、ROS ラッパーを実装。

```
RMF (ROS 2) ↔ fleet_adapter ↔ 1×/fleet/states + 1×/fleet/navigate ↔ bridge_node ↔ sim_core
                 (rclpy)                 ROS 2 O(1)                     (rclpy)      (Python)
```

**Changes:**
- **pybullet_fleet:** `AgentManager.get_fleet_states()` → numpy flat array
- **bridge_node:** Replace N `RobotHandler` with single `FleetROSInterface`
  - `/fleet/states` publisher (FleetStates.msg)
  - `/fleet/navigate` service
- **fleet_adapter:** Replace N `RobotClientAPI` with single `FleetClientAPI`
  - Subscribe `/fleet/states`, dispatch to per-robot `RobotAdapter.update()`
- **RobotAdapter remains** — EasyFullControl requires per-robot callbacks

**Benefits:** 100 robots: 200 endpoints → 2-3 endpoints

**Depends on:** Item 3 (FleetStates.msg)

**Status:** 🔲 Not started

---

### Phase 3 — After Phase 2

#### 12. Cleaning Simulation [P3 — Low Priority]

fleet_adapter の `execute_action("clean")` でロボットが指定ゾーンを巡回するシミュレーション。
現在は即 `finished()` でスキップしている (rmf_demos も fleet_manager にパスを返すだけで実際の掃除はしない)。

**Scope:**
- bridge_node に `/{robot}/start_activity` ROS 2 service を追加
- fleet_adapter の `execute_action("clean")` → `api.start_activity("clean", zone)` 呼び出し
- bridge_node 側でゾーン名から巡回パスを生成 (bounding box の周回 or zig-zag)
- `override_schedule` でRMFに実際の走行パスを通知

**Priority:** 低。rmf_demos でも cleaning は `airport_terminal` デモでのみ使用。
現状の即完了でデモ目的には十分。

**Status:** 🔲 Not started

---

#### 9. Multi-Floor Visualization [P3 — Low Priority]

フロア表示 ON/OFF で視認性を確保。Deployment Patterns (2, 3, 4) が優先。

- `p.changeVisualShape(rgbaColor=[..., alpha=0])` で非表示フロアを透明化
- キーボードでフロア切替 (1, 2, 3 キー等)
- 論理座標の Z は正しく扱う (elevator teleport と整合)

**Depends on:** Item 6 (Elevator)

**Status:** 🔲 Not started (low priority — Deployment Patterns でカバーされていれば後回し)

---

## Deployment Patterns (fleet_adapter ↔ sim_core)

### Overview

| # | Name | fleet_adapter↔sim | bridge_node | ROS 2 endpoints | Primary Use |
|---|---|---|---|---|---|
| 1 | Per-Robot ROS 2 | ROS topics (per-robot) | ✅ | O(N) | **Current (v1)** |
| 2 | Batch ROS 2 | ROS topics (batch) | ✅ | O(1) | **Production. Scalable** |
| 3 | Plugin Only | Direct Python | ❌ | 0 | CI benchmarks, headless |
| 4 | Plugin + Bridge | Direct Python | ✅ (observer) | 0 (adapter↔sim) | **Dev/Debug. Primary** |

**Primary targets: Patterns 2 and 4.**

### Pattern 1: Per-Robot ROS 2 (Current — v1)

```
RMF (ROS 2) ↔ fleet_adapter ↔ N×odom + N×NavigateToPose ↔ bridge_node ↔ sim_core
```

- `RobotClientAPI` × N + `RobotHandler` × N
- 10 robots = 20 endpoints; 100 robots = 200 endpoints
- **Status:** ✅ Implemented

### Pattern 2: Batch ROS 2 ⭐

```
RMF (ROS 2) ↔ fleet_adapter ↔ 1×/fleet/states + 1×/fleet/navigate ↔ bridge_node ↔ sim_core
```

- Full ROS 2 ecosystem (rviz2, ros2 cli, Nav2)
- O(1) endpoints regardless of fleet size
- **Implemented by:** Action Item 8
- **Status:** 🔲 Not started

### Pattern 3: Plugin Only (Headless)

```
RMF (ROS 2) ↔ fleet_adapter (plugin) ↔ direct Python ↔ sim_core
```

- Zero ROS 2 between adapter ↔ sim
- No `/clock`, `/odom`, `/tf` → no rviz2
- Best for CI benchmarks with 100+ robots

```python
class RMFPlugin:
    def __init__(self, sim_core, fleet_config_yaml, nav_graph):
        self.sim = sim_core
        self.adapter = Adapter.make("fleet_adapter")
        self.fleet_handle = self.adapter.add_easy_fleet(fleet_config)

    def update(self, sim_core, dt):
        for agent in sim_core.agents:
            state = rmf_easy.RobotState("L1", [pos[0], pos[1], yaw], 1.0)
            self.update_handles[agent.name].update(state, ...)

    def navigate(self, agent_name, destination):
        agent = self.sim.get_agent(agent_name)
        agent.add_action(MoveAction(path=Path.from_positions([destination])))
```

- **Depends on:** Action Item 2 (EventBus refactor)
- **Status:** 🔲 Not started

### Pattern 4: Plugin + Bridge (Dev/Debug) ⭐

```
RMF schedule/dispatcher (ROS 2)
        ↑↓
  fleet_adapter (plugin)  ←── direct Python ──→  sim_core
                                                     ↑
                                               bridge_node (read-only observer)
                                                     ↓
                                          /clock, /odom, /tf, services
                                                     ↓
                                              rviz2, ros2 topic echo
```

- Zero-overhead adapter↔sim + full observability from bridge
- bridge_node is optional — start/stop without affecting RMF
- `/clock` enables sim_time acceleration
- **Depends on:** Action Item 2 (EventBus refactor), Pattern 3
- **Status:** 🔲 Not started

---

## RMF Delivery — Two Mechanisms

RMF (rmf_demos) には2つの配送メカニズムがあり、PyBulletFleet は両方をサポートする。
RMF 関連ドキュメント作成時にはこの区別を明記すること。

### Standard Delivery（インフラ主導 — Workcell 方式）

**dispatch:** `dispatch_delivery.py` → RMF 組み込み `delivery` タスクカテゴリ

```
ロボット → dispenser 位置に到着
         → RMF タスクエンジンが DispenserRequest を自動発行
         → WorkcellHandler が受信 → PickAction でアイテムをロボットに取り付け
         → ロボット → ingestor 位置に到着
         → RMF タスクエンジンが IngestorRequest を自動発行
         → WorkcellHandler が受信 → DropAction でアイテムをロボットから取り外し
```

**特徴:**
- ロボットは搬送プラットフォーム（受動的）— 移動するだけ
- アイテムの載せ降ろしは **ステーション側の設備 (Workcell)** が行う
- Attach パラメータ（取り付け位置 `spawn_offset` 等）は **WorkcellConfig** が保持
- 現実例: コンベア付き AMR — ステーションのコンベアが荷物を載せ替える
- Gazebo 対応: `TeleportDispenser` / `TeleportIngestor`
- PyBulletFleet 対応: `WorkcellHandler` + `WorkcellPlugin`

### Cart Delivery（ロボット主導 — toggle_attach 方式）

**dispatch:** `dispatch_cart_delivery.py` → `compose` タスク (PerformAction phases)

```
ロボット → カート位置に到着
         → execute_action("delivery_pickup") → toggle_attach(True) → カートに結合
         → ロボット → 目的地に到着
         → execute_action("delivery_dropoff") → toggle_attach(False) → カートを切り離し
```

**特徴:**
- ロボットが **自分でカート/棚を掴んで運ぶ**（能動的）
- 外部設備 (Workcell) は不要
- Attach パラメータ（結合位置等）は **ロボット側** が保持
- 現実例: リフト式 AMR (Amazon Kiva 的) — ロボットが棚ごと持ち上げて運ぶ
- fleet_adapter の `execute_action()` が直接 `toggle_attach` を呼ぶ

### 対比表

| | Standard (Workcell) | Cart (toggle_attach) |
|---|---|---|
| dispatch script | `dispatch_delivery.py` | `dispatch_cart_delivery.py` |
| RMF task category | 組み込み `delivery` | `compose` (PerformAction) |
| 荷物操作の主体 | Workcell (インフラ側) | ロボット自身 |
| attach 位置の決定 | `WorkcellConfig.spawn_offset` | ロボット URDF / attach point |
| fleet_adapter の関与 | なし (RMF + WorkcellHandler) | `execute_action` → `toggle_attach` |
| Gazebo 対応 | TeleportDispenser/Ingestor | fleet_adapter `toggle_teleop` |
| PyBulletFleet 対応 | WorkcellHandler + WorkcellPlugin | fleet_adapter + RobotClientAPI |

### Documentation TODO

RMF ドキュメント作成時に以下のフロー図（シーケンス図）を含めること:

1. **Standard Delivery フロー図** — dispatch_delivery → RMF Engine → fleet_adapter (navigate) →
   DispenserRequest → WorkcellHandler → PickAction → DispenserResult →
   navigate → IngestorRequest → DropAction → IngestorResult → task done
2. **Cart Delivery フロー図** — dispatch_cart_delivery → compose (4 Phase) →
   navigate → execute_action("delivery_pickup") → toggle_attach(True) →
   navigate → execute_action("delivery_dropoff") → toggle_attach(False) → task done
3. **通信手段の違い** — Standard は ROS 2 トピック (`/dispenser_requests`, `/dispenser_results`)
   による非同期メッセージング。Cart は fleet_adapter コールバック → ROS 2 サービス。
   RMF Engine と WorkcellHandler は fleet_adapter を介さず直接 pub/sub で通信する点を明記。

---

## PickAction / DropAction Integration

### Current State
- `workcell_adapter` returns instant SUCCESS (no actual item simulation)
- `PickAction` / `DropAction` exist in `pybullet_fleet` but are not wired to RMF
- `PickAction` は既に `target_position` + `search_radius` による最近傍 pickable オブジェクト探索を実装済み

### Goal
When a delivery task reaches a dispenser/ingestor, actually pick up / drop off
a SimObject in the simulation.

### Approach: Gazebo 方式 (既存 PickAction の近傍探索を利用)

Gazebo の `TeleportDispenser` と同じアプローチ: RMF メッセージにオブジェクト名は含まれない。
dispenser の位置から最近傍の pickable SimObject を自動選択する。

`PickAction` の既存機能 (`target_position` + `search_radius`) をそのまま使うため、
カスタム srv もオブジェクト名の config マッピングも不要。`rmf_internal_msgs` だけで完結する。

**適用範囲:** Standard Delivery のみ。Cart Delivery は PickAction/DropAction ではなく
toggle_attach による物理的結合/切り離しで完結する。

**Pattern 2 (Batch ROS 2):**
```
RMF DispenserRequest(target_guid="coke_dispenser")
    → workcell_adapter: dispenser の位置を取得
    → bridge_node: agent.add_action(PickAction(
          target_position=dispenser_pos,
          search_radius=1.0,
      ))
    → on complete → workcell_adapter publishes DispenserResult(SUCCESS)
```

**Patterns 3 & 4 (Plugin):**
```
RMF DispenserRequest → RMFPlugin.on_dispenser_request()
    → dispenser_pos = self.dispensers[request.target_guid].position
    → agent.add_action(PickAction(
          target_position=dispenser_pos,
          search_radius=1.0,
      ))
    → on complete → publish DispenserResult(SUCCESS)
```

### Why Not Custom srv?

Gazebo `TeleportDispenser` もオブジェクト名を指定せず空間近傍探索で識別する。
`PickAction` が同じ機能を既に持つため、`pybullet_fleet_msgs` パッケージは不要。
将来的に明示的なオブジェクト指定が必要になった場合は `target_object_id` パラメータで対応可能。

---

## Attach/Detach Interfaces — Immediate vs Action-Based

**Status:** ✅ Implemented

### Design: Two Layers

| Layer | Interface | 実行方式 | 用途 |
|---|---|---|---|
| **Immediate** | `toggle_attach` (SetBool) | `agent.attach_object()` 直接呼び出し | RMF cart delivery |
| **Immediate** | `attach_object` (AttachObject.srv) | `agent.attach_object()` 直接呼び出し | 外部連携・スクリプト |
| **Action-based** | `ExecuteAction.action` | PickAction/DropAction を queue に投入 | approach 込みの完全フロー |

**Immediate layer** は action queue をバイパスし、`agent.attach_object()` / `agent.detach_object()`
を直接呼ぶ。キューの状態に関係なく即時実行される。teleport + constraint 設定のみ。

**Action-based layer** は PickAction/DropAction を queue に投入。approach → teleport → attach →
retreat の完全フェーズを実行。先行アクション完了後に開始される。

### Immediate: toggle_attach vs attach_object

| | `toggle_attach` (SetBool) | `attach_object` (AttachObject.srv) |
|---|---|---|
| 用途 | RMF cart delivery 専用 | 汎用・外部連携 |
| 対象選択 | 最近傍（暗黙的） | `object_name` で明示指定可 |
| parent link | base_link 固定 | `parent_link` で指定可 |
| offset | keep_world_pose (現在位置保持) | `geometry_msgs/Pose` で指定可 |
| search radius | 1.0m 固定 | `search_radius` で指定可 |
| 結果 | success/message | success/message + attached_object_name |

### Files

- `pybullet_fleet_msgs/srv/AttachObject.srv` — サービス定義
- `robot_handler.py` — `_toggle_attach_cb`, `_attach_object_cb`, `_find_nearest_pickable`
- `robot_client_api.py` — `toggle_attach()`, `attach_object()`

---

## ROS 2 Action Server — Direct Action Control

RMF を介さず、ROS 2 経由で PyBulletFleet のアクション (Move, Pick, Drop, Wait, Joint) を直接操作する。

### Motivation
- RMF は fleet レベルのタスク管理向け。JointAction やアーム制御はカバー外
- テスト・デバッグ時に `ros2 action send_goal` で即座にアクション投入したい
- 外部システム (WMS, MES, カスタム UI) からロボットを直接操作するユースケース

### Design: Unified ROS 2 Action

```
# ExecuteAction.action (pybullet_fleet_ros or pybullet_fleet_msgs)

# Goal
string robot_name
string action_type          # "move" | "pick" | "drop" | "wait" | "joint"
string action_params_json   # JSON でパラメータを柔軟に渡す
---
# Result
bool success
string message
---
# Feedback
string status               # "NOT_STARTED" | "IN_PROGRESS" | "COMPLETED" | "FAILED"
float32 progress            # 0.0 ~ 1.0
```

### bridge_node Implementation

```python
class ActionBridge:
    """ROS 2 Action → PyBulletFleet Action 変換"""

    ACTION_MAP = {
        "move": MoveAction,
        "pick": PickAction,
        "drop": DropAction,
        "wait": WaitAction,
        "joint": JointAction,
    }

    def execute_callback(self, goal_handle):
        params = json.loads(goal_handle.request.action_params_json)
        action_cls = self.ACTION_MAP[goal_handle.request.action_type]
        action = action_cls(**params)

        agent = sim_core.get_agent(goal_handle.request.robot_name)
        agent.add_action(action)

        # フィードバックループ
        while action.status == ActionStatus.IN_PROGRESS:
            feedback.status = action.status.name
            feedback.progress = action.progress
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        result.success = (action.status == ActionStatus.COMPLETED)
        return result
```

### Usage Examples

```bash
# MoveAction
ros2 action send_goal /execute_action ExecuteAction \
  "{robot_name: 'robot0', action_type: 'move', \
    action_params_json: '{\"path\": [[5,5,0]]}' }"

# JointAction (アーム制御 — RMF ではカバー不可)
ros2 action send_goal /execute_action ExecuteAction \
  "{robot_name: 'ur5e_0', action_type: 'joint', \
    action_params_json: '{\"joint_targets\": [0, -1.57, 1.57, 0, 0, 0]}' }"

# WaitAction
ros2 action send_goal /execute_action ExecuteAction \
  "{robot_name: 'robot0', action_type: 'wait', \
    action_params_json: '{\"duration\": 3.0}' }"
```

### Benefits
- JSON パラメータで全アクションを統一 — 新アクション追加時に msg 定義変更不要
- Cancel / Feedback は ROS 2 Action の仕組みで自動対応
- RMF タスクとの共存可能 — 同じロボットに RMF タスクと直接操作を混在できる
- `ros2 action send_goal` でコマンドラインからテスト可能

### Depends on
- Item 3 (Custom msg パッケージ選択) — `.action` ファイルの配置先
- Item 2 (EventBus refactor) — bridge_node が sim_core にアクセスする方法

### Status: 🔲 Not started

---

## Package Separation Plan

When API stabilizes (v1.0), split into:

```
# PyPI / apt: generic ROS 2 bridge library
pybullet_fleet_ros/
├── bridge_node.py
├── robot_handler.py
├── fleet_adapter.py
├── rmf_plugin.py          # Pattern 3 & 4
├── door_adapter.py
├── lift_adapter.py
├── workcell_adapter.py
├── robot_client_api.py
├── conversions.py
└── sim_services.py

# PyPI / apt: custom ROS 2 message definitions
# 配置方法は Item 3 で選択 (方法 A: pybullet_fleet_ros に統合 / 方法 B: 別パッケージ)
pybullet_fleet_msgs/  # 方法 B の場合
├── msg/
│   └── FleetStates.msg
├── srv/
│   └── (reserved)
└── action/
    └── ExecuteAction.action   # Item 10: Direct action control

# Separate repo: demo-specific configs and launch files
pybullet_fleet_demos/
├── config/
│   ├── office/
│   ├── airport_terminal/
│   ├── hotel/
│   └── campus/
├── launch/
├── nav_graphs/
└── docker/
```

- `pybullet_fleet_ros` → `apt install ros-jazzy-pybullet-fleet-ros`
- `pybullet_fleet_demos` → user forks and customizes
- Current co-location is for development velocity

---

## Demo Compatibility Matrix

| Demo | Floors | Lifts | Doors (RMF) | Workcells | Fleets | Robots | External | Crowd sim | Required Items | Phase 2 後 |
|------|--------|-------|-------------|-----------|--------|--------|----------|-----------|----------------|------------|
| **office** | 1 | 0 | 3 | 4 | 1 | 2 | 0 | No | ✅ v1 | ✅ 完全対応 |
| **battle_royale** | 1 | 0 | 0 | 0 | 1 | 4 | 0 | No | config only | ✅ 完全対応 |
| **campus** | 1 | 0 | 0 | 0 | 3 | ~6 | 0 | No | config only | ✅ 完全対応 |
| **airport_terminal** | 1 | 0 | 5 | 0 | 5 | 10+ | caddy | Yes (7 goal sets) | **6, 7, 11** | 🟡 ~85% (crowd sim 簡易版) |
| **hotel** | 3 | 2 | 12 | 0 | 3 | 4 | 0 | No | **6, 11** | 🟡 ~95% (可視化のみ) |
| **clinic** | 2 | 2 active + 2 cosmetic | ~10 | 0 | 2 | 4 | 0 | No | **6, 11** | 🟡 ~95% (可視化のみ) |

**Notes:**
- hotel/clinic は Phase 2 後に**機能的には完全動作** (Door, Elevator, Multi-fleet)。Multi-floor visualization (item 9) がないとフロアが重なって見える
- airport_terminal の crowd sim は RandomWalkController で簡易代替。rmf_demos の `crowd_simulator` (nav graph goal sets) との完全互換は将来対応
- clinic の Door 76 枚中 ~10 枚のみ RMF 制御 (`plugin: normal`)。残りは装飾 (cosmetic)

---

## Implementation Summary

```
Phase 1: [1, 2, 3, 4, 5] — ✅ 完了 (item 3 は部分完了)
    1. Multi Fleet (launch)            ✅ Done
    2. bridge_node EventBus refactor   ✅ Done
    3. Custom message package          🟡 Partial (ExecuteAction.action のみ)
    4. Camera control GUI              ✅ Done
    5. sim_time acceleration           ✅ Done

Phase 2: [6, 7, 11, 8, 10] — Device / Controller / Demos
    6. DoorDevice / ElevatorDevice     ← Agent subclass + URDF joint
    7. Controller chain + ExternalAgent ← PatrolCtrl / RandomWalkCtrl
   11. Demo launch files & configs     ← battle_royale, hotel, clinic, airport 更新
    8. Batch API ROS wrapper           ← depends on 3 (FleetStates.msg)
   10. ROS 2 Action Server            ← depends on 3 (ExecuteAction build)

Phase 3: [9, 12] — Phase 2 の 6 完了後
    9. Multi-floor visualization       ← depends on 6 (low priority)
   12. Cleaning simulation             ← low priority, 即完了で十分
```
