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

**Status:** 🔲 Not started

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

**Status:** 🔲 Not started

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

**Status:** 🔲 Not started

---

#### 4. Camera Control GUI [P3]

大規模マップのデバッグ支援。

- Top-down orthographic ビュー切替 (キー)
- Shift+drag でカメラ平行移動 (`p.getMouseEvents()`)
- `p.resetDebugVisualizerCamera()` でプログラム制御
- (将来) フロア切替ボタン (マルチフロア対応時)

**Location:** `pybullet_fleet/camera_controller.py`

**Status:** 🔲 Not started

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

**Status:** 🔲 Not started

---

### Phase 2 — After Phase 1 items 2, 3

#### 6. Device Base Class + Door / Elevator [P1]

ROS 非依存の Device クラスを PyBulletFleet コアに実装。現 door_adapter のロジックを分離。

**Location:**
```
pybullet_fleet/devices/
├── __init__.py
├── device.py          # ExternalDevice base class
├── door.py            # Door (open/closed/moving states)
├── elevator.py        # Elevator (floor management, teleport)
└── workcell.py        # Workcell (dispenser/ingestor)

ros2_bridge/pybullet_fleet_ros/
├── door_adapter.py        # ROS wrapper: /door_requests → Door → /door_states
├── lift_adapter.py        # ROS wrapper: /lift_requests → Elevator → /lift_states
└── workcell_adapter.py    # ROS wrapper: requests → Workcell → results
```

**Elevator design:**
```python
class Elevator:
    floors: Dict[str, float]  # {"L1": 0.0, "L2": 8.0, "L3": 16.0}
    current_floor: str
    travel_speed: float = 2.0  # m/s
    mode: str = "velocity"     # "velocity" | "instant"

    def request_floor(self, floor_name: str):
        if self.mode == "velocity":
            # 毎ステップ travel_speed × dt ずつ Z を teleport
            # 到着判定: abs(current_z - target_z) < epsilon
            # GUI で移動が見える
        else:  # "instant"
            # 一発 teleport (CI / 高速シミュレーション向け)
        # 内部の attach 済み物体は attachment 伝搬で自動追従
```

方式:
- **`velocity` (デフォルト):** `travel_speed` で毎ステップ Z を更新。`set_pose()` + attachment 伝搬で実現。GUI でエレベータの移動が視覚的に確認できる。所要時間は `distance / travel_speed` から自動計算。
- **`instant`:** 一発 teleport。CI や高速シミュレーション用。

**Unlocks:** hotel (2 lifts, 3 floors), clinic, マルチフロア対応

**Status:** 🔲 Not started

---

#### 7. ExternalAgent Plugin [P1]

RMF に制御されないが traffic schedule に報告される Agent を実装。

**Gazebo equivalent:** `crowd_simulator` plugin + `robot_state_aggregator` + `read_only` fleet adapter

**Note:** Agent の subclass **ではなく**、既存 Agent を操作するコールバック・プラグイン。
Agent を subclass 化するとファクトリ (`from_params`, `from_yaml`) やシリアライゼーションに影響するため、
振る舞いの差異は plugin 層で吸収する。

**Design:**
```python
class ExternalAgentPlugin:
    """RMF 非管理ロボット: 固定ルート巡回、ランダム移動、外部制御に対応。

    既存の Agent インスタンスをそのまま使い、MoveAction を投入して動かす。
    Agent の subclass ではなくコントローラ層として機能する。
    """

    def __init__(self, sim_core, agents, mode="patrol"):
        self.mode = mode  # "static" | "patrol" | "random" | "external"
        self.agents = agents  # 通常の Agent インスタンスのリスト
        sim_core.register_callback(self.update, frequency=10.0)

    def update(self, sim_core, dt):
        for agent in self.agents:
            if self.mode == "patrol":
                # ウェイポイントリスト → MoveAction で巡回
                # 到着 → 次の MoveAction を投入するループ
            elif self.mode == "random":
                # ランダムな近傍目標に MoveAction を投入
            elif self.mode == "external":
                # REST/gRPC/ROS topic で位置を受信
            elif self.mode == "static":
                pass
```

**Location:** `pybullet_fleet/plugins/external_agent.py`

**ROS wrapper:** `/robot_state` を publish → `robot_state_aggregator` → `read_only` adapter → traffic schedule

**Use cases:**
- 空港 caddy (patrol: 固定ルート巡回)
- 歩行者シミュレーション (random: ランダム歩行)
- テスト障害物 (static: 固定位置)
- 外部制御ロボット (external: REST API で位置受信)

**Unlocks:** airport caddy fleet, 交通管理テスト

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

| Demo | Floors | Lifts | Doors | Workcells | Fleets | External Agents | Required Items |
|------|--------|-------|-------|-----------|--------|-----------------|----------------|
| **office** | 1 | 0 | 3 | 4 | 1 | 0 | ✅ v1 complete |
| **battle_royale** | 1 | 0 | 0 | 0 | 1 (×4) | 0 | config only |
| **campus** | 1 | 0 | 0 | 0 | 1 (×3) | 0 | config only |
| **airport_terminal** | 1 | 0 | 5 | 2 | 5 (×10) | 1 (caddy) | **1, 7** |
| **hotel** | 3 | 2 | 3 | 0 | 3 | 0 | **1, 6, 9** |
| **clinic** | 2 | 1 | ? | ? | ? | 0 | **1, 6, 9** |

---

## Implementation Summary

```
Phase 1: [1, 2, 3, 4, 5] — 並行着手可能、依存なし
    1. Multi Fleet (launch)
    2. bridge_node EventBus refactor
    3. Custom message package
    4. Camera control GUI
    5. sim_time acceleration

Phase 2: [6, 7, 8, 10] — Phase 1 の 2, 3 完了後
    6. Device base + Door/Elevator     ← depends on 2
    7. ExternalAgent plugin            ← depends on 2
    8. Batch API ROS wrapper           ← depends on 3
   10. ROS 2 Action Server            ← depends on 2, 3

Phase 3: [9] — Phase 2 の 6 完了後
    9. Multi-floor visualization       ← depends on 6 (low priority)
```
