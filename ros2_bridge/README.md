# ROS 2 Bridge & Open-RMF Integration

<table>
<tr>
<td align="center"><b>Office (Open-RMF)</b><br>
<video src="https://raw.githubusercontent.com/yuokamoto/PyBulletFleet/feat/ros2-bridge-v2/ros2_bridge/videos/office.mp4" width="380" controls muted></video></td>
<td align="center"><b>Hotel (Open-RMF)</b><br>
<video src="https://raw.githubusercontent.com/yuokamoto/PyBulletFleet/feat/ros2-bridge-v2/ros2_bridge/videos/hotel.mp4" width="380" controls muted></video></td>
</tr>
<tr>
<td align="center"><b>TurtleBot3 Demo</b><br>
<video src="https://raw.githubusercontent.com/yuokamoto/PyBulletFleet/feat/ros2-bridge-v2/ros2_bridge/videos/tb3.mp4" width="380" controls muted></video></td>
<td align="center"><b>UR5 Arm</b><br>
<video src="https://raw.githubusercontent.com/yuokamoto/PyBulletFleet/feat/ros2-bridge-v2/ros2_bridge/videos/ur5.mp4" width="380" controls muted></video></td>
</tr>
</table>

ROS 2 interface layer for [PyBulletFleet](../README.md).
Three packages provide a clean separation between general ROS 2 connectivity, Open-RMF integration, and custom message definitions.

| Package | Description |
|---------|-------------|
| **pybullet_fleet_ros** | General-purpose ROS 2 bridge — per-robot topics, action servers, services |
| **pybullet_fleet_rmf** | Open-RMF fleet adapter + door / lift / workcell handlers |
| **pybullet_fleet_msgs** | Custom message, service, and action definitions |

## Quick Start

See **[docker/README.md](../docker/README.md)** for build instructions and demo walkthroughs.

---

## pybullet_fleet_ros — ROS 2 Bridge

`bridge_node` creates a `RobotHandler` for each simulated robot, exposing standard ROS 2 interfaces.

### Per-Robot Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `/{robot}/odom` | `nav_msgs/Odometry` | Odometry + TF (`odom → base_link`) |
| Pub | `/{robot}/joint_states` | `sensor_msgs/JointState` | Joint positions / velocities |
| Pub | `/{robot}/plan` | `nav_msgs/Path` | Current planned path |
| Pub | `/{robot}/current_goal` | `geometry_msgs/PoseStamped` | Active navigation goal |
| Pub | `/{robot}/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Robot diagnostics |
| Sub | `/{robot}/cmd_vel` | `geometry_msgs/Twist` | Velocity command |
| Sub | `/{robot}/goal_pose` | `geometry_msgs/PoseStamped` | Navigation goal (fire-and-forget) |
| Sub | `/{robot}/path` | `nav_msgs/Path` | Path to follow |
| Sub | `/{robot}/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Joint trajectory command |
| Sub | `/{robot}/joint_commands` | `std_msgs/Float64MultiArray` | Direct joint position command |

### Per-Robot Action Servers

| Action Server | Type | Description |
|---------------|------|-------------|
| `/{robot}/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Point-to-point navigation with feedback |
| `/{robot}/follow_path` | `nav2_msgs/FollowPath` | Path-following with feedback |
| `/{robot}/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Arm trajectory execution |
| `/{robot}/execute_action_blocking` | `pybullet_fleet_msgs/ExecuteAction` | Generic PyBulletFleet action (blocking) |

### Per-Robot Services

| Service | Type | Description |
|---------|------|-------------|
| `/{robot}/toggle_attach` | `std_srvs/SetBool` | Attach/detach nearest object |
| `/{robot}/attach_object` | `pybullet_fleet_msgs/AttachObject` | Attach specific object by name |

### Per-Robot Topics (fire-and-forget)

| Topic | Type | Description |
|-------|------|-------------|
| `/{robot}/execute_action` | `pybullet_fleet_msgs/ExecuteActionGoal` | Generic action (non-blocking) |

### Simulation Services

| Service | Description |
|---------|-------------|
| `/sim/spawn_entity` | Spawn robot or object |
| `/sim/delete_entity` | Remove entity |
| `/sim/get_entity_state` | Query pose of an entity |
| `/sim/set_entity_state` | Teleport entity |
| `/sim/get_entities` | List all entities |
| `/sim/get_entities_states` | Batch state query |
| `/sim/get_entity_info` | Entity metadata (URDF, type) |
| `/sim/get_entity_bounds` | AABB bounds |
| `/sim/step_simulation` | Advance simulation by N steps |
| `/sim/get_simulation_state` | Query sim state (paused, time, FPS) |
| `/sim/set_simulation_state` | Pause / resume / set speed |
| `/sim/reset_simulation` | Reset to initial state |
| `/sim/get_spawnables` | List available URDFs |
| `/sim/get_simulator_features` | Query supported features |
| `/sim/simulate_steps` | Step simulation (action, with feedback) |

---

## pybullet_fleet_rmf — Open-RMF Integration

Fleet adapter and infrastructure handlers for [Open-RMF](https://www.open-rmf.org/).

### Fleet Adapter

`fleet_adapter` registers simulated robots with the RMF fleet manager via `rmf_adapter.easy_full_control`. Commands are forwarded to the bridge node through `RobotClientAPI`:

```
RMF Schedule ← FleetAdapterNode → RobotClientAPI → BridgeNode → PyBulletFleet
```

Supported RMF task actions:
- **navigate** — `NavigateToPose` action goal
- **delivery_pickup** — `toggle_attach(True)` → PickAction
- **delivery_dropoff** — `toggle_attach(False)` → DropAction
- **stop** — Cancel current navigation

Currently supported RMF task types: **patrol** and **delivery** only.
`charge` is not simulated (sim battery is always 100%; `finishing_request` should be `"nothing"` or `"park"` to avoid deadlock).
`clean` is a no-op (immediately finishes without zone patrol).

### Infrastructure Handlers

| Handler | RMF Protocol | Subscribe | Publish |
|---------|-------------|-----------|---------|
| `DoorHandler` | Door adapter | `/adapter_door_requests` | `/door_states` |
| `LiftHandler` | Lift adapter | `/adapter_lift_requests` | `/lift_states` |
| `WorkcellHandler` | Dispenser + Ingestor | `/dispenser_requests`, `/ingestor_requests` | `/{dispenser,ingestor}_{states,results}` |

### Demo Scenarios

| Launch File | Environment |
|-------------|-------------|
| `office_pybullet.launch.py` | Office (rmf_demos standard) |
| `hotel_pybullet.launch.py` | Hotel |
| `airport_terminal_pybullet.launch.py` | Airport terminal |

```bash
# Example: Office demo (via Docker)
cd docker && docker compose up
```

See **[docker/README.md](../docker/README.md)** for detailed setup and walkthroughs.

---

## Roadmap

Future improvements. Items already implemented have been removed — see git log for history.

---

### Near-Term

#### Batch API ROS Wrapper (Pattern 2)

`AgentManager` batch API に numpy flat array 一括取得を追加し、ROS ラッパーで O(1) endpoint 化。

```
fleet_adapter ↔ 1×/fleet/states + 1×/fleet/navigate ↔ bridge_node ↔ sim_core
```

- `/fleet/states` publisher (FleetStates.msg) — N 台分を1メッセージ
- `/fleet/navigate` service — batch navigation
- 100 robots: 200 endpoints → 2–3 endpoints

**Depends on:** `pybullet_fleet_msgs` に FleetStates.msg 定義

---

### Mid-Term

#### Direct Python Connection (no ROS)

`fleet_adapter` を `MultiRobotSimulationCore` に Python API で直接接続。
ROS 2 topic/service 層をバイパスし、低レイテンシ・シンプルデプロイを実現。

- EventBus でアダプタ ↔ sim_core 通信
- 同一プロセスで実行、シリアライゼーションオーバーヘッドゼロ

**Deployment Patterns:**

| # | Name | 通信 | bridge | ROS endpoints | 用途 |
|---|---|---|---|---|---|
| 1 | Per-Robot ROS 2 | ROS topics | ✅ | O(N) | **現行 (v1)** |
| 2 | Batch ROS 2 | ROS topics (batch) | ✅ | O(1) | Scalable |
| 3 | Plugin Only | Direct Python | ❌ | 0 | CI / headless |
| 4 | Plugin + Bridge | Direct Python | ✅ | 0 | Dev/Debug |

---

#### Handler Decomposition

`RobotHandler` を focused, composable handlers に分解 (Gazebo-plugin style):

- **OdometryHandler** — `/odom` + TF `odom → base_link`
- **JointStateHandler** — `/joint_states`
- **CmdVelHandler** — `/cmd_vel` subscriber
- **NavigationHandler** — `navigate_to_pose` action server
- **DiagnosticsHandler** — Status/heartbeat

`handler_classes:` config でロボットタイプごとに必要なハンドラだけを組み合わせ可能。

---

#### `cargo_relative_pose` on Agent

`WorkcellPlugin._attach_z_offset` を `Agent` / `AgentSpawnParams` に `cargo_relative_pose: Pose` として移動。
`dispense()` が `robot.cargo_relative_pose` → `PickAction.attach_relative_pose` を設定。
ロボットごとの取り付け位置（フォークリフト先端 vs AMR 上面 vs グリッパリンク）に対応。

---

#### Multi-Floor Visualization

フロア表示 ON/OFF で視認性確保:

- `p.changeVisualShape(rgbaColor=[..., alpha=0])` で非表示フロアを透明化
- キーボードでフロア切替 (1, 2, 3 キー)

---

### Low Priority

#### Cart Delivery (toggle_attach 方式)

`dispatch_cart_delivery.py` → `compose` タスク (PerformAction phases)。
`toggle_attach` (SetBool) / `attach_object` (AttachObject.srv) は実装済み。
fleet_adapter の `execute_action("delivery_pickup/dropoff")` → `toggle_attach` の接続が残り。

---

#### Cleaning Simulation

`execute_action("clean")` でゾーン巡回。現在は即 `finished()` でスキップ。
rmf_demos でも airport_terminal のみ使用。

---

#### Device Enhancements

- **Elevator doors** — Elevator プラットフォームに revolute/prismatic ドアジョイントを追加。到着/出発時に JointAction で開閉
- **Double-hinge door** — 2枚リーフドア用の URDF + 協調 JointAction
- **Xacro-parameterised device URDFs** — `door_hinge.urdf` 等を xacro テンプレート化。YAML config からドア幅/高さを指定可能に

---

### Documentation TODO

- Standard Delivery フロー図 (dispatch_delivery → RMF → WorkcellHandler → PickAction → DropAction)
- Cart Delivery フロー図 (dispatch_cart_delivery → compose → toggle_attach)
- 通信手段の違い (ROS topic vs fleet_adapter callback)
