# Device Base + Door/Elevator & ExternalAgent Plugin — Agent Specification

## Requirements

### Functional
- DoorDevice(Agent): URDF revolute/prismatic ジョイントで開閉シミュレーション
- ElevatorDevice(Agent): URDF prismatic Z ジョイントでフロア間移動 + auto_attach
- WorkcellDevice(SimObject): 静的位置マーカー (PickAction target)
- PatrolController: ウェイポイント巡回、loop/wait_time 対応
- RandomWalkController: ランダム近傍歩行、radius/wait_range 対応 (crowd_simulator 簡易版)
- Controller チェーン: Agent に複数 Controller をスタック可能
- SimObject.update() + _needs_update: 統一更新ループ
- Demo launch files & configs: battle_royale, hotel, clinic, airport_terminal 更新
  - 各デモが `DEMO_WORLD=xxx docker compose -f docker-compose.rmf.yaml up` で起動可能

### Non-functional
- 既存テスト 1255 件全パス
- Coverage 75%+ 維持
- ROS 非依存 (device/controller は pybullet_fleet コアに実装)
- p.DIRECT のみ (テストで p.GUI 禁止)

## Constraints

- Agent は SimObject を継承。DoorDevice/ElevatorDevice は Agent を継承
- Agent.from_params() は cls パラメータを正しくサブクラスに伝搬する必要あり
- `_agents` リストに Device が追加されるが、fleet_adapter は fleet config YAML の known_robots のみ管理するため影響なし
- controller.py の CONTROLLER_REGISTRY に新 Controller を登録
- entity_registry.py の ENTITY_REGISTRY に新 Device を登録

## Approach

### 1. SimObject.update() + _needs_update (基盤変更)

SimObject に update(dt) と _needs_update クラス属性を追加。
step_once() を統一ループに変更。

```python
# sim_object.py
class SimObject:
    _needs_update: bool = False  # 静的オブジェクトはスキップ

    def update(self, dt: float) -> bool:
        """毎ステップ呼ばれる。サブクラスでオーバーライド。
        Returns: True if object moved/changed.
        """
        return False
```

```python
# core_simulation.py step_once() — 現在の Agent ループを置換
# BEFORE:
#   for agent in self._agents:
#       moved = agent.update(self._params.timestep)
#       if moved and agent.object_id in self._kinematic_objects:
#           self._moved_this_step.add(agent.object_id)
#
# AFTER:
for obj in self._sim_objects:
    if obj._needs_update:
        moved = obj.update(self._params.timestep)
        if moved and obj.object_id in self._kinematic_objects:
            self._moved_this_step.add(obj.object_id)
```

注意: Agent._needs_update = True は Agent クラスで設定（既存の update() が呼ばれる）。

### 2. Controller チェーン (Agent 変更)

```python
# agent.py — Controller リスト化
class Agent(SimObject):
    _needs_update = True

    def __init__(self, spawn_params, sim_core):
        ...
        self._controllers: list[Controller] = []
        # motion_mode から基底 Controller を自動生成
        base_ctrl = self._create_base_controller(spawn_params)
        self._controllers.append(base_ctrl)

        # controllers: YAML セクションから高レベル Controller を追加
        for ctrl_cfg in spawn_params.controllers or []:
            ctrl = create_controller(ctrl_cfg["type"],
                                     {k:v for k,v in ctrl_cfg.items() if k != "type"})
            self._controllers.append(ctrl)

    @property
    def controller(self) -> Controller | None:
        """後方互換: 基底 Controller を返す"""
        return self._controllers[0] if self._controllers else None

    def set_controller(self, controller: Controller):
        """後方互換: 基底 Controller を差し替え"""
        if self._controllers:
            self._controllers[0] = controller
        else:
            self._controllers.append(controller)

    def add_controller(self, controller: Controller):
        """高レベル Controller をチェーン末尾に追加"""
        self._controllers.append(controller)

    def remove_controller(self, controller: Controller):
        """Controller をチェーンから除去（基底以外）"""
        if controller in self._controllers[1:]:
            self._controllers.remove(controller)

    def update(self, dt) -> bool:
        ...
        # 現在の self._controller.compute(self, dt) を以下に置換:
        for ctrl in reversed(self._controllers):
            ctrl.compute(self, dt)
        ...
```

AgentSpawnParams に `controllers` フィールドを追加:
```python
@dataclass
class AgentSpawnParams(SimObjectSpawnParams):
    ...
    controllers: list[dict] | None = None  # [{"type": "patrol", "waypoints": [...]}]
```

### 3. PatrolController / RandomWalkController

```python
# pybullet_fleet/controllers/patrol_controller.py
class PatrolController(Controller):
    """ウェイポイント巡回。set_goal_pose() を呼ぶだけ。
    実移動は下位の KinematicController に委譲。"""

    def __init__(self, waypoints: list[list[float]],
                 wait_time: float = 0.0, loop: bool = True):
        super().__init__()
        self._waypoints = waypoints
        self._current_idx = 0
        self._wait_time = wait_time
        self._wait_timer = 0.0
        self._loop = loop

    def compute(self, agent, dt: float) -> bool:
        if self._wait_timer > 0:
            self._wait_timer -= dt
            return False
        if not agent.is_moving:
            self._current_idx = (self._current_idx + 1) % len(self._waypoints) if self._loop \
                else min(self._current_idx + 1, len(self._waypoints) - 1)
            if self._current_idx < len(self._waypoints):
                agent.set_goal_pose(Pose.from_xyz(*self._waypoints[self._current_idx]))
            self._wait_timer = self._wait_time
        return False  # 実移動は KinematicController が処理

# pybullet_fleet/controllers/random_walk_controller.py
class RandomWalkController(Controller):
    """ランダム近傍歩行。set_goal_pose() を設定するだけ。"""

    def __init__(self, radius: float = 5.0,
                 wait_range: tuple[float, float] = (1.0, 5.0)):
        super().__init__()
        self._radius = radius
        self._wait_range = wait_range
        self._origin: list[float] | None = None

    def compute(self, agent, dt: float) -> bool:
        if self._origin is None:
            self._origin = list(agent.get_pose().position[:2])
        if not agent.is_moving:
            import random, math
            angle = random.uniform(0, 2 * math.pi)
            dist = random.uniform(0, self._radius)
            z = agent.get_pose().position[2]
            target = [self._origin[0] + dist * math.cos(angle),
                      self._origin[1] + dist * math.sin(angle), z]
            agent.set_goal_pose(Pose.from_xyz(*target))
        return False
```

CONTROLLER_REGISTRY に登録:
```python
# controller.py
CONTROLLER_REGISTRY["patrol"] = PatrolController
CONTROLLER_REGISTRY["random_walk"] = RandomWalkController
```

### 4. DoorDevice(Agent)

```python
# pybullet_fleet/devices/door.py
from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.action import JointAction
from pybullet_fleet.types import ActionStatus

class DoorDevice(Agent):
    """URDF ジョイントで開閉をシミュレーションするドア。

    revolute joint → ヒンジドア
    prismatic joint → スライドドア
    JointAction で制御 → Agent.update() が自動で interpolation。
    """

    @classmethod
    def from_params(cls, spawn_params: AgentSpawnParams, sim_core) -> "DoorDevice":
        # Agent.from_params() の結果を DoorDevice に変換
        # 実装方法は Agent factory の構造を確認して決定
        instance = super().from_params(spawn_params, sim_core)
        # device_config from user_data
        cfg = spawn_params.user_data or {}
        instance._open_positions = cfg.get("open_positions", {})
        instance._close_positions = cfg.get("close_positions", {})
        return instance

    @property
    def door_state(self) -> str:
        """ジョイント位置とアクション状態からドア状態を導出"""
        action = self.current_action
        if action and isinstance(action, JointAction) and action.status == ActionStatus.IN_PROGRESS:
            # target が open_positions なら "opening", close_positions なら "closing"
            if action.target_joint_positions == self._open_positions:
                return "opening"
            return "closing"
        if self._open_positions and self.are_joints_at_targets(self._open_positions):
            return "open"
        return "closed"

    def request_open(self):
        """ドアを開く (JointAction を投入)"""
        if self.door_state in ("closed", "closing"):
            self.cancel_current_action()  # closing 中なら cancel
            self.add_action(JointAction(target_joint_positions=self._open_positions))

    def request_close(self):
        """ドアを閉じる (JointAction を投入)"""
        if self.door_state in ("open", "opening"):
            self.cancel_current_action()
            self.add_action(JointAction(target_joint_positions=self._close_positions))
```

### 5. ElevatorDevice(Agent)

```python
# pybullet_fleet/devices/elevator.py
from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.action import JointAction
from pybullet_fleet.logging_utils import get_lazy_logger

logger = get_lazy_logger(__name__)

class ElevatorDevice(Agent):
    """Prismatic Z ジョイントでフロア間移動するエレベータ。

    URDF:
      <joint name="lift" type="prismatic">
        <axis xyz="0 0 1"/>
        <limit lower="0" upper="20"/>
      </joint>

    auto_attach=True でプラットフォーム上のオブジェクトを自動 attach/detach。
    """

    @classmethod
    def from_params(cls, spawn_params: AgentSpawnParams, sim_core) -> "ElevatorDevice":
        instance = super().from_params(spawn_params, sim_core)
        cfg = spawn_params.user_data or {}
        instance._floors = cfg.get("floors", {})  # {"L1": 0.0, "L2": 8.0}
        instance._current_floor = cfg.get("initial_floor", "L1")
        instance._auto_attach = cfg.get("auto_attach", True)
        instance._platform_radius = cfg.get("platform_radius", 2.0)
        instance._joint_name = cfg.get("joint_name", "lift")
        return instance

    @property
    def current_floor(self) -> str:
        return self._current_floor

    @property
    def available_floors(self) -> list[str]:
        return list(self._floors.keys())

    def request_floor(self, floor_name: str):
        """指定フロアへ移動"""
        if floor_name not in self._floors:
            logger.warning("Unknown floor: %s", floor_name)
            return
        if floor_name == self._current_floor:
            return

        target_z = self._floors[floor_name]

        # auto_attach: プラットフォーム上のオブジェクトを attach
        if self._auto_attach:
            self._attach_platform_objects()

        # JointAction でリフトジョイントを移動
        self.add_action(JointAction(
            target_joint_positions={self._joint_name: target_z}
        ))
        self._target_floor = floor_name

    def _attach_platform_objects(self):
        """プラットフォーム上のオブジェクトを自動 attach"""
        my_pos = self.get_pose().position
        for obj in self._sim_core.sim_objects:
            if obj is self or obj.object_id == self.object_id:
                continue
            pos = obj.get_pose().position
            dist_2d = ((pos[0] - my_pos[0])**2 + (pos[1] - my_pos[1])**2)**0.5
            if dist_2d < self._platform_radius:
                self.attach_object(obj)
                logger.info("Auto-attached %s to elevator", obj.name)

    def _detach_all(self):
        """全 attached オブジェクトを detach"""
        for child in list(getattr(self, '_attached_objects', [])):
            self.detach_object(child)

    def update(self, dt: float) -> bool:
        result = super().update(dt)

        # 到着判定: JointAction 完了 → current_floor 更新 + detach
        if hasattr(self, '_target_floor') and self._target_floor:
            target_z = self._floors[self._target_floor]
            joint_name = self._joint_name
            if self.are_joints_at_targets({joint_name: target_z}):
                self._current_floor = self._target_floor
                self._target_floor = None
                if self._auto_attach:
                    self._detach_all()
                logger.info("Elevator arrived at %s", self._current_floor)

        return result
```

### 6. WorkcellDevice(SimObject)

```python
# pybullet_fleet/devices/workcell.py
from pybullet_fleet.sim_object import SimObject

class WorkcellDevice(SimObject):
    """Dispenser/Ingestor の位置マーカー。
    物理ボディあり (可視で位置確認可能)。
    PickAction の target_position として使用。
    """
    _needs_update = False  # 静的 — update() 不要
```

### 7. URDF ファイル

```xml
<!-- robots/door_hinge.urdf — ヒンジドア -->
<robot name="door_hinge">
  <link name="frame">
    <visual><geometry><box size="0.1 0.1 2.0"/></geometry></visual>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="frame"/>
    <child link="panel"/>
    <origin xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="1.5708" effort="10" velocity="1.0"/>
  </joint>
  <link name="panel">
    <visual><geometry><box size="1.0 0.05 2.0"/></geometry></visual>
    <collision><geometry><box size="1.0 0.05 2.0"/></geometry></collision>
  </link>
</robot>
```

```xml
<!-- robots/door_slide.urdf — スライドドア -->
<robot name="door_slide">
  <link name="frame">
    <visual><geometry><box size="0.1 0.1 2.0"/></geometry></visual>
  </link>
  <joint name="slide" type="prismatic">
    <parent link="frame"/>
    <child link="panel"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.2" effort="10" velocity="1.0"/>
  </joint>
  <link name="panel">
    <visual><geometry><box size="1.0 0.05 2.0"/></geometry></visual>
    <collision><geometry><box size="1.0 0.05 2.0"/></geometry></collision>
  </link>
</robot>
```

```xml
<!-- robots/elevator.urdf — エレベータ -->
<robot name="elevator">
  <link name="shaft">
    <visual><geometry><box size="0.01 0.01 0.01"/></geometry></visual>
  </link>
  <joint name="lift" type="prismatic">
    <parent link="shaft"/>
    <child link="platform"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="20.0" effort="100" velocity="2.0"/>
  </joint>
  <link name="platform">
    <visual><geometry><box size="3.0 3.0 0.2"/></geometry></visual>
    <collision><geometry><box size="3.0 3.0 0.2"/></geometry></collision>
  </link>
</robot>
```

### 8. entity_registry 登録

```python
# entity_registry.py — 組み込み登録に追加
from pybullet_fleet.devices.door import DoorDevice
from pybullet_fleet.devices.elevator import ElevatorDevice
from pybullet_fleet.devices.workcell import WorkcellDevice

ENTITY_REGISTRY["door"] = DoorDevice
ENTITY_REGISTRY["elevator"] = ElevatorDevice
ENTITY_REGISTRY["workcell"] = WorkcellDevice
```

または YAML の `entity_classes:` で動的登録 (import 文字列)。

### 9. ROS Adapter 変更

**door_adapter.py 変更:**
```python
# 現在: スタブ (即座に状態変更)
# 変更後: sim_core の DoorDevice を制御

class DoorAdapter(Node):
    def __init__(self, sim_core, ...):
        self._sim_core = sim_core
        # DoorRequest → DoorDevice.request_open/close

    def _on_door_request(self, msg):
        door = self._sim_core.get_agent(msg.door_name)
        if isinstance(door, DoorDevice):
            if msg.requested_mode == DoorMode.MODE_OPEN:
                door.request_open()
            else:
                door.request_close()

    def _publish_states(self):
        # DoorDevice.door_state → DoorState msg
        for agent in self._sim_core.agents:
            if isinstance(agent, DoorDevice):
                state_msg = DoorState()
                state_msg.door_name = agent.name
                state_msg.current_mode = self._state_to_mode(agent.door_state)
                self._pub.publish(state_msg)
```

**lift_adapter.py (新規):**
```python
class LiftAdapter(Node):
    def _on_lift_request(self, msg):
        elevator = self._sim_core.get_agent(msg.lift_name)
        if isinstance(elevator, ElevatorDevice):
            elevator.request_floor(msg.destination_floor)

    def _publish_states(self):
        for agent in self._sim_core.agents:
            if isinstance(agent, ElevatorDevice):
                state_msg = LiftState()
                state_msg.lift_name = agent.name
                state_msg.current_floor = agent.current_floor
                state_msg.available_floors = agent.available_floors
                self._pub.publish(state_msg)
```

**ExternalAgentHandler (新規):**
```python
class ExternalAgentHandler(RobotHandlerBase):
    """ExternalAgent 用: Odometry + TF のみ publish。
    NavigateToPose action server は提供しない。"""

    def post_step(self, dt, stamp):
        self._publish_odom(stamp)
        self._broadcast_tf(stamp)

    def destroy(self):
        self._odom_pub.destroy()
```

## Design

### Architecture

```
pybullet_fleet/
├── sim_object.py          # + update() + _needs_update
├── agent.py               # + _controllers list, add_controller()
├── controller.py          # + CONTROLLER_REGISTRY entries
├── core_simulation.py     # step_once() unified loop
├── entity_registry.py     # + door, elevator, workcell
├── controllers/
│   ├── __init__.py
│   ├── patrol_controller.py
│   └── random_walk_controller.py
├── devices/
│   ├── __init__.py
│   ├── door.py            # DoorDevice(Agent)
│   ├── elevator.py        # ElevatorDevice(Agent)
│   └── workcell.py        # WorkcellDevice(SimObject)
robots/
├── door_hinge.urdf
├── door_slide.urdf
└── elevator.urdf

ros2_bridge/pybullet_fleet_ros/
├── door_adapter.py        # 変更: sim_core 連携
├── lift_adapter.py        # 新規
└── external_agent_handler.py  # 新規
```

### Key Components

| Component | Responsibility | Location |
|-----------|---------------|----------|
| SimObject.update() | 統一更新フック | `pybullet_fleet/sim_object.py` |
| Controller chain | 複数 Controller スタック | `pybullet_fleet/agent.py` |
| PatrolController | ウェイポイント巡回 | `pybullet_fleet/controllers/patrol_controller.py` |
| RandomWalkController | ランダム歩行 | `pybullet_fleet/controllers/random_walk_controller.py` |
| DoorDevice | URDF ジョイント開閉 | `pybullet_fleet/devices/door.py` |
| ElevatorDevice | Z 移動 + auto_attach | `pybullet_fleet/devices/elevator.py` |
| WorkcellDevice | 位置マーカー | `pybullet_fleet/devices/workcell.py` |
| DoorAdapter | ROS wrapper for Door | `ros2_bridge/.../door_adapter.py` |
| LiftAdapter | ROS wrapper for Elevator | `ros2_bridge/.../lift_adapter.py` |
| ExternalAgentHandler | Odom/TF publish only | `ros2_bridge/.../external_agent_handler.py` |

### Data Flow

#### Door open request:
```
RMF DoorRequest → door_adapter → DoorDevice.request_open()
    → JointAction(target=open_positions) → Agent.update()
    → joint interpolation → door_state: "opening" → "open"
    → door_adapter publishes DoorState(OPEN)
```

#### Elevator floor request:
```
RMF LiftRequest → lift_adapter → ElevatorDevice.request_floor("L2")
    → auto_attach(platform objects)
    → JointAction(target={lift: 8.0}) → Agent.update()
    → Z interpolation → attached objects follow
    → current_floor = "L2", auto_detach
    → lift_adapter publishes LiftState(L2)
```

#### ExternalAgent patrol:
```
YAML controllers: [{type: patrol, waypoints: [...]}]
    → Agent.__init__: add_controller(PatrolController)
    → Agent.update() → PatrolController.compute() → set_goal_pose(wp)
    → KinematicController.compute() → 実移動
    → ExternalAgentHandler.post_step() → Odom/TF publish
```

### Code Patterns

```python
# Controller chain — 既存パターンとの互換性
agent = Agent.from_params(spawn_params, sim_core)
agent.controller  # → KinematicController (基底)
agent.add_controller(PatrolController(waypoints=[...]))
# update() → PatrolController.compute() → KinematicController.compute()

# set_controller() は後方互換
agent.set_controller(new_kinematic)  # 基底を差し替え

# Device YAML スポーン
# entity_classes: {door: "pybullet_fleet.devices.DoorDevice"}
# entities:
#   - name: main_door
#     type: door
#     urdf_path: robots/door_hinge.urdf
#     user_data:
#       open_positions: {hinge: 1.57}
#       close_positions: {hinge: 0.0}
```

## File References

Files the plan agent MUST read before planning:

- `pybullet_fleet/sim_object.py` — SimObject class, update hook location
- `pybullet_fleet/agent.py` — Agent.update(), _controller, from_params(), action queue
- `pybullet_fleet/controller.py` — Controller ABC, CONTROLLER_REGISTRY, KinematicController
- `pybullet_fleet/core_simulation.py:2962-3060` — step_once() agent update loop
- `pybullet_fleet/action.py` — JointAction, ActionStatus
- `pybullet_fleet/entity_registry.py` — ENTITY_REGISTRY, spawn dispatch
- `pybullet_fleet/types.py` — MotionMode, ActionStatus enums
- `pybullet_fleet/geometry.py` — Pose class
- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/door_adapter.py` — current door adapter
- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler_base.py` — handler base
- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py` — existing handler
- `tests/conftest.py` — MockSimCore, fixtures
- `tests/test_action.py` — JointAction test patterns

## Success Criteria

- [ ] DoorDevice: request_open() → door_state transitions closed → opening → open
- [ ] DoorDevice: request_close() → door_state transitions open → closing → closed
- [ ] ElevatorDevice: request_floor("L2") → Z=8.0 → current_floor="L2"
- [ ] ElevatorDevice: auto_attach Agent on platform → follows Z movement
- [ ] ElevatorDevice: auto_detach on floor arrival
- [ ] WorkcellDevice: _needs_update=False, static position
- [ ] PatrolController: cycles through waypoints with loop=True
- [ ] PatrolController: stops at last waypoint with loop=False
- [ ] RandomWalkController: moves within radius of origin
- [ ] Controller chain: high-level compute() runs before low-level
- [ ] set_controller() backward compatibility (replaces index 0)
- [ ] add_controller() appends to chain
- [ ] entity_registry: type: door → DoorDevice spawned
- [ ] YAML controllers: section → controllers attached at spawn
- [ ] step_once() unified loop: _sim_objects with _needs_update
- [ ] Demo: battle_royale launch + config → patrol tasks work
- [ ] Demo: hotel launch + config → patrol tasks + door + elevator work
- [ ] Demo: clinic launch + config → patrol tasks + door + elevator work
- [ ] Demo: airport_terminal config updated → doors + caddy patrol work
- [ ] All demos launchable via DEMO_WORLD=xxx docker compose
- [ ] Existing 1255 tests pass
- [ ] Coverage ≥ 75%
- [ ] `make verify` passes
