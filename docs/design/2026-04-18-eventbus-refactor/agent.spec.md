# bridge_node EventBus Refactor — Agent Specification

## Requirements

- bridge_node を EventBus ベースの observer に全面移行 (owner モード廃止)
- sim_core を外部から受け取り、EventBus でのみアクセス
- ROS timer (step + publish) を EventBus callback に置換

## Constraints

- `rclpy.spin()` は必ず呼ぶ必要がある (ROS 2 action server 等のため)
- `sim.run_simulation()` はブロッキング → 別スレッドで実行
- bridge_node が sim_core を所有しないケースでは `from_dict()` を呼ばない
- `MultiThreadedExecutor` は既に使用中 (L228)

## Approach

bridge_node の publish ロジックを `_publish_all_states()` に集約し、
`POST_STEP` EventBus callback から呼び出す。ROS timer は廃止。

### 変更箇所

| File | Change |
|------|--------|
| `bridge_node.py` | ROS timer 駆動を削除、EventBus callback に移行 |
| `bridge_node.py` | `_publish_all_states()` を既存 `_publish_callback` から抽出 |
| `bridge_node.py` | `POST_STEP` → `_publish_all_states()` |
| `bridge_node.py` | sim_core を外部から注入する `create_observer()` classmethod |

### Before (v1 — ROS timer 駆動)

```
BridgeNode.__init__():
    self.sim = MultiRobotSimulationCore.from_dict(config)  # L87-101
    self._step_timer = create_timer(dt/rtf, _step_callback)  # L129-135
    self._publish_timer = create_timer(1/rate, _publish_callback)  # L138-139

_step_callback():
    self.sim.step_once()  # L200
    publish /clock  # L204-206

_publish_callback():
    for handler in self._handlers.values():
        handler.publish_robot_state()  # L210-213
```

### After (v2 — EventBus observer)

```
BridgeNode.create_observer(sim_core):
    self.sim = sim_core  # 外部から注入
    sim_core.events.on(POST_STEP, self._on_post_step)
    sim_core.events.on(AGENT_SPAWNED, self._on_agent_spawned)
    sim_core.events.on(AGENT_REMOVED, self._on_agent_removed)
    # ROS timer は不要

_on_post_step(event_data):
    self._publish_all_states()
    # /clock は内部 throttle (100 Hz)
    if event_data.sim_time - self._last_clock >= 0.01:
        self._publish_clock(event_data.sim_time)
        self._last_clock = event_data.sim_time
```

### Code Pattern — EventBus callback 登録

```python
# events.py の SimEvents を使用
from pybullet_fleet.events import SimEvents

def _init_observer(self, sim_core):
    self.sim = sim_core
    self.sim.events.on(SimEvents.POST_STEP, self._on_post_step)
    self.sim.events.on(SimEvents.AGENT_SPAWNED, self._on_agent_spawned)
    self.sim.events.on(SimEvents.AGENT_REMOVED, self._on_agent_removed)
    # register_callback は使わない — EventBus に統一
```

### スレッド配置

```
Thread 1 (main): rclpy.spin(bridge_node)
    ↑ ROS 2 action server callbacks, service callbacks

Thread 2: sim_core.run_simulation()
    ↓ POST_STEP → bridge_node._on_post_step()
```

## File References

- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/bridge_node.py` — 全変更箇所
- `pybullet_fleet/events.py` — SimEvents enum, EventBus API
- `pybullet_fleet/core_simulation.py:L503-L524` — register_callback
- `pybullet_fleet/core_simulation.py:L2723` — run_simulation
- `pybullet_fleet/core_simulation.py:L2910` — step_once 実行順序

## Success Criteria

- [ ] bridge_node が EventBus 経由で ROS publish を実行
- [ ] `/clock`, `/odom`, `/tf`, `/joint_states` が正しく publish
- [ ] `NavigateToPose` action server が動作
- [ ] `/clock` throttle が 100 Hz で動作
- [ ] 既存テスト全通過
- [ ] `make verify` 通過
