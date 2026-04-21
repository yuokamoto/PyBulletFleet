# Multi Fleet Support — Agent Specification

## Requirements

- demo-specific launch file (e.g. `airport_pybullet.launch.py`) から複数 fleet_adapter を起動
- `pybullet_common.launch.py` はそのまま使用 (bridge + door + workcell + 1 fleet)
- 追加フリートは demo launch file 側で Node を追加
- bridge_config YAML に全フリートのロボットを列挙

## Constraints

- `fleet_adapter.py` のコード変更なし
- `pybullet_common.launch.py` の変更は最小限 (fleet_adapter の名前を configurable にする程度)
- 各 fleet_adapter は rmf_demos の fleet config YAML をそのまま使用

## Approach

rmf_demos の `airport_terminal.launch.xml` と同じパターン:
launch file で fleet ごとに fleet_adapter Node を宣言。

### 変更点

1. `pybullet_common.launch.py` — fleet_adapter の `fleet_name` を launch arg 化
2. `airport_pybullet.launch.py` (新規) — 5 fleet 用 demo launch
3. `config/bridge_airport.yaml` (新規) — 全フリートのロボット定義
4. rmf_demos の fleet config (`tinyRobot_config.yaml` 等) をそのまま参照

## Design

### Launch Structure (airport_terminal 例)

```
airport_pybullet.launch.py
├── IncludeLaunchDescription(rmf_demos/common.launch.xml)
│   └── traffic schedule, map server, task dispatcher, etc.
├── IncludeLaunchDescription(pybullet_common.launch.py)
│   ├── bridge_node (config_yaml=bridge_airport.yaml)
│   ├── door_adapter
│   ├── workcell_adapter
│   └── fleet_adapter (fleet_config=tinyRobot_config.yaml)
├── Node: fleet_adapter_2 (fleet_config=deliveryRobot_config.yaml)
├── Node: fleet_adapter_3 (fleet_config=cleanerBot_config.yaml)
├── Node: fleet_adapter_4 (fleet_config=caddyRobot_config.yaml)
└── Node: fleet_adapter_5 (fleet_config=securityBot_config.yaml)
```

### bridge_airport.yaml

```yaml
simulation:
  gui: true
  physics: false
  target_rtf: 10.0

world:
  world_file: "/rmf_demos_ws/.../airport_terminal.world"
  skip_models: [TinyRobot, DeliveryRobot, CleanerBot, ...]

robots:
  # Fleet 1: TinyRobot
  - name: tinyRobot1
    sdf_path: ".../TinyRobot/model.sdf"
    pose: [10, -5, 0]
    motion_mode: "differential"
  - name: tinyRobot2
    sdf_path: ".../TinyRobot/model.sdf"
    pose: [20, -5, 0]
    motion_mode: "differential"
  # Fleet 2: DeliveryRobot
  - name: deliveryRobot1
    sdf_path: ".../DeliveryRobot/model.sdf"
    pose: [30, -5, 0]
    motion_mode: "differential"
  # ... etc
```

### Code Pattern — 追加 fleet_adapter Node

```python
# In airport_pybullet.launch.py
Node(
    package="pybullet_fleet_ros",
    executable="fleet_adapter",
    name="deliveryRobot_fleet_adapter",  # ← unique name per fleet
    arguments=[
        "-c", delivery_fleet_config,
        "-n", delivery_nav_graph,
    ],
    parameters=[{"server_uri": LaunchConfiguration("server_uri")}],
    output="screen",
),
```

## File References

- `ros2_bridge/pybullet_fleet_ros/launch/office_pybullet.launch.py` — 現在の single-fleet launch
- `ros2_bridge/pybullet_fleet_ros/launch/pybullet_common.launch.py` — 共通ノード起動
- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/fleet_adapter.py` — fleet adapter (変更なし)
- `ros2_bridge/pybullet_fleet_ros/config/bridge_office.yaml` — 現在の bridge config

## Success Criteria

- [ ] airport_pybullet.launch.py が5つの fleet_adapter を起動
- [ ] bridge_airport.yaml に5フリートのロボットが定義
- [ ] 各フリートが独立に RMF タスクを受信・実行
- [ ] pybullet_common.launch.py の変更は fleet_name arg 追加のみ
- [ ] office demo は影響なし
