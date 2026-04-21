# sim_time Acceleration — Agent Specification

## Requirements

- `pybullet_common.launch.py` に `use_sim_time` launch argument 追加
- 全ノードに `use_sim_time` パラメータを伝播
- `fleet_adapter` に `-sim` flag を渡す
- `door_adapter` / `workcell_adapter` に `use_sim_time` パラメータ追加
- デフォルトは `true` (高速化がメイン目的)

## Constraints

- `/clock` は bridge_node が step 毎に publish 済み (L204-206)
- `fleet_adapter.py` L91-96: 既に `-sim` argparse flag あり
- `fleet_adapter.py` L115-118: `-sim` 時に `adapter.node.use_sim_time()` 呼び出し
- `door_adapter.py` / `workcell_adapter.py`: `self.get_clock().now()` 使用 →
  `use_sim_time` ROS param を True にすれば自動的に `/clock` を使う
- rclpy の `use_sim_time` は Node パラメータ経由で設定:
  `parameters=[{"use_sim_time": True}]`

## Approach

launch.py → 全ノードに `use_sim_time` パラメータを追加。
fleet_adapter は `-sim` flag を追加で渡す (Open-RMF の独自処理)。

## Design

### pybullet_common.launch.py Changes

```python
# Add launch argument
DeclareLaunchArgument(
    "use_sim_time",
    default_value="true",
    description="Use simulation clock (/clock) for all nodes. Default true for acceleration.",
)

# Create parameter
use_sim_time = LaunchConfiguration("use_sim_time")

# Pass to all nodes
Node(
    package="pybullet_fleet_ros",
    executable="bridge_node",
    parameters=[{"use_sim_time": use_sim_time}],
    ...
)

Node(
    package="pybullet_fleet_ros",
    executable="door_adapter",
    parameters=[{"use_sim_time": use_sim_time}],
    ...
)

Node(
    package="pybullet_fleet_ros",
    executable="workcell_adapter",
    parameters=[{"use_sim_time": use_sim_time}],
    ...
)

# fleet_adapter needs -sim flag
Node(
    package="pybullet_fleet_ros",
    executable="fleet_adapter",
    arguments=[
        ...existing_args...,
        # Conditionally add -sim flag
        IfCondition(use_sim_time), "-sim",
    ],
    parameters=[{"use_sim_time": use_sim_time}],
    ...
)
```

### door_adapter.py Changes

```python
# No code change needed.
# rclpy.Node automatically uses /clock when
# the 'use_sim_time' parameter is True.
# self.get_clock().now() returns sim_time.
```

### workcell_adapter.py Changes

```python
# Same as door_adapter — no code change needed.
# The launch file passes the parameter.
```

### fleet_adapter.py Changes

```python
# Launch passes -sim flag via IfCondition.
# Existing code at L115-118 already handles it:
#   if args.use_sim_time:
#       adapter.node.use_sim_time()
```

### bridge_node.py Changes

```python
# Existing /clock publish is sufficient.
# No code change — launch passes use_sim_time param
# so bridge_node itself also uses sim_time internally.
```

## File References

- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/bridge_node.py:L204-L206` — /clock publish
- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/fleet_adapter.py:L91-L96` — `-sim` flag
- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/fleet_adapter.py:L115-L118` — use_sim_time()
- `ros2_bridge/pybullet_fleet_ros/launch/pybullet_common.launch.py` — launch file
- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/door_adapter.py` — door adapter
- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/workcell_adapter.py` — workcell adapter

## Success Criteria

- [ ] launch arg `use_sim_time:=true` で全ノードが sim_time 使用
- [ ] デフォルト (`use_sim_time:=true`) で全ノードが sim_time 使用
- [ ] `use_sim_time:=false` で wall clock モードに切替可能
- [ ] `ros2 topic echo /clock` で正しい sim_time が確認可能
- [ ] `target_rtf: 5.0` + `use_sim_time:=true` で加速動作
- [ ] `make verify` 通過 (pybullet_fleet 側)
