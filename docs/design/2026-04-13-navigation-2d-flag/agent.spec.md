# `navigation_2d` Flag — Agent Specification

## Requirements

- `KinematicController.__init__` に `navigation_2d: bool = True` パラメータ追加
- `AgentSpawnParams` に `navigation_2d: bool = True` フィールド追加
- Agent が controller を生成する際に config dict 経由でフラグを渡す
- Pose モード（TPI）のZ上書きを条件分岐化（3箇所）
- Velocity モードは変更なし

## Constraints

- `MotionMode` enum は変更しない
- `Pose` / `Path` クラスは変更しない
- デフォルト `True` で既存テスト全通過が必須

## Approach

controller.py 内の3箇所の `goal_pos[2] = current_pos[2]` を
`if self._navigation_2d:` ガードで囲む。最小変更。

## Design

### 変更箇所

| File | Location | Change |
|------|----------|--------|
| `pybullet_fleet/controller.py` L155付近 | `KinematicController.__init__` | `self._navigation_2d = navigation_2d` 追加 |
| `pybullet_fleet/controller.py` L583 | `_handle_waypoint_reached` | `if self._navigation_2d:` で囲む |
| `pybullet_fleet/controller.py` L764 | `OmniController._init_pose_trajectory` | `if self._navigation_2d:` で囲む |
| `pybullet_fleet/controller.py` L891 | `DifferentialController._init_pose_trajectory` | `if self._navigation_2d:` で囲む |
| `pybullet_fleet/agent.py` | `AgentSpawnParams` | `navigation_2d: bool = True` 追加 |
| `pybullet_fleet/agent.py` | controller config 生成部分 | フラグを config dict に含める |

### コードパターン

```python
# controller.py — KinematicController.__init__ に追加
def __init__(self, ..., navigation_2d: bool = True) -> None:
    ...
    self._navigation_2d: bool = navigation_2d

# 3箇所すべて同じパターン:
# Before:
goal_pos[2] = current_pos[2]
# After:
if self._navigation_2d:
    goal_pos[2] = current_pos[2]
```

## File References

- `pybullet_fleet/controller.py` — 全変更箇所
- `pybullet_fleet/agent.py:40-80` — AgentSpawnParams dataclass
- `pybullet_fleet/agent.py` — `from_params` での controller 生成
- `tests/test_controller.py` — 既存テスト（変更不要だが確認必要）
- `examples/mobile/path_following_demo.py` — 3D パスのデモ例

## Success Criteria

- [ ] 既存テスト全通過（`make test`）
- [ ] 新テスト `test_omni_3d_path`: Omni + `navigation_2d=False` でZ到達確認
- [ ] 新テスト `test_diff_3d_path`: Differential + `navigation_2d=False` でZ到達確認
- [ ] 新テスト `test_2d_preserves_z`: `navigation_2d=True` でZ保持確認
- [ ] `make verify` 通過
