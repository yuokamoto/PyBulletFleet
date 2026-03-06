# core_simulation.py テスト計画

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** `core_simulation.py` の包括的テストスイートを構築する。既存の collision テストをリファクタリングし、オブジェクトライフサイクル・ステップ実行・コールバック・空間ハッシュの単体テストを追加し、デモシナリオに基づくE2Eテストを作成する。

**Architecture:** 3 段階で進める。(1) 既存のスタンドアロンスクリプト 5 ファイルを整理し、pytest 互換の `test_collision_comprehensive.py` に統合する。(2) `test_core_simulation.py` を新規作成し、`add_object`/`remove_object`/`step_once` 等の単体テストを書く。(3) `test_e2e.py` を新規作成し、Pick→Move→Drop ワークフローやマルチエージェント衝突検知の E2E テストを書く。

**Tech Stack:** Python 3.8+, pytest, PyBullet (DIRECT mode), numpy

---

## 現状分析

### テスト済み（カバレッジあり）

| メソッド | テストファイル | 品質 |
|---|---|---|
| `check_collisions()` | test_collision_comprehensive.py | Good — 32 モード組合せ、margin 検出 |
| `SimulationParams` auto-selection | test_collision_comprehensive.py | Good |
| 空間ハッシュ multi-cell | test_multi_cell_*.py (3ファイル) | **pytest非互換** — `__main__` スクリプト |

### 未テスト（主要ギャップ）

| メソッド / 機能 | 備考 |
|---|---|
| `add_object()` / `remove_object()` | MockSimCore 経由のみ。実装未テスト |
| `step_once()` | 1 件の skip テストのみ。ループ未テスト |
| `_mark_object_moved()` → 空間ハッシュ更新 | MockSimCore の no-op のみ |
| コールバックシステム | `register_callback` の頻度制御未テスト |
| `from_yaml()` / `from_dict()` | 未テスト |
| `ignore_static_collision` フラグ | 部分的（手動挿入のみ） |
| 実行時 collision mode 変更 | SimObject 側のみテスト済み |
| E2E ワークフロー（Move→Pick→Drop） | 未テスト |

### 削除候補ファイル

| ファイル | 理由 |
|---|---|
| `tests/test_collision_optimization.py` (82行) | pytest 非互換。リスト内包表記のマイクロベンチマーク。core_simulation 無関係 |
| `tests/test_getaabb_performance.py` (52行) | pytest 非互換。`p.getAABB()` の計測スクリプト。core_simulation 無関係 |
| `tests/test_multi_cell_internal.py` (134行) | pytest 非互換。内容を test_collision_comprehensive.py に統合 |
| `tests/test_multi_cell_registration.py` (275行) | pytest 非互換。内容を test_collision_comprehensive.py に統合 |
| `tests/test_multi_cell_simple.py` (163行) | pytest 非互換。内容を test_collision_comprehensive.py に統合 |

---

## Task 1: スタンドアロンスクリプト削除 (SERIAL)

**Files:**
- Delete: `tests/test_collision_optimization.py`
- Delete: `tests/test_getaabb_performance.py`

これらは core_simulation.py と無関係なマイクロベンチマークであり、`benchmark/` ディレクトリに既に類似スクリプトがある。

**Step 1: テストスイート全体を実行して現状を確認**

```bash
python -m pytest tests/ -q --tb=short 2>&1 | tail -5
```

Expected: 全テスト PASS（既存スイートが壊れていないことを確認）

**Step 2: 2 ファイルを削除**

```bash
rm tests/test_collision_optimization.py tests/test_getaabb_performance.py
```

**Step 3: テスト再実行して影響なしを確認**

```bash
python -m pytest tests/ -q --tb=short 2>&1 | tail -5
```

Expected: PASS（削除ファイルは pytest で収集されないため影響ゼロ）

**Step 4: コミット**

```bash
git add -A && git commit -m "chore: remove standalone benchmark scripts from tests/"
```

---

## Task 2: multi-cell スクリプトを pytest テストとしてリファクタリング (SERIAL)

**Files:**
- Delete: `tests/test_multi_cell_internal.py`
- Delete: `tests/test_multi_cell_registration.py`
- Delete: `tests/test_multi_cell_simple.py`
- Modify: `tests/test_collision_comprehensive.py`

3 つのスタンドアロンスクリプトから有用なテストケースを抽出し、`test_collision_comprehensive.py` に pytest クラスとして統合する。低レベル PyBullet API (`p.createCollisionShape` 等) の直接使用は `SimObject.from_mesh()` に置き換える。

**Step 1: test_collision_comprehensive.py に `TestMultiCellRegistration` クラスを追加**

`test_collision_comprehensive.py` の末尾（`if __name__ == "__main__"` の前）に以下を追加:

```python
# ============================================================================
# Category 6: Multi-cell Registration
# ============================================================================


class TestMultiCellRegistration:
    """Test spatial hash multi-cell registration for large objects."""

    @pytest.fixture
    def sim_core_multicell(self):
        """Simulation with small cell size to trigger multi-cell registration."""
        params = SimulationParams(
            gui=False,
            physics=False,
            monitor=False,
            collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
            collision_margin=0.02,
            spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
            spatial_hash_cell_size=2.0,
            multi_cell_threshold=1.5,
            ignore_static_collision=False,
            log_level="warning",
        )
        sim_core = MultiRobotSimulationCore(params)
        sim_core.set_collision_spatial_hash_cell_size_mode()
        yield sim_core

    def test_small_object_single_cell(self, sim_core_multicell):
        """Small objects (< threshold * cell_size) use single cell."""
        obj = create_test_box(
            sim_core_multicell, [0, 0, 0], size=0.25,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        cells = sim_core_multicell._cached_object_to_cell.get(obj.object_id, [])
        assert len(cells) == 1, f"Small object should use 1 cell, got {len(cells)}"

    def test_large_object_multi_cell(self, sim_core_multicell):
        """Large objects (>= threshold * cell_size) use multiple cells."""
        obj = create_test_box(
            sim_core_multicell, [0, 0, 0], size=2.5,
            collision_mode=CollisionMode.STATIC,
        )
        cells = sim_core_multicell._cached_object_to_cell.get(obj.object_id, [])
        assert len(cells) > 1, f"Large object (5m) should use multiple cells, got {len(cells)}"

    def test_should_use_multi_cell_registration(self, sim_core_multicell):
        """_should_use_multi_cell_registration returns correct bool."""
        small = create_test_box(
            sim_core_multicell, [0, 0, 0], size=0.25,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        large = create_test_box(
            sim_core_multicell, [5, 0, 0], size=2.5,
            collision_mode=CollisionMode.STATIC,
        )
        assert not sim_core_multicell._should_use_multi_cell_registration(small.body_id)
        assert sim_core_multicell._should_use_multi_cell_registration(large.body_id)

    def test_get_overlapping_cells(self, sim_core_multicell):
        """_get_overlapping_cells returns cells covering object AABB."""
        large = create_test_box(
            sim_core_multicell, [0, 0, 0], size=2.5,
            collision_mode=CollisionMode.STATIC,
        )
        cells = sim_core_multicell._get_overlapping_cells(large.body_id)
        # 5m object with 2m cells → should span at least 3x3 = 9 cells in XY
        assert len(cells) >= 4, f"Expected ≥4 overlapping cells, got {len(cells)}"

    def test_large_wall_collision_with_small_robot(self, sim_core_multicell):
        """Small robot collides with large wall via multi-cell lookup."""
        wall = create_test_box(
            sim_core_multicell, [0, 0, 0], size=2.5,
            collision_mode=CollisionMode.STATIC,
        )
        robot = create_test_box(
            sim_core_multicell, [2.3, 2.3, 0], size=0.25,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        # Mark robot as moved so check_collisions considers it
        sim_core_multicell._moved_this_step.add(robot.object_id)
        assert_collision_detected(
            sim_core_multicell, wall, robot, should_collide=True,
            message="Small robot near large wall edge should collide",
        )

    def test_threshold_affects_registration(self):
        """Lower threshold causes medium objects to use multi-cell."""
        params = SimulationParams(
            gui=False, physics=False, monitor=False,
            collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
            spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
            spatial_hash_cell_size=2.0,
            multi_cell_threshold=1.2,  # Lower threshold → 2.4m
            ignore_static_collision=False, log_level="warning",
        )
        sim_core = MultiRobotSimulationCore(params)
        sim_core.set_collision_spatial_hash_cell_size_mode()

        # 2.5m object: should be single-cell at 1.5x (3.0m) but multi-cell at 1.2x (2.4m)
        obj = create_test_box(sim_core, [0, 0, 0], size=1.25, collision_mode=CollisionMode.NORMAL_3D)
        cells = sim_core._cached_object_to_cell.get(obj.object_id, [])
        assert len(cells) > 1, f"Medium object with low threshold should use multi-cell, got {len(cells)}"
```

**Step 2: テスト実行して PASS を確認**

```bash
python -m pytest tests/test_collision_comprehensive.py -v -k "MultiCell" --tb=short
```

Expected: 7 tests PASS

**Step 3: スタンドアロンスクリプトを削除**

```bash
rm tests/test_multi_cell_internal.py tests/test_multi_cell_registration.py tests/test_multi_cell_simple.py
```

**Step 4: 全テスト実行**

```bash
python -m pytest tests/ -q --tb=short 2>&1 | tail -5
```

Expected: 全 PASS

**Step 5: コミット**

```bash
git add -A && git commit -m "refactor: migrate multi-cell tests to pytest in test_collision_comprehensive"
```

---

## Task 3: collision テストに Movement Detection / Runtime Mode Change カテゴリを追加 (SERIAL)

**Files:**
- Modify: `tests/test_collision_comprehensive.py`

既存テストは static な配置のみ。オブジェクト移動後の衝突検出と、実行時の collision mode 変更をテストする。

**Step 1: `TestMovementDetection` クラスを追加**

```python
# ============================================================================
# Category 7: Movement Detection
# ============================================================================


class TestMovementDetection:
    """Test that moved objects are correctly re-evaluated for collisions."""

    def test_move_into_collision(self, sim_core_kinematics):
        """Object moved into collision range is detected."""
        obj1 = create_test_box(sim_core_kinematics, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        obj2 = create_test_box(sim_core_kinematics, [5, 0, 0], collision_mode=CollisionMode.NORMAL_3D)

        # Initially separated → no collision
        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=False)

        # Move obj2 next to obj1
        obj2.set_pose(Pose.from_xyz(0.5, 0, 0))
        sim_core_kinematics._mark_object_moved(obj2.object_id)
        sim_core_kinematics._update_object_aabb(obj2.body_id, update_grid=True)
        sim_core_kinematics._moved_this_step.add(obj2.object_id)

        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=True)

    def test_move_out_of_collision(self, sim_core_kinematics):
        """Object moved away from collision range is no longer detected."""
        obj1 = create_test_box(sim_core_kinematics, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        obj2 = create_test_box(sim_core_kinematics, [0.5, 0, 0], collision_mode=CollisionMode.NORMAL_3D)

        # Initially overlapping → collision
        sim_core_kinematics._moved_this_step.add(obj2.object_id)
        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=True)

        # Move obj2 far away
        obj2.set_pose(Pose.from_xyz(10, 0, 0))
        sim_core_kinematics._mark_object_moved(obj2.object_id)
        sim_core_kinematics._update_object_aabb(obj2.body_id, update_grid=True)
        sim_core_kinematics._moved_this_step.add(obj2.object_id)

        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=False)


# ============================================================================
# Category 8: Runtime Mode Changes
# ============================================================================


class TestRuntimeModeChanges:
    """Test collision mode changes at runtime."""

    def test_disable_collision_at_runtime(self, sim_core_kinematics):
        """Disabling collision mode removes object from detection."""
        obj1 = create_test_box(sim_core_kinematics, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        obj2 = create_test_box(sim_core_kinematics, [0.5, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        sim_core_kinematics._moved_this_step.add(obj2.object_id)

        # Initially colliding
        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=True)

        # Disable obj2 collision
        obj2.set_collision_mode(CollisionMode.DISABLED)
        sim_core_kinematics._update_object_collision_mode(obj2)

        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=False)

    def test_enable_collision_at_runtime(self, sim_core_kinematics):
        """Re-enabling collision mode re-adds object to detection."""
        obj1 = create_test_box(sim_core_kinematics, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        obj2 = create_test_box(sim_core_kinematics, [0.5, 0, 0], collision_mode=CollisionMode.DISABLED)

        sim_core_kinematics._moved_this_step.add(obj1.object_id)

        # Initially disabled → no collision
        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=False)

        # Enable obj2
        obj2.set_collision_mode(CollisionMode.NORMAL_3D)
        sim_core_kinematics._update_object_collision_mode(obj2)
        sim_core_kinematics._moved_this_step.add(obj2.object_id)

        assert_collision_detected(sim_core_kinematics, obj1, obj2, should_collide=True)
```

**Step 2: テスト実行**

```bash
python -m pytest tests/test_collision_comprehensive.py -v -k "Movement or RuntimeMode" --tb=short
```

Expected: 4 tests PASS

**Step 3: コミット**

```bash
git add tests/test_collision_comprehensive.py && git commit -m "test: add movement detection and runtime mode change tests"
```

---

## Task 4: `test_core_simulation.py` 新規作成 — オブジェクトライフサイクル (SERIAL)

**Files:**
- Create: `tests/test_core_simulation.py`

`add_object()` / `remove_object()` の実装を real `MultiRobotSimulationCore` で テストする。MockSimCore ではなく実際のキャッシュ更新を検証する。

**Step 1: ファイル作成 — テストクラス `TestObjectLifecycle`**

```python
"""
Unit tests for MultiRobotSimulationCore.

Tests object lifecycle, simulation step, callback system, and configuration
using a real PyBullet environment (DIRECT mode).
"""

import pytest
import numpy as np
import pybullet as p

from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import SimObject, ShapeParams
from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.geometry import Pose
from pybullet_fleet.types import (
    CollisionMode,
    CollisionDetectionMethod,
    SpatialHashCellSizeMode,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def sim_core():
    """Headless kinematics sim with collision enabled."""
    params = SimulationParams(
        gui=False,
        physics=False,
        monitor=False,
        collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
        collision_margin=0.02,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.0,
        ignore_static_collision=False,
        log_level="warning",
    )
    sc = MultiRobotSimulationCore(params)
    sc.set_collision_spatial_hash_cell_size_mode()
    return sc


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def make_box(sim_core, position, size=0.25, collision_mode=CollisionMode.NORMAL_3D, mass=0.0):
    """Create a box SimObject registered to sim_core."""
    return SimObject.from_mesh(
        visual_shape=ShapeParams(shape_type="box", half_extents=[size, size, size]),
        collision_shape=ShapeParams(shape_type="box", half_extents=[size, size, size]),
        pose=Pose.from_xyz(*position),
        mass=mass,
        sim_core=sim_core,
        collision_mode=collision_mode,
    )


def make_agent(sim_core, position=(0, 0, 0)):
    """Create an Agent registered to sim_core."""
    return Agent.from_params(
        AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(*position),
            collision_mode=CollisionMode.NORMAL_3D,
        ),
        sim_core=sim_core,
    )


# ============================================================================
# Object Lifecycle
# ============================================================================


class TestObjectLifecycle:
    """Test add_object / remove_object with real sim_core caches."""

    def test_add_object_registers_in_dict(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0])
        assert obj.object_id in sim_core._sim_objects_dict
        assert obj in sim_core.sim_objects

    def test_add_object_sets_collision_mode_cache(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        assert sim_core._cached_collision_modes[obj.object_id] == CollisionMode.NORMAL_3D

    def test_add_object_registers_aabb(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0])
        assert obj.object_id in sim_core._cached_aabbs_dict

    def test_add_object_registers_spatial_grid_cell(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0])
        assert obj.object_id in sim_core._cached_object_to_cell
        cells = sim_core._cached_object_to_cell[obj.object_id]
        assert len(cells) >= 1

    def test_add_agent_registers_in_agents_list(self, sim_core):
        agent = make_agent(sim_core)
        assert agent in sim_core.agents
        assert agent in sim_core.sim_objects

    def test_add_disabled_object_skips_collision_system(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.DISABLED)
        assert obj.object_id in sim_core._disabled_collision_objects
        assert obj.object_id not in sim_core._cached_aabbs_dict

    def test_add_static_object_registers_in_static_set(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.STATIC)
        assert obj.object_id in sim_core._static_objects
        assert obj.object_id not in sim_core._cached_non_static_dict

    def test_add_normal_object_registers_in_non_static_dict(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        assert obj.object_id in sim_core._cached_non_static_dict

    def test_add_duplicate_is_noop(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0])
        count_before = len(sim_core.sim_objects)
        sim_core.add_object(obj)  # duplicate
        assert len(sim_core.sim_objects) == count_before

    def test_remove_object_clears_all_caches(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0])
        obj_id = obj.object_id
        sim_core.remove_object(obj)

        assert obj_id not in sim_core._sim_objects_dict
        assert obj not in sim_core.sim_objects
        assert obj_id not in sim_core._cached_collision_modes
        assert obj_id not in sim_core._cached_aabbs_dict
        assert obj_id not in sim_core._cached_object_to_cell
        assert obj_id not in sim_core._cached_non_static_dict

    def test_remove_agent_clears_agents_list(self, sim_core):
        agent = make_agent(sim_core)
        sim_core.remove_object(agent)
        assert agent not in sim_core.agents
        assert agent not in sim_core.sim_objects

    def test_remove_nonexistent_is_noop(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0])
        sim_core.remove_object(obj)
        # Second remove should not raise
        sim_core.remove_object(obj)

    def test_remove_clears_active_collision_pairs(self, sim_core):
        obj1 = make_box(sim_core, [0, 0, 0])
        obj2 = make_box(sim_core, [0.3, 0, 0])
        sim_core._moved_this_step.add(obj2.object_id)
        sim_core.check_collisions()

        sim_core.remove_object(obj2)
        # No pair should reference removed object
        for pair in sim_core._active_collision_pairs:
            assert obj2.object_id not in pair
```

**Step 2: テスト実行**

```bash
python -m pytest tests/test_core_simulation.py -v -k "TestObjectLifecycle" --tb=short
```

Expected: 13 tests PASS

**Step 3: コミット**

```bash
git add tests/test_core_simulation.py && git commit -m "test: add object lifecycle tests for core_simulation"
```

---

## Task 5: `test_core_simulation.py` — SimulationParams / 初期化テスト (PARALLEL with Task 6)

**Files:**
- Modify: `tests/test_core_simulation.py`

**Step 1: `TestSimulationParams` と `TestInitialization` クラスを追加**

```python
# ============================================================================
# SimulationParams
# ============================================================================


class TestSimulationParams:
    """Test SimulationParams construction and from_dict."""

    def test_defaults(self):
        params = SimulationParams(gui=False, monitor=False)
        assert params.physics is False
        assert params.timestep == pytest.approx(1.0 / 240.0)
        assert params.collision_margin == 0.02
        assert params.multi_cell_threshold == 1.5

    def test_auto_select_closest_points_for_kinematics(self):
        params = SimulationParams(gui=False, physics=False, monitor=False)
        assert params.collision_detection_method == CollisionDetectionMethod.CLOSEST_POINTS

    def test_auto_select_contact_points_for_physics(self):
        params = SimulationParams(gui=False, physics=True, monitor=False)
        assert params.collision_detection_method == CollisionDetectionMethod.CONTACT_POINTS

    def test_explicit_method_overrides_auto(self):
        params = SimulationParams(
            gui=False, physics=False, monitor=False,
            collision_detection_method=CollisionDetectionMethod.HYBRID,
        )
        assert params.collision_detection_method == CollisionDetectionMethod.HYBRID

    def test_from_dict(self):
        cfg = {
            "gui": False, "physics": True, "monitor": False,
            "timestep": 0.01, "collision_margin": 0.05,
        }
        params = SimulationParams.from_dict(cfg)
        assert params.physics is True
        assert params.timestep == 0.01
        assert params.collision_margin == 0.05

    def test_from_yaml(self, tmp_path):
        import yaml
        cfg = {
            "gui": False, "physics": False, "monitor": False,
            "timestep": 0.05, "speed": 2.0,
        }
        yaml_file = tmp_path / "test_config.yaml"
        yaml_file.write_text(yaml.dump(cfg))
        params = SimulationParams.from_config(str(yaml_file))
        assert params.timestep == 0.05
        assert params.speed == 2.0


# ============================================================================
# Initialization
# ============================================================================


class TestInitialization:
    """Test MultiRobotSimulationCore initialization."""

    def test_pybullet_connected(self, sim_core):
        assert sim_core.client is not None
        assert p.isConnected(physicsClientId=sim_core.client)

    def test_empty_initial_state(self, sim_core):
        assert len(sim_core.sim_objects) == 0
        assert len(sim_core.agents) == 0
        assert sim_core.step_count == 0
        assert sim_core.sim_time == 0.0

    def test_from_dict(self):
        cfg = {"gui": False, "physics": False, "monitor": False}
        sc = MultiRobotSimulationCore.from_dict(cfg)
        assert sc.params.gui is False
        assert p.isConnected(physicsClientId=sc.client)
```

**Step 2: テスト実行**

```bash
python -m pytest tests/test_core_simulation.py -v -k "TestSimulationParams or TestInitialization" --tb=short
```

Expected: 10 tests PASS

**Step 3: コミット**

```bash
git add tests/test_core_simulation.py && git commit -m "test: add SimulationParams and initialization tests"
```

---

## Task 6: `test_core_simulation.py` — step_once / コールバックテスト (PARALLEL with Task 5)

**Files:**
- Modify: `tests/test_core_simulation.py`

**Step 1: `TestStepOnce` と `TestCallbackSystem` クラスを追加**

```python
# ============================================================================
# step_once
# ============================================================================


class TestStepOnce:
    """Test step_once execution."""

    def test_step_increments_count(self, sim_core):
        sim_core.step_once()
        assert sim_core.step_count == 1

    def test_step_advances_sim_time(self, sim_core):
        dt = sim_core.params.timestep
        sim_core.step_once()
        assert sim_core.sim_time == pytest.approx(dt, abs=1e-9)

    def test_step_updates_agent(self, sim_core):
        agent = make_agent(sim_core, position=(0, 0, 0))
        goal = Pose.from_xyz(5, 0, 0)
        agent.set_goal_pose(goal)

        for _ in range(10):
            sim_core.step_once()

        pos = agent.get_pose().position
        # Agent should have moved towards goal
        assert pos[0] > 0.0, "Agent should move towards goal"

    def test_step_return_profiling(self, sim_core):
        result = sim_core.step_once(return_profiling=True)
        assert result is not None
        assert "total" in result
        assert "agent_update" in result
        assert "collision_check" in result
        assert all(v >= 0 for v in result.values())

    def test_multiple_steps_accumulate(self, sim_core):
        n = 50
        dt = sim_core.params.timestep
        for _ in range(n):
            sim_core.step_once()
        assert sim_core.step_count == n
        assert sim_core.sim_time == pytest.approx(n * dt, abs=1e-6)

    def test_step_triggers_collision_check(self, sim_core):
        obj1 = make_box(sim_core, [0, 0, 0])
        obj2 = make_box(sim_core, [0.3, 0, 0])
        sim_core._moved_this_step.add(obj1.object_id)

        sim_core.step_once()

        # After step, active_collision_pairs should be populated
        pairs = sim_core.get_active_collision_pairs()
        expected = (
            min(obj1.object_id, obj2.object_id),
            max(obj1.object_id, obj2.object_id),
        )
        assert expected in pairs


# ============================================================================
# Callback System
# ============================================================================


class TestCallbackSystem:
    """Test register_callback and execution."""

    def test_callback_called_every_step(self, sim_core):
        calls = []
        sim_core.register_callback(lambda sc, dt: calls.append(dt), frequency=None)
        for _ in range(5):
            sim_core.step_once()
        assert len(calls) == 5

    def test_callback_frequency_control(self, sim_core):
        """Callback at 1 Hz should fire ~once per second of sim_time."""
        calls = []
        sim_core.register_callback(lambda sc, dt: calls.append(sc.sim_time), frequency=1.0)

        dt = sim_core.params.timestep
        # Run 2 seconds of sim time
        steps = int(2.0 / dt)
        for _ in range(steps):
            sim_core.step_once()

        # Should fire approximately 2 times (at t≈1.0 and t≈2.0)
        assert 1 <= len(calls) <= 3, f"Expected ~2 calls at 1Hz over 2s, got {len(calls)}"

    def test_multiple_callbacks(self, sim_core):
        calls_a, calls_b = [], []
        sim_core.register_callback(lambda sc, dt: calls_a.append(1), frequency=None)
        sim_core.register_callback(lambda sc, dt: calls_b.append(1), frequency=None)
        sim_core.step_once()
        assert len(calls_a) == 1
        assert len(calls_b) == 1
```

**Step 2: テスト実行**

```bash
python -m pytest tests/test_core_simulation.py -v -k "TestStepOnce or TestCallbackSystem" --tb=short
```

Expected: 9 tests PASS

**Step 3: コミット**

```bash
git add tests/test_core_simulation.py && git commit -m "test: add step_once and callback system tests"
```

---

## Task 7: `test_core_simulation.py` — ignore_static_collision / collision_check_frequency (SERIAL)

**Files:**
- Modify: `tests/test_core_simulation.py`

**Step 1: `TestIgnoreStaticCollision` と `TestCollisionCheckFrequency` を追加**

```python
# ============================================================================
# ignore_static_collision
# ============================================================================


class TestIgnoreStaticCollision:
    """Test ignore_static_collision flag behaviour."""

    @pytest.fixture
    def sim_ignore_static(self):
        params = SimulationParams(
            gui=False, physics=False, monitor=False,
            collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
            spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
            spatial_hash_cell_size=2.0,
            ignore_static_collision=True,
            log_level="warning",
        )
        sc = MultiRobotSimulationCore(params)
        sc.set_collision_spatial_hash_cell_size_mode()
        return sc

    def test_static_collision_ignored(self, sim_ignore_static):
        static = make_box(sim_ignore_static, [0, 0, 0], collision_mode=CollisionMode.STATIC)
        normal = make_box(sim_ignore_static, [0.3, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        sim_ignore_static._moved_this_step.add(normal.object_id)

        pairs, _ = sim_ignore_static.check_collisions()
        pair = (min(static.object_id, normal.object_id), max(static.object_id, normal.object_id))
        assert pair not in pairs, "Static collision should be ignored"

    def test_static_collision_detected_when_not_ignored(self, sim_core):
        static = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.STATIC)
        normal = make_box(sim_core, [0.3, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        sim_core._moved_this_step.add(normal.object_id)

        pairs, _ = sim_core.check_collisions()
        pair = (min(static.object_id, normal.object_id), max(static.object_id, normal.object_id))
        assert pair in pairs, "Static collision should be detected when ignore_static=False"


# ============================================================================
# Collision Check Frequency
# ============================================================================


class TestCollisionCheckFrequency:
    """Test collision_check_frequency parameter."""

    def test_frequency_zero_skips_checks(self):
        params = SimulationParams(
            gui=False, physics=False, monitor=False,
            collision_check_frequency=0,
            log_level="warning",
        )
        sc = MultiRobotSimulationCore(params)
        obj1 = make_box(sc, [0, 0, 0])
        obj2 = make_box(sc, [0.3, 0, 0])
        sc._moved_this_step.add(obj1.object_id)

        sc.step_once()

        # With frequency=0, collision check is skipped
        assert len(sc.get_active_collision_pairs()) == 0

    def test_frequency_none_checks_every_step(self, sim_core):
        obj1 = make_box(sim_core, [0, 0, 0])
        obj2 = make_box(sim_core, [0.3, 0, 0])
        sim_core._moved_this_step.add(obj1.object_id)

        sim_core.step_once()
        assert len(sim_core.get_active_collision_pairs()) > 0
```

**Step 2: テスト実行**

```bash
python -m pytest tests/test_core_simulation.py -v -k "IgnoreStatic or CollisionCheckFrequency" --tb=short
```

Expected: 4 tests PASS

**Step 3: コミット**

```bash
git add tests/test_core_simulation.py && git commit -m "test: add ignore_static and collision check frequency tests"
```

---

## Task 8: `test_e2e.py` 新規作成 — E2E ワークフローテスト (SERIAL)

**Files:**
- Create: `tests/test_e2e.py`

デモシナリオに基づく E2E テスト。実際の `MultiRobotSimulationCore` + `Agent` + `Action` で `step_once()` ループを回し、最終状態を検証する。

**Step 1: ファイル作成**

```python
"""
End-to-end tests for PyBulletFleet simulation workflows.

Each test runs a complete scenario through MultiRobotSimulationCore.step_once(),
verifying final state after action sequences complete.

Based on examples/:
- action_system_demo.py (Pick → Drop → Move → Wait)
- pick_drop_mobile_100robots_demo.py (bulk Pick → Move → Drop)
- path_following_demo.py (waypoint path following)
- collision_features_demo.py (multi-mode collision detection)
"""

import numpy as np
import pytest

from pybullet_fleet.action import (
    DropAction,
    MoveAction,
    PickAction,
    WaitAction,
)
from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Path, Pose
from pybullet_fleet.sim_object import ShapeParams, SimObject, SimObjectSpawnParams
from pybullet_fleet.types import (
    CollisionDetectionMethod,
    CollisionMode,
    MotionMode,
    SpatialHashCellSizeMode,
)

TOL = 0.15  # E2E position tolerance (generous — accounts for kinematic stepping)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def sim():
    """Headless kinematics sim (fast timestep for quick convergence)."""
    params = SimulationParams(
        gui=False,
        physics=False,
        monitor=False,
        timestep=0.1,
        speed=0,  # max speed (no sleep)
        collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.0,
        ignore_static_collision=True,
        log_level="warning",
    )
    sc = MultiRobotSimulationCore(params)
    sc.set_collision_spatial_hash_cell_size_mode()
    return sc


def make_mobile_agent(sim, position=(0, 0, 0), motion_mode=MotionMode.OMNIDIRECTIONAL):
    """Spawn a mobile robot agent."""
    return Agent.from_params(
        AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(*position),
            collision_mode=CollisionMode.NORMAL_3D,
            motion_mode=motion_mode,
        ),
        sim_core=sim,
    )


def make_pickable_box(sim, position=(2, 0, 0)):
    """Spawn a pickable box object."""
    return SimObject.from_params(
        SimObjectSpawnParams(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            initial_pose=Pose.from_xyz(*position),
            mass=0.0,
            pickable=True,
        ),
        sim_core=sim,
    )


def run_until_done(sim, agents, *, max_steps=5000):
    """Step sim until all agent action queues are empty."""
    for step in range(max_steps):
        sim.step_once()
        if all(a.is_action_queue_empty() and not a.is_moving for a in agents):
            return step + 1
    raise AssertionError(f"Agents did not finish within {max_steps} steps")


# ============================================================================
# E2E: Pick → Move → Drop
# ============================================================================


class TestPickMoveDropE2E:
    """Full pick-move-drop workflow (based on action_system_demo.py)."""

    def test_pick_move_drop_single_agent(self, sim):
        agent = make_mobile_agent(sim, position=(0, 0, 0))
        box = make_pickable_box(sim, position=(2, 0, 0))

        drop_pose = Pose.from_xyz(5, 3, 0)

        agent.add_action_sequence([
            PickAction(
                target_object_id=box.object_id,
                use_approach=False,
                pick_offset=0.3,
                attach_relative_pose=Pose.from_xyz(0, 0, 0.3),
            ),
            MoveAction(
                path=Path.from_positions([drop_pose.position]),
                final_orientation_align=False,
            ),
            DropAction(
                drop_pose=drop_pose,
                use_approach=False,
                drop_offset=0.3,
            ),
        ])

        run_until_done(sim, [agent])

        # Box should be near drop position
        box_pos = np.array(box.get_pose().position)
        drop_pos = np.array(drop_pose.position)
        assert np.linalg.norm(box_pos[:2] - drop_pos[:2]) < TOL, (
            f"Box should be near drop pose. box={box_pos}, drop={drop_pos}"
        )

    def test_pick_move_drop_with_approach(self, sim):
        agent = make_mobile_agent(sim, position=(0, 0, 0))
        box = make_pickable_box(sim, position=(3, 0, 0))
        drop_pose = Pose.from_xyz(6, 4, 0)

        agent.add_action_sequence([
            PickAction(
                target_object_id=box.object_id,
                use_approach=True,
                approach_offset=1.0,
                pick_offset=0.3,
                attach_relative_pose=Pose.from_xyz(0, 0, 0.3),
            ),
            MoveAction(
                path=Path.from_positions([drop_pose.position]),
            ),
            DropAction(
                drop_pose=drop_pose,
                use_approach=True,
                approach_offset=1.0,
                drop_offset=0.3,
            ),
        ])

        run_until_done(sim, [agent])

        box_pos = np.array(box.get_pose().position)
        drop_pos = np.array(drop_pose.position)
        assert np.linalg.norm(box_pos[:2] - drop_pos[:2]) < TOL

    def test_move_then_wait(self, sim):
        agent = make_mobile_agent(sim, position=(0, 0, 0))
        goal = [4, 2, 0]

        agent.add_action_sequence([
            MoveAction(path=Path.from_positions([goal])),
            WaitAction(duration=0.5, action_type="idle"),
        ])

        steps = run_until_done(sim, [agent])

        pos = np.array(agent.get_pose().position)
        assert np.linalg.norm(pos[:2] - np.array(goal[:2])) < TOL
        assert steps > 0


# ============================================================================
# E2E: Path Following
# ============================================================================


class TestPathFollowingE2E:
    """Path following to completion (based on path_following_demo.py)."""

    def test_square_path(self, sim):
        agent = make_mobile_agent(sim, position=(0, 0, 0))

        waypoints = [[2, 0, 0], [2, 2, 0], [0, 2, 0], [0, 0, 0]]
        agent.add_action_sequence([
            MoveAction(path=Path.from_positions(waypoints)),
        ])

        run_until_done(sim, [agent])

        pos = np.array(agent.get_pose().position)
        assert np.linalg.norm(pos[:2]) < TOL, f"Agent should return to origin, got {pos}"

    def test_multi_agent_paths(self, sim):
        agents = [
            make_mobile_agent(sim, position=(i * 3, 0, 0))
            for i in range(3)
        ]

        for i, agent in enumerate(agents):
            goal = [i * 3, 5, 0]
            agent.add_action_sequence([
                MoveAction(path=Path.from_positions([goal])),
            ])

        run_until_done(sim, agents)

        for i, agent in enumerate(agents):
            pos = np.array(agent.get_pose().position)
            expected = np.array([i * 3, 5, 0])
            assert np.linalg.norm(pos[:2] - expected[:2]) < TOL


# ============================================================================
# E2E: Collision Detection During Movement
# ============================================================================


class TestCollisionDuringMovementE2E:
    """Collision detection while agents are moving (based on collision_features_demo.py)."""

    @pytest.fixture
    def sim_with_collision(self):
        params = SimulationParams(
            gui=False, physics=False, monitor=False,
            timestep=0.1,
            collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
            collision_margin=0.05,
            spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
            spatial_hash_cell_size=2.0,
            ignore_static_collision=False,
            log_level="warning",
        )
        sc = MultiRobotSimulationCore(params)
        sc.set_collision_spatial_hash_cell_size_mode()
        return sc

    def test_collision_detected_during_step(self, sim_with_collision):
        """Two agents converging should produce collision pairs at some step."""
        sc = sim_with_collision
        a1 = make_mobile_agent(sc, position=(0, 0, 0))
        a2 = make_mobile_agent(sc, position=(3, 0, 0))

        a1.set_goal_pose(Pose.from_xyz(3, 0, 0))
        a2.set_goal_pose(Pose.from_xyz(0, 0, 0))

        collision_ever = False
        for _ in range(500):
            sc.step_once()
            if len(sc.get_active_collision_pairs()) > 0:
                collision_ever = True
                break

        assert collision_ever, "Converging agents should collide at some point"

    def test_disabled_mode_no_collision(self, sim_with_collision):
        """Agent with DISABLED collision mode is never in collision pairs."""
        sc = sim_with_collision
        a1 = make_mobile_agent(sc, position=(0, 0, 0))
        a2 = Agent.from_params(
            AgentSpawnParams(
                urdf_path="robots/mobile_robot.urdf",
                initial_pose=Pose.from_xyz(0.3, 0, 0),
                collision_mode=CollisionMode.DISABLED,
            ),
            sim_core=sc,
        )
        sc._moved_this_step.add(a1.object_id)

        sc.step_once()
        pairs = sc.get_active_collision_pairs()
        ids = {obj_id for pair in pairs for obj_id in pair}
        assert a2.object_id not in ids, "DISABLED agent should never appear in collision pairs"


# ============================================================================
# E2E: Multi-Agent Bulk Operations
# ============================================================================


class TestMultiAgentBulkE2E:
    """Multi-agent scenarios (based on pick_drop_mobile_100robots_demo.py)."""

    def test_multiple_agents_pick_and_drop(self, sim):
        """3 agents each pick a box, move, and drop it."""
        num_agents = 3
        agents = []
        boxes = []

        for i in range(num_agents):
            agent = make_mobile_agent(sim, position=(i * 4, 0, 0))
            box = make_pickable_box(sim, position=(i * 4, 1, 0))
            agents.append(agent)
            boxes.append(box)

        drop_y = 5.0
        for i, (agent, box) in enumerate(zip(agents, boxes)):
            drop_pose = Pose.from_xyz(i * 4, drop_y, 0)
            agent.add_action_sequence([
                PickAction(
                    target_object_id=box.object_id,
                    use_approach=False,
                    pick_offset=0.3,
                    attach_relative_pose=Pose.from_xyz(0, 0, 0.3),
                ),
                MoveAction(path=Path.from_positions([drop_pose.position])),
                DropAction(drop_pose=drop_pose, use_approach=False, drop_offset=0.3),
            ])

        run_until_done(sim, agents, max_steps=10000)

        for i, box in enumerate(boxes):
            box_pos = np.array(box.get_pose().position)
            expected_y = drop_y
            assert abs(box_pos[1] - expected_y) < TOL, (
                f"Box {i} should be near y={expected_y}, got y={box_pos[1]}"
            )
```

**Step 2: テスト実行**

```bash
python -m pytest tests/test_e2e.py -v --tb=short
```

Expected: 8 tests PASS

**Step 3: コミット**

```bash
git add tests/test_e2e.py && git commit -m "test: add E2E workflow tests for core_simulation"
```

---

## Task 9: pre-commit / 最終検証 (SERIAL)

**Files:**
- All test files

**Step 1: 全テスト実行**

```bash
python -m pytest tests/ -q --tb=short
```

Expected: 全テスト PASS

**Step 2: pre-commit チェック**

```bash
pre-commit run --all-files
```

Expected: 全 PASS。flake8/black/pyright エラーがあれば修正。

**Step 3: 最終コミット**

```bash
git add -A && git commit -m "test: complete core_simulation test suite"
```

---

## タスク依存関係

| Task | 依存 | 並列可否 |
|---|---|---|
| Task 1: ベンチマークスクリプト削除 | なし | SERIAL (先頭) |
| Task 2: multi-cell pytest 化 | Task 1 | SERIAL |
| Task 3: MovementDetection / RuntimeModeChange | Task 2 | SERIAL |
| Task 4: ObjectLifecycle | Task 1 | SERIAL |
| Task 5: SimulationParams / 初期化 | Task 4 | **PARALLEL with Task 6** |
| Task 6: step_once / Callback | Task 4 | **PARALLEL with Task 5** |
| Task 7: ignore_static / frequency | Task 6 | SERIAL |
| Task 8: E2E テスト | Task 4 | SERIAL (Task 7 の後推奨) |
| Task 9: 最終検証 | 全タスク | SERIAL (最後) |

## 予想テスト数

| ファイル | 追加テスト数 |
|---|---|
| test_collision_comprehensive.py | +11 (multi-cell 7 + movement 2 + mode change 2) |
| test_core_simulation.py (新規) | +26 (lifecycle 13 + params 7 + step 6 + callback 3 + static 2 + freq 2) |
| test_e2e.py (新規) | +8 |
| **合計** | **+45** |
