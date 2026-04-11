# SDF Loader — Design Spec

**Date:** 2026-04-09
**Status:** Draft
**Depends on:** EventBus (PR 1) — `object_spawned` イベントで自動登録
**Enables:** Open-RMF Fleet Adapter Example (PR 3)

## Context

PyBulletFleet は URDF (Agent) と primitive shapes / OBJ mesh (SimObject) をサポートしているが、
SDF (Simulation Description Format) には未対応。Roadmap に `SimObject.from_sdf()` が記載済み。

Open-RMF の `rmf_demos` ワールドを読み込むには SDF 対応が必要だが、PyBullet の SDF パーサには
`<world>`, `<include>`, `model://` URI の非対応という制約がある。

## Decision

**2層アプローチ:**

1. **`SimObject.from_sdf()`** — `p.loadSDF()` で個別 SDF モデルを読み込み、SimObject でラップ
2. **`load_rmf_world()`** — rmf_building_map_tools が生成した OBJ メッシュ群を読み込む環境ローダー

SDF `<world>` ファイルを直接パースする Level 2 (Gazebo Fuel 統合、`<include>` 解決等) は
スコープ外。rmf_demos 環境は OBJ メッシュ経由で読み込む。

### 根拠

| アプローチ | 工数 | rmf_demos 対応 | pybullet_data 対応 |
|---|---|---|---|
| `p.loadSDF()` wrapper のみ | 小 | ✗ (world 非対応) | ✓ (kiva_shelf 等) |
| SDF XML パーサ + Fuel 連携 | 大 | △ (plugin/DAE 制約) | ✓ |
| **OBJ mesh ローダー** | **小** | **✓** | — |
| **両方 (選択)** | **小+小** | **✓** | **✓** |

## Requirements

### 1. `SimObject.from_sdf()` — 個別 SDF モデル読み込み

```python
# pybullet_data の SDF モデル
shelves = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim)
# → List[SimObject] (body_id ごとに1つ)

# ローカル SDF ファイル
objects = SimObject.from_sdf("/path/to/custom_model.sdf", sim_core=sim, global_scaling=2.0)
```

- `p.loadSDF()` を呼び、返された各 body_id を SimObject でラップ
- `resolve_model()` と同じパス解決 (robots/, pybullet_data, search_path)
- `global_scaling` パラメータサポート
- 各 SimObject に自動命名 (SDF model name or fallback)
- `sim_core` に自動登録 (`add_object()` → `object_spawned` イベント)
- `List[SimObject]` を返す (SDF は複数 model を含みうる)

### 2. `load_rmf_world()` — rmf_demos 環境読み込みヘルパー

```python
from pybullet_fleet.world_loader import load_rmf_world

# rmf_building_map_tools が生成したディレクトリを指定
objects = load_rmf_world(
    mesh_dir="/path/to/rmf_demos/maps/office/meshes/",
    sim_core=sim,
    collision_mode=CollisionMode.STATIC,
)
# → 壁、床、家具が SimObject として配置される
```

- 指定ディレクトリ内の `*.obj` ファイルを走査
- 各 OBJ を `SimObject.from_mesh()` で読み込み
- すべて原点 `Pose(0,0,0)` に配置 — `rmf_building_map_tools` はワールド座標を頂点に焼き込むため位置指定不要
- すべて mass=0 (static) で配置
- collision_mode は STATIC or DISABLED を選択可能 (環境オブジェクトはロボットと衝突させるか)
- 返り値: `List[SimObject]`

### 3. パス解決の拡張

`robot_models.py` の `resolve_model()` は既に `.sdf` 拡張子を走査している。
これを `resolve_model()` にリネーム (or エイリアス追加) して、SDF 解決を公式サポート:

```python
# 現在
path = resolve_model("kiva_shelf")  # → .../kiva_shelf/model.sdf

# 拡張 (エイリアス)
path = resolve_model("kiva_shelf")  # 同じ。URDF/SDF 両方を解決
```

## Constraints

- PyBullet の `p.loadSDF()` 制約:
  - `<world>` タグ非対応 (model 直下のみ)
  - `<include>` / `model://` URI 非対応
  - DAE メッシュは visual のみ (collision 非対応)
- `from_sdf()` は `SimObject.__init__(body_id=...)` を使う — 既存の lifecycle 管理をそのまま活用
- OBJ メッシュは PyBullet が完全サポート (visual + collision)
- `load_rmf_world()` は事前に `rmf_building_map_tools` で OBJ 生成が必要 (PyBulletFleet 自身は変換しない)

## Out of Scope

- SDF `<world>` ファイルの直接パース
- `<include>` タグ解決 / `model://` URI 解決
- Gazebo Fuel モデルダウンロード
- DAE → OBJ 自動変換
- SDF `<plugin>` の PyBulletFleet 実装 (ドア、リフト等)
- `traffic-editor` / `building.yaml` のパース

## Implementation Approach

1. `pybullet_fleet/sim_object.py` に `from_sdf()` classmethod を追加
2. `pybullet_fleet/world_loader.py` を新規作成 (`load_rmf_world()`)
3. `pybullet_fleet/robot_models.py` に `resolve_model()` エイリアス追加
4. `pybullet_fleet/__init__.py` に export 追加
5. `tests/test_sdf_loader.py` を新規作成
6. `examples/basics/sdf_demo.py` で動作デモ

## Success Criteria

- [ ] `SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim)` で kiva_shelf が読み込める
- [ ] 返り値が `List[SimObject]` で、各オブジェクトが sim_core に登録済み
- [ ] `global_scaling` が正しく効く
- [ ] `load_rmf_world(mesh_dir, sim_core)` で OBJ 群が読み込める
- [ ] 読み込んだ環境オブジェクトの collision が正しく動作
- [ ] `object_spawned` イベントが各オブジェクトで発火 (EventBus 依存)
- [ ] `resolve_model("kiva_shelf")` で SDF パスが解決できる
- [ ] 全既存テスト修正なしで通る
- [ ] `make verify` 通過

## References

- [roadmap.md](../../../roadmap.md) — `SimObject.from_sdf()` の記載
- [plugin-architecture/spec.md](../plugin-architecture/spec.md) — World loader (Phase 4)
- [rmf_building_map_tools](https://github.com/open-rmf/rmf_traffic_editor) — building.yaml → OBJ/SDF 変換ツール
- [PyBullet SDF support](https://pybullet.org) — `p.loadSDF()` API
