# Roadmap

Planned additions and improvements for PyBulletFleet.
Items are grouped by category; ordering within a group does not imply priority.


## Assets

New robot and infrastructure models:

- **Physics Mobile Robot** — Wheeled robot driven by PyBullet physics (motor torques, friction, contact forces)
- **Physics Mobile Manipulator** — Physics-mode mobile manipulator with motor-driven base and arm
- **Conveyor / Elevator / Mobile Rack** — Warehouse infrastructure entities for material handling scenarios

## Features

- **Snapshot & Replay** — Full and delta snapshot serialization for logging, replay, and external synchronization ([USO](https://github.com/yuokamoto/Unified-Simulation-Orchestrator) integration)
- **Behavior tree integration** - Create agent behavior from behavior tree.

## Interfaces

External communication layers:

- **ROS 2** — Topic / service / action bridge for ROS 2 ecosystem integration
- **gRPC** — Language-agnostic RPC interface for orchestrators, WMS, and fleet managers

## Refactoring

- **Remove scipy dependency** — Currently only `scipy.spatial.transform.Rotation` is used (9 call sites for quat↔euler, quat↔matrix, relative rotation). Replace with PyBullet utilities + lightweight helpers in `geometry.py` to eliminate the ~150 MB transitive dependency. Low priority: no runtime performance impact, only install size.

- **Manual quaternion helpers in `geometry.py`** — `SlerpPrecomp` / `quat_slerp` で確立したパターン（scipy を避け、scalar math で手書き）を拡張し、以下のヘルパーを `geometry.py` に追加する。ホットパスでの scipy `Rotation` オブジェクト生成オーバーヘッドを排除するのが目的。

  必要なヘルパー関数:
  - `quat_rotate_vector(q, v)` — クォータニオン `q` でベクトル `v` を回転（body→world 変換）。`Rotation.from_quat(q).apply(v)` の置換
  - `quat_multiply(q1, q2)` — クォータニオン積。`(Rotation.from_quat(q1) * Rotation.from_quat(q2)).as_quat()` の置換
  - `quat_from_rotvec(rotvec)` — 回転ベクトルからクォータニオン生成。`Rotation.from_rotvec(rotvec).as_quat()` の置換（角速度→姿勢更新で使用）

  主な適用箇所:
  - `OmniVelocityController._apply_velocity()` — body→world 速度変換 + 角速度によるクォータニオン更新
  - `tools.body_to_world_velocity_3d()` — 同上
  - `Path._calculate_orientation_for_plane()` — 回転行列→クォータニオン変換
  - `Path.visualize_waypoints()` — クォータニオン→回転行列変換

  設計方針:
  - pure Python scalar math (`math.sin`/`math.cos`) or small numpy array ops
  - `SlerpPrecomp` と同じファイル・同じスタイルで実装
  - プロファイリングでボトルネック確認後に段階的に適用（YAGNI）
  - scipy 完全除去は全 call site 置換後に別 PR で実施

## CI / DevOps

- **GitHub Actions refactoring** — Streamlined CI pipeline
- **Automated performance tracking** — Run time / memory benchmarks in CI, auto-update results in documentation, and alert on significant performance regressions

## Long-Term: Backend Abstraction & Beyond PyBullet

Current architecture is tightly coupled to PyBullet (`body_id`, per-entity FFI calls). At 1000 agents PyBullet's per-call overhead dominates step time (88% of 40.9 ms). The following items explore decoupling from PyBullet to unlock 10–100× performance gains while keeping the Python user API unchanged.

### Phase 1: SimBackend ABC + Numpy Pure Kinematic Backend

- **SimBackend ABC** — Abstract interface (`set_positions_batch`, `detect_collisions`, `load_model`, `step_physics`) that `Agent` and `SimObject` program against instead of raw `pybullet` calls
- **NumpyBackend (default)** — Contiguous numpy arrays for positions/orientations, `scipy.spatial.cKDTree` for collision. No physics engine dependency. Expected: 1000 agents in ~2–5 ms (RTF 20–50×), 5000+ agents at RTF > 1.0
- **PyBulletBackend (compat)** — Wraps existing PyBullet calls behind SimBackend ABC for backward compatibility and physics-mode users
- **URDF parsing without PyBullet** — Use `yourdfpy` or similar to parse URDF into internal model data, removing the last hard dependency on PyBullet for kinematic-only mode
- **Visualization decoupling** — Replace `p.GUI` with Rerun, Open3D, or RViz for rendering. Backend-agnostic scene display

### Phase 2: Native Backend (Rust/C++ via PyO3/pybind11)

Only justified if Phase 1 numpy performance is insufficient (e.g., 5000+ agents at 240 Hz).

- **Rust kinematic core** — Position update, yaw integration, AABB collision in Rust. Exposed to Python via PyO3. Expected: further 3–10× over numpy (RTF 100–300× for 1000 agents)
- **Batch collision in Rust** — Sweep-and-prune or spatial hash for O(n log n) broad-phase, replacing Python KDTree
- **Zero-copy interop** — Share numpy arrays directly with Rust via buffer protocol (no serialization)

### Phase 3: GPU Backend (optional)

For 10,000+ agent scenarios or RL training workloads.

- **MuJoCo MJX backend** — JAX-accelerated batch physics on GPU. URDF via MJCF conversion
- **JAX pure kinematic** — `jax.numpy` drop-in for NumpyBackend, JIT-compiled, GPU-parallel

### Phase 4: ECS Architecture (v2.0 candidate)

Considered when entity diversity explodes beyond what Agent/SimObject OOP can express cleanly (e.g., drones + ground robots + conveyors + elevators + dynamic obstacles all in one scene with distinct component sets).

- **Component-based entity model** — Replace Agent/SimObject inheritance with composable components (Transform, Collision, Kinematics, JointState, ActionQueue, etc.)
- **System pipeline** — Each system (MovementSystem, CollisionSystem, ActionSystem) operates on component arrays. Maps naturally from current Controller ABC → System
- **Rust ECS runtime** — Leverage `bevy_ecs` or `hecs` for cache-friendly SoA memory layout. Python API via PyO3 for user-facing logic
- **Migration path** — Controller ABC → System, EventBus → ECS Events, Registry → ECS resource/archetype queries. Plugin Architecture phases are designed as stepping stones toward this

> **Note:** At this point the project may evolve beyond "PyBulletFleet" in name, as PyBullet would be just one optional backend among several, and the core would no longer depend on it. A rename (e.g., *FleetSim*, *KinematicFleet*) may be appropriate.
