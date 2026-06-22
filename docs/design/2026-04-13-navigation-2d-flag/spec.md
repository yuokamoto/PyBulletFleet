# `navigation_2d` Flag Specification

**Date:** 2026-04-13
**Status:** Validated

## Context

Controller の Pose モード（TPI）は `goal_pos[2] = current_pos[2]` をハードコードしており、
3D パス追従（エレベーター、ドローン、アーム、スロープ）が不可能。
Velocity モードは既に3D対応済み。

## Decision

`KinematicController` に `navigation_2d: bool = True` フラグを追加。
デフォルト `True` で既存動作を保持し、`False` で目標Z座標をそのまま使用する。

## Requirements

- デフォルト `True` で後方互換性を維持
- `AgentSpawnParams` から設定可能
- Omni / Differential 両コントローラで動作
- Velocity モードには影響しない

## Constraints

- controller.py の条件分岐3箇所のみ変更
- MotionMode, Pose, Path の API は変更しない

## Out of Scope

- ウェイポイント単位のZ制約切替
- 3D専用コントローラクラスの追加
- 衝突検出の2D/3D連動（別機能）

## Success Criteria

- [ ] 既存テスト全通過（デフォルト True）
- [ ] `navigation_2d=False` で Z が目標値に到達するテスト通過
- [ ] `navigation_2d=True` で Z が保持されるテスト通過
- [ ] `make verify` 通過
