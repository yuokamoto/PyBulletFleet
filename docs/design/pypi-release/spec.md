# PyPI Release Process Specification

**Date:** 2026-03-13
**Status:** Validated

## Context

PyBulletFleet を `pip install pybullet-fleet` でインストール可能にする。リリースプロセスを自動化し、tag push だけで PyPI 公開・GitHub Release 作成・リリースノート生成を完結させる。CI を強化して PR レベルで品質を保証する。

## Decision

3層の品質保証体制を導入する：

1. **GitHub Actions (ci.yml)** — PR ごとの自動チェック（ゲートキーパー）
2. **GitHub Actions (release.yml)** — tag push での自動リリース
3. **Copilot Skill (releasing)** — リリース作業のオーケストレーション（判断・対話）

手動バージョン管理 + tag push トリガー。PyPI 認証は Trusted Publisher (OIDC)。

## Requirements

- `pip install pybullet-fleet` で PyPI からインストール可能
- `git tag v0.1.0 && git push --tags` で PyPI アップロードが自動実行される
- PR ごとに lint・テスト・Sphinx・機密情報・ライセンス・セキュリティを自動チェック
- リリースノートが conventional commits から自動生成される
- `CHANGELOG.md` にリリース履歴が蓄積される
- GitHub Release が自動作成される
- admin 以外はリリースできない
- Copilot skill がリリース作業全体をガイドする

## Constraints

- バージョンは `pyproject.toml` で手動管理（`setuptools-scm` 不使用）
- PyPI 認証は Trusted Publisher (OIDC)（API トークン不使用）
- Tag protection rule で `v*` を admin のみに制限
- Environment protection で `release` environment に承認者設定
- 初回リリースは `v0.1.0`

## Out of Scope

- TestPyPI での事前テスト（将来追加可能）
- 自動バージョンバンプ（手動管理で十分）
- Conda パッケージ公開

## 品質チェックの分担

| チェック | PR (ci.yml) | Release (release.yml / skill) |
|---|---|---|
| テスト + カバレッジ閾値 | ✅ 自動 | ✅ 自動（再実行） |
| lint / 型チェック | ✅ 自動（既存） | - |
| Sphinx -W ビルド | ✅ 自動 | - |
| 機密情報スキャン | ✅ 自動 | - |
| ライセンス互換性 | ✅ 自動 | - |
| セキュリティ (pip-audit) | ✅ 自動 | - |
| パフォーマンスベンチマーク | - | ✅ skill（計測・比較・記録） |
| API 互換性チェック | - | ✅ skill（判断が必要） |
| パッケージ検証 (twine check) | - | ✅ 自動 |
| リリースノート生成 | - | ✅ skill + pre-release.sh |
| README バージョン確認 | - | ✅ skill |

## リリースフロー

```
Copilot Skill (releasing) がオーケストレーション:
  1. CI が全部通っているか確認
  2. パフォーマンスベンチマーク実行、前回比較
  3. API 互換性チェック（breaking change の有無）
  4. README・ドキュメントの確認
  5. pre-release.sh を実行:
     → 事前チェック（ブランチ、クリーン、同期、バージョン一致）
     → semver バリデーション（+1 patch/minor/major のみ許可）
     → CHANGELOG.md にリリースノート draft 追記
  6. AI が CHANGELOG をリライト → ユーザー承認
  7. publish-release.sh を実行:
     → CHANGELOG 表示 → 確認プロンプト（y/n）
     → commit & tag & push

GitHub Actions (release.yml) — 2つのトリガー:
  A. tag push (v*) → 自動起動（通常フロー）
  B. workflow_dispatch → GitHub UI から手動起動（再実行・リカバリ用）

  6. テスト再実行
  7. パッケージビルド (sdist + wheel) + twine check
  8. PyPI にアップロード (Trusted Publisher)
  9. GitHub Release 作成（CHANGELOG.md から抽出）
```

### 手動トリガー (workflow_dispatch)

- GitHub Actions → Release → "Run workflow" から実行
- version 入力必須（例: `0.1.0`）
- 対応する tag `v0.1.0` が存在しない場合はエラー
- 用途: PyPI アップロード失敗時の再実行、GitHub Release の再作成

## 成果物

| ファイル | 用途 |
|---|---|
| `.github/workflows/ci.yml` | PR 品質ゲート（既存を拡張） |
| `.github/workflows/release.yml` | 自動リリースワークフロー |
| `pre-release.sh` | 事前チェック + semver検証 + CHANGELOG draft生成 |
| `publish-release.sh` | CHANGELOG確認 + commit + tag + push |
| `CHANGELOG.md` | リリース履歴 |
| `.copilot/skills/releasing/SKILL.md` | リリースオーケストレーション skill |

## GitHub 設定（手動）

| 設定 | 場所 | 内容 |
|---|---|---|
| Tag protection | Settings → Rules → Rulesets → `v*` | admin のみ tag 作成可 |
| Environment | Settings → Environments → `release` | 承認者設定 |
| Trusted Publisher | PyPI → Publishing → Add | リポジトリ・ワークフロー・environment 指定 |
| Required status checks | Settings → Branches → `main` | 新規 CI ジョブを required に追加 |

## Success Criteria

- [ ] `pip install pybullet-fleet==0.1.0` でインストール成功
- [ ] PR で Sphinx・機密情報・ライセンス・セキュリティチェックが自動実行される
- [ ] CI が失敗した PR はマージできない
- [ ] `pre-release.sh` がチェック → semver検証 → CHANGELOG draft 生成
- [ ] `publish-release.sh` が確認 → commit → tag push を実行
- [ ] tag push で GitHub Actions が自動でビルド → PyPI アップロード
- [ ] GitHub Release にリリースノートが含まれる
- [ ] CHANGELOG.md にリリース履歴が蓄積される
- [ ] admin 以外が tag を push できない
- [ ] releasing skill がリリース全工程をガイドする
