# PyPI Release Process — Agent Specification

## Requirements

- `.github/workflows/ci.yml` を拡張。Sphinx -W, 機密情報スキャン, ライセンス互換性, セキュリティ監査, カバレッジ閾値を追加
- `pre-release.sh <version>` スクリプトを作成。事前チェック→semver検証→リリースノートdraft生成→CHANGELOG更新
- `publish-release.sh <version>` スクリプトを作成。CHANGELOG確認→commit & tag push
- `.github/workflows/release.yml` を作成。tag push → test → build → twine check → PyPI upload → GitHub Release
- `CHANGELOG.md` を作成（初期状態は空テンプレート）
- `.copilot/skills/releasing/SKILL.md` を作成。リリースオーケストレーション skill
- PyPI 認証は Trusted Publisher (OIDC) — API トークンは使わない
- GitHub Environment `release` を使用（手動設定の手順をドキュメント化）

## Constraints

- バージョンは `pyproject.toml` の `version` フィールドで手動管理
- `pre-release.sh` と `publish-release.sh` は bash スクリプト（サードパーティツール不使用、POSIX標準ユーティリティ + git のみ使用）
- Conventional Commits 形式を前提にパース
- 既存の `.github/workflows/ci.yml` のジョブ構造を維持しつつ拡張
- Python 3.11 でビルド（`.readthedocs.yaml` と統一）
- Skill は `.copilot/skills/releasing/SKILL.md` に配置（プロジェクトローカル）

## Approach

5成果物: ci.yml 拡張 + release.yml + pre-release.sh + publish-release.sh + CHANGELOG.md + releasing skill。手動設定手順のドキュメント化。

## Design

### 1. ci.yml 拡張（PR 品質ゲート）

既存の `lint` + `test` ジョブに加えて以下を追加:

```yaml
  docs:
    name: Documentation Build
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v5
      - uses: actions/setup-python@v5
        with:
          python-version: '3.11'
      - run: |
          pip install -e ".[dev,docs]"
      - run: |
          cd docs && sphinx-build -W -b html . _build/html

  security:
    name: Security and License Check
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v5
      - uses: actions/setup-python@v5
        with:
          python-version: '3.11'
      - run: pip install -e . pip-audit pip-licenses
      # 機密情報スキャン
      - name: Scan for secrets
        run: |
          # API keys, tokens, passwords
          ! grep -rn \
            -e 'AKIA[0-9A-Z]\{16\}' \
            -e 'sk-[a-zA-Z0-9]\{20,\}' \
            -e 'password\s*=' \
            -e 'secret\s*=' \
            --include='*.py' --include='*.yaml' --include='*.yml' --include='*.toml' \
            pybullet_fleet/ docs/ .github/ || exit 1
      # セキュリティ監査
      - name: Security audit
        run: pip-audit
      # ライセンス互換性チェック (Apache 2.0 互換)
      - name: License check
        run: |
          pip-licenses --format=csv --with-license-file --no-license-path > licenses.csv
          # GPL/AGPL/SSPL 等の非互換ライセンスを検出
          if grep -iE '(GNU General Public|GPL|AGPL|SSPL|EUPL|CC-BY-NC|CC-BY-ND)' licenses.csv; then
            echo "::error::Incompatible license detected"
            exit 1
          fi
```

`test` ジョブにカバレッジ閾値を追加:

```yaml
      - name: Run tests
        run: |
          pytest tests/ -v --tb=short --cov=pybullet_fleet --cov-report=term-missing --cov-fail-under=75
```

### 2. pre-release.sh + publish-release.sh

```bash
#!/bin/bash
set -euo pipefail

# pre-release.sh <version>
# 1. 事前チェック（ブランチ、クリーン、同期、バージョン一致、tag重複なし）
# 2. semver バリデーション（前回タグから +1 patch/minor/major のみ許可）
# 3. リリースノート draft 生成（conventional commits パース）
# 4. CHANGELOG.md の ## [Unreleased] の直後に挿入

# publish-release.sh <version>
# 1. 再検証（ブランチ、バージョン、tag重複）
# 2. CHANGELOG.md のみ変更されていることを確認
# 3. リリースノート表示 + 確認プロンプト
# 4. git add CHANGELOG.md && git commit && git tag && git push
```

### リリースノート生成ロジック（pre-release.sh 内）

```bash
generate_release_notes() {
    local prev_tag="$1"
    local version="$2"
    local date=$(date +%Y-%m-%d)

    echo "## v${version} (${date})"
    echo ""

    # カテゴリ別に commit を収集
    local categories=("feat:Features" "fix:Bug Fixes" "docs:Documentation" "perf:Performance" "refactor:Refactoring" "test:Testing" "chore:Chores")

    for entry in "${categories[@]}"; do
        local prefix="${entry%%:*}"
        local title="${entry##*:}"
        local commits
        if [ -n "$prev_tag" ]; then
            commits=$(git log "${prev_tag}..HEAD" --pretty=format:"- %s" --grep="^${prefix}" --extended-regexp)
        else
            commits=$(git log --pretty=format:"- %s" --grep="^${prefix}" --extended-regexp)
        fi
        if [ -n "$commits" ]; then
            echo "### ${title}"
            echo ""
            echo "$commits"
            echo ""
        fi
    done

    # カテゴリに属さない commit
    local other
    if [ -n "$prev_tag" ]; then
        other=$(git log "${prev_tag}..HEAD" --pretty=format:"%s" | grep -vE "^(feat|fix|docs|perf|refactor|test|chore)" || true)
    else
        other=$(git log --pretty=format:"%s" | grep -vE "^(feat|fix|docs|perf|refactor|test|chore)" || true)
    fi
    if [ -n "$other" ]; then
        echo "### Other Changes"
        echo ""
        echo "$other" | sed 's/^/- /'
        echo ""
    fi
}
```

### 3. release.yml

```yaml
name: Release

on:
  push:
    tags:
      - 'v*'
  workflow_dispatch:
    inputs:
      version:
        description: 'Release version (e.g., 0.1.0)'
        required: true
        type: string

# バージョン解決: tag push → GITHUB_REF_NAME (v0.1.0)
#                 manual   → inputs.version から v prefix 付与
# 最初の job で tag 存在確認を行い、manual 時に tag がなければ失敗させる

jobs:
  validate:
    name: Validate Release
    runs-on: ubuntu-latest
    outputs:
      version: ${{ steps.version.outputs.version }}
      tag: ${{ steps.version.outputs.tag }}
    steps:
      - uses: actions/checkout@v5
        with:
          fetch-depth: 0
      - name: Resolve version
        id: version
        run: |
          if [ "${{ github.event_name }}" = "workflow_dispatch" ]; then
            TAG="v${{ inputs.version }}"
          else
            TAG="${GITHUB_REF_NAME}"
          fi
          VERSION="${TAG#v}"
          echo "tag=${TAG}" >> "$GITHUB_OUTPUT"
          echo "version=${VERSION}" >> "$GITHUB_OUTPUT"
          # tag が存在するか確認
          if ! git rev-parse "refs/tags/${TAG}" >/dev/null 2>&1; then
            echo "::error::Tag ${TAG} does not exist"
            exit 1
          fi

  test:
    name: Test
    needs: validate
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v5
      - uses: actions/setup-python@v5
        with:
          python-version: '3.11'
      - run: pip install -e ".[dev]"
      - run: pytest tests/ -v --tb=short

  build:
    name: Build and Verify Package
    needs: test
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v5
      - uses: actions/setup-python@v5
        with:
          python-version: '3.11'
      - run: pip install build twine
      - run: python -m build
      - run: twine check dist/*
      - uses: actions/upload-artifact@v4
        with:
          name: dist
          path: dist/

  publish-pypi:
    name: Publish to PyPI
    needs: build
    runs-on: ubuntu-latest
    environment: release
    permissions:
      id-token: write
    steps:
      - uses: actions/download-artifact@v4
        with:
          name: dist
          path: dist/
      - uses: pypa/gh-action-pypi-publish@release/v1

  github-release:
    name: Create GitHub Release
    needs: publish-pypi
    runs-on: ubuntu-latest
    permissions:
      contents: write
    steps:
      - uses: actions/checkout@v5
      - name: Extract changelog for this version
        id: changelog
        run: |
          VERSION="${{ needs.validate.outputs.version }}"
          awk "/^## v${VERSION}/,/^## v[0-9]/" CHANGELOG.md | head -n -1 > release_notes.md
          if [ ! -s release_notes.md ]; then
            echo "Release ${GITHUB_REF_NAME}" > release_notes.md
          fi
      - uses: softprops/action-gh-release@v2
        with:
          body_path: release_notes.md
          generate_release_notes: false
```

### 4. CHANGELOG.md

```markdown
# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

```

### 5. Releasing Skill

Location: `.copilot/skills/releasing/SKILL.md` (プロジェクトローカル)

Skill がエージェントに指示するフロー:

```
Phase 1: 事前検証
  1. CI ステータス確認（GitHub Actions の最新結果）
  2. git status / branch 確認

Phase 2: リリース固有チェック（skill が実行）
  3. パフォーマンスベンチマーク実行
     - benchmark/ のスクリプトを実行
     - 前回リリースの結果と比較（results/ 配下）
     - 結果をリリースノートに追記
  4. API 互換性チェック
     - __init__.py の public exports を前回 tag と diff
     - breaking change があればユーザーに報告
  5. README バージョン確認
     - インストール手順のバージョン表記が正しいか
  6. ドキュメント確認
     - 新規 public API に docstring があるか
     - CHANGELOG.md に記載漏れがないか

Phase 3: リリース実行
  7. pre-release.sh を実行（draft 生成）
  8. AI が CHANGELOG をリライト → ユーザー承認
  9. publish-release.sh を実行（commit, tag, push）
  10. tag push 後、GitHub Actions の進行を報告

Phase 4: リリース後確認
  9. PyPI にパッケージが公開されたか確認
  10. GitHub Release が作成されたか確認
```

Skill の設計方針:
- GitHub Actions = 決定的チェック（pass/fail で自動判定）
- Skill = 判断が必要な作業（パフォーマンス比較、API 互換性判断、リリースノート品質）
- Skill は Actions の結果を読み取り、追加チェックを実行し、対話的に進行する

## File References

実装エージェントが読むべきファイル:

- `pyproject.toml` — パッケージメタデータ、`version = "0.1.0"`
- `.github/workflows/ci.yml` — 既存 CI（拡張対象）
- `pybullet_fleet/__init__.py` — パッケージの `__version__` の有無を確認
- `LICENSE` — PyPI メタデータに必要
- `docs/conf.py` — Sphinx 設定（CI で使用）
- `pyproject.toml` の `[project.optional-dependencies.docs]` — ドキュメント依存（`pip install -e ".[docs]"` で使用）
- `benchmark/run_benchmark.py` — ベンチマーク実行スクリプト（skill で使用）
- `~/.copilot/skills/verification-before-completion/SKILL.md` — 既存 skill の構造を参考
- `~/.copilot/skills/finishing-a-development-branch/SKILL.md` — 既存 skill の構造を参考

## 手動設定手順（ドキュメント化必須）

### 1. PyPI アカウントと Trusted Publisher

1. https://pypi.org/ でアカウント作成（未作成の場合）
2. https://pypi.org/manage/account/publishing/ → **Add a new pending publisher**
   - PyPI project name: `pybullet-fleet`
   - Owner: `yuokamoto`
   - Repository: `PyBulletFleet`
   - Workflow name: `release.yml`
   - Environment name: `release`

### 2. GitHub Tag Protection

1. Settings → Rules → Rulesets → **New ruleset**
   - Target: Tags → `v*`
   - Bypass list: Repository admin のみ
   - Restrict creations: ON

### 3. GitHub Environment

1. Settings → Environments → **New environment** → `release`
2. **Required reviewers** → 自分を追加（任意だが推奨）
3. **Deployment branches and tags** → `v*` tags のみに制限

### 4. Required Status Checks 更新

1. Settings → Branches → `main` protection rule
2. Required status checks に追加:
   - `Documentation Build`
   - `Security and License Check`

## Success Criteria

- [ ] ci.yml に docs, security ジョブが追加されている
- [ ] PR で Sphinx -W, 機密情報スキャン, ライセンスチェック, pip-audit が自動実行される
- [ ] カバレッジ 75% 未満で CI が失敗する
- [ ] `pre-release.sh 0.1.0` が事前チェック → semver検証 → CHANGELOG draft 生成を実行
- [ ] `pre-release.sh` がブランチ不一致・バージョン不一致・dirty tree・semver不正で正しくエラー終了
- [ ] `publish-release.sh 0.1.0` が CHANGELOG 確認 → commit → tag push を実行
- [ ] tag push で `release.yml` が起動し、test → build → twine check → PyPI → GitHub Release の順に実行
- [ ] `pip install pybullet-fleet==0.1.0` でインストール成功
- [ ] GitHub Release にリリースノートが含まれる
- [ ] CHANGELOG.md にリリース履歴が記録される
- [ ] releasing skill が全リリース工程をガイドする
- [ ] `__version__` が `pyproject.toml` と一致（存在する場合）
