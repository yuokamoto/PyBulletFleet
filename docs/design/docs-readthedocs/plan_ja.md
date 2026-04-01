# ドキュメント ReadTheDocs 移行 — 実装計画

> **Claude 向け:** 必須サブスキル: executing-plans を使用してこの計画をタスクごとに実行してください。

**目標:** PyBulletFleet の全 Markdown ドキュメントを Sphinx + MyST による ReadTheDocs 対応サイトに再構成する。適切な階層構造、重複の排除、英語統一を実現する。

**アーキテクチャ:** Sphinx に `myst-parser` を組み合わせ、全ドキュメントを Markdown のまま維持する（reStructuredText への変換は不要）。既存の `.md` ファイルを `docs/` 配下の RTD セクションに対応するサブディレクトリに再配置する。`docs/` ルートの `index.md` + `conf.py` でビルドを制御する。古いドキュメントやセッションログは `docs/archive/` に移動。空ファイルは削除する。

**技術スタック:** Sphinx, myst-parser, sphinx-rtd-theme, sphinx-autodoc-typehints, sphinx-copybutton

---

## 現状

**35個の Markdown ファイル**が6箇所に散在：
- `./README.md`, `./DESIGN.md`（レポルート）
- `./docs/`（12ファイル — 設計ドキュメント、ガイド、プロファイリング結果）
- `./docs_conversation/`（8ファイル — セッションダンプ、移行計画）
- `./benchmark/`（10ファイル — ベンチマークスイート関連）
- `./config/`（1ファイル）
- `./tests/`（1ファイル）

**問題点：**
1. 統一的な目次やナビゲーションがない — フラットなファイルが散在
2. 重複・重複する内容（パフォーマンスレポート3件、プロファイリングガイド2件、設定ガイド2件）
3. 空のプレースホルダーファイルが5件（0行）
4. 日本語と英語が混在し、一貫性がない
5. セッション会話のダンプと意図的なドキュメントが混在
6. ビルドシステムがない — ブラウズ可能なサイトを生成できない

## 目標構成

```
docs/
├── conf.py                        # Sphinx 設定
├── index.md                       # ランディングページ（プロジェクト紹介）
├── getting-started/
│   └── quickstart.md              # インストール + 初回シミュレーション
├── architecture/
│   ├── index.md                   # セクションインデックス
│   ├── overview.md                # システムアーキテクチャ（DESIGN.md から）
│   ├── collision-detection.md     # 衝突検出サブシステム（COLLISION_DETECTION_DESIGN_v3.md から）
│   └── realtime-sync.md           # リアルタイム同期設計（REALTIME_SYNC_DESIGN.md から）
├── how-to/
│   ├── index.md                   # セクションインデックス
│   ├── memory-profiling.md        # メモリプロファイリングガイド
│   ├── logging.md                 # LazyLogger の使い方
│   └── spatial-hash-config.md     # セルサイズモード設定
├── configuration/
│   ├── index.md                   # セクションインデックス
│   └── reference.md               # YAML 設定リファレンス（config/README.md から）
├── benchmarking/
│   ├── index.md                   # セクションインデックス
│   ├── optimization-guide.md      # パフォーマンス最適化ガイド
│   ├── results.md                 # ベンチマーク結果（統合版）
│   ├── profiling.md               # プロファイリングツール（統合+翻訳）
│   ├── configs.md                 # ベンチマーク設定ファイル
│   └── experiments.md             # 実験スクリプト
├── testing/
│   ├── index.md                   # セクションインデックス
│   └── overview.md                # テストガイド（翻訳版）
├── api/
│   └── index.md                   # autodoc プレースホルダー
├── design/                        # そのまま維持（エージェント作業用）
│   ├── core-simulation-tests/
│   ├── pybullet-fleet-skills/
│   └── docs-readthedocs/          # この計画
└── archive/                       # toctree に含めない、参照用に保持
    ├── development-history.md
    ├── test-action-plan.md
    ├── collision-test-design.md
    ├── agent-profiling-results.md
    ├── optimization-results.md
    ├── performance-analysis.md
    └── conversation/              # docs_conversation/ の全内容
```

---

## タスク 1: Sphinx の初期セットアップ（直列）

**ファイル：**
- 作成: `docs/conf.py`
- 作成: `docs/index.md`
- 作成: `docs/requirements-docs.txt`
- 修正: `requirements-dev.txt`（ドキュメント依存追加）

**ステップ 1: ドキュメント用 requirements ファイルを作成**

```
# docs/requirements-docs.txt
sphinx>=7.0
sphinx-rtd-theme>=2.0
sphinx-autodoc-typehints>=2.0
sphinx-copybutton>=0.5
myst-parser>=3.0
```

**ステップ 2: `docs/conf.py` を作成**

```python
import os
import sys
sys.path.insert(0, os.path.abspath('..'))

project = 'PyBulletFleet'
copyright = '2026, Yu Okamoto'
author = 'Yu Okamoto'
release = '0.1.0'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.intersphinx',
    'sphinx_autodoc_typehints',
    'sphinx_copybutton',
    'myst_parser',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'archive', 'design']

html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'navigation_depth': 3,
    'collapse_navigation': False,
}

myst_enable_extensions = [
    'colon_fence',
    'deflist',
    'tasklist',
]
myst_heading_anchors = 3

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = True

autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'undoc-members': True,
}

intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
    'pybullet': ('https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA', None),
}
```

**ステップ 3: `docs/index.md` を作成**

全セクションへの toctree を含むルートインデックスを作成。

**ステップ 4: Sphinx ビルドの確認**

実行: `cd docs && sphinx-build -b html . _build/html`
期待結果: 警告のみでビルド成功（エラーなし）

**ステップ 5: コミット**

```bash
git add docs/conf.py docs/index.md docs/requirements-docs.txt
git commit -m "docs: bootstrap Sphinx with MyST for ReadTheDocs"
```

---

## タスク 2: 空ファイル・旧ファイルの削除（並列可）

**削除対象ファイル：**
- `docs/AGENT_PROFILING_RESULTS_EN.md`（0行、空のプレースホルダー）
- `docs/OPTIMIZATION_RESULTS_EN.md`（0行、空のプレースホルダー）
- `docs_conversation/MEMORY_ANALYSIS_RESULTS.md`（0行）
- `docs_conversation/SYSTEM_PERFORMANCE_ANALYSIS.md`（0行）
- `docs_conversation/SYSTEM_RESOURCE_EVIDENCE.md`（0行）
- `benchmark/archive/CONFIG_GUIDE.md`（`benchmark/configs/README.md` に置き換え済み）
- `benchmark/archive/PERFORMANCE_REPORT_old.md`（`benchmark/PERFORMANCE_REPORT.md` に置き換え済み）

**ステップ 1: ファイル削除**

```bash
git rm docs/AGENT_PROFILING_RESULTS_EN.md \
       docs/OPTIMIZATION_RESULTS_EN.md \
       docs_conversation/MEMORY_ANALYSIS_RESULTS.md \
       docs_conversation/SYSTEM_PERFORMANCE_ANALYSIS.md \
       docs_conversation/SYSTEM_RESOURCE_EVIDENCE.md \
       benchmark/archive/CONFIG_GUIDE.md \
       benchmark/archive/PERFORMANCE_REPORT_old.md
```

**ステップ 2: コミット**

```bash
git commit -m "docs: remove empty placeholders and superseded files"
```

---

## タスク 3: セッションログ・計画ドキュメントのアーカイブ（並列可）

参照専用のファイル（会話ダンプ、時点計画）を `docs/archive/` に移動する。

**ステップ 1: アーカイブディレクトリの作成**

```bash
mkdir -p docs/archive/conversation
```

**ステップ 2: docs_conversation/ の内容を移動**

```bash
git mv docs_conversation/COMPREHENSIVE_REVIEW_AND_IMPROVEMENTS.md docs/archive/conversation/
git mv docs_conversation/REFACTORING_SUMMARY.md docs/archive/conversation/
git mv docs_conversation/SPHINX_MIGRATION_PLAN.md docs/archive/conversation/
git mv docs_conversation/MEMORY_PROFILING_IMPROVEMENTS.md docs/archive/conversation/
git mv docs_conversation/README.md docs/archive/conversation/
```

**ステップ 3: 計画・履歴ドキュメントの移動**

```bash
git mv docs/DEVELOPMENT_HISTORY.md docs/archive/development-history.md
git mv docs/TEST_ACTION_PLAN.md docs/archive/test-action-plan.md
git mv docs/COLLISION_TEST_DESIGN.md docs/archive/collision-test-design.md
```

**ステップ 4: 空の docs_conversation/ ディレクトリを削除**

```bash
rmdir docs_conversation
```

**ステップ 5: コミット**

```bash
git commit -m "docs: archive session logs and planning docs"
```

---

## タスク 4: Getting Started セクションの作成（直列、タスク1に依存）

**ファイル：**
- 作成: `docs/getting-started/quickstart.md`
- 修正: `README.md`（スリム化、ドキュメントへのリンク追記）

**ステップ 1: quickstart.md を作成**

`README.md` から Quick Start、インストール、初回シミュレーション、基本的な使い方を抽出する。内容：
- インストール（`pip install -e .`）
- 最初のサンプル実行（`python examples/basics/robot_demo.py`）
- YAML 設定の基本
- 詳細は設定リファレンスへのリンク

**ステップ 2: README.md をスリム化**

README.md はプロジェクト概要（動機、機能一覧テーブル、インストールコマンド、ドキュメントリンク）に限定。RTD に移動した詳細セクションを削除：
- 詳細な設定セクションを削除（→ `configuration/reference.md`）
- 詳細なアーキテクチャセクションを削除（→ `architecture/overview.md`）
- 「📖 **完全なドキュメント:** https://pybulletfleet.readthedocs.io」リンクを追加

**ステップ 3: Sphinx ビルドの確認**

実行: `cd docs && sphinx-build -b html . _build/html`
期待結果: PASS

**ステップ 4: コミット**

```bash
git add docs/getting-started/ README.md
git commit -m "docs: add getting-started section, slim README"
```

---

## タスク 5: Architecture セクションの作成（直列、タスク1に依存）

**ファイル：**
- 作成: `docs/architecture/index.md`
- 移動+編集: `DESIGN.md` → `docs/architecture/overview.md`
- 移動: `docs/COLLISION_DETECTION_DESIGN_v3.md` → `docs/architecture/collision-detection.md`
- 移動: `docs/REALTIME_SYNC_DESIGN.md` → `docs/architecture/realtime-sync.md`

**ステップ 1: セクションインデックスを作成**

```markdown
# Architecture

PyBulletFleet のシステム設計の概要。

```{toctree}
:maxdepth: 2
overview
collision-detection
realtime-sync
```
```

**ステップ 2: DESIGN.md を移動・修正**

```bash
git mv DESIGN.md docs/architecture/overview.md
```

1行目に MyST 対応の見出しを追加。README との重複を削除。ASCII ダイアグラムは維持。

**ステップ 3: 衝突検出と RT 同期ドキュメントを移動**

```bash
git mv docs/COLLISION_DETECTION_DESIGN_v3.md docs/architecture/collision-detection.md
git mv docs/REALTIME_SYNC_DESIGN.md docs/architecture/realtime-sync.md
```

**ステップ 4: Sphinx ビルドの確認**

実行: `cd docs && sphinx-build -b html . _build/html`
期待結果: PASS

**ステップ 5: コミット**

```bash
git add docs/architecture/
git commit -m "docs: create architecture section with design, collision, and RT sync"
```

---

## タスク 6: How-to ガイドセクションの作成（直列、タスク1に依存）

**ファイル：**
- 作成: `docs/how-to/index.md`
- 移動: `docs/MEMORY_PROFILING_GUIDE.md` → `docs/how-to/memory-profiling.md`
- 移動: `docs/LOGGING_UTILS.md` → `docs/how-to/logging.md`
- 移動: `docs/spatial_hash_cell_size_modes.md` → `docs/how-to/spatial-hash-config.md`

**ステップ 1: セクションインデックスを作成**

```markdown
# How-to ガイド

よくあるタスクの実践的なガイド。

```{toctree}
:maxdepth: 2
memory-profiling
logging
spatial-hash-config
```
```

**ステップ 2: ファイルを移動**

```bash
mkdir -p docs/how-to
git mv docs/MEMORY_PROFILING_GUIDE.md docs/how-to/memory-profiling.md
git mv docs/LOGGING_UTILS.md docs/how-to/logging.md
git mv docs/spatial_hash_cell_size_modes.md docs/how-to/spatial-hash-config.md
```

**ステップ 3: Sphinx ビルド確認とコミット**

```bash
cd docs && sphinx-build -b html . _build/html
git add docs/how-to/
git commit -m "docs: create how-to section with memory profiling, logging, spatial hash guides"
```

---

## タスク 7: Configuration セクションの作成（直列、タスク1に依存）

**ファイル：**
- 作成: `docs/configuration/index.md`
- 移動: `config/README.md` → `docs/configuration/reference.md`

**ステップ 1: セクションインデックスを作成し設定ドキュメントを移動**

```bash
mkdir -p docs/configuration
```

toctree 付きの `docs/configuration/index.md` を作成。`config/README.md` の内容を `docs/configuration/reference.md` にコピー（`config/README.md` はドキュメントへの短いポインターとして維持）。

**ステップ 2: 確認とコミット**

```bash
git add docs/configuration/
git commit -m "docs: create configuration reference section"
```

---

## タスク 8: Benchmarking セクションの作成（直列、タスク1に依存）

最大のセクション — 重複するドキュメントの統合、日本語の翻訳を含む。

**ファイル：**
- 作成: `docs/benchmarking/index.md`
- 移動: `benchmark/PERFORMANCE_OPTIMIZATION_GUIDE.md` → `docs/benchmarking/optimization-guide.md`
- 統合: `benchmark/PERFORMANCE_REPORT.md` + `benchmark/COLLISION_BENCHMARK_RESULTS.md` → `docs/benchmarking/results.md`
- 統合: `benchmark/profiling/README.md` + `benchmark/profiling/PROFILING_GUIDE.md` → `docs/benchmarking/profiling.md`（日→英翻訳）
- 移動+翻訳: `benchmark/configs/README.md` → `docs/benchmarking/configs.md`
- 移動: `benchmark/experiments/README.md` → `docs/benchmarking/experiments.md`

**ステップ 1: セクションインデックスを作成**

**ステップ 2: 最適化ガイドを移動**

```bash
mkdir -p docs/benchmarking
git mv benchmark/PERFORMANCE_OPTIMIZATION_GUIDE.md docs/benchmarking/optimization-guide.md
```

**ステップ 3: パフォーマンス結果を統合**

`docs/benchmarking/results.md` を作成。以下を統合：
- `PERFORMANCE_REPORT.md` からハードウェアスペック + RTF/タイミング/メモリテーブル
- `COLLISION_BENCHMARK_RESULTS.md` から衝突ベンチマーク（物理 ON vs OFF）

**ステップ 4: プロファイリングガイドの統合（翻訳）**

`docs/benchmarking/profiling.md` を作成。以下を統合：
- `benchmark/profiling/README.md`（ツール説明 — 日本語部分を翻訳）
- `benchmark/profiling/PROFILING_GUIDE.md`（手法 — 日本語部分を翻訳）

**ステップ 5: ベンチマーク設定の移動+翻訳**

`benchmark/configs/README.md` を翻訳して `docs/benchmarking/configs.md` にコピー

**ステップ 6: 実験を移動**

```bash
git mv benchmark/experiments/README.md docs/benchmarking/experiments.md
```

**ステップ 7: 統合元のオリジナルをアーカイブ**

統合されたソースファイル（移動ではなく）を `docs/archive/benchmark/` に移動して git 履歴を保持：

```bash
mkdir -p docs/archive/benchmark
git mv benchmark/PERFORMANCE_REPORT.md docs/archive/benchmark/
git mv benchmark/COLLISION_BENCHMARK_RESULTS.md docs/archive/benchmark/
git mv benchmark/profiling/README.md docs/archive/benchmark/profiling-readme.md
git mv benchmark/profiling/PROFILING_GUIDE.md docs/archive/benchmark/
```

**ステップ 8: 残りのパフォーマンス分析ドキュメントもアーカイブ**

```bash
git mv docs/PERFORMANCE_ANALYSIS.md docs/archive/performance-analysis.md
git mv docs/AGENT_PROFILING_RESULTS.md docs/archive/agent-profiling-results.md
git mv docs/OPTIMIZATION_RESULTS.md docs/archive/optimization-results.md
```

**ステップ 9: benchmark/README.md を更新**

`benchmark/README.md` をドキュメントは `docs/benchmarking/` セクションを参照するよう更新し、スクリプト使用方法のみインライン参照として残す。

**ステップ 10: 確認とコミット**

```bash
cd docs && sphinx-build -b html . _build/html
git add docs/benchmarking/ docs/archive/ benchmark/
git commit -m "docs: create benchmarking section with merged results and translated profiling guides"
```

---

## タスク 9: Testing セクションの作成（直列、タスク1に依存）

**ファイル：**
- 作成: `docs/testing/index.md`
- 作成: `docs/testing/overview.md`（`tests/README.md` から翻訳）

**ステップ 1: セクションインデックスを作成**

**ステップ 2: tests/README.md を英語に翻訳**

`docs/testing/overview.md` を作成 — テストディレクトリガイドの英語版。`tests/README.md` はドキュメントへの短いポインターとして維持。

**ステップ 3: 確認とコミット**

```bash
git add docs/testing/
git commit -m "docs: create testing section with translated overview"
```

---

## タスク 10: API リファレンスのプレースホルダー作成（直列、タスク1に依存）

**ファイル：**
- 作成: `docs/api/index.md`

**ステップ 1: autodoc スタブ付き API インデックスを作成**

```markdown
# API リファレンス

ソースコードの docstring から自動生成。

```{toctree}
:maxdepth: 2
```

```{eval-rst}
.. autosummary::
   :toctree: generated
   :recursive:

   pybullet_fleet
```
```

**ステップ 2: 確認とコミット**

```bash
git add docs/api/
git commit -m "docs: add API reference placeholder with autodoc"
```

---

## タスク 11: .readthedocs.yaml の追加（直列、タスク1に依存）

**ファイル：**
- 作成: `.readthedocs.yaml`
- 作成: `docs/Makefile`
- 修正: `.gitignore`（`docs/_build/` を追加）

**ステップ 1: `.readthedocs.yaml` を作成**

```yaml
version: 2

build:
  os: ubuntu-24.04
  tools:
    python: "3.11"

sphinx:
  configuration: docs/conf.py

python:
  install:
    - requirements: docs/requirements-docs.txt
    - method: pip
      path: .
```

**ステップ 2: `docs/Makefile` を作成**

ローカルビルド用の標準的な Sphinx Makefile。

**ステップ 3: .gitignore を更新**

`.gitignore` に `docs/_build/` を追加。

**ステップ 4: フルビルドの確認**

```bash
cd docs && sphinx-build -b html . _build/html -W
```

`-W` は警告をエラーとして扱う。壊れた相互参照を修正する。

**ステップ 5: コミット**

```bash
git add .readthedocs.yaml docs/Makefile .gitignore
git commit -m "docs: add ReadTheDocs config and Makefile"
```

---

## タスク 12: 最終クリーンアップとリンク検証（直列、タスク2-11すべてに依存）

**ステップ 1: 内部リンク切れがないか確認**

```bash
cd docs && sphinx-build -b linkcheck . _build/linkcheck
```

**ステップ 2: 全旧ドキュメントの対応を確認**

スクリプトを実行：
```bash
find . -name "*.md" -not -path "./.git/*" -not -path "./.copilot/*" -not -path "./.pytest_cache/*" | sort
```

すべての `.md` は以下のいずれかに該当するべき：
- `docs/` toctree 内（公開）
- `docs/archive/` 内（保持、ビルドから除外）
- `docs/design/` 内（作業ドキュメント、ビルドから除外）
- `README.md`（レポルート、スリム化済み）
- `benchmark/README.md`（スクリプト使用方法の参照として維持）
- `config/README.md`（短いポインターとして維持）
- `tests/README.md`（短いポインターとして維持）

**ステップ 3: ローカルで開いてブラウズ**

```bash
python -m http.server 8000 -d docs/_build/html
```

**ステップ 4: 最終コミット**

```bash
git add -A
git commit -m "docs: complete ReadTheDocs migration"
```

---

## タスク依存関係

```
タスク 1（Sphinx 初期セットアップ）←── 他の全タスクがこれに依存
  ├── タスク  2（空ファイル削除）          並列可
  ├── タスク  3（セッションログのアーカイブ）  並列可
  ├── タスク  4（Getting Started）        直列（1の後）
  ├── タスク  5（Architecture）           直列（1の後）
  ├── タスク  6（How-to）                直列（1の後）
  ├── タスク  7（Configuration）          直列（1の後）
  ├── タスク  8（Benchmarking）           直列（1の後、最大タスク）
  ├── タスク  9（Testing）               直列（1の後）
  ├── タスク 10（API リファレンス）         直列（1の後）
  └── タスク 11（.readthedocs.yaml）     直列（1の後）
タスク 12（最終クリーンアップ）←── 上記すべてに依存
```

**並列タスク:** 2, 3（相互依存なし）
**タスク1完了後に並列可:** 4, 5, 6, 7, 8, 9, 10, 11（独立したセクション）
**直列ゲート:** タスク 12 は全タスクの完了を待つ

---

## ファイル処理まとめ

| アクション | 対象ファイル | 件数 |
|-----------|------------|------|
| **RTD セクションへ移動** | DESIGN.md, COLLISION_DETECTION_DESIGN_v3.md, REALTIME_SYNC_DESIGN.md, MEMORY_PROFILING_GUIDE.md, LOGGING_UTILS.md, spatial_hash_cell_size_modes.md, PERFORMANCE_OPTIMIZATION_GUIDE.md | 7 |
| **RTD へ統合** | PERFORMANCE_REPORT.md + COLLISION_BENCHMARK_RESULTS.md → results.md; profiling/README.md + PROFILING_GUIDE.md → profiling.md | 4 → 2 |
| **翻訳 + 移動** | benchmark/configs/README.md, benchmark/profiling/*（日本語部分）, tests/README.md | 3 |
| **アーカイブ** | DEVELOPMENT_HISTORY.md, TEST_ACTION_PLAN.md, COLLISION_TEST_DESIGN.md, PERFORMANCE_ANALYSIS.md, AGENT_PROFILING_RESULTS.md, OPTIMIZATION_RESULTS.md, docs_conversation/ 全体（5ファイル） | 11 |
| **削除** | 空ファイル5件 + 旧バージョン2件 | 7 |
| **現状維持** | README.md（スリム化）, benchmark/README.md, config/README.md, tests/README.md, docs/design/* | 7+ |
| **新規作成** | conf.py, index.md, セクションインデックス8件, quickstart.md, results.md, profiling.md, testing/overview.md, .readthedocs.yaml, Makefile, requirements-docs.txt | 約15 |
