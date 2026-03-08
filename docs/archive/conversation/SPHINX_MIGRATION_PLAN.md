# Sphinx 移行計画

## 目的

既存のMarkdownドキュメント（18個、250KB）を活かしながら、Sphinxによる包括的なドキュメントシステムを構築する。

---

## 戦略: ハイブリッドアプローチ

### ✅ やること
1. Sphinx で API リファレンスを自動生成
2. Sphinx で検索可能な統合ドキュメント作成
3. 既存MDを Sphinx から参照（シンボリックリンク or include）
4. ReadTheDocs で公開

### ❌ やらないこと
1. 既存MDを全て reStructuredText に書き換え（不要）
2. docs/ ディレクトリを削除（保持）

---

## Phase 1: Sphinx セットアップ (1-2時間)

### Step 1.1: 依存関係インストール

```bash
pip install sphinx sphinx-rtd-theme sphinx-autodoc-typehints myst-parser
```

### Step 1.2: Sphinx 初期化

```bash
cd /home/rapyuta/rr_sim_evaluation/PyBulletFleet
mkdir -p docs_sphinx
cd docs_sphinx

# クイックスタート (対話式)
sphinx-quickstart

# プロンプトへの回答:
# > Separate source and build directories (y/n) [n]: y
# > Project name: PyBulletFleet
# > Author name(s): Yu Okamoto
# > Project release []: 0.1.0
# > Project language [en]: en
```

### Step 1.3: conf.py 設定

```python
# docs_sphinx/source/conf.py

import os
import sys
sys.path.insert(0, os.path.abspath('../../'))  # pybullet_fleet へのパス

# Project information
project = 'PyBulletFleet'
copyright = '2026, Yu Okamoto'
author = 'Yu Okamoto'
release = '0.1.0'

# Extensions
extensions = [
    'sphinx.ext.autodoc',           # 自動ドキュメント生成
    'sphinx.ext.napoleon',          # Google/NumPy docstring サポート
    'sphinx.ext.viewcode',          # ソースコードリンク
    'sphinx.ext.intersphinx',       # 他ドキュメントへのリンク
    'sphinx.ext.autosummary',       # サマリーテーブル
    'sphinx_autodoc_typehints',     # 型ヒント表示
    'myst_parser',                  # Markdown サポート
]

# Napoleon settings (Google style docstrings)
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = True

# Autodoc settings
autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'special-members': '__init__',
    'undoc-members': True,
    'exclude-members': '__weakref__'
}

# Autosummary
autosummary_generate = True

# MyST (Markdown) settings
myst_enable_extensions = [
    "colon_fence",      # ::: で code fence
    "deflist",          # 定義リスト
    "html_image",       # HTML img タグ
]

# Source file types
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

# HTML output
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# Intersphinx (外部ドキュメントリンク)
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
    'pybullet': ('https://pybullet.org/wordpress/', None),
}
```

### Step 1.4: index.rst 作成

```rst
.. docs_sphinx/source/index.rst

PyBulletFleet Documentation
===========================

Welcome to PyBulletFleet's documentation!

PyBulletFleet is a general-purpose PyBullet simulation library for multi-robot fleets.

Quick Links
-----------

* :doc:`getting_started` - Installation and first simulation
* :doc:`tutorials/index` - Step-by-step tutorials
* :doc:`api/index` - API Reference
* :doc:`design/index` - Architecture and design documents

Contents
--------

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   getting_started
   tutorials/index
   examples/index

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/index

.. toctree::
   :maxdepth: 2
   :caption: Design & Advanced

   design/index
   advanced/index

.. toctree::
   :maxdepth: 1
   :caption: Development

   changelog
   contributing


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
```

---

## Phase 2: API リファレンス自動生成 (30分)

### Step 2.1: sphinx-apidoc で自動生成

```bash
cd docs_sphinx
sphinx-apidoc -o source/api ../pybullet_fleet -f -e
```

生成されるファイル:
```
source/api/
├── modules.rst                          # モジュール一覧
├── pybullet_fleet.action.rst
├── pybullet_fleet.agent.rst
├── pybullet_fleet.agent_manager.rst
├── pybullet_fleet.core_simulation.rst
├── pybullet_fleet.geometry.rst
├── pybullet_fleet.rst                   # トップ
└── ... (すべてのモジュール)
```

### Step 2.2: api/index.rst 作成

```rst
.. docs_sphinx/source/api/index.rst

API Reference
=============

This section contains the API documentation for PyBulletFleet, automatically
generated from the source code docstrings.

Core Modules
------------

.. autosummary::
   :toctree: generated
   :recursive:

   pybullet_fleet.core_simulation
   pybullet_fleet.agent
   pybullet_fleet.agent_manager
   pybullet_fleet.sim_object

Actions & Geometry
------------------

.. autosummary::
   :toctree: generated
   :recursive:

   pybullet_fleet.action
   pybullet_fleet.geometry

Utilities
---------

.. autosummary::
   :toctree: generated
   :recursive:

   pybullet_fleet.tools
   pybullet_fleet.types
   pybullet_fleet.logging_utils
   pybullet_fleet.data_monitor
   pybullet_fleet.collision_visualizer

Full API
--------

.. toctree::
   :maxdepth: 2

   modules
```

---

## Phase 3: 既存MDの統合 (1時間)

### Step 3.1: シンボリックリンクで既存MDを参照

```bash
cd docs_sphinx/source

# design/ ディレクトリ作成
mkdir -p design

# 既存MDへのシンボリックリンク
ln -s ../../../DESIGN.md design/architecture.md
ln -s ../../../docs/COLLISION_DETECTION_DESIGN.md design/collision.md
ln -s ../../../docs/COLLISION_DETECTION_DESIGN_v3.md design/collision_v3.md

# advanced/ ディレクトリ作成
mkdir -p advanced

ln -s ../../../docs/PERFORMANCE_ANALYSIS.md advanced/performance.md
ln -s ../../../docs/MEMORY_PROFILING_GUIDE.md advanced/memory_profiling.md
ln -s ../../../docs/MEMORY_PROFILING_IMPROVEMENTS.md advanced/memory_improvements.md
```

### Step 3.2: design/index.rst 作成

```rst
.. docs_sphinx/source/design/index.rst

Design & Architecture
=====================

This section contains design documents and architectural decisions.

Core Architecture
-----------------

.. toctree::
   :maxdepth: 2

   architecture
   collision
   collision_v3

See also: :doc:`../api/pybullet_fleet.core_simulation` for API details.
```

### Step 3.3: advanced/index.rst 作成

```rst
.. docs_sphinx/source/advanced/index.rst

Advanced Topics
===============

Performance Optimization
------------------------

.. toctree::
   :maxdepth: 2

   performance
   memory_profiling
   memory_improvements

Development History
-------------------

For historical context, see the following documents in the ``docs/`` directory:

* ``DEVELOPMENT_HISTORY.md``
* ``REFACTORING_SUMMARY.md``
* ``OPTIMIZATION_RESULTS.md``
```

---

## Phase 4: チュートリアル作成 (2-4時間)

### Step 4.1: tutorials/index.rst

```rst
.. docs_sphinx/source/tutorials/index.rst

Tutorials
=========

Step-by-step guides for common tasks.

.. toctree::
   :maxdepth: 2

   basic_simulation
   spawning_robots
   action_system
   collision_detection
   performance_profiling
```

### Step 4.2: tutorials/basic_simulation.rst (例)

```rst
Basic Simulation
================

This tutorial shows how to create your first PyBulletFleet simulation.

Installation
------------

.. code-block:: bash

   cd PyBulletFleet
   pip install -e .

Creating a Simple Simulation
-----------------------------

.. code-block:: python

   from pybullet_fleet.core_simulation import MultiRobotSimulationCore

   # Load configuration
   sim = MultiRobotSimulationCore.from_yaml("config/config.yaml")

   # Initialize
   sim.initialize_simulation()

   # Run for 10 seconds
   sim.run_simulation()

See :class:`pybullet_fleet.core_simulation.MultiRobotSimulationCore` for
API details.

Next Steps
----------

* :doc:`spawning_robots` - Add robots to your simulation
* :doc:`action_system` - Make robots perform tasks
```

---

## Phase 5: ビルドとデプロイ (30分)

### Step 5.1: ローカルビルド

```bash
cd docs_sphinx
make html

# ブラウザで確認
firefox build/html/index.html
# or
python -m http.server --directory build/html 8000
# → http://localhost:8000
```

### Step 5.2: ReadTheDocs 統合

```yaml
# .readthedocs.yml (新規作成)
version: 2

build:
  os: ubuntu-22.04
  tools:
    python: "3.10"

sphinx:
  configuration: docs_sphinx/source/conf.py

python:
  install:
    - method: pip
      path: .
      extra_requirements:
        - dev
```

### Step 5.3: GitHub Actions で自動ビルド

```yaml
# .github/workflows/docs.yml (新規作成)
name: Documentation

on:
  push:
    branches: [main, devel]
  pull_request:
    branches: [main]

jobs:
  build-docs:
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v3

      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.10'

      - name: Install dependencies
        run: |
          python -m pip install --upgrade pip
          pip install -e .[dev]
          pip install sphinx sphinx-rtd-theme sphinx-autodoc-typehints myst-parser

      - name: Build documentation
        run: |
          cd docs_sphinx
          make html

      - name: Upload artifacts
        uses: actions/upload-artifact@v3
        with:
          name: documentation
          path: docs_sphinx/build/html/
```

---

## Phase 6: pyproject.toml 更新 (5分)

```toml
# pyproject.toml に追加

[project.optional-dependencies]
dev = [
    "black>=23.0.0",
    "isort>=5.12.0",
    "flake8>=6.0.0",
    "mypy>=1.0.0",
    "pre-commit>=3.0.0",
    "pytest>=7.0.0",
    "pytest-cov>=4.0.0",
]
docs = [
    "sphinx>=5.0.0",
    "sphinx-rtd-theme>=1.2.0",
    "sphinx-autodoc-typehints>=1.22.0",
    "myst-parser>=1.0.0",
]

[tool.sphinx]
source-dir = "docs_sphinx/source"
build-dir = "docs_sphinx/build"
```

---

## メリット比較

### 現状 (Markdown のみ)

| 項目 | 状態 |
|------|------|
| API リファレンス | ❌ なし |
| 検索機能 | ❌ なし (grep のみ) |
| クロスリファレンス | ❌ 手動リンク |
| バージョン管理 | ❌ なし |
| 外部公開 | ⚠️ GitHub のみ |
| メンテナンス | ✅ 簡単 |
| 学習コスト | ✅ 低い |

### Sphinx 移行後

| 項目 | 状態 |
|------|------|
| API リファレンス | ✅ 自動生成 |
| 検索機能 | ✅ 全文検索 |
| クロスリファレンス | ✅ 自動リンク |
| バージョン管理 | ✅ 複数バージョン対応 |
| 外部公開 | ✅ ReadTheDocs |
| メンテナンス | ⚠️ やや複雑 |
| 学習コスト | ⚠️ 中程度 |

---

## タイムライン

### 即日実施 (2-3時間)
- [x] Phase 1: Sphinx セットアップ
- [x] Phase 2: API リファレンス自動生成
- [x] Phase 3: 既存MDの統合

### 1週間以内 (4-6時間)
- [ ] Phase 4: チュートリアル作成
- [ ] Phase 5: ビルドとデプロイ
- [ ] Phase 6: pyproject.toml 更新

### 継続的 (随時)
- [ ] 既存MDの内容を Sphinx に統合
- [ ] チュートリアルの拡充
- [ ] ReadTheDocs での公開

---

## FAQ

### Q: 既存のMarkdownファイルは削除すべき？
**A: いいえ。** シンボリックリンクで参照し続けることで、両方の利点を活かせます。

### Q: reStructuredText を学ぶ必要がある？
**A: 最小限でOK。** MyST Parser により Markdown でほとんど書けます。

### Q: ReadTheDocs は必須？
**A: いいえ。** ローカルビルドや GitHub Pages でも公開可能です。

### Q: 既存ドキュメントの品質は十分？
**A: はい。** docstring が豊富なので、autodoc で高品質なAPIリファレンスが生成されます。

---

## 結論

**推奨: Phase 1-3 を今すぐ実施 (2-3時間)**

これにより:
1. ✅ API リファレンス自動生成
2. ✅ 既存MDの検索可能化
3. ✅ 将来の拡張に備えた基盤

**既存のMarkdownは削除せず、シンボリックリンクで活用**することで、移行コストを最小化しつつ、Sphinxの恩恵を受けられます。
