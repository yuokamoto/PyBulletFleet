#　PyBulletを用いた多ロボット・高速シミュレーションのサンプルプロジェクトです。

## 主な機能
- 複数ロボットの同時シミュレーション（URDFベース）
- パレット搬送・協調動作・衝突判定
- 高速化（10倍速以上、物理演算無効化対応）
- 外部データモニター（tkinter/console対応）
- URDFキャッシュ・バッチ生成・レンダリング制御

## セットアップ
1. Python 3.8+ をインストール
2. 仮想環境推奨: `python -m venv venv && source venv/bin/activate`
3. 依存パッケージ: `pip install -r requirements.txt`

## 実行例
config/config.yaml を必要に応じて編集
```bash
python example/100robots_demo.py
```