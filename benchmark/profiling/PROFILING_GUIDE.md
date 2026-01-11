# Profiling Guide - cProfile vs CPU Time Measurement

## Overview

このガイドでは、benchmark/profiling/ で使用している2つの計測手法の違いを説明します。

---

## 1. `cProfile` (プロファイラ)

### 使用場所
- `profiling/agent_update.py` の `profile_# CPU使用率
cpu_utilization = (cpu_time / wall_time) * 100
# 注: 1コア=100%換算。マルチスレッドなら100%超える（例: 4コア使用で最大400%）
```

**なぜこの方法？**
- ✅ オーバーヘッドがない（正確な性能測定）
- ✅ 繰り返し実行して統計を取る（median, mean, stdev）
- ✅ CPU使用率が分かる（マルチコア活用度の指標）
- ✅ 同一環境での比較や回帰検知に強いfile()`
- Python標準ライブラリ

### 目的
**「どの関数が遅いか」を特定する**

### 計測内容
- 各関数の呼び出し回数
- 各関数の累積実行時間
- 関数ごとの平均実行時間
- 関数呼び出しのツリー構造

### 使い方
```python
import cProfile
import pstats

profiler = cProfile.Profile()
profiler.enable()

# 計測したいコード
for agent in agents:
    agent.update(dt=0.01)

profiler.disable()

# 結果を出力
stats = pstats.Stats(profiler)
stats.sort_stats("cumulative")
stats.print_stats(30)  # Top 30関数
```

### 出力例
```
   ncalls  tottime  percall  cumtime  percall filename:lineno(function)
     1000    0.050    0.000    0.150    0.000 agent.py:123(update)
     1000    0.080    0.000    0.080    0.000 {method 'getBasePositionAndOrientation'}
     1000    0.020    0.000    0.020    0.000 geometry.py:45(__init__)
```

### メリット
✅ **詳細な分析**: 関数レベルでボトルネックを特定
✅ **コールグラフ**: 呼び出し関係が分かる
✅ **標準ライブラリ**: 追加インストール不要

### デメリット
❌ **オーバーヘッド**: 計測自体がコードを遅くする（特に関数呼び出しが多いコードで顕著）
❌ **粒度の限界**: Python関数/呼び出し単位の内訳は分かるが、C拡張内部（例: PyBullet API内部）や行単位の詳細までは見えない
❌ **読みにくい**: 大量の関数リストで分かりにくい

### 注意点
⚠️ **C拡張の扱い**: PyBulletはC++側で動作するため、cProfileでは「呼び出し」として時間が記録されるが、C++内部の詳細は出ない
⚠️ **オーバーヘッドの変動**: コードの呼び出し粒度に強く依存（小さい関数を大量に呼ぶと悪化、ループ中身が重いC拡張やI/O中心なら影響小）

### 使うべきケース
- 🔍 **最適化前**: どこが遅いか分からない時
- 🐛 **デバッグ**: 予期しない関数が呼ばれていないか確認
- 📊 **詳細分析**: 関数レベルの詳細が必要な時

---

## 2. `cpu_time` (CPU時間計測)

### 使用場所
- `benchmark/performance_benchmark.py`
- `psutil.Process.cpu_times()` 使用

### 目的
**「実際にCPUを使った時間」を測定する**

### 計測内容
- User time: ユーザー空間でのCPU時間
- System time: カーネル空間でのCPU時間
- Total CPU time = user + system

### 使い方
```python
import psutil

proc = psutil.Process()

# 開始時のCPU時間
cpu_start = proc.cpu_times()
start_cpu_time = cpu_start.user + cpu_start.system

# 計測したいコード
run_simulation()

# 終了時のCPU時間
cpu_end = proc.cpu_times()
end_cpu_time = cpu_end.user + cpu_end.system

# CPU時間の差分
cpu_time_used = end_cpu_time - start_cpu_time
```

### Wall Time との比較
```python
import time

wall_start = time.perf_counter()
cpu_start = cpu_time_s(proc)

# 実行
run_simulation()

wall_time = time.perf_counter() - wall_start
cpu_time = cpu_time_s(proc) - cpu_start

# CPU使用率
cpu_utilization = (cpu_time / wall_time) * 100
```

### メリット
✅ **オーバーヘッドなし**: 計測の影響がほぼゼロ
✅ **安定性**: wall time より安定しやすい（ただし他プロセス負荷でスケジューリングが変わると、wall time や CPU utilization は変動する）
✅ **シンプル**: 数値1つで分かりやすい
✅ **比較可能**: 同一環境での比較や回帰検知に強い（異なる環境比較は条件を揃えないと難しい）

### デメリット
❌ **詳細不明**: どの関数が遅いかは分からない
❌ **粒度が粗い**: プロセス全体の合計時間のみ
❌ **I/O待ちを含まない**: 純粋なCPU時間のみ（スリープやネットワーク待ちは含まれない）

### 注意点
⚠️ **マルチコア環境**: `cpu_time / wall_time * 100` で計算するCPU使用率は、1コア=100%換算。マルチスレッド/マルチプロセスで並列実行すれば100%を超える（例: 4コア分使えば最大400%）
⚠️ **環境依存**: CPU時間はマシン差（CPU速度・コア数・周波数・ターボ・省電力設定）で変わる。異なるマシン間の絶対値比較は慎重に

### 使うべきケース
- ⚡ **性能測定**: 全体的な性能を数値化
- 📈 **最適化効果の検証**: 改善前後の比較
- 🏆 **ベンチマーク**: 異なる設定/環境の比較
- 📊 **統計分析**: 繰り返し実行して平均・標準偏差を取る

---

## 3. `time.perf_counter()` (Wall Time)

### 目的
**「実際にかかった時間」を測定する**

### 計測内容
- 実際の経過時間（壁時計時間）
- CPU時間 + 待ち時間（I/O、スリープなど）

### 使い方
```python
import time

start = time.perf_counter()
# 処理
elapsed = time.perf_counter() - start
```

### 特徴
- **高精度**: ナノ秒レベル
- **実時間**: ユーザーが体感する時間
- **影響を受けやすい**: バックグラウンドプロセス、OS負荷、スケジューリングの影響大

### 注意点
⚠️ **統計的手法**: wall time でも繰り返し測定して統計（median等）を取れば使用可能だが、ノイズが大きいので十分な回数や環境の固定が必要
⚠️ **再現性**: 同じコードでも実行タイミングで結果が変動しやすい

---

## 比較表

| 項目 | cProfile | CPU Time | Wall Time |
|------|----------|----------|-----------|
| **目的** | ボトルネック特定 | CPU使用量測定 | 実時間測定 |
| **粒度** | 関数レベル | プロセス全体 | プロセス全体 |
| **オーバーヘッド** | 有（呼び出し多で大） | ほぼなし | ほぼなし |
| **詳細度** | 高い（Python層） | 低い | 低い |
| **安定性** | 中 | 高（同一環境） | 低（環境影響大） |
| **用途** | 最適化ポイント発見 | 性能評価・回帰検知 | 体感速度・RTF測定 |
| **統計分析** | 不向き（重い） | 向いている | 可能（要多数回） |

---

## 実例: benchmark での使い分け

### `performance_benchmark.py` (CPU Time + Wall Time)

```python
# Spawn時の計測
cpu_spawn_start = cpu_time_s(proc)      # CPU時間
wall_spawn_start = time.perf_counter()  # Wall時間

agents = spawn_agents()

wall_spawn_time = time.perf_counter() - wall_spawn_start
cpu_spawn_time = cpu_time_s(proc) - cpu_spawn_start

# CPU使用率
cpu_percent = (cpu_spawn_time / wall_spawn_time * 100)
```

**なぜこの方法？**
- ✅ オーバーヘッドがない（正確な性能測定）
- ✅ 繰り返し実行して統計を取る（median, mean, stdev）
- ✅ CPU使用率が分かる（マルチコア活用度）
- ✅ 環境依存が少ない（他のマシンと比較可能）

### `profiling/agent_update.py` (cProfile)

```python
profiler = cProfile.Profile()
profiler.enable()

for agent in agents:
    agent.update(dt=0.01)

profiler.disable()
stats = pstats.Stats(profiler)
stats.print_stats(30)
```

**なぜこの方法？**
- 🔍 `Agent.update()` のどこが遅いか分からない
- 🎯 関数レベルでボトルネックを特定したい
- 📊 PyBullet API呼び出しの頻度を知りたい
- 🐛 予期しない処理が走っていないか確認

**制限**:
- ⚠️ PyBulletはC++実装のため、`p.getAABB()`等の呼び出し全体の時間は分かるが、C++内部の詳細（AABB計算、行列演算等）は見えない
- ⚠️ オーバーヘッドがあるため、絶対的な性能値は測れない（相対比較や構造分析に使う）

---

## 補足: その他の計測ツール

### `time.process_time()`

**概要**: Python標準ライブラリの軽量CPU時間計測

```python
import time

start = time.process_time()
# 処理
elapsed = time.process_time() - start
```

**特徴**:
- ✅ 標準ライブラリ（psutil不要）
- ✅ プロセスのCPU時間（user + system）を返す
- ✅ スリープ時間を含まない

**psutil.Process.cpu_times() との違い**:
- `process_time()`: シンプルで手軽、単一プロセス向き
- `psutil`: 詳細情報（user/system分離、メモリ、子プロセス等）が取れる

**使い分け**:
- 単純なCPU時間計測なら `process_time()` で十分
- 詳細な分析やクロスプラットフォーム対応なら `psutil`

---

## 重要な注意事項

### 1. PyBullet（C拡張）のプロファイリング限界

PyBulletはC++で実装されているため：

```python
# cProfile で見えるのは...
p.getBasePositionAndOrientation(body_id)  # ← この呼び出し全体の時間
# C++内部の詳細（AABB計算、行列演算等）は見えない
```

**対策**:
- C拡張の「呼び出し単位」で時間を測る
- 複数のPyBullet APIの相対的なコストを比較
- C++側の詳細が必要なら別のプロファイラ（例: `perf`, `gprof`）が必要

### 2. マルチコア環境でのCPU使用率

```python
cpu_utilization = (cpu_time / wall_time) * 100
```

この値の意味:
- **100%**: 1コアをフル活用
- **200%**: 2コアをフル活用（並列処理）
- **400%**: 4コアをフル活用
- **50%**: 1コアの半分を使用（待ち時間が多い、または軽い処理）

PyBulletシミュレーションは主にシングルスレッドのため、通常は100%前後になります。

### 3. 環境間の性能比較

**同一環境での比較** ✅:
```
マシンA: 改善前 50ms → 改善後 30ms (40%高速化)
```

**異なる環境の比較** ⚠️:
```
マシンA (i7-1185G7): 30ms
マシンB (Ryzen 9):   25ms
→ 単純比較は難しい（CPU性能、周波数、メモリ速度等が異なる）
```

**推奨**:
- 改善効果は「改善率」で評価（30ms → 20ms = 33%改善）
- 異なる環境比較は条件を揃える（同じエージェント数、同じシナリオ、同じPython/PyBulletバージョン）
- システム情報（CPU型番、メモリ等）を記録する ← **performance_benchmark.py で自動記録済み** ✅

---

## 使い分けのガイドライン

### ✅ cProfile を使うべき時

1. **最適化前の調査**
   - 「何が遅いか分からない」→ cProfile で調査

2. **予期しない遅延**
   - 「なぜか遅い」→ cProfile で原因特定

3. **関数レベルの分析**
   - 特定の関数の内部を詳しく見たい

**コマンド例**:
```bash
python benchmark/profiling/agent_update.py --test cprofile --agents 1000
```

---

### ✅ CPU Time を使うべき時

1. **性能ベンチマーク**
   - 全体的な性能を数値化
   - 複数回実行して統計を取る

2. **最適化効果の検証**
   - 改善前: 50ms (CPU), 改善後: 30ms (CPU) → 40%高速化

3. **環境間の比較**
   - マシンAとマシンBで性能比較
   - シナリオAとシナリオBで比較

**コマンド例**:
```bash
python benchmark/run_benchmark.py --sweep 100 500 1000
```

---

### ✅ Wall Time を使うべき時

1. **体感速度の測定**
   - ユーザーが実際に待つ時間

2. **Real-Time Factor (RTF)**
   - シミュレーション速度 / 実時間

3. **タイムアウト制御**
   - 「X秒以内に終わらせる」

---

## 実際のワークフロー

### ステップ1: 全体性能測定（CPU Time）
```bash
python benchmark/run_benchmark.py --agents 1000 --repetitions 5
```
→ 結果: RTF=5.34x, Step=18.72ms

### ステップ2: ボトルネック特定（cProfile）
```bash
python benchmark/profiling/simulation_profiler.py --agents 1000 --steps 100
```
→ 結果: Collision Check = 77.8%の時間を占める

### ステップ3: 詳細プロファイリング
```bash
python benchmark/profiling/collision_check.py --agents 1000
```
→ 結果: getAABB()とspatial hashingが遅い

### ステップ4: 最適化実装
- 2D collision mode実装
- Frequency制御実装

### ステップ5: 効果検証（CPU Time）
```bash
python benchmark/run_benchmark.py --compare no_collision collision_2d_10hz collision_3d_full
```
→ 結果: 3倍高速化を確認！

---

## まとめ

| やりたいこと | 使うツール | 理由 |
|------------|----------|------|
| 性能ベンチマーク | CPU Time + Wall Time | オーバーヘッドなし、統計可能 |
| ボトルネック特定 | cProfile | 関数レベルの詳細分析（Python層） |
| 体感速度測定 | Wall Time | 実際の経過時間（RTF計算に使用） |
| 最適化効果検証 | CPU Time | 同一環境での比較に強い |
| 関数内部の分析 | cProfile + 手動タイマー | 詳細な内訳（ただしC拡張内部は見えない） |
| 環境間の性能比較 | CPU Time + システム情報記録 | 条件を揃えて相対比較 |

**基本方針**:
1. まず **CPU Time + Wall Time** で全体性能を測定
2. 遅い部分を **cProfile** で特定（Python層のボトルネック）
3. 最適化後、再度 **CPU Time** で効果検証
4. **システム情報を記録**して再現性を確保

**注意**:
- cProfileは「どこが遅いか」を知るための診断ツール
- CPU Timeは「どれくらい速いか」を測るベンチマークツール
- 両者を組み合わせて効率的に最適化を進める
