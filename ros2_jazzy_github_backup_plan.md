# バックアッププラン: GitHub自前ホスティングによるapt配布

作成日: 2026-07-25

## 位置づけ

**これは本流(rosdistro/bloom)ルートを置き換えるものではない。公式 APT を標準経路としつつ、先行版・固定デモ版・公式反映待ちの配布経路として継続運用する。**

- rosdistro PR #52864 はそのまま並行してレビュー待ちを継続する(こちらの作業は不要、待つだけ)
- 本流がマージされれば official APT を標準経路として案内する
- 本手順で作った自前リポジトリは、公式反映待ち・固定デモ版・先行検証版の経路として維持する
- RosCon 向けは amd64/Jazzy/Noble の単一構成でよいが、継続公開するなら署名、更新・撤回方針、対応 matrix、CI を整備する

### 【重要・スコープ変更】RMF dispatchはデモ必須と確定

以前は「RMFは第二弾(msgs/rosのros-testing反映後)」としていたが、ROSConのデモで
**RMFのdispatch(フリート間の交通制御)を見せることが必須**と確定したため、本バックアップ
プランも含めて**msgs + ros + rmfの3パッケージすべてを対象とする**方針に変更する。

ただし、以前から指摘していた「bloomが通ることと、apt install後にlaunchが実際に動くことは別」
というリスクは、必須になった今こそ最優先で潰すべき項目になる。残り4日という制約下では、
**RMFの実起動検証(rmf_demosのassets/launch前提の有無)が最大のクリティカルパス**。

**リスク分散策**: RMF部分は不確実性が最も高いため、apt化を目指しつつ、並行して
「動作確認済みのDockerイメージ」もRMFデモ用フォールバックとして用意しておく。
8/3が近づいた時点で、RMFのapt化が間に合っていればそちらを、間に合わなければ
Docker側でRMF部分のデモを実施する、という二段構えにする。

## すでに使える資産(ゼロから作らなくてよい部分)

- `debian/rules`は`bloom-generate rosdebian`で生成済み・動作確認済み(dh_auto_test || trueのガードも確認済み)
- pybulletのPEP668対応pip導入手順(system Python + `pip install --break-system-packages`)は検証中〜確定間近
- package.xml(依存・license・maintainer)は確認済み

### 【要対応】レビューで発覚した2つの問題

1. **`pybullet_fleet_ros`の`debian/control`に`pybullet_fleet_msgs`への依存が入っていない**
   - 原因: debian/rules確認のために一時的にpackage.xmlの依存をコメントアウトして
     `bloom-generate`した際の生成物が、そのまま残っていた可能性が高い
   - 対応: 下記「Step 0」でローカルrosdep上書きを追加した上で、`debian/`を完全に削除して
     再生成し、`cat debian/control`で依存が正しく入っているか確認してから先に進む

2. **dirtyな作業ツリーでビルドしない**
   - 上記の一時的な変更が未コミットのまま残っている可能性があるため、rosdistroの
     source entryと同じタグ(例: `v0.1.0-ros2`)からのclean worktreeでビルドする
   - 生成元commitをバックアップリポジトリのREADMEに記録し、再現性を担保する

## 作業内訳と時間見積もり(RMF必須化により更新)

| # | 作業 | 所要時間 | 備考 |
|---|---|---|---|
| 0 | clean worktree作成、ローカルrosdep上書き整備 | 1時間 | msgs用の上書きを追加、dirty tree問題の解消 |
| 1 | 3パッケージ(msgs+ros+rmf)の`.deb`ビルド | 4〜6時間 | 依存順序厳守。rmf追加で工数増、RMF側は要トラブルシュート |
| 2 | 簡易リポジトリメタデータ生成 + SHA256SUMS | 1〜2時間 | `dpkg-scanpackages`、配布物のチェックサムも同梱 |
| 3 | ホスティング(GitHub Pages優先) | 30分〜1時間 | raw URLは緊急時のみ |
| 4 | GPG署名 | 0時間(省略・デモ限定) | `[trusted=yes]`で対応 |
| 5 | ビルド・index生成のスクリプト化 | 1〜2時間 | コピペ手順ではなく`build_backup_repo.sh`として version管理 |
| 6 | **RMFの実起動検証(最重要・最大の不確実性)** | 4〜8時間+ | rmf_demosのassets/launch前提の有無を洗い出し、必要なら同梱 |
| 7 | クリーン環境での通し動作検証 | 3〜5時間 | apt install、`ros2 pkg prefix`、`ros2 run ... bridge_node`(timeout付き)、core importまで |
| 8 | (保険)RMFデモ用Dockerフォールバックの準備 | 2〜4時間 | apt化が間に合わない場合の代替手段として並行準備 |

**合計目安: 1.5〜2.5日**(bridge単体のみなら半日〜1日で済んでいたが、RMF必須化により増加)

残り4日という期限を考えると、**項目6(RMF実起動検証)と項目8(Dockerフォールバック)を
最優先で今日から並行着手**するのが安全。ここで問題が出るほど後工程への皺寄せが大きい。

---

## 具体的作業手順

### Step 0: clean worktree準備 + ローカルrosdep上書き

```bash
# rosdistroのsource entryと同じタグからclean worktreeを作成(dirty tree問題の解消)
cd ~/PyBulletFleet
git tag v0.1.0-ros2  # 未タグならここで打つ。rosdistro側と揃える
git worktree add /tmp/pbf-clean-build v0.1.0-ros2
cd /tmp/pbf-clean-build
# 以降のビルドはすべてこのclean worktree内で行う(~/PyBulletFleetの作業ツリーは使わない)

# pybullet_fleet_msgs をROSパッケージ名としてローカルで解決可能にする上書き
mkdir -p ~/rosdep_custom
cat > ~/rosdep_custom/pybullet_fleet_msgs.yaml << 'EOF'
pybullet_fleet_msgs:
  ubuntu: [ros-jazzy-pybullet-fleet-msgs]
EOF
echo "yaml file://$HOME/rosdep_custom/pybullet_fleet_msgs.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/09-pybullet-fleet-msgs.list
rosdep update
```

### Step 1: .debビルド(msgs → ros → rmf の順、依存順序厳守)

```bash
# --- pybullet_fleet_msgs ---
cd /tmp/pbf-clean-build/ros2_bridge/pybullet_fleet_msgs
rm -rf debian   # 過去の一時生成物が残っていないことを保証
bloom-generate rosdebian --ros-distro jazzy
rosdep install --from-paths . --ignore-src -r -y
dpkg-buildpackage -us -uc -b
sudo dpkg -i ../ros-jazzy-pybullet-fleet-msgs_*.deb

# --- pybullet_fleet_ros ---
cd /tmp/pbf-clean-build/ros2_bridge/pybullet_fleet_ros
rm -rf debian   # 【重要】以前の一時コメントアウト時の生成物を必ず消してから再生成
bloom-generate rosdebian --ros-distro jazzy
cat debian/control   # ★ここで ros-jazzy-pybullet-fleet-msgs への Depends: が入っているか必ず確認
rosdep install --from-paths . --ignore-src -r -y
dpkg-buildpackage -us -uc -b
sudo dpkg -i ../ros-jazzy-pybullet-fleet-ros_*.deb

# --- pybullet_fleet_rmf(今回からスコープに追加) ---
cd /tmp/pbf-clean-build/ros2_bridge/pybullet_fleet_rmf
rm -rf debian
bloom-generate rosdebian --ros-distro jazzy
cat debian/control   # rmf_fleet_adapter, ros-jazzy-pybullet-fleet-msgs 等の依存を確認
rosdep install --from-paths . --ignore-src -r -y
dpkg-buildpackage -us -uc -b
```

- pybullet(core)は`_ros`・`rmf`どちらもpackage.xmlに書かれていないので、このビルド自体には
  一切影響しない。実行時のpip導入(PEP668対応)はStep 7の検証で別途扱う
- `debian/control`の確認を必ず挟むこと(レビューで発覚した依存欠落の再発防止)

### Step 2: 簡易リポジトリの作成(flatリポジトリ形式・最速)

`reprepro`/`aptly`のような本格ツールは使わず、`dpkg-scanpackages`で素朴に済ませる。

```bash
mkdir -p ~/pybulletfleet-apt-backup/repo
cp ros-jazzy-pybullet-fleet-msgs_*.deb ros-jazzy-pybullet-fleet-ros_*.deb ros-jazzy-pybullet-fleet-rmf_*.deb \
   ~/pybulletfleet-apt-backup/repo/
cd ~/pybulletfleet-apt-backup/repo

dpkg-scanpackages . /dev/null > Packages
gzip -9c Packages > Packages.gz

# 配布物の完全性確認用にチェックサムを同梱
sha256sum *.deb > SHA256SUMS

# 生成元commitを記録(再現性の担保)
echo "Built from commit: $(cd /tmp/pbf-clean-build && git rev-parse HEAD) (tag v0.1.0-ros2)" > BUILD_INFO.txt
```

これで「flatリポジトリ形式」(dists/以下の複雑な階層が不要な簡易形式)が出来上がる。

### Step 3: ホスティング(GitHub Pagesを優先、raw URLは緊急時のみ)

**案A: GitHub Pages(優先・推奨)**
```bash
cd ~/pybulletfleet-apt-backup
git init
git add .
git commit -m "apt backup repo for jazzy demo (commit: $(cd /tmp/pbf-clean-build && git rev-parse --short HEAD))"
git remote add origin https://github.com/yuokamoto/pybulletfleet-apt-backup.git
git push -u origin main
# GitHubのSettings → Pages で公開ブランチを指定して有効化
```

**案B: raw.githubusercontent.com(緊急時のみ・Pages反映が間に合わない場合の代替)**
上記のgit pushだけ済ませれば、`https://raw.githubusercontent.com/yuokamoto/pybulletfleet-apt-backup/main/repo/`
で直接ファイルにアクセス可能。Pagesの反映を待てない緊急時だけ使う。

### Step 4: 利用側(デモ機)の設定

```bash
echo "deb [trusted=yes] https://yuokamoto.github.io/pybulletfleet-apt-backup/repo ./" \
  | sudo tee /etc/apt/sources.list.d/pybulletfleet-backup.list
sudo apt update
sudo apt install ros-jazzy-pybullet-fleet-msgs ros-jazzy-pybullet-fleet-ros ros-jazzy-pybullet-fleet-rmf
```

(Pagesの反映が間に合わない緊急時のみ、URLを`raw.githubusercontent.com`側に置き換える)

### Step 6: RMFの実起動検証(最優先・最大のリスク)

RMFがデモ必須になったため、以下を**他の何よりも先に**着手する:

- `pybullet_fleet_rmf`のlaunchファイルが要求するアセット(building map、traffic graph等)を洗い出す
- `rmf_demos`由来のアセットに依存している場合、それらが`pybullet_fleet_rmf`パッケージ自体に
  同梱されているか、それとも別途`rmf_demos`のインストール・設定が必要かを確認
- 必要な設定・アセットが揃った状態で、実際にdispatch(タスクの割当・複数ロボットの交通制御)が
  動くところまで確認する。「ノードが起動する」だけでは不十分

### Step 7: クリーン環境での通し動作検証

新規のDocker/VM(既存の開発環境ではなく、まっさらな`osrf/ros:jazzy-desktop`等)で以下をtimeout付きで確認する:

```bash
# 1. sources.list設定 → apt install(msgs, ros, rmf)
sudo apt install -y ros-jazzy-pybullet-fleet-msgs ros-jazzy-pybullet-fleet-ros ros-jazzy-pybullet-fleet-rmf

# 2. パッケージが正しくインストールされているか
ros2 pkg prefix pybullet_fleet_ros
ros2 pkg prefix pybullet_fleet_rmf

# 3. pybullet-fleet(core)のpip導入(確定済みのPEP668対応手順)
pip install --break-system-packages pybullet-fleet
python3 -c "import pybullet_fleet; print('OK:', pybullet_fleet.__file__)"

# 4. bridgeノードの起動確認(timeout付き、クラッシュせず一定時間動き続けるか)
timeout 15 ros2 run pybullet_fleet_ros bridge_node --ros-args -p config:=<デモ用config>

# 5. RMF dispatchの起動確認(timeout付き)
timeout 30 ros2 launch pybullet_fleet_rmf <デモ用launchファイル>
```

ここで問題が出た場合のデバッグ時間を多めに見込んでおく(見積もり表の項目6・7はこのバッファ込み)。

---

## rosdistro側との関係(再確認)

- 本手順(msgs+ros+rmf)を残り4日のうち**できるだけ早く**一度通しで済ませ、
  「いつでもデモに使える状態」を作っておく
- rosdistro PR #52864 が8/3までにマージ・ros-testing反映まで完了すれば、デモは本流ルートで実施
- 間に合わなければ、本バックアップの`sources.list`をデモ機に設定して代替する
- どちらのルートで進めても、デモ本番でユーザー(聴衆)に見せるコマンド自体はほぼ同じなので、
  切り替えのリスクは小さい
- **ただしRMFだけは別枠**: apt化(本流・バックアップいずれも)が間に合わない場合に備え、
  動作確認済みのDockerイメージをRMFデモ用フォールバックとして必ず並行準備しておく

## 次のアクション(優先順位順)

- [ ] **(最優先・今日)** RMFの実起動検証に着手(rmf_demosアセット依存の洗い出し、dispatch動作確認)
- [ ] **(最優先・今日と並行)** RMFデモ用Dockerフォールバックの準備開始(保険)
- [ ] clean worktree(`v0.1.0-ros2`タグ)を作成、`~/PyBulletFleet`の未コミット変更を整理
- [ ] msgs用のローカルrosdep上書きを追加
- [ ] `pybullet_fleet_msgs` → `pybullet_fleet_ros`(control確認必須)→ `pybullet_fleet_rmf` の順で`.deb`ビルド
- [ ] flatリポジトリ作成(SHA256SUMS・BUILD_INFO.txt同梱)、GitHub Pagesへpush
- [ ] ビルド・index生成手順を`build_backup_repo.sh`としてスクリプト化・バージョン管理
- [ ] クリーンなDocker/VMで、apt install → ros2 pkg prefix → pip install → bridge起動 →
      RMF dispatch起動まで、timeout付きで通しリハーサル
- [ ] rosdistro PR #52864の状況を継続的に確認、マージされ次第そちらへ一本化
