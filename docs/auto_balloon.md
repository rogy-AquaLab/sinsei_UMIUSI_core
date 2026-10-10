# 風船割り（Auto モード）

Auto モードに入ると `auto_target_generator` が有効になり、前カメラの映像から風船を探して
`/cmd/target`（前後・上下）と `/cmd/attitude_target`（yaw_rate・hold_yaw）を出す。
姿勢の安定化は control の `attitude_controller` に任せる。

```
control: gst_camera_node ──共有メモリ /tmp/umiusi_cam1.sock（BGR 320x240）──┐
                                                                          ▼
core: auto_target_generator
   ├ sinsei_umiusi_core/balloon/detector.py  色のしきい値で風船を探す
   ├ sinsei_umiusi_core/balloon/mission.py   行動決定（ROS 非依存）
   └ /cmd/target, /cmd/attitude_target, ~/status（JSON）
```

検出器の評価ツールと評価セットは別リポジトリ `sinsei_umiusi_ballon_detector` にある。
`detector.py` / `config.py` はそこからのコピーなので、直すときは両方を揃える。

## 前提

- control を共有メモリ付きで起動する:
  `ros2 launch sinsei_umiusi_control main.yaml cameras_param_file:=<share>/params/cameras_shm.yaml`
- 共有メモリを読む `shmsrc` は GStreamer の plugins-bad に入っている（control の `shmsink` と同じ）。
  入っていないと `no element "shmsrc"` と出て映像が来ない（`sudo apt install gstreamer1.0-plugins-bad`）
- autonomy（`umiusi_autonomy`）の launch は**起動しない**。navigator などが `/cmd/target` を出して取り合いになる
- Auto の間は rosbridge のクライアントが 0 になっても電源を落とさない（LAN を抜いて走るため）。
  止めるには、持ち時間切れを待つ（推力 0、正浮力で浮く）か、LAN をつなぎ直して Standby に戻す

## 行動の流れ

| 状態 | やること | 次へ |
|---|---|---|
| WAIT | Auto に入ってから `start_delay` 秒、推力 0（LAN を抜いて手を離す） | 時間 |
| DIVE | 真下へ潜る（`dive_heave`） | 狙う色が見えた / `dive_time` |
| APPROACH | 目標を照準（`aim_x`, `aim_y`）に合わせながら近づく。青が目標より大きく正面にあれば止まって避ける | 大きく・照準内 / 見失った |
| RAM | `ram_time` 秒まっすぐ突っ込む | 時間 |
| BACKOFF | `backoff_time` 秒下がる | 時間 |
| SEARCH | その場で回って探す | 狙う色が見えた |
| END | `mission_time` 秒経ったら推力 0 | — |

最初は `first_color`（黄）だけを狙い、一度突っ込むか `first_only_time` 秒経ったら `priority`（赤 → 黄）を狙う。
青は狙わない。割れたかどうかは判定しない（まだ見えていればもう一度狙う。同じ色に `max_rams` 回続いたら
`ignore_time` 秒無視する）。

パラメータは `config/auto_target_generator.yaml`。数値は型を合わせる（`15` ではなく `15.0`）。

## プールに入る前に（陸上）

### 1. 向きの確認（必須）

行動決定の約束は「surge 正 = 前、heave 正 = 上、yaw_rate 正 = 左旋回（上から見て反時計回り）」。
control 側の向きと合っているかを、陸上で 1 つずつ確かめる。逆なら `sign_surge` / `sign_heave` / `sign_yaw` を `-1.0` にする。

```bash
# Manual で、UI を使わずに 1 軸だけ出す例（スラスタの水流の向きを手で確かめる）
ros2 topic pub -r 10 /user_input/target sinsei_umiusi_msgs/msg/Target "{velocity: {z: 0.2}}"   # 浮上のはず
ros2 topic pub -r 10 /user_input/attitude_target sinsei_umiusi_msgs/msg/AttitudeTarget "{attitude: {w: 1.0}, yaw_rate: 0.3}"  # 左旋回のはず
```

### 2. 再生で反応の向きを確かめる（ROS 不要）

```bash
python3 scripts/replay_auto.py <録画> --detector config/balloon/noir.json --out /tmp/replay
```

一覧画像の矢印（yaw）・縦棒（heave）・下の帯（surge）が、見えている風船へ向かう向きになっていること。

### 3. 単体テスト

```bash
python3 -m pytest test/python
```

## プールで

1. LAN をつないだまま Auto に入れ、`ros2 topic echo /auto_target_generator/status` で状態と検出を見る
2. 毎回 bag を取る（`/auto_target_generator/status`, `/cmd/*`, `/state/imu`）。
   `save_dir` に生フレームと検出結果が残るので、色合わせに使う
3. 本番の手順: Auto に入れる → `start_delay` 秒以内に LAN を抜いて機体を放す

## 色のしきい値（色合わせ）

**同じカメラでも日・水・照明で色の写り方が大きく変わる**（NoIR では 10/03 の濁った水で青い風船が青く、9/13 ではピンクに写った。
2024 年の本番プールでは、他チームのカメラで 黄 → 黄緑、赤 → えんじ〜紫 に写った）。プールに入るたびに合わせ直す。
`config/balloon/noir.json` / `normal.json` は叩き台。

画像を PC に送らずに、機体の上で色合わせできる:

1. 初回だけ: `config/auto_target_generator.yaml` の `detector_config` を `"/home/pi/umiusi_balloon/current.json"`
   （絶対パス）にする。`current.json` が無いとノードが起動しないので、先に 2〜4 を一度やっておく
2. 撮る（どのモードでもよい。手動で機体を回しながら、3 色の風船を近く・遠く、背景も入れて 1 分ほど）:

   ```bash
   python3 scripts/capture_frames.py
   ```

   `~/umiusi_balloon/capture/<日時>/` に 0.5 秒ごとに保存される。Ctrl-C で止める
3. 色合わせの画面を機体で起動し、**ノート PC のブラウザで `http://<機体のホスト名>.local:8766` を開く**:

   ```bash
   python3 scripts/calibrate_server.py
   ```

   一番新しい撮影フォルダを読む。撮り直したら画面の「最新の撮影を読み込む」
4. 色ごとに近い風船と遠い風船を 5〜10 か所クリック、背景（水・奥の暗い水・壁・床・錘）も同じくらい →「提案」→
   マスク表示（`M`）で確かめる → 名前を付けて保存。`~/umiusi_balloon/<名前>.json` と `current.json` ができる
5. `auto_target_generator` は `current.json` を 1 秒ごとに見ていて、**保存すると止めずに読み直す**
   （ログに「色の設定を読み直した」）。UI の枠で確かめ、合っていなければ 4 に戻る

- 走行中は色合わせの画面を止めておく（CPU を使う）
- 書き込みは一時ファイルからの置き換えなので、書きかけを読むことはない。壊れたファイルを手で置いた場合は前の設定のまま動く
- 撮影スクリプトとノードは同じ共有メモリを同時に読む（GStreamer の shmsink は複数の読み手を受け付ける。実機で要確認）
- 提案の計算は `sinsei_umiusi_core/balloon/calibrate.py`（色相は外れ値を除いて余白を足す。赤は 0/179 をまたぐので 2 つ。
  S と V は下限だけで、背景のクリックが 5% 以下しか入らなくなるまで上げる）
