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
- Auto の間は rosbridge のクライアントが 0 になっても電源を落とさない（水中では無線が届かず、競技中は通信もできないため）。
  止めるには、持ち時間切れを待つ（推力 0、正浮力で浮く）か、つなぎ直して Standby に戻す
- `auto_target_generator` が落ちても Auto は続く。launch が立ち上げ直し（respawn）、robot_strategy の Auto は
  指令が 1 s 来なければ推力 0 を出しつつ configure → activate し直す。持ち時間は `run_started_at`（Auto に入った時刻）を
  渡して数え続ける（立ち上げ直しで持ち時間が延びない）
- `preview: true`（既定）で、Auto でない間も認識だけ回して `~/status` に出す（UI で認識と色合わせを確かめる用。推力は出さない）。
  その分 Standby / Manual でもカメラの読み込みと検出の CPU を使う。Pi が重いときは `preview: false`

## 行動の流れ

| 状態 | やること | 次へ |
|---|---|---|
| WAIT | Auto に入れて（= スタートの合図）から `start_delay` 秒、推力 0（水に入れて手を離す） | 時間 |
| DIVE | 真下へ潜る（`dive_heave`） | 狙う色が見えた / `dive_time` |
| APPROACH | 目標を照準（`aim_x`, `aim_y`）に合わせながら近づく。青が目標より大きく正面にあれば止まって避ける | 大きく・照準内 / 見失った |
| RAM | `ram_time` 秒まっすぐ突っ込む | 時間 |
| BACKOFF | `backoff_time` 秒下がる | 時間 |
| SEARCH | その場で回って探す | 狙う色が見えた |
| END | 動き出してから `mission_time` 秒経ったら推力 0（正浮力で浮く） | — |

最初は `first_color`（黄）だけを狙い、一度突っ込むか `first_only_time` 秒経ったら `priority`（赤 → 黄）を狙う。
青は狙わない。割れたかどうかは判定しない（まだ見えていればもう一度狙う。同じ色に `max_rams` 回続いたら
`ignore_time` 秒無視する）。

パラメータは `config/auto_target_generator.yaml`。数値は型を合わせる（`15` ではなく `15.0`）。
`mission.*` は Auto に入るたびに読み直すので、持ち時間などは起動したまま変えられる（次の Auto から効く）:

```bash
ros2 param set /auto_target_generator mission.mission_time 300.0
ros2 param set /auto_target_generator mission.start_delay 10.0
```

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
3. 本番の手順: 無線でつないだまま Auto に入れる（スタートの合図）→ `start_delay` 秒以内に水に入れて手を離す

## 色のしきい値（色合わせ）

**同じカメラでも日・水・照明で色の写り方が大きく変わる**（NoIR では 10/03 の濁った水で青い風船が青く、9/13 ではピンクに写った。
2024 年の本番プールでは、他チームのカメラで 黄 → 黄緑、赤 → えんじ〜紫 に写った）。プールに入るたびに合わせ直す。
`config/balloon/noir.json` / `normal.json` は叩き台。

画像を PC に送らずに、機体の上で色合わせできる:

1. 何も設定しなくてよい: ノードは既定で `~/umiusi_balloon/current.json` を読み、無ければ同梱の `normal.json` で動く。
   **UI の前カメラの左上に「色の設定: current.json」と出ていれば、色合わせの結果で動いている**
   （「normal.json（予備）」なら、まだ色合わせの結果を読んでいない）
2. 撮る（どのモードでもよい。手動で機体を回しながら、3 色の風船を近く・遠く、背景も入れて 1 分ほど）:

   ```bash
   python3 scripts/capture_frames.py
   ```

   `~/umiusi_balloon/capture/<日時>/` に 0.5 秒ごとに PNG で保存される（JPEG だと色がにじんで、走行中とずれる）。Ctrl-C で止める
3. 色合わせの画面を機体で起動し、**ノート PC のブラウザで `http://<機体のホスト名>.local:8766` を開く**:

   ```bash
   python3 scripts/calibrate_server.py
   ```

   一番新しい撮影フォルダを読む。撮り直したら画面の「最新の撮影を読み込む」
4. 画面の手順どおりに進める: 1 黄 → 2 赤 → 3 青 → 4 風船以外（それぞれクリックで 5 か所以上）→ 5 結果。
   結果では見つかった所が色で塗られ、色ごとに「OK」や次にすること（例: 風船以外も拾っている → 4 でその場所をクリック）が出る。
   良ければ名前を付けて「保存して機体に反映」。`~/umiusi_balloon/<名前>.json` と `current.json` ができる。
   「詳しい設定」には色の範囲の数字、水の色を打ち消す補正（gray world）の切り替え、撮り直した画像の読み込みがある
5. `auto_target_generator` は `current.json` を 1 秒ごとに見ていて、**保存すると止めずに読み直す**
   （ログに「色の設定を読み直した」）。UI の枠で確かめ、合っていなければ 4 に戻る

- 走行中は色合わせの画面を止めておく（CPU を使う）
- 書き込みは一時ファイルからの置き換えなので、書きかけを読むことはない。壊れたファイルを手で置いた場合は前の設定のまま動く
- 撮影スクリプトとノードは同じ共有メモリを同時に読む（GStreamer の shmsink は複数の読み手を受け付ける。実機で要確認）
- 提案の計算は `sinsei_umiusi_core/balloon/calibrate.py`（色相は外れ値を除いて余白を足す。赤は 0/179 をまたぐので 2 つ。
  S と V は下限だけで、背景のクリックが 5% 以下しか入らなくなるまで上げる）

## CNN で検出する（色合わせ不要）

色のしきい値の代わりに、学習した CNN（umiusi_sim の TinyBalloonNet、ONNX）でも検出できる。色合わせは要らない。
`detector` で切り替え、走行中でも切り替わる（行動決定はどちらでも同じ）:

```bash
ros2 param set /auto_target_generator detector cnn   # 戻すときは hsv
ros2 param set /auto_target_generator cnn_model balloon_cnn_20261015.onnx
```

- モデルは `config/balloon/models/`。`balloon_cnn_20261007.onnx` = 実機の autonomy で使ってきたもの（既定）、
  `balloon_cnn_20261015.onnx` = ラベルを直して学び直したもの（F1: JAMSTEC の映像 0.30 → 0.62、反射の誤検出 85 → 35、
  10/03 0.56 → 0.62、9/13 自プール 0.61 → 0.62。実機では未確認）
- `cnn_conf`（既定 0.3）より確信度の低い検出は捨てる。0.4 にすると濁ったプール（10/03）でほぼ全部消える
- CNN を読めなければログに出して色の検出で続ける。`~/status` の `detector` が今使っている方、検出ごとの `s` が確信度
- 推論は onnxruntime があればそれ、無ければ OpenCV の dnn。結果は同じ（Pi の OpenCV 4.6 と onnxruntime 1.31 で 174 コマ一致）。
  スレッド数は `cnn_threads`（既定 2）。Pi（umiusi2、カメラ・制御・UI を動かしたまま、1 コマ平均）:
  onnxruntime 2 スレッド 76 ms / OpenCV 2 スレッド 89 ms / OpenCV 4 スレッド 106 ms（色の検出は 19 ms）。
  UI をつないだままだと rosbridge と映像の配信が重く、CNN の周期が 3〜7 Hz に落ちる（切れば 9.5 Hz）
- onnxruntime は Ubuntu 24.04 の apt に無い。Pi では `pip install --no-deps --break-system-packages --target ~/ws-claude/ortlib onnxruntime`
  で入れ、`PYTHONPATH` に足す（システムの numpy 1.26 のまま動く）
- 入力の大きさは onnxruntime ならモデルから読む。OpenCV の dnn で読むときは 320 決め打ちなので、320 以外のモデル（軽量化した 224 など）は onnxruntime で使う
- `radius` は色の検出器に合わせて「本来の比率に戻した画像で、箱の長い辺の半分 / 画像幅」。
  `source_aspect` は `detector_config` の値を使う。近さのしきい値（`ram_radius` など）は色の検出器で決めたものなので、
  CNN の箱が風船より少し大きく / 小さく出ると突っ込む距離が変わる。プールで確かめる
- 復号は `sinsei_umiusi_core/balloon/cnn.py`。学習側（umiusi_sim `learned_detector.decode`）と 44 枚で検出が一致する

### 検出を別スレッドで回す / 色が主で CNN が補助（hsv+cnn）
- 検出は別スレッド（`sinsei_umiusi_core/balloon/async_detect.py` の `DetectionWorker`）で回し、指令は検出の遅さに関係なく `rate_hz` で出す。
  CNN を周期の中で回していたときは、指令の更新が 3〜6 Hz に落ちて接近中に左右へ振れた（2026-10-15 プール）。
  `~/status` の `det_ms` が検出 1 回にかかった時間
- `detector: hsv+cnn`: 色の検出が主。色で何も見つからないコマだけ、補助のスレッドの CNN の結果を使う（`~/status` の `assist_used`）。
  - `cnn_assist_max_age`（0.5 s）より古いコマの結果は使わない
  - `cnn_assist_conf`（0.5）より確信度の低い検出は使わない（`cnn_conf` とは別。CNN は反射・タイルに高い確信度を出すため）
  - `cnn_assist_confirm`（true）なら、前回の CNN でも近く（画面の幅・高さの 0.15 以内）に同じ色があったものだけ使う
- 10/15 のプールでは、hsv+cnn は検出 9.2 Hz・突進 3 回で、ほぼ hsv と同じ動きで割れた（CNN 単独は検出 4.4 Hz・突進 0）

### CNN の目標のときだけ照準を変える / 重なった箱をまとめる
- 追っている目標が CNN の検出（`detector: cnn`、または hsv+cnn で CNN の結果を使ったコマ）のときだけ、
  `mission.k_yaw_cnn`（既定 0.4、色は `k_yaw` 0.8）と `mission.ram_radius_cnn`（既定 0 = `ram_radius` と同じ）を使う。`~/status` の `target_cnn`
  - CNN は遅れ（周期 + 推論 0.4〜0.5 s）が大きく、10/15 の CNN の走行は `k_yaw` 0.8 で左右に振れ続けた（反転 22 回/分）。
    水中の画角は横の半分が約 0.4 rad と狭く、x 1 あたりの旋回が強く効く。sim を実機の旋回と画角に合わせると 0.4 で振れが消えた
  - CNN の箱は実寸の約 0.86 倍（色は約 1.23 倍）に出て、10/15 は半径が `ram_radius` 0.16 に一度も届かず突進 0 回。
    下げるなら 0.12 前後が候補（未検証）
- `cnn_merge`（既定 true）: CNN が 1 個の風船の上に重ねて出す同じ色の小さい箱（中心が大きい方の箱の半分以内）を、大きい方の 1 個にまとめる。
  小さい方を追うと目標の大きさ（= 距離）が半分に化ける。検出ごとの `src`（hsv / cnn）も `~/status` に出す
