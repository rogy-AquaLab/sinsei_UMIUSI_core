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

## 色のしきい値

`config/balloon/noir.json`（NoIR カメラ・9/13 の市民プールで決めた値）と `normal.json`（通常カメラ・2019 公式動画）は叩き台。
**同じ NoIR カメラでも日によって色の写り方が大きく違う**（10/03 の濁った水では青い風船が青く、9/13 ではピンクに写った）ので、
プールに入るたびに色を合わせ直す。手順は検出器リポジトリの README。
