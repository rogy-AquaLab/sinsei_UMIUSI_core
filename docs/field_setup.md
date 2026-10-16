# 実機（Pi）の現場の運用と設定

2026-10-15 のプール実験で使った起動の道具（`scripts/field/`）と、Pi に入れたシステム設定の記録。
Pi を作り直すときは「Pi の設定」を上から順に。

## 起動の流れ（scripts/field/）
```bash
~/ros2-ws/src/sinsei_umiusi_core/scripts/field/start_all.sh --record
#  偽装（fake_health.sh）-> core -> UI で Connect -> Main Power On -> control（control.sh、前カメラはピント固定）-> 記録（record.sh）
~/ros2-ws/src/sinsei_umiusi_core/scripts/field/stop_all.sh     # 全部止める（bag・映像はきれいに閉じる）
```

| 道具 | 何をするか |
|---|---|
| `env.sh` | ROS・libcamera・DDS（LOCALHOST）・onnxruntime の環境。ssh の非対話シェルからでも同じになるよう、道具はみなこれを読む。`UMIUSI_WS` / `UMIUSI_OVERLAY` / `UMIUSI_ORTLIB` で場所を変えられる |
| `fake_health.sh` | ヘルスチェックの偽装（分電盤の実装まで）。low power は control が本物を出し始めたら自動で止める（両方出ると UI の値が混ざる） |
| `control.sh [af]` | control を `cameras_shm.yaml`（前カメラを共有メモリにも出す）で起動。既定は前カメラのピント固定（`LENS`、既定 1.0 = 1 m）。`af` で連続 AF |
| `record.sh [名前]` | bag（指令・状態・自律の status・IMU・スラスタ）と映像（前・下、H.264 のまま、autonomy の `tools/record_camera.sh`）を記録 |
| `power_reenable.sh` | control の起動後に強電が落ちたとき、入れ直す応急（下の「既知の問題」） |
| `calib.sh capture/serve` | 色合わせ。`serve` で `http://<機体>:8766/`。保存すると auto_target_generator が 1 秒以内に読み直す |
| `meas.sh` | 15 秒の CPU（プロセスごと）と status の周期 |
| `bag_stats.py <bag>` | bag の status から、検出の周期・接近中に見えたコマ・見失い・旋回の反転を集計 |
| `det_probe.py` | 前カメラで HSV と CNN（モデルを複数並べられる）を回し、1 秒ごとに色ごとの見えた数・一番大きい半径 r・確信度、最後に検出 1 回の時間を出す。`--save <dir>` で枠を描いた jpg。会場でどこまで遠くが見えるか・モデルの Pi での時間を測る用（走行中でも使える）。onnxruntime を使うので `source scripts/field/env.sh` してから |
| `watch_run.py <out.tsv>` | 1 秒ごとに robot_state・クライアント数・自律の状態・指令を記録（rosbridge を使わない。無線の試験用） |
| `rosbridge_keepalive.py` | 何も購読しない接続を 1 本張る。UI を閉じて計測するとき、Standby で電源が落ちないように |

## 現場で気をつけること
- **Standby / Manual で UI（rosbridge のクライアント）が全部切れると強電が落ちる。** AUTO の間は落ちない（競技中は通信が切れる前提）。
- **AUTO は「スタートの合図」**: `mission.start_delay`（既定 10 s）で動き出し、`mission.mission_time` で推力 0。値は UI の Auto タブで変えられ、次の AUTO から効く。
- 前カメラの連続 AF は水中でピントを探し続ける → `control.sh`（ピント固定）。
- CNN は Pi で重い（負荷下で 1 回 76〜226 ms）。UI をつないだままだと rosbridge と映像の配信が CPU を取り、さらに落ちる。本線は `detector: hsv`（現場で色合わせ）か `hsv+cnn`。

## Pi の設定（2026-10-15 時点、umiusi2 = Ubuntu 24.04、ROS 2 Jazzy）
### DDS を Pi の中に閉じる
`~/.bashrc` に（`env.sh` と揃える）:
```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```
有線のアドレスが変わったとき、起動済みのノード間の通信が壊れた（UI から control が止まって見えた）ため。PC から `ros2` の CLI で Pi のトピックは見えなくなる（Pi に ssh して見る）。

### 会場用の Wi-Fi アクセスポイント
Pi が AP になり、スマホや PC を直接つなぐ（会場の Wi-Fi に頼らない）。UI は `http://192.168.50.1:8080/`。有線と同時に使える。
```bash
sudo apt install --no-install-recommends hostapd
# /etc/hostapd/hostapd.conf（600）: interface=wlan0, driver=nl80211, ssid=<SSID>, country_code=JP, ieee80211d=1,
#   hw_mode=g, channel=6, ieee80211n=1, wmm_enabled=1, wpa=2, wpa_key_mgmt=WPA-PSK, rsn_pairwise=CCMP, wpa_passphrase=<8 文字以上>
# /etc/systemd/network/30-wlan0-ap.network:
#   [Match] Name=wlan0
#   [Network] Address=192.168.50.1/24  DHCPServer=yes  ConfigureWithoutCarrier=yes  LinkLocalAddressing=no
#   [DHCPServer] PoolOffset=10  PoolSize=40  EmitDNS=no  EmitRouter=no
sudo systemctl unmask hostapd && sudo systemctl enable --now hostapd && sudo networkctl reload
```
- 戻すとき: `sudo systemctl disable --now hostapd; sudo rm /etc/systemd/network/30-wlan0-ap.network; sudo networkctl reload`
- AP を立てた直後に Pi が一度予告なく再起動した（原因未確定。電源の落ち込みの疑い）。AP を立てたままスラスタを回して確かめること。
- 水中では無線は届かない。AUTO は切れても続き、上がったら UI が自動でつなぎ直す（ui の自動再接続）。
- rosbridge は切れた UI を、websocket の ping（launch で 2 s / 5 s）で数秒で外す。無いと約 2 分残った。

### onnxruntime（任意。CNN を少し速くする）
Ubuntu 24.04 の apt に無い。システムの numpy（1.26）のまま動く:
```bash
python3 -m pip install --no-deps --break-system-packages --target ~/ortlib onnxruntime
```
`env.sh` が `~/ortlib` を `PYTHONPATH` に足す。Pi（負荷下、2 スレッド）で OpenCV の dnn 89 ms → 76 ms、検出は一致。

### PC から Pi へ（研究室の構成）
mk2（Windows）の ICS 配下。**mk2 の USB LAN アダプタを挿し直すと「イーサネット N」の番号が変わり、ICS が外れることがある**
→ Wi-Fi のプロパティの「共有」で共有先を選び直す。Pi のアドレスは DHCP なので `umiusi2.local` で指す。
ICS が外れると両側が 169.254（link-local）になり、Pi は外に出られず時計も合わず、UI の映像（WebRTC）も届かない。

## 既知の問題
- **control を起動すると CAN の健全性が ERROR（`/state/low_power_circuit_info` の can: 1）になり、強電を入れ直せない。**
  VESC の "Unsupported packet status variant" が原因とみられる（control 側の修正が要る）。応急は `power_reenable.sh`。
- control に `/cmd/target` の受信タイムアウトが無い（指令を出すノードが止まると最後の指令が残る）。auto_target_generator が落ちたときは robot_strategy の Auto が推力 0 を出す（core #88）。
