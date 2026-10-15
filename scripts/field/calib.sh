#!/bin/bash
# 色合わせ（手動のクリック / CNN からの自動提案）。scripts/calibrate_server.py を使う。
#   calib.sh capture [秒]   前カメラを 0.5 秒ごとに撮る（既定 30 秒）。風船を近く・遠く・暗い所など色々写す
#   calib.sh serve          一番新しい撮影で色合わせの画面を出す -> http://<機体>:8766/
# 保存すると ~/umiusi_balloon/current.json が置き換わり、auto_target_generator が 1 秒以内に読み直す。
# serve のたびに今の current.json を ~/umiusi_balloon/backup/ に控える（戻すときはそれを current.json へ cp）
F="$(dirname "$(readlink -f "$0")")"
source "$F/env.sh"
case "$1" in
  capture) python3 "$F/../capture_frames.py" --seconds "${2:-30}" ;;
  serve)
    mkdir -p ~/umiusi_balloon/backup
    [ -f ~/umiusi_balloon/current.json ] && cp ~/umiusi_balloon/current.json ~/umiusi_balloon/backup/$(date +%m%d-%H%M%S)-current.json
    python3 "$F/../calibrate_server.py" --port 8766 ;;
  *) sed -n 2,6p "$0" ;;
esac
