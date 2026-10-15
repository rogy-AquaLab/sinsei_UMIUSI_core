#!/bin/bash
# 実験の起動をまとめる（tmux セッション rov）。強電 ON だけは UI で人が押す。
#   start_all.sh            偽装 -> core -> （UI で Main Power On を待つ）-> control（ピント固定）
#   start_all.sh --record   上に加えて、control が上がったら bag と映像の記録も始める
# 止める: stop_all.sh（各ウィンドウに Ctrl-C を送って閉じる）
F="$(dirname "$(readlink -f "$0")")"
source "$F/env.sh"
if tmux has-session -t rov 2>/dev/null; then echo "rov が既にある（stop_all.sh で止めてから）"; exit 1; fi
cd "${UMIUSI_WS:-$HOME/ros2-ws}"
tmux new -d -s rov -n fake "bash -c '$F/fake_health.sh; exec bash'"
tmux new-window -d -t rov -n core "bash -c 'source $F/env.sh && ros2 launch sinsei_umiusi_core main.yaml 2>&1 | tee /tmp/umiusi_core_launch.log; exec bash'"
# UI のビルド済みファイルがあれば :8080 で配る（AP なら http://192.168.50.1:8080/）
UI_DIST="${UMIUSI_UI_DIST:-${UMIUSI_WS:-$HOME/ros2-ws}/src/sinsei_umiusi_ui/dist}"
if [ -d "$UI_DIST" ] && ! tmux has-session -t ui 2>/dev/null; then
  tmux new -d -s ui "python3 -m http.server 8080 -d $UI_DIST"
fi
echo "core を起動した。UI で Connect -> Main Power On を押す（robot_state が 0 になるのを待つ）"
until timeout 6 ros2 topic echo --once --no-daemon /robot_state 2>/dev/null | grep -q "state: 0"; do sleep 2; done
echo "強電 ON を確認 -> control を起動"
tmux new-window -d -t rov -n control "bash -c '$F/control.sh; exec bash'"
if [ "$1" = "--record" ]; then
  sleep 20
  "$F/record.sh"
fi
echo "起動完了。tmux a -t rov で中を見る"
