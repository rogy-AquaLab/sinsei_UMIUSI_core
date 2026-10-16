#!/bin/bash
# rov の全ウィンドウに Ctrl-C を送ってから閉じる（bag と映像はきれいに閉じる。kill -9 は使わない）
for w in $(tmux list-windows -t rov -F "#W" 2>/dev/null); do tmux send-keys -t "rov:$w" C-c; done
sleep 10
tmux kill-session -t rov 2>/dev/null && echo "rov を閉じた"
