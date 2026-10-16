#!/bin/bash
# bag と映像（前 cam1・下 cam2、H.264 のまま）の記録を rov に足す。名前は日時 + 引数
#   record.sh [名前]   止める: rov の bag / cam ウィンドウで Ctrl-C
# 映像は RTSP から録る（カメラは gst_camera_node が掴んでいるので）。録画の本体は autonomy の tools/record_camera.sh
F="$(dirname "$(readlink -f "$0")")"
T=$(date +%Y%m%d_%H%M%S)${1:+_$1}
tmux new-window -d -t rov -n bag "bash -c 'source $F/env.sh && ros2 bag record -o ~/rosbags/$T /cmd/target /cmd/attitude_target /robot_state /auto_target_generator/status /client_count /state/imu /state/low_power_circuit_info /state/thruster_state_all /rosout; exec bash'"
REC="${UMIUSI_RECORD_CAMERA:-${UMIUSI_WS:-$HOME/ros2-ws}/src/sinsei_UMIUSI_autonomy/tools/record_camera.sh}"
if [ -x "$REC" ]; then
  tmux new-window -d -t rov -n cam "bash -c 'UMIUSI_REC_DIR=~/recordings/$T $REC --both; exec bash'"
else
  echo "映像は録らない（$REC が無い）"
fi
echo "記録開始: ~/rosbags/$T  ~/recordings/$T"
