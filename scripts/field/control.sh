#!/bin/bash
# control を、前カメラを共有メモリにも出す設定（cameras_shm.yaml）で起動する。UI で Main Power On を押してから。
#   control.sh            前カメラのピントを固定（既定 1 m。LENS=0.5 で 2 m、2.0 で 0.5 m。lens-position = 1/距離[m]）
#   control.sh af         前カメラは連続オートフォーカス（cameras_shm.yaml のまま。水中でピントを探し続けることがある）
F="$(dirname "$(readlink -f "$0")")"
source "$F/env.sh"
P=$(ros2 pkg prefix sinsei_umiusi_control)/share/sinsei_umiusi_control/params
CAM=$P/cameras_shm.yaml
if [ "$1" != "af" ]; then
  CAM=/tmp/cameras_shm_fixedfocus.yaml
  sed "s/libcamerasrc af-mode=continuous/libcamerasrc af-mode=manual lens-position=${LENS:-1.0}/" "$P/cameras_shm.yaml" > "$CAM"
fi
echo "cameras: $CAM"
cd "${UMIUSI_WS:-$HOME/ros2-ws}"
ros2 launch sinsei_umiusi_control main.yaml cameras_param_file:="$CAM" 2>&1 | tee /tmp/umiusi_control_launch.log
