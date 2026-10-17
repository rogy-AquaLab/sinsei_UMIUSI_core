#!/bin/bash
# control を、前カメラを共有メモリにも出す設定（cameras_shm.yaml）で起動する。UI で Main Power On を押してから。
#   control.sh            前カメラのピントを固定（既定 1 m。LENS=0.5 で 2 m、2.0 で 0.5 m。lens-position = 1/距離[m]）
#   control.sh af         前カメラは連続オートフォーカス（水中でピントを探し続けることがある）
# cameras_shm.yaml の af-mode が continuous でも manual でも、ここで上書きする（control の既定は固定 1 m）
# ホワイトバランス（既定は libcamera の自動。10/17 プールでは自動が数秒ごとに真っ青（赤 0）へ飛んだ）
#   AWB=daylight control.sh        色温度の範囲を絞る（自動のまま。揺れは減るが止まらない [未確認]。auto / daylight / cloudy / indoor / fluorescent / incandescent / tungsten）
#   GAINS=3.0,1.2 control.sh       赤・青のゲインを手で固定（自動を止める）。awb_probe.sh で選ぶ
F="$(dirname "$(readlink -f "$0")")"
source "$F/env.sh"
P=$(ros2 pkg prefix sinsei_umiusi_control)/share/sinsei_umiusi_control/params
if [ "$1" = "af" ]; then
  CAM=/tmp/cameras_shm_af.yaml
  FOCUS="af-mode=continuous"
else
  CAM=/tmp/cameras_shm_fixedfocus.yaml
  FOCUS="af-mode=manual lens-position=${LENS:-1.0}"
fi
[ -n "$AWB" ] && FOCUS="$FOCUS awb-mode=$AWB"
[ -n "$GAINS" ] && FOCUS="$FOCUS awb-enable=false colour-gains=<$GAINS>"
sed -E "s/libcamerasrc af-mode=[a-z]+( lens-position=[0-9.]+)?/libcamerasrc $FOCUS/" "$P/cameras_shm.yaml" > "$CAM"
grep -qF "libcamerasrc $FOCUS" "$CAM" || { echo "cameras_shm.yaml の libcamerasrc を書き換えられない: $P/cameras_shm.yaml"; exit 1; }
echo "cameras: $CAM ($FOCUS)"
cd "${UMIUSI_WS:-$HOME/ros2-ws}"
ros2 launch sinsei_umiusi_control main.yaml cameras_param_file:="$CAM" 2>&1 | tee /tmp/umiusi_control_launch.log
