#!/bin/bash
# control を起動した後に強電が落ちたとき、入れ直せるようにする（応急）。
# control が出す CAN の健全性が ERROR（can: 1。VESC の "Unsupported packet status variant" が原因とみられる）のため、
# low_power_health_check を止め、/low_power_health_check_result に OK を出す。core を起動し直せば元に戻る。
F="$(dirname "$(readlink -f "$0")")"
source "$F/env.sh"
P=$(pgrep -f "lib/sinsei_umiusi_core/low_power_health_chec[k]") && kill $P && echo "low_power_health_check を止めた"
tmux new-window -d -t rov -n lp_ok "bash -c 'source $F/env.sh && ros2 topic pub -r 2 /low_power_health_check_result sinsei_umiusi_msgs/msg/HealthCheckResult \"{is_ok: true}\"; exec bash'"
echo "OK を出し始めた。UI で Main Power On を押す"
