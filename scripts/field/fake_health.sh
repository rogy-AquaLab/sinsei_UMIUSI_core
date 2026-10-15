#!/bin/bash
# ヘルスチェックの偽装（分電盤の実装が終わるまで）。Ctrl-C で全部止まる。
# - /high_power_health_check_result {is_ok: true} を出し続ける（出している node が無いため）
# - /state/low_power_circuit_info を全部 0 で出す。control が本物を出し始めたら自動で止める
#   （両方が出ると UI の値が混ざる。強電 ON の判定に要るのは control を起動する前だけ）
F="$(dirname "$(readlink -f "$0")")"
source "$F/env.sh"
ros2 topic pub /high_power_health_check_result sinsei_umiusi_msgs/msg/HealthCheckResult "{is_ok: true}" > /dev/null &
HP=$!
ros2 topic pub /state/low_power_circuit_info sinsei_umiusi_msgs/msg/LowPowerCircuitInfo \
  "{can: 0, headlights: 0, imu: 0, indicator_led: 0}" > /dev/null &
LP=$!
trap 'kill $HP $LP 2>/dev/null; echo; echo "fake 停止"; exit 0' INT TERM
echo "fake: high_power ok / low_power を出し中（control が出し始めたら low_power は止める）。Ctrl-C で終了"
while kill -0 $LP 2>/dev/null; do
  sleep 3
  n=$(ros2 topic info /state/low_power_circuit_info 2>/dev/null | awk '/Publisher count/ {print $3}')
  if [ "${n:-0}" -ge 2 ]; then
    kill $LP; echo "control の low_power_circuit_info を確認 -> low_power の偽装を止めた（high_power は続ける）"
  fi
done
wait $HP
