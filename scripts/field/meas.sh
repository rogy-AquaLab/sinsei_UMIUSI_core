#!/bin/bash
# 15 秒の CPU（プロセスごと）と負荷、status の周期を測る
source "$(dirname "$(readlink -f "$0")")/env.sh"
python3 - $(pgrep -f "lib/sinsei_umiusi_core/auto_targe[t]_generator" | head -1) $(pgrep -f "rosbridge_websocke[t]" | head -1) $(pgrep -f "ros2_control_nod[e]" | head -1) $(pgrep -f "gst_camera_node.*pi_camer[a]" | head -1) $(pgrep -x mediamtx) <<PY &
import sys, time, os
pids = sys.argv[1:]; names = ["auto", "rosbridge", "control", "pi_camera", "mediamtx"]
def t(p):
    f = open(f"/proc/{p}/stat").read().split(")")[1].split(); return int(f[11]) + int(f[12])
hz = os.sysconf("SC_CLK_TCK"); a = [t(p) for p in pids]; time.sleep(15); b = [t(p) for p in pids]
print("  ".join(f"{n} {100*(y-x)/hz/15:.0f}%" for n, x, y in zip(names, a, b)), "| load", open("/proc/loadavg").read().split()[0])
PY
python3 - <<PY
import json, time, rclpy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node("claude_hz"); ts = []; dets = []
def cb(m):
    s = json.loads(m.data); ts.append(time.monotonic()); dets.append(len(s["detections"]))
n.create_subscription(String, "/auto_target_generator/status", cb, 10)
end = time.monotonic() + 15
while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.1)
new = len(ts)
print(f"status {new/15:.1f} Hz, 検出/コマ {sum(dets)/max(1,len(dets)):.1f}")
PY
wait
