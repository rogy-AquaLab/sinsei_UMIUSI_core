"""1 秒ごとに robot_state・クライアント数・自律の状態・指令を記録する（rosbridge を使わない）。usage: watch_run.py <out.tsv>"""

import json
import sys
import time
import rclpy
from std_msgs.msg import Int32, String
from sinsei_umiusi_msgs.msg import RobotState, Target

rclpy.init()
n = rclpy.create_node('claude_watch')
st = {
    'robot': None,
    'clients': None,
    'auto': None,
    'surge': None,
    'heave': None,
    'n_target': 0,
}
n.create_subscription(
    RobotState, '/robot_state', lambda m: st.update(robot=m.state), 10
)
n.create_subscription(Int32, '/client_count', lambda m: st.update(clients=m.data), 10)
n.create_subscription(
    String,
    '/auto_target_generator/status',
    lambda m: st.update(auto=json.loads(m.data)['state']),
    10,
)


def tgt(m):
    st.update(surge=round(m.velocity.x, 2), heave=round(m.velocity.z, 2))
    st['n_target'] += 1


n.create_subscription(Target, '/cmd/target', tgt, 50)
out = open(sys.argv[1], 'a')
out.write('time\trobot_state\tclients\tauto\tsurge\theave\ttarget_hz\n')
out.flush()
last = time.monotonic()
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.monotonic() - last >= 1.0:
        last = time.monotonic()
        out.write(
            f'{time.strftime("%H:%M:%S")}\t{st["robot"]}\t{st["clients"]}\t{st["auto"]}\t{st["surge"]}\t{st["heave"]}\t{st["n_target"]}\n'
        )
        out.flush()
        st['n_target'] = 0
