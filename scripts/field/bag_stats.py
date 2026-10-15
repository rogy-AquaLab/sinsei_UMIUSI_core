"""auto_target_generator/status を bag から集計する。usage: bag_stats.py <bag_dir>..."""

import json
import sys
import collections
import statistics
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    r.set_filter(rosbag2_py.StorageFilter(topics=['/auto_target_generator/status']))
    while r.has_next():
        _, data, t = r.read_next()
        yield t / 1e9, json.loads(deserialize_message(data, String).data)


for bag in sys.argv[1:]:
    rows = list(read(bag))
    print(f'== {bag}  status {len(rows)}')
    # 1 秒ごと: (検出器, 走行中か) -> 新しいコマの数
    sec = collections.defaultdict(lambda: collections.Counter())
    for t, s in rows:
        key = (
            s.get('detector'),
            'AUTO' if s['state'] not in ('PREVIEW',) else 'PREVIEW',
        )
        sec[(int(t), key)]['new'] += s['new_frame']
    per = collections.defaultdict(list)
    for (_, key), c in sec.items():
        per[key].append(c['new'])
    for key, v in sorted(per.items(), key=str):
        v = sorted(v)
        print(
            f'  {key}: {len(v)} 秒  処理コマ/秒 中央 {statistics.median(v)}  下位10% {v[len(v) // 10]}  最小 {v[0]}'
        )
    # 走行（PREVIEW 以外）の区間ごと
    runs, cur = [], []
    for t, s in rows:
        if s['state'] == 'PREVIEW':
            if cur:
                runs.append(cur)
                cur = []
        else:
            cur.append((t, s))
    if cur:
        runs.append(cur)
    for run in runs:
        if run[-1][0] - run[0][0] < 5:
            continue
        st = [s['state'] for _, s in run]
        trans = collections.Counter(
            (a, b) for a, b in zip(st, st[1:], strict=False) if a != b
        )
        det = collections.Counter(s.get('detector') for _, s in run)
        newf = [s for _, s in run if s['new_frame']]
        hit = sum(
            1
            for s in newf
            if s['target'] and any(d['color'] == s['target'] for d in s['detections'])
        )
        appr = [s for s in newf if s['state'] == 'APPROACH']
        appr_hit = sum(
            1 for s in appr if any(d['color'] == s['target'] for d in s['detections'])
        )
        yaw = [s['cmd']['yaw_rate'] for _, s in run if s['state'] == 'APPROACH']
        flips = sum(
            1
            for a, b in zip(yaw, yaw[1:], strict=False)
            if a * b < 0 and abs(a) > 0.05 and abs(b) > 0.05
        )
        dur = run[-1][0] - run[0][0]
        nf_t = [t for t, s in run if s['new_frame']]
        g = sorted(b - a for a, b in zip(nf_t, nf_t[1:], strict=False))
        print(
            f'  -- 走行 {dur:.0f}s 検出器 {dict(det)}  新コマ {len(newf) / dur:.1f}/s  コマ間隔 中央 {g[len(g) // 2] * 1e3:.0f}ms 90% {g[int(len(g) * 0.9)] * 1e3:.0f}ms 最大 {g[-1] * 1e3:.0f}ms'
            if g
            else f'  -- 走行 {dur:.0f}s'
        )
        print(f'     状態 {dict(collections.Counter(st))}')
        ms = sorted(s['det_ms'] for _, s in run if s.get('det_ms') is not None)
        if ms:
            print(
                f'     検出 1 回 中央 {ms[len(ms) // 2]} ms  90% {ms[int(len(ms) * 0.9)]} ms   CNN の結果を使ったコマ {sum(1 for _, s in run if s.get("assist_used"))}  指令 {len(run) / dur:.1f}/s'
            )
        print(
            f'     見失い APPROACH->SEARCH {trans[("APPROACH", "SEARCH")]}  突進 {trans[("APPROACH", "RAM")]}  接近中に目標が見えたコマ {appr_hit}/{len(appr)}  接近中の旋回の反転 {flips}/{len(yaw)}'
        )
