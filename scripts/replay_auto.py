#!/usr/bin/env python3
"""録画した動画を、風船検出 -> 行動決定 に通し、各コマの指令を重ねた画像で確かめる。ROS 不要。

機体は動画どおりにしか動かない（指令は映像に反映されない）ので、確かめられるのは
「見えたものに対して、どちらへ動こうとしたか」だけ。プールの前に符号の取り違えを潰す用。

  python3 scripts/replay_auto.py <動画> --detector config/balloon/noir.json --out /tmp/replay
  （画面録画なら --crop x,y,w,h で前カメラ枠を切り出す）

出力: <out>/sheet_XX.jpg（矢印 = yaw の向き、縦棒 = heave、下の帯 = surge）と <out>/log.csv
"""

import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sinsei_umiusi_core.balloon import detect, load_params  # noqa: E402
from sinsei_umiusi_core.balloon.mission import BalloonMission, MissionParams  # noqa: E402

DRAW = {'yellow': (0, 230, 255), 'red': (0, 0, 255), 'blue': (255, 120, 0)}


def draw(img, dets, cmd, t):
    h, w = img.shape[:2]
    for d in dets or []:
        cx, cy = int((d.x + 1) / 2 * w), int((d.y + 1) / 2 * h)
        r = max(3, int(d.radius * w))
        thick = 3 if d.color == cmd.target else 1
        cv2.circle(img, (cx, cy), r, DRAW.get(d.color, (255, 255, 255)), thick)
    c = (w // 2, h // 2)
    # yaw: 正 = 左旋回 -> 左向きの矢印
    cv2.arrowedLine(
        img,
        c,
        (int(c[0] - cmd.yaw_rate * w * 0.4), c[1]),
        (255, 255, 255),
        2,
        tipLength=0.3,
    )
    # heave: 正 = 浮上 -> 上向き
    cv2.line(
        img,
        (w - 12, h // 2),
        (w - 12, int(h // 2 - cmd.heave * h * 0.4)),
        (0, 255, 0),
        4,
    )
    # surge: 下の帯
    cv2.rectangle(img, (0, h - 6), (int(max(0.0, cmd.surge) * w), h), (0, 255, 0), -1)
    if cmd.surge < 0:
        cv2.rectangle(img, (0, h - 6), (int(-cmd.surge * w), h), (0, 0, 255), -1)
    label = f'{t:5.1f}s {cmd.state} {cmd.target or ""}'
    cv2.putText(img, label, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
    cv2.putText(img, label, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return img


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument('video', type=Path)
    ap.add_argument('--detector', type=Path, required=True, help='色のしきい値の JSON')
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--crop', help='x,y,w,h（画面録画の前カメラ枠）')
    ap.add_argument('--rate', type=float, default=10.0, help='行動決定を回す周期 [Hz]')
    ap.add_argument(
        '--start-delay', type=float, default=0.0, help='WAIT の長さ（既定 0 で即 DIVE）'
    )
    ap.add_argument(
        '--sheet-every', type=float, default=1.0, help='一覧に載せる間隔 [s]'
    )
    args = ap.parse_args()

    det_params = load_params(args.detector)
    mission = BalloonMission(MissionParams(start_delay=args.start_delay))
    mission.reset(0.0)
    crop = [int(v) for v in args.crop.split(',')] if args.crop else None

    cap = cv2.VideoCapture(str(args.video))
    fps = cap.get(cv2.CAP_PROP_FPS) or 15.0
    args.out.mkdir(parents=True, exist_ok=True)
    tiles, i, next_tick, next_tile = [], 0, 0.0, 0.0
    with open(args.out / 'log.csv', 'w', newline='') as f:
        log = csv.writer(f)
        log.writerow(['t', 'state', 'target', 'surge', 'heave', 'yaw_rate', 'n_dets'])
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            t = i / fps
            i += 1
            if t < next_tick:
                continue
            next_tick = t + 1.0 / args.rate
            if crop:
                x, y, w, h = crop
                frame = frame[y : y + h, x : x + w]
            small = cv2.resize(
                frame, (320, 240), interpolation=cv2.INTER_AREA
            )  # 実機と同じ入力
            dets, _ = detect(small, det_params)
            cmd = mission.step(t, dets)
            log.writerow(
                [
                    f'{t:.2f}',
                    cmd.state,
                    cmd.target or '',
                    cmd.surge,
                    cmd.heave,
                    f'{cmd.yaw_rate:.3f}',
                    len(dets),
                ]
            )
            if t >= next_tile:
                next_tile = t + args.sheet_every
                view = cv2.resize(frame, (320, 180), interpolation=cv2.INTER_AREA)
                tiles.append(draw(view, dets, cmd, t))

    per = 24
    for s in range(0, len(tiles), per):
        chunk = tiles[s : s + per]
        chunk += [np.zeros_like(tiles[0])] * (per - len(chunk))
        grid = np.vstack([np.hstack(chunk[r * 4 : (r + 1) * 4]) for r in range(6)])
        cv2.imwrite(
            str(args.out / f'sheet_{s // per:02d}.jpg'),
            grid,
            [cv2.IMWRITE_JPEG_QUALITY, 80],
        )
    print(f'{len(tiles)} tiles -> {args.out}')


if __name__ == '__main__':
    main()
