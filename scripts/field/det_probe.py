#!/usr/bin/env python3
"""前カメラで、色の検出（HSV）と CNN（複数のモデル）を並べて回し、1 秒ごとに見えたものと時間を出す。

現場で次を決めるための道具（auto_target_generator とは別に読む。走行中でも使える）:
- 会場の水で、風船がどこまで遠くで見えるか（HSV と CNN のそれぞれ。距離ごとの半径 r）
- CNN のモデルごとの Pi での推論時間（ms）

  scripts/field/det_probe.py                                     # 前カメラ・current.json・20261015 を 30 秒
  scripts/field/det_probe.py --models balloon_cnn_20261015.onnx ~/p1015_w8ft.onnx --seconds 60
  scripts/field/det_probe.py --save ~/probe/3m                   # 1 秒ごとに枠を描いた jpg を残す（巻き尺と一緒に撮る）
  scripts/field/det_probe.py --source <録画.mp4>                  # 録画で試す

1 秒ごとの行: 色ごとに「HSV の個数・一番大きい r」「CNN の一番大きい r・その確信度」。r は半径 / 画像幅（mission の ram_radius と同じ）。
最後に検出器ごとの時間（中央 / 90%）。
"""

import argparse
import sys
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from sinsei_umiusi_core.balloon import detect, load_params  # noqa: E402
from sinsei_umiusi_core.balloon.cnn import CnnDetector, detect_cnn  # noqa: E402
from sinsei_umiusi_core.balloon.reflection import split_reflection  # noqa: E402

try:  # 重なった箱をまとめる（core の cnn_merge）。無い版の core では何もしない
    from sinsei_umiusi_core.balloon.cnn import merge_overlaps  # noqa: E402
except ImportError:  # pragma: no cover

    def merge_overlaps(dets):
        return list(dets)


# auto_target_generator と同じ（control の cameras_shm.yaml の共有メモリ、BGR 320x240）
SHM = (
    'shmsrc socket-path=/tmp/umiusi_cam1.sock is-live=true ! '
    'video/x-raw,format=BGR,width=320,height=240 ! '
    'appsink drop=true max-buffers=1 sync=false'
)
COLORS = ('yellow', 'red', 'blue')
MODELS = REPO / 'config' / 'balloon' / 'models'
DRAW = {'yellow': (0, 220, 255), 'red': (0, 0, 255), 'blue': (255, 128, 0)}


def resolve_config(arg):
    if arg:
        return Path(arg).expanduser()
    cur = Path('~/umiusi_balloon/current.json').expanduser()
    return cur if cur.exists() else REPO / 'config' / 'balloon' / 'normal.json'


def resolve_model(name):
    p = Path(name).expanduser()
    return p if p.exists() else MODELS / name


def best(dets, color):
    same = [d for d in dets if d.color == color]
    return max(same, key=lambda d: d.radius) if same else None


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        '--source', default=SHM, help='GStreamer パイプラインか動画。既定は前カメラ'
    )
    ap.add_argument('--config', help='色の設定。既定 ~/umiusi_balloon/current.json')
    ap.add_argument('--models', nargs='*', default=['balloon_cnn_20261015.onnx'])
    ap.add_argument('--conf', type=float, default=0.3, help='CNN の確信度の下限')
    ap.add_argument('--threads', type=int, default=2)
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--every', type=float, default=1.0, help='まとめて出す間隔 [s]')
    ap.add_argument('--save', type=Path, help='間隔ごとに枠を描いた jpg を残すフォルダ')
    a = ap.parse_args()

    cfg = resolve_config(a.config)
    params = load_params(cfg)
    cnns = []
    for m in a.models:
        path = resolve_model(m)
        try:
            det = CnnDetector(path, conf=a.conf, threads=a.threads)
        except Exception as e:  # noqa: BLE001 読めないモデルは飛ばして続ける
            print(f'CNN {path.name}: 読めない（{type(e).__name__}）。飛ばす')
            continue
        cnns.append((path.stem, det))
        print(f'C{len(cnns)} = {path.name}: {det.backend}, 入力 {det.input_size}')
        if det.backend == 'opencv':
            print(
                '  注意: OpenCV では入力を 320 とみなす。320 以外のモデルは onnxruntime で（env.sh）'
            )
    print(f'色の設定: {cfg}')

    gst = '!' in a.source
    cap = cv2.VideoCapture(a.source, cv2.CAP_GSTREAMER if gst else cv2.CAP_ANY)
    if not cap.isOpened():
        sys.exit(f'カメラ / 動画を開けない: {a.source}')
    if a.save:
        a.save.mkdir(parents=True, exist_ok=True)

    names = ['hsv'] + [n for n, _ in cnns]
    ms = defaultdict(list)
    per = defaultdict(list)  # 名前 -> この間隔の [dets]
    head = '時刻     ' + '  '.join(f'{c:<27}' for c in COLORS)
    print(head)
    t_end = time.monotonic() + a.seconds
    t_next = time.monotonic() + a.every
    frames = 0
    last = None
    while time.monotonic() < t_end:
        ok, frame = cap.read()
        if not ok:
            if gst:
                time.sleep(0.05)
                continue
            break
        frames += 1
        t0 = time.perf_counter()
        hsv, _ = detect(frame, params)
        hsv = split_reflection(hsv, params, frame.shape)
        ms['hsv'].append((time.perf_counter() - t0) * 1e3)
        per['hsv'].append(hsv)
        out = {'hsv': hsv}
        for name, model in cnns:
            t0 = time.perf_counter()
            d = detect_cnn(frame, model, params)
            d = split_reflection(merge_overlaps(d), params, frame.shape)
            ms[name].append((time.perf_counter() - t0) * 1e3)
            per[name].append(d)
            out[name] = d
        last = (frame, out)
        if time.monotonic() < t_next:
            continue
        t_next += a.every
        cols = []
        for c in COLORS:
            seen = [x for x in (best(ds, c) for ds in per['hsv']) if x]
            s = f'H {len(seen)}/{len(per["hsv"])} r{max((x.radius for x in seen), default=0):.3f}'
            for k, (name, _) in enumerate(cnns, 1):
                b = [x for x in (best(ds, c) for ds in per[name]) if x]
                top = max(b, key=lambda x: x.radius) if b else None
                s += f' C{k} r{top.radius:.3f} s{top.score:.2f}' if top else f' C{k} -'
            cols.append(f'{s:<27}')
        print(time.strftime('%H:%M:%S') + '  ' + '  '.join(cols), flush=True)
        per.clear()
        if a.save and last is not None:
            img = last[0].copy()
            for name, ds in last[1].items():
                for d in ds:
                    x, y, w, h = d.bbox
                    sx = img.shape[1] / params.work_width
                    sy = img.shape[0] / round(
                        params.work_width
                        / (params.source_aspect or img.shape[1] / img.shape[0])
                    )
                    p0 = (int(x * sx), int(y * sy))
                    p1 = (int((x + w) * sx), int((y + h) * sy))
                    th = 1 if name == 'hsv' else 2
                    cv2.rectangle(img, p0, p1, DRAW.get(d.color, (255, 255, 255)), th)
                    k = [n for n, _ in cnns].index(name) + 1 if name != 'hsv' else 0
                    label = 'H' if name == 'hsv' else f'C{k} s{d.score:.2f}'
                    cv2.putText(
                        img,
                        f'{label} r{d.radius:.3f}',
                        (p0[0], max(10, p0[1] - 2)),
                        cv2.FONT_HERSHEY_PLAIN,
                        0.8,
                        DRAW.get(d.color, (255, 255, 255)),
                        1,
                    )
            cv2.imwrite(str(a.save / f'{time.strftime("%H%M%S")}.jpg'), img)
    cap.release()
    print(f'\n{frames} コマ。検出 1 回の時間 [ms]（中央 / 90%）:')
    for n in names:
        if ms[n]:
            v = np.array(ms[n])
            print(f'  {n:<28} {np.median(v):6.1f} / {np.percentile(v, 90):6.1f}')


if __name__ == '__main__':
    main()
