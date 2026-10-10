#!/usr/bin/env python3
"""色合わせ用に、前カメラのコマを一定間隔で保存する。ROS もモードも問わない（手動で動かしながら撮れる）。

control の共有メモリ（cameras_shm.yaml）から、auto_target_generator と同じものを読む。

  python3 scripts/capture_frames.py                  # ~/umiusi_balloon/capture/<日時>/ に 0.5 秒ごと
  python3 scripts/capture_frames.py --every 0.25 --seconds 60

Ctrl-C で止める。撮ったフォルダは scripts/calibrate_server.py にそのまま渡せる。
"""

import argparse
import time
from datetime import datetime
from pathlib import Path

import cv2

DEFAULT_CAMERA = (
    'shmsrc socket-path=/tmp/umiusi_cam1.sock is-live=true ! '
    'video/x-raw,format=BGR,width=320,height=240 ! '
    'appsink drop=true max-buffers=1 sync=false'
)


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        '--source', default=DEFAULT_CAMERA, help='GStreamer パイプライン か 動画'
    )
    ap.add_argument('--out', type=Path, help='既定: ~/umiusi_balloon/capture/<日時>')
    ap.add_argument('--every', type=float, default=0.5, help='保存する間隔 [s]')
    ap.add_argument(
        '--seconds',
        type=float,
        default=0.0,
        help='この秒数で止める。0 なら Ctrl-C まで',
    )
    args = ap.parse_args()

    out = args.out or (
        Path.home()
        / 'umiusi_balloon'
        / 'capture'
        / datetime.now().strftime('%Y%m%d-%H%M%S')
    )
    out.mkdir(parents=True, exist_ok=True)
    cap = (
        cv2.VideoCapture(args.source, cv2.CAP_GSTREAMER)
        if '!' in args.source
        else cv2.VideoCapture(args.source)
    )
    if not cap.isOpened():
        raise SystemExit(
            f'カメラを開けない: {args.source}\n'
            'control を cameras_shm.yaml で起動しているか、gstreamer1.0-plugins-bad が入っているかを確かめる'
        )

    print(f'保存先: {out}（Ctrl-C で止める）')
    n, start, last = 0, time.monotonic(), -1e9
    live = '!' in args.source
    try:
        while True:
            now = time.monotonic()
            if args.seconds and now - start >= args.seconds:
                break
            ok, frame = cap.read()
            if not ok:
                if not live:
                    break  # 動画ファイルの終わり
                print('読めない。カメラが止まった？')
                time.sleep(0.5)
                continue
            if now - last >= args.every:
                last = now
                cv2.imwrite(
                    str(out / f'{n:05d}.jpg'), frame, [cv2.IMWRITE_JPEG_QUALITY, 92]
                )
                n += 1
                if n % 20 == 0:
                    print(f'{n} 枚')
    except KeyboardInterrupt:
        pass
    print(f'{n} 枚を保存した: {out}')


if __name__ == '__main__':
    main()
