#!/usr/bin/env python3
"""色合わせの道具（Pi の上で動かし、ノート PC のブラウザから開く）。画像を PC に送らなくてよい。

  python3 scripts/calibrate_server.py                      # 一番新しい撮影フォルダを読む
  python3 scripts/calibrate_server.py ~/umiusi_balloon/capture/20261011-101500

ブラウザで http://<機体のホスト名>:8766 を開く（例: http://umiusi2.local:8766）。
保存すると <out-dir>/<名前>.json と <out-dir>/current.json ができる。auto_target_generator の
detector_config を current.json にしておけば、保存するたびに自動で読み直す（ノードを止めなくてよい）。

撮影は scripts/capture_frames.py。走行中は止めておく（CPU を使うので）。
元: sinsei_umiusi_ballon_detector tools/calibrate_server.py（PC 版）。
"""

import argparse
import copy
import json
import os
import socket
import sys
from dataclasses import asdict, replace
from datetime import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from sinsei_umiusi_core.balloon import (  # noqa: E402
    HsvRange,
    color_masks,
    detect,
    load_params,
    preprocess,
)
from sinsei_umiusi_core.balloon.calibrate import propose  # noqa: E402

COLORS = ('yellow', 'red', 'blue')
LABELS = COLORS + ('background',)
DRAW = {
    'yellow': (0, 230, 255),
    'red': (0, 0, 255),
    'blue': (255, 120, 0),
    'background': (200, 200, 200),
}
IMAGE_EXTS = {'.jpg', '.jpeg', '.png', '.bmp'}
PAINT_GRAY = (128, 128, 128)
DEFAULT_OUT = Path.home() / 'umiusi_balloon'


def newest_capture(out_dir: Path) -> Path:
    caps = sorted(p for p in (out_dir / 'capture').glob('*') if p.is_dir())
    if not caps:
        raise SystemExit(
            f'撮影フォルダが無い: {out_dir / "capture"}（先に scripts/capture_frames.py で撮る）'
        )
    return caps[-1]


def load_frames(src: Path, every: float, limit: int):
    """[(名前, BGR)]。フォルダなら画像を名前順に、動画なら every 秒ごとに。"""
    if src.is_dir():
        files = sorted(p for p in src.iterdir() if p.suffix.lower() in IMAGE_EXTS)
        step = max(1, len(files) // limit) if len(files) > limit else 1
        return [(p.name, cv2.imread(str(p))) for p in files[::step]]
    cap = cv2.VideoCapture(str(src))
    fps = cap.get(cv2.CAP_PROP_FPS) or 15.0
    step = max(1, round(every * fps))
    out, i = [], 0
    while len(out) < limit:
        ok = cap.grab()
        if not ok:
            break
        if i % step == 0:
            ok, img = cap.retrieve()
            if ok:
                out.append((f'{src.stem}_{i:06d}', img))
        i += 1
    return out


def write_atomic(path: Path, text: str) -> None:
    """一時ファイルに書いてから置き換える（読む側が書きかけを読まないように）。"""
    tmp = path.with_name(f'.{path.name}.tmp')
    tmp.write_text(text)
    os.replace(tmp, path)


class State:
    def __init__(self, base_path: Path, out_dir: Path):
        self.base_path = base_path
        self.out_dir = out_dir
        self.base_raw = json.loads(base_path.read_text())
        self.params = load_params(base_path)
        self.frames = []
        self.source = None
        self.last_proposal = {}

    def load(self, source: Path, every: float, limit: int) -> None:
        frames = load_frames(source, every, limit)
        if not frames:
            raise ValueError(f'画像が無い: {source}')
        self.frames = frames
        self.source = source
        self.clicks = []  # (label, frame_idx, x, y, r) 正規化座標
        self._recompute()

    def _recompute(self) -> None:
        frames = self.frames
        self.hsv = [preprocess(img, self.params) for _, img in frames]
        # 塗りつぶした灰色 (128,128,128) は見本に入れない
        self.painted = [
            cv2.resize(
                cv2.inRange(img, np.array(PAINT_GRAY), np.array(PAINT_GRAY)),
                (h.shape[1], h.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )
            > 0
            for (_, img), h in zip(frames, self.hsv, strict=True)
        ]

    def set_white_balance(self, mode: str) -> None:
        """色かぶり補正を切り替える（クリックはそのまま。見本の色はこの補正で計算し直す）。"""
        if mode not in ('none', 'grayworld'):
            raise ValueError(f'white_balance は none か grayworld: {mode}')
        self.params = replace(self.params, white_balance=mode)
        self.base_raw['white_balance'] = mode
        self._recompute()

    def pixel(self, idx: int, x: float, y: float):
        """その点の HSV と、どの色の範囲に入るか（カーソルの読み取り用）。"""
        hsv = self.hsv[idx]
        h, w = hsv.shape[:2]
        px = hsv[min(h - 1, int(y * h)), min(w - 1, int(x * w))]
        inside = [
            c
            for c, rs in self.params.colors.items()
            if any(all(r.lo[k] <= int(px[k]) <= r.hi[k] for k in range(3)) for r in rs)
        ]
        return {'h': int(px[0]), 's': int(px[1]), 'v': int(px[2]), 'inside': inside}

    def mask_hit(self, label):
        """クリックした所のうち、最終的なマスク（ノイズ除去の後）に入っている割合。"""
        hit = total = 0
        masks_cache = {}
        for lab, fi, x, y, r in self.clicks:
            if lab != label:
                continue
            if fi not in masks_cache:
                masks_cache[fi] = color_masks(self.hsv[fi], self.params)
            m = masks_cache[fi].get(label)
            if m is None:
                continue
            h, w = m.shape
            cx, cy, rr = x * w, y * h, max(1.0, r * w)
            yy, xx = np.ogrid[:h, :w]
            sel = ((xx - cx) ** 2 + (yy - cy) ** 2 <= rr * rr) & ~self.painted[fi]
            total += int(sel.sum())
            hit += int((m[sel] > 0).sum())
        return round(hit / total, 3) if total else None

    def samples(self, label):
        out = []
        for lab, fi, x, y, r in self.clicks:
            if lab != label:
                continue
            hsv = self.hsv[fi]
            h, w = hsv.shape[:2]
            cx, cy, rr = x * w, y * h, max(1.0, r * w)
            yy, xx = np.ogrid[:h, :w]
            m = ((xx - cx) ** 2 + (yy - cy) ** 2 <= rr * rr) & ~self.painted[fi]
            out.append(hsv[m])
        return np.concatenate(out) if out else np.zeros((0, 3), np.uint8)

    def counts(self):
        return {lab: int(len(self.samples(lab))) for lab in LABELS}

    def propose_all(self):
        bg = self.samples('background')
        result = {}
        for c in COLORS:
            s = self.samples(c)
            if len(s) < 20:
                result[c] = {'skipped': 'クリックが足りない（元の値のまま）'}
                continue
            p = propose(s, bg if len(bg) else None)
            self.params.colors[c] = p.ranges
            result[c] = {
                'keep': round(p.keep, 3),
                'bg_in': None if p.bg_in is None else round(p.bg_in, 3),
                'n': int(len(s)),
                'median': [int(v) for v in np.median(s, axis=0)],
            }
        for c in COLORS:
            if 'skipped' not in result[c]:
                result[c]['mask_hit'] = self.mask_hit(c)
        if len(bg):
            result['background'] = {
                'n': int(len(bg)),
                'median': [int(v) for v in np.median(bg, axis=0)],
            }
        self.last_proposal = result
        return result

    def set_ranges(self, colors):
        for c, ranges in colors.items():
            self.params.colors[c] = [
                HsvRange(tuple(r['lo']), tuple(r['hi'])) for r in ranges
            ]

    def ranges_json(self):
        return {
            c: [{'lo': list(r.lo), 'hi': list(r.hi)} for r in rs]
            for c, rs in self.params.colors.items()
        }

    def render(self, idx: int, view: str) -> bytes:
        _name, img = self.frames[idx]
        hsv = self.hsv[idx]
        h, w = hsv.shape[:2]
        shown = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        if view == 'mask':
            masks = color_masks(hsv, self.params)
            over = shown.copy() // 3
            for c, m in masks.items():
                over[m > 0] = DRAW[c]
            shown = over
        dets, _ = detect(img, self.params)
        for d in dets:
            x, y, bw, bh = d.bbox
            cv2.rectangle(shown, (x, y), (x + bw, y + bh), DRAW[d.color], 1)
        for lab, fi, x, y, r in self.clicks:
            if fi == idx:
                cv2.circle(
                    shown, (int(x * w), int(y * h)), max(1, int(r * w)), DRAW[lab], 1
                )
        big = cv2.resize(shown, (w * 2, h * 2), interpolation=cv2.INTER_NEAREST)
        _ok, buf = cv2.imencode('.jpg', big, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return buf.tobytes()

    def summary(self):
        """全コマで、色ごとに検出のあったコマの番号。"""
        hits = {c: [] for c in self.params.colors}
        for i, (_, img) in enumerate(self.frames):
            for c in {d.color for d in detect(img, self.params)[0]}:
                hits[c].append(i)
        return hits

    def save(self, name: str) -> Path:
        out = copy.deepcopy(self.base_raw)
        out['colors'] = self.ranges_json()
        stamp = datetime.now().strftime('%Y-%m-%d %H:%M')
        out['_comment'] = (
            f'{stamp} 色合わせの道具で作成（元: {self.base_path.name}、画像: {self.source}）。'
            f'クリック数 {self.counts()}、提案時の成績 {self.last_proposal}'
        )
        text = json.dumps(out, ensure_ascii=False, indent=2) + '\n'
        self.out_dir.mkdir(parents=True, exist_ok=True)
        named = self.out_dir / f'{name}.json'
        write_atomic(named, text)
        write_atomic(self.out_dir / 'current.json', text)  # ノードはこれを見て読み直す
        return named


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        'source',
        type=Path,
        nargs='?',
        help='画像フォルダ か 動画。省略で一番新しい撮影',
    )
    ap.add_argument(
        '--out-dir',
        type=Path,
        default=DEFAULT_OUT,
        help='保存先（current.json もここ）',
    )
    ap.add_argument(
        '--base',
        type=Path,
        help='元にする設定（色以外の値を引き継ぐ）。既定: <out-dir>/current.json、無ければ normal.json',
    )
    ap.add_argument('--every', type=float, default=1.0, help='動画から抜く間隔 [s]')
    ap.add_argument('--limit', type=int, default=200, help='読み込む最大コマ数')
    ap.add_argument(
        '--host', default='0.0.0.0', help='待ち受けるアドレス（既定: 全部）'
    )
    ap.add_argument('--port', type=int, default=8766)
    args = ap.parse_args()

    base = args.base
    if base is None:
        current = args.out_dir / 'current.json'
        base = (
            current if current.exists() else REPO / 'config' / 'balloon' / 'normal.json'
        )
    state = State(base, args.out_dir)
    fixed_source = args.source
    state.load(fixed_source or newest_capture(args.out_dir), args.every, args.limit)
    page = (REPO / 'scripts' / 'calibrate.html').read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, body: bytes, ctype='application/json'):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj, ensure_ascii=False).encode())

        def do_GET(self):  # noqa: N802
            u = urlparse(self.path)
            q = parse_qs(u.query)
            if u.path == '/':
                self._send(200, page, 'text/html; charset=utf-8')
            elif u.path == '/api/state':
                self._json(
                    {
                        'frames': [n for n, _ in state.frames],
                        'source': str(state.source),
                        'base': str(state.base_path),
                        'out_dir': str(state.out_dir),
                        'ranges': state.ranges_json(),
                        'counts': state.counts(),
                        'params': {
                            k: v
                            for k, v in asdict(state.params).items()
                            if k != 'colors'
                        },
                    }
                )
            elif u.path == '/api/render':
                view = q.get('view', ['image'])[0]
                self._send(200, state.render(int(q['i'][0]), view), 'image/jpeg')
            elif u.path == '/api/pixel':
                self._json(
                    state.pixel(int(q['i'][0]), float(q['x'][0]), float(q['y'][0]))
                )
            elif u.path == '/api/summary':
                self._json(state.summary())
            else:
                self._send(404, b'{}')

        def do_POST(self):  # noqa: N802
            body = json.loads(
                self.rfile.read(int(self.headers['Content-Length'])) or b'{}'
            )
            try:
                if self.path == '/api/click':
                    state.clicks.append(
                        (body['label'], int(body['i']), body['x'], body['y'], body['r'])
                    )
                    self._json({'counts': state.counts()})
                elif self.path == '/api/undo':
                    if state.clicks:
                        state.clicks.pop()
                    self._json({'counts': state.counts()})
                elif self.path == '/api/propose':
                    self._json(
                        {'result': state.propose_all(), 'ranges': state.ranges_json()}
                    )
                elif self.path == '/api/ranges':
                    state.set_ranges(body['ranges'])
                    self._json({'ranges': state.ranges_json()})
                elif self.path == '/api/white_balance':
                    state.set_white_balance(body['mode'])
                    self._json({'white_balance': state.params.white_balance})
                elif self.path == '/api/reload':
                    # 撮り直したら押す。しきい値はそのまま、クリックは消える
                    src = fixed_source or newest_capture(state.out_dir)
                    state.load(src, args.every, args.limit)
                    self._json({'source': str(src), 'frames': len(state.frames)})
                elif self.path == '/api/save':
                    name = ''.join(
                        ch for ch in body['name'] if ch.isalnum() or ch in '-_'
                    )
                    if not name:
                        raise ValueError('名前が空')
                    self._json(
                        {
                            'path': str(state.save(name)),
                            'current': str(state.out_dir / 'current.json'),
                        }
                    )
                else:
                    self._send(404, b'{}')
            except (Exception, SystemExit) as e:  # noqa: BLE001 画面に理由を出す
                self._json({'error': str(e)}, 400)

        def log_message(self, *a):
            pass

    print(f'{len(state.frames)} コマ（{state.source}）、元の設定 {base}')
    print(
        f'ブラウザで http://{socket.gethostname()}.local:{args.port}/ を開く。終わったら Ctrl-C'
    )
    try:
        # 1 本で順に処理（状態を共有するため）
        HTTPServer((args.host, args.port), Handler).serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
