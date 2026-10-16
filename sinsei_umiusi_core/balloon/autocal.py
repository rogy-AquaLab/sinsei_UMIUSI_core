"""色合わせの自動提案: 学習した CNN の検出を先生にして、クリックの代わりに風船と背景の画素を集める。

手順（人のクリックは使わない）:
  1. 色合わせ用のフレームに CNN を掛け、確信度 conf_floor 以上の箱の中心付近（楕円の内側 r_in）を風船の画素にする。
     箱のまわりの輪（水の色）から遠い画素だけを残す（fg_keep）
  2. CNN が何か見ている箱（1.3 倍）の外の画素を背景にする。他の色の画素も背景に足す（色どうしを分ける）
  3. 色ごとに確信度の高い順に top_k 個の箱を取り、中心の色相が多数派から hue_tol 以内のものだけを使う
     （確信度の目盛りは会場で大きく変わるので、固定のしきい値は使わない）
  4. calibrate.propose で範囲を出し、S・V の下限を「CNN の箱との一致（F1）」が一番よいところへ寄せる

元: umiusi_sim 側の試作 data/yk_hsv_eval_20261012/autocal/autocal.py（fg=True, r_in=0.5, min_px=16, refine=True）。
10/03 の濁ったプールで、手動の色合わせ並み（同じ run の後半で 黄 F1 0.98 / 青 0.96）。
"""

from __future__ import annotations

import collections
import copy
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Sequence, Tuple

import cv2
import numpy as np

from sinsei_umiusi_core.balloon.calibrate import propose
from sinsei_umiusi_core.balloon.config import DetectorParams, HsvRange
from sinsei_umiusi_core.balloon.detector import detect, preprocess

COLORS = ('red', 'yellow', 'blue')

# (bgr) -> [(色, 確信度, cx, cy, bw, bh)]（入力画像の画素）。CnnDetector.boxes と同じ形
Teacher = Callable[[np.ndarray], List[Tuple[str, float, float, float, float, float]]]


@dataclass
class AutoCalResult:
    params: DetectorParams  # 合わせた色だけ差し替えたもの。合わなかった色は元のまま
    report: Dict[str, dict] = field(default_factory=dict)  # 色ごとの結果（画面に出す）


def _hue_mean(px: np.ndarray) -> float:
    a = px[:, 0].astype(float) * np.pi / 90
    return (np.degrees(np.arctan2(np.sin(a).mean(), np.cos(a).mean())) / 2) % 180


def _hue_dist(a, b):
    d = np.abs(a - b)
    return np.minimum(d, 180 - d)


def autocal(
    params: DetectorParams,
    frames: Sequence[np.ndarray],
    teacher: Teacher,
    conf_floor: float = 0.2,
    top_k: int = 40,
    r_in: float = 0.5,
    min_px: int = 16,
    fg_keep: float = 0.5,
    hue_tol: int = 12,
    min_dets: int = 3,
    min_agree: float = 0.5,
    max_bg_in: float = 0.15,
    min_keep: float = 0.85,
    refine: bool = True,
    min_f1: float = 0.5,
    seed: int = 0,
) -> AutoCalResult:
    """frames: BGR 画像。teacher は確信度 0.15 くらいまで低く出すこと（背景の除外と誤りの判定に使う）。

    min_f1: 合わせた範囲の f1（先生の箱との一致）がこれ未満の色は採らず、元の値のままにする。
    10/15 のデータでは、見せていない青を CNN の誤検出から合わせ、緑を青にした（f1 0.10 で採用されていた）。
    """
    rng = np.random.default_rng(seed)
    cand = collections.defaultdict(list)  # 色 -> [(確信度, 画素, フレーム番号, 箱)]
    bg, cache = [], []
    for fi, bgr in enumerate(frames):
        hsv = preprocess(bgr, params)
        H, W = hsv.shape[:2]
        sx, sy = W / bgr.shape[1], H / bgr.shape[0]
        boxes = teacher(bgr)
        cache.append((bgr, boxes))
        yy, xx = np.mgrid[0:H, 0:W]
        outside = np.ones((H, W), bool)
        lab = None
        for c, s, bcx, bcy, bw, bh in boxes:
            cx, cy, hw, hh = bcx * sx, bcy * sy, bw * sx / 2, bh * sy / 2
            dx, dy = np.abs(xx - cx), np.abs(yy - cy)
            outside &= ~((dx <= hw * 1.3) & (dy <= hh * 1.3))
            if s < conf_floor or min(hw, hh) * 2 < min_px:
                continue
            m = ((xx - cx) / (hw * r_in)) ** 2 + ((yy - cy) / (hh * r_in)) ** 2 <= 1
            px = hsv[m]
            # 箱のまわり（1.2〜1.6 倍の輪）の水の色から遠い画素だけを風船とみなす（Lab の距離で上位 fg_keep）
            ring = (
                (dx <= hw * 1.6)
                & (dy <= hh * 1.6)
                & ~((dx <= hw * 1.2) & (dy <= hh * 1.2))
            )
            if ring.sum() >= 10 and m.sum() >= 10:
                if lab is None:
                    lab = cv2.cvtColor(
                        cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2LAB
                    ).astype(float)
                dist = np.linalg.norm(lab[m] - np.median(lab[ring], axis=0), axis=1)
                px = px[dist >= np.quantile(dist, 1 - fg_keep)]
            if len(px) > 60:
                px = px[rng.choice(len(px), 60, replace=False)]
            if len(px) >= 5:
                cand[c].append((s, px, fi, (bcx - bw / 2, bcy - bh / 2, bw, bh)))
        idx = np.flatnonzero(outside.ravel())
        if len(idx):
            bg.append(
                hsv.reshape(-1, 3)[rng.choice(idx, min(300, len(idx)), replace=False)]
            )
    bg = np.concatenate(bg) if bg else np.zeros((0, 3), np.uint8)

    report: Dict[str, dict] = {}
    col_px, pos = {}, {}
    for c in COLORS:
        lst = sorted(cand.get(c, []), key=lambda t: -t[0])[:top_k]
        if not lst:
            report[c] = {'skipped': 'CNN が見つけていない（元の値のまま）'}
            continue
        hues = np.array([_hue_mean(t[1]) for t in lst])
        votes = [(_hue_dist(hues, h) <= hue_tol).sum() for h in hues]
        center = hues[int(np.argmax(votes))]
        ok = _hue_dist(hues, center) <= hue_tol
        info = {
            'boxes': len(lst),
            'conf': [round(lst[-1][0], 2), round(lst[0][0], 2)],
            'hue': round(float(center)),
            'agree': int(ok.sum()),
        }
        if ok.sum() < min_dets or ok.mean() < min_agree:
            report[c] = {**info, 'skipped': '色相がそろわない（元の値のまま）'}
            continue
        col_px[c] = np.concatenate([t[1] for t, k in zip(lst, ok, strict=True) if k])
        pos[c] = [(t[2], t[3]) for t, k in zip(lst, ok, strict=True) if k]
        report[c] = info

    out = copy.deepcopy(params)
    for c, px in col_px.items():
        others = [v for k, v in col_px.items() if k != c]
        if len(px) < 20:
            report[c]['skipped'] = '画素が足りない（元の値のまま）'
            continue
        pr = propose(px, np.concatenate([bg] + others), hue_pct=5.0, min_keep=min_keep)
        if pr.bg_in is not None and pr.bg_in > max_bg_in:
            report[c]['skipped'] = f'背景が {pr.bg_in:.2f} 入る（元の値のまま）'
            continue
        ranges = pr.ranges
        if refine:
            ranges, f1 = _refine_sv(out, c, ranges, cache, pos[c])
            report[c]['f1'] = round(f1, 3)
            if f1 < min_f1:
                report[c]['skipped'] = f'f1 が {f1:.2f} < {min_f1}（元の値のまま）'
                continue
        out.colors[c] = ranges
        report[c].update(
            keep=round(pr.keep, 3),
            bg_in=None if pr.bg_in is None else round(pr.bg_in, 3),
            ranges=[{'lo': list(r.lo), 'hi': list(r.hi)} for r in ranges],
        )
    return AutoCalResult(params=out, report=report)


def _refine_sv(params, color, ranges, cache, want_boxes):
    """S・V の下限を振り、色合わせ用フレームで「CNN の箱との一致」が一番よいものを選ぶ。

    見つけるべきもの = 色相のそろった CNN の箱、誤り = CNN が何も見ていない所の検出。
    背景の画素の割合では「風船のまわりの水が塊につながって形の判定で落ちる」失敗が見えないので、検出の結果で選ぶ。
    20 きざみで探してから、よかった所のまわりを 10 きざみで探す（Pi で 1 色数十秒に収めるため）。
    """
    want = collections.defaultdict(list)
    for fi, b in want_boxes:
        want[fi].append(b)

    def inside(pt, b, m=0.15):
        return (
            b[0] - m * b[2] <= pt[0] <= b[0] + (1 + m) * b[2]
            and b[1] - m * b[3] <= pt[1] <= b[1] + (1 + m) * b[3]
        )

    def score(s_lo, v_lo):
        q = copy.copy(params)
        q.colors = {color: [HsvRange((r.lo[0], s_lo, v_lo), r.hi) for r in ranges]}
        tp = fp = fn = 0
        for fi, (bgr, boxes) in enumerate(cache):
            H0, W0 = bgr.shape[:2]
            ww = q.work_width
            hh = round(ww / (q.source_aspect or W0 / H0))
            sx, sy = W0 / ww, H0 / hh
            dets, _ = detect(bgr, q)
            ctr = [
                ((d.bbox[0] + d.bbox[2] / 2) * sx, (d.bbox[1] + d.bbox[3] / 2) * sy)
                for d in dets
            ]
            for b in want.get(fi, []):
                if any(inside(pt, b) for pt in ctr):
                    tp += 1
                else:
                    fn += 1
            seen = [
                (cx - bw / 2, cy - bh / 2, bw, bh) for _, _, cx, cy, bw, bh in boxes
            ]
            fp += sum(1 for pt in ctr if not any(inside(pt, b) for b in seen))
        f1 = 2 * tp / (2 * tp + fp + fn) if tp + fp + fn else 0.0
        return (f1, s_lo + v_lo)  # 同点なら締める方

    tried = {}

    def best_of(points):
        for p in points:
            if p not in tried:
                tried[p] = score(*p)
        return max(points, key=lambda p: tried[p])

    coarse = best_of([(s, v) for s in range(0, 160, 20) for v in range(0, 160, 20)])
    s0, v0 = coarse
    fine = best_of(
        [
            (s, v)
            for s in range(max(0, s0 - 10), s0 + 11, 10)
            for v in range(max(0, v0 - 10), v0 + 11, 10)
        ]
    )
    s_lo, v_lo = fine
    return [HsvRange((r.lo[0], s_lo, v_lo), r.hi) for r in ranges], tried[fine][0]


__all__ = ['AutoCalResult', 'autocal']
