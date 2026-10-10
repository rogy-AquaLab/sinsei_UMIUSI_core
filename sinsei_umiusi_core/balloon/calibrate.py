"""クリックで集めた画素から、色ごとの HSV の範囲を提案する（色合わせの道具の計算部分）。

- 色相は 0 と 179 がつながっている（赤がまたぐ）。サンプルの無い一番広い隙間で円を切り開いてから範囲を取る
- 鮮やかさ (S)・明るさ (V) は下限だけを決める（上限は 255）
- 背景のサンプルがあれば、背景が範囲に入る割合が bg_max 以下になるまで S・V の下限を上げる
  （上げ過ぎて風船の画素を min_keep より多く落とすなら、そこで止める）

元: sinsei_umiusi_ballon_detector 2e25c28（評価ツールと評価セットはそちら）。直すときは両方を揃えること。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from sinsei_umiusi_core.balloon.config import HsvRange


@dataclass
class Proposal:
    ranges: List[HsvRange]
    keep: float  # 風船のサンプルのうち範囲に入る割合
    bg_in: Optional[
        float
    ]  # 背景のサンプルのうち範囲に入る割合（背景サンプルが無ければ None）


def in_ranges(hsv: np.ndarray, ranges: List[HsvRange]) -> np.ndarray:
    """(N, 3) の HSV が範囲のどれかに入るか。"""
    ok = np.zeros(len(hsv), bool)
    for r in ranges:
        lo, hi = np.array(r.lo), np.array(r.hi)
        ok |= np.all((hsv >= lo) & (hsv <= hi), axis=1)
    return ok


def hue_interval(h: np.ndarray, lo_pct: float = 2.0, hi_pct: float = 98.0) -> tuple:
    """円周上の色相の、外れ値を除いた区間 (始まり, 終わり) を返す。終わり < 始まり なら 179 をまたぐ。"""
    hs = np.sort(np.unique(h.astype(int)))
    if len(hs) == 1:
        return int(hs[0]), int(hs[0])
    gaps = np.diff(np.concatenate([hs, [hs[0] + 180]]))
    cut = int(np.argmax(gaps))  # この後ろに一番広い隙間がある -> その次の値から始める
    start = hs[(cut + 1) % len(hs)]
    unwrapped = (h.astype(int) - start) % 180
    a = np.percentile(unwrapped, lo_pct)
    b = np.percentile(unwrapped, hi_pct)
    return int(round(start + a)) % 180, int(round(start + b)) % 180


def _hue_ranges(h0: int, h1: int, margin: int, s_lo: int, v_lo: int) -> List[HsvRange]:
    width = (h1 - h0) % 180 + 2 * margin
    if width >= 179:
        return [HsvRange((0, s_lo, v_lo), (179, 255, 255))]
    a = (h0 - margin) % 180
    b = (h1 + margin) % 180
    if a <= b:
        return [HsvRange((a, s_lo, v_lo), (b, 255, 255))]
    return [
        HsvRange((0, s_lo, v_lo), (b, 255, 255)),
        HsvRange((a, s_lo, v_lo), (179, 255, 255)),
    ]


def propose(
    color_hsv: np.ndarray,
    bg_hsv: Optional[np.ndarray] = None,
    hue_margin: int = 4,
    sv_margin: int = 15,
    bg_max: float = 0.05,
    min_keep: float = 0.85,
    hue_pct: float = 2.0,
) -> Proposal:
    """風船の画素 (N, 3) と背景の画素 (M, 3)（任意）から範囲を提案する。

    hue_pct: 色相の両端で外れ値として捨てる割合 [%]。見本が汚れている（別の色が混ざる）なら上げる。
    """
    if len(color_hsv) < 20:
        raise ValueError('風船の画素が少なすぎる（もっとクリックする）')
    h0, h1 = hue_interval(color_hsv[:, 0], hue_pct, 100.0 - hue_pct)
    s_lo = int(max(0, np.percentile(color_hsv[:, 1], 3) - sv_margin))
    v_lo = int(max(0, np.percentile(color_hsv[:, 2], 3) - sv_margin))
    ranges = _hue_ranges(h0, h1, hue_margin, s_lo, v_lo)

    if bg_hsv is not None and len(bg_hsv):
        # 背景が多く入るなら、S と V の下限を交互に上げて締める
        for _ in range(60):
            if in_ranges(bg_hsv, ranges).mean() <= bg_max:
                break
            s_next, v_next = min(255, s_lo + 5), min(255, v_lo + 5)
            cand_s = _hue_ranges(h0, h1, hue_margin, s_next, v_lo)
            cand_v = _hue_ranges(h0, h1, hue_margin, s_lo, v_next)
            # 背景をより多く落とせる方を選ぶ（同じなら風船を多く残す方）
            score_s = (
                in_ranges(bg_hsv, cand_s).mean(),
                -in_ranges(color_hsv, cand_s).mean(),
            )
            score_v = (
                in_ranges(bg_hsv, cand_v).mean(),
                -in_ranges(color_hsv, cand_v).mean(),
            )
            cand, s_c, v_c = (
                (cand_s, s_next, v_lo) if score_s <= score_v else (cand_v, s_lo, v_next)
            )
            if in_ranges(color_hsv, cand).mean() < min_keep:
                break
            ranges, s_lo, v_lo = cand, s_c, v_c

    keep = float(in_ranges(color_hsv, ranges).mean())
    bg_in = (
        float(in_ranges(bg_hsv, ranges).mean())
        if bg_hsv is not None and len(bg_hsv)
        else None
    )
    return Proposal(ranges=ranges, keep=keep, bg_in=bg_in)
