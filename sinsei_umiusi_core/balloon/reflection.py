"""水面の鏡像とつながった検出を、下側の本物の風船に直す。

水面近くでは風船の像が水面で全反射し、風船とその鏡像が縦につながって 1 つの縦長の塊として検出される。
中心が本物より上にずれるので、狙うと浮上し続ける（プールの実機 2026-10-15、sim でも中心が高さの約 0.4 倍上にずれる）。
風船はほぼ丸く、鏡像は必ず上にできるので、縦長（高さ / 幅 > aspect）の検出は「下端から幅 1 個ぶんの正方形」を本物とみなす。
"""

from __future__ import annotations

from dataclasses import replace
from typing import List, Sequence, Tuple

from sinsei_umiusi_core.balloon.config import DetectorParams
from sinsei_umiusi_core.balloon.detector import Detection


def split_reflection(
    dets: Sequence[Detection],
    params: DetectorParams,
    frame_shape: Tuple[int, ...],
    aspect: float = 1.3,
) -> List[Detection]:
    """frame_shape: 入力画像の (高さ, 幅, ...)。bbox は作業解像度（detect / detect_cnn と同じ）。"""
    h0, w0 = frame_shape[:2]
    w = params.work_width
    h = round(w / (params.source_aspect or (w0 / h0)))
    out = []
    for d in dets:
        bx, by, bw, bh = d.bbox
        if bw <= 0 or bh <= aspect * bw:
            out.append(d)
            continue
        cy = by + bh - bw / 2
        out.append(
            replace(
                d,
                y=(cy - h / 2) / (h / 2),
                radius=bw / 2 / w,
                bbox=(bx, by + bh - bw, bw, bw),
            )
        )
    return out


__all__ = ['split_reflection']
