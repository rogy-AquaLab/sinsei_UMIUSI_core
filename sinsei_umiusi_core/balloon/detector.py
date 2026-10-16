"""色のしきい値で風船を見つける。

流れ（1 フレームあたり）:
  1. 作業解像度へ縮小し、（設定があれば）ホワイトバランスを取り、軽くぼかす
  2. HSV に変換し、色ごとに inRange で 2 値マスクを作る
  3. 開閉処理で小さなノイズと穴を消す
  4. 輪郭を取り、面積・充填率（輪郭面積 / 外接円面積）・凸度（輪郭面積 / 凸包面積）でふるう

座標は作業解像度に依存しないよう、画像幅・高さで正規化して返す。

元: sinsei_umiusi_ballon_detector 2e25c28（評価ツールと評価セットはそちら）。直すときは両方を揃えること。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from sinsei_umiusi_core.balloon.config import DetectorParams


@dataclass
class Detection:
    color: str
    # 中心。画像中心が (0, 0)、右・下が正、端が ±1
    x: float
    y: float
    # 外接円の半径を画像幅で割ったもの
    radius: float
    # 輪郭面積を画像面積で割ったもの
    area: float
    # 輪郭面積 / 外接円面積。真円なら 1
    fill: float
    # 輪郭面積 / 凸包面積。凸なら 1
    solidity: float
    # 作業解像度でのバウンディングボックス (x, y, w, h)
    bbox: Tuple[int, int, int, int]


def gray_world(bgr: np.ndarray, max_gain: float) -> np.ndarray:
    """各チャンネルの中央値をそろえる（水の青緑かぶりを打ち消す）。

    中央値を使うのは、大きな風船が画面の半分近くを占めても背景の色を基準にするため。
    ゲインは max_gain で頭打ちにする（真っ暗・単色の画面で暴れないように）。
    """
    med = np.median(bgr.reshape(-1, 3), axis=0).astype(np.float32)
    gains = np.clip(med.mean() / np.maximum(med, 1.0), 1.0 / max_gain, max_gain)
    return cv2.convertScaleAbs(bgr.astype(np.float32) * gains)


def color_masks(hsv: np.ndarray, params: DetectorParams) -> Dict[str, np.ndarray]:
    """色ごとの 2 値マスク（開閉処理済み）を返す。"""
    k = params.morph_kernel
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)) if k > 1 else None
    masks = {}
    for color, ranges in params.colors.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for r in ranges:
            mask |= cv2.inRange(hsv, np.array(r.lo, np.uint8), np.array(r.hi, np.uint8))
        if kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        masks[color] = mask
    return masks


def preprocess(bgr: np.ndarray, params: DetectorParams) -> np.ndarray:
    """色を判定する HSV 画像（作業解像度）を作る。色合わせの道具も同じものを使う。"""
    h0, w0 = bgr.shape[:2]
    w = params.work_width
    # 入力が縦横比を潰されていても（16:9 のカメラを 320x240 に縮めた等）、カメラ本来の比率に戻してから
    # 形を見る。潰れたままだと風船が縦長になり、充填率で落ちる
    aspect = params.source_aspect or (w0 / h0)
    h = round(w / aspect)
    img = cv2.resize(bgr, (w, h), interpolation=cv2.INTER_AREA)
    if params.white_balance == 'grayworld':
        img = gray_world(img, params.wb_max_gain)
    elif params.white_balance != 'none':
        raise ValueError(f'unknown white_balance: {params.white_balance}')
    if params.blur_ksize > 1:
        img = cv2.GaussianBlur(img, (params.blur_ksize, params.blur_ksize), 0)
    return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


def detect(
    bgr: np.ndarray,
    params: DetectorParams,
    ignore_mask: Optional[np.ndarray] = None,
) -> Tuple[List[Detection], Dict[str, np.ndarray]]:
    """BGR 画像から風船を探す。

    ignore_mask: 入力と同じ範囲を覆う uint8（大きさは問わない。作業解像度に合わせて縮める）。
                 非 0 の画素は無視する（画面録画の UI 部品など）。
    戻り値: (検出のリスト（面積の大きい順）, 色ごとのマスク)
    """
    hsv = preprocess(bgr, params)
    h, w = hsv.shape[:2]

    masks = color_masks(hsv, params)
    if ignore_mask is not None:
        ignore_mask = cv2.resize(ignore_mask, (w, h), interpolation=cv2.INTER_NEAREST)
        keep = cv2.bitwise_not(ignore_mask)
        masks = {c: cv2.bitwise_and(m, keep) for c, m in masks.items()}

    img_area = float(w * h)
    detections = []
    for color, mask in masks.items():
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            area_frac = area / img_area
            if not (params.min_area_frac <= area_frac <= params.max_area_frac):
                continue
            (cx, cy), r = cv2.minEnclosingCircle(c)
            fill = area / (np.pi * r * r) if r > 0 else 0.0
            bx, by, bw, bh = cv2.boundingRect(c)
            on_border = bx <= 0 or by <= 0 or bx + bw >= w or by + bh >= h
            # 画面端で切れた風船は丸くないので、充填率の下限を緩める。
            # 近づいて突っ込む直前はほぼ必ずこの状態になる
            if fill < (params.border_min_fill if on_border else params.min_fill):
                continue
            # 水面の映り込み・手・紐はギザギザ / 細長い。風船は凸に近い
            hull_area = cv2.contourArea(cv2.convexHull(c))
            solidity = area / hull_area if hull_area > 0 else 0.0
            if solidity < params.min_solidity:
                continue
            detections.append(
                Detection(
                    color=color,
                    x=(cx - w / 2) / (w / 2),
                    y=(cy - h / 2) / (h / 2),
                    radius=r / w,
                    area=area_frac,
                    fill=fill,
                    solidity=solidity,
                    bbox=(bx, by, bw, bh),
                )
            )
    detections.sort(key=lambda d: d.area, reverse=True)
    return detections, masks
