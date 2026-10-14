"""学習した CNN（ONNX）で風船を見つける。色のしきい値（detector.detect）の代わりに使える。

モデル: umiusi_sim の TinyBalloonNet（CenterNet 風、出力のきざみ 8 画素）。
  入力 x: (1, 3, N, N) の RGB / 255（画像を N x N へそのまま縮める。縦横比は保たない）
  出力 hm: (1, 3, N/8, N/8) 色ごと（赤・黄・青の順）の中心らしさのロジット
       wh: (1, 2, N/8, N/8) 箱の幅・高さ（入力の一辺に対する割合）
学習側の復号（umiusi_perception.learned_detector.decode）と同じ手順で箱にする。直すときは両方を揃えること。

推論は onnxruntime があればそれ、無ければ OpenCV の dnn（Conv と ReLU だけのモデルなのでどちらでも動く）。
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Union

import cv2
import numpy as np

from sinsei_umiusi_core.balloon.config import DetectorParams
from sinsei_umiusi_core.balloon.detector import Detection

# 出力の色の並び（学習側の COLOURS と同じ。COCO の category_id の順ではない）
COLORS = ('red', 'yellow', 'blue')
STRIDE = 8
TOPK = 40


class CnnDetector:
    def __init__(
        self, path: Union[str, Path], conf: float = 0.3, backend: str = 'auto'
    ) -> None:
        self.path = Path(path)
        self.conf = conf
        self._ort = None
        self._net = None
        if backend in ('auto', 'onnxruntime'):
            try:
                import onnxruntime as ort

                opts = ort.SessionOptions()
                opts.intra_op_num_threads = (
                    2  # Pi の 4 コアを使い切らない（カメラ・制御が同居）
                )
                self._ort = ort.InferenceSession(
                    str(self.path), opts, providers=['CPUExecutionProvider']
                )
            except ImportError:
                if backend == 'onnxruntime':
                    raise
        if self._ort is None:
            self._net = cv2.dnn.readNetFromONNX(str(self.path))
        self.backend = 'onnxruntime' if self._ort is not None else 'opencv'
        self.input_size = self._input_size()

    def _input_size(self) -> int:
        if self._ort is not None:
            n = self._ort.get_inputs()[0].shape[-1]
            if isinstance(n, int):
                return n
        return 320  # 形が埋め込まれていないモデル（OpenCV では読めない）は学習時の既定

    def infer(self, bgr: np.ndarray):
        """(hm ロジット (3, n, n), wh (2, n, n))。"""
        n = self.input_size
        # cv2.resize の INTER_LINEAR は torch の interpolate(bilinear, align_corners=False) と同じ標本点。
        # uint8 のまま縮めると重みが固定小数点に丸められ、しきい値近くの検出が学習側とずれるので float で縮める
        img = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        img = cv2.resize(img, (n, n), interpolation=cv2.INTER_LINEAR)
        x = np.ascontiguousarray(img.transpose(2, 0, 1)[None])
        if self._ort is not None:
            hm, wh = self._ort.run(['hm', 'wh'], {'x': x})
        else:
            self._net.setInput(x)
            hm, wh = self._net.forward(['hm', 'wh'])
        return hm[0], wh[0]

    def boxes(self, bgr: np.ndarray):
        """[(色, 確信度, cx, cy, bw, bh)]（入力画像の画素）。"""
        h0, w0 = bgr.shape[:2]
        n = self.input_size
        logits, wh = self.infer(bgr)
        hm = 1.0 / (1.0 + np.exp(-logits))
        # 3x3 の極大だけ残す（学習側の max_pool2d と同じ。端は -inf で埋める）
        pad = np.pad(hm, ((0, 0), (1, 1), (1, 1)), constant_values=-np.inf)
        fh, fw = hm.shape[1:]
        mx = np.max(
            [pad[:, dy : dy + fh, dx : dx + fw] for dy in range(3) for dx in range(3)],
            axis=0,
        )
        peaks = np.where(mx == hm, hm, 0.0)
        out = []
        for ci, color in enumerate(COLORS):
            flat = peaks[ci].ravel()
            k = min(TOPK, flat.size)
            idx = np.argpartition(-flat, k - 1)[:k]
            for i in idx[np.argsort(-flat[idx])]:
                s = float(flat[i])
                if s < self.conf:
                    break
                gy, gx = divmod(int(i), fw)
                cx = (gx + 0.5) * STRIDE * w0 / n
                cy = (gy + 0.5) * STRIDE * h0 / n
                bw = max(float(wh[0, gy, gx]), 1e-3) * w0
                bh = max(float(wh[1, gy, gx]), 1e-3) * h0
                out.append((color, s, cx, cy, bw, bh))
        return out


def detect_cnn(
    bgr: np.ndarray, model: CnnDetector, params: DetectorParams
) -> List[Detection]:
    """detector.detect と同じ形の Detection を返す（面積の大きい順）。

    params は作業解像度（work_width）とカメラ本来の比率（source_aspect）だけを使う。
    radius は色の検出器と揃えて「本来の比率に戻した画像で、箱の長い辺の半分 / 画像幅」。
    fill・solidity は 1、area は箱に内接する楕円の面積 / 画像面積。
    """
    h0, w0 = bgr.shape[:2]
    w = params.work_width
    aspect = params.source_aspect or (w0 / h0)
    h = round(w / aspect)
    dets = []
    for color, s, cx, cy, bw, bh in model.boxes(bgr):
        u0, v0 = max(0.0, cx - bw / 2), max(0.0, cy - bh / 2)
        u1, v1 = min(float(w0), cx + bw / 2), min(float(h0), cy + bh / 2)
        if u1 <= u0 or v1 <= v0:
            continue
        fw, fh = (u1 - u0) / w0, (v1 - v0) / h0  # 画像に対する割合
        dets.append(
            Detection(
                color=color,
                x=(cx - w0 / 2) / (w0 / 2),
                y=(cy - h0 / 2) / (h0 / 2),
                radius=max(fw * w, fh * h) / 2 / w,
                area=float(np.pi / 4 * fw * fh),
                fill=1.0,
                solidity=1.0,
                bbox=(
                    round(u0 / w0 * w),
                    round(v0 / h0 * h),
                    round(fw * w),
                    round(fh * h),
                ),
                score=s,
            )
        )
    dets.sort(key=lambda d: d.area, reverse=True)
    return dets
