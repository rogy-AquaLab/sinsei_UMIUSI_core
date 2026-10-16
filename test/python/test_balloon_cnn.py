import dataclasses
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from sinsei_umiusi_core.balloon import load_params  # noqa: E402
from sinsei_umiusi_core.balloon.cnn import CnnDetector, detect_cnn  # noqa: E402

CONFIG = Path(__file__).resolve().parents[2] / 'config' / 'balloon'
MODELS = sorted((CONFIG / 'models').glob('*.onnx'))
PARAMS = dataclasses.replace(load_params(CONFIG / 'normal.json'), source_aspect=None)


@pytest.mark.parametrize('path', MODELS, ids=lambda p: p.name)
def test_shipped_models_run_on_opencv(path):
    det = CnnDetector(path, backend='opencv')
    hm, wh = det.infer(np.zeros((240, 320, 3), np.uint8))
    n = det.input_size // 8
    assert hm.shape == (3, n, n) and wh.shape == (2, n, n)


class FakeNet(CnnDetector):
    """推論の代わりに決めた hm / wh を返す（復号だけを確かめる）。"""

    def __init__(self, hm, wh, conf=0.3):
        self.conf, self.input_size, self._out = conf, 320, (hm, wh)

    def infer(self, bgr):
        return self._out


def peak_map(cells):
    """cells: [(色の番号, gy, gx, 確信度)] -> ロジット (3, 40, 40)。"""
    hm = np.full((3, 40, 40), -10.0, np.float32)
    for ci, gy, gx, p in cells:
        hm[ci, gy, gx] = np.log(p / (1 - p))
    return hm


def test_decodes_centre_size_and_colour():
    wh = np.zeros((2, 40, 40), np.float32)
    wh[:, 10, 30] = (0.25, 0.5)  # 入力の一辺に対する割合
    det = FakeNet(peak_map([(2, 10, 30, 0.9)]), wh)
    img = np.zeros((240, 320, 3), np.uint8)
    (d,) = detect_cnn(img, det, PARAMS)
    assert d.color == 'blue'  # 出力の並びは 赤・黄・青
    # 中心はセルの中央: (30.5 * 8 / 320 * 320, 10.5 * 8 / 320 * 240) = (244, 63)
    assert d.x == pytest.approx((244 - 160) / 160)
    assert d.y == pytest.approx((63 - 120) / 120)
    # 箱 80 x 120 px。長い辺の半分 / 画像幅
    assert d.radius == pytest.approx(60 / 320)
    assert d.score == pytest.approx(0.9)


def test_drops_low_confidence_and_non_peaks():
    hm = peak_map([(0, 5, 5, 0.6), (0, 5, 6, 0.5), (1, 20, 20, 0.2)])
    wh = np.full((2, 40, 40), 0.05, np.float32)
    dets = detect_cnn(np.zeros((240, 320, 3), np.uint8), FakeNet(hm, wh), PARAMS)
    # 隣の 0.5 は 0.6 の極大に負ける。黄の 0.2 は下限 0.3 未満
    assert [(d.color, round(d.score, 2)) for d in dets] == [('red', 0.6)]


def test_radius_uses_source_aspect():
    """16:9 のカメラを 320x240 に潰した画像でも、色の検出器と同じく本来の比率で大きさを測る。"""
    wh = np.zeros((2, 40, 40), np.float32)
    wh[:, 20, 20] = (0.1, 0.1)
    det = FakeNet(peak_map([(0, 20, 20, 0.9)]), wh)
    img = np.zeros((240, 320, 3), np.uint8)
    squashed = dataclasses.replace(PARAMS, source_aspect=16 / 9)
    (d,) = detect_cnn(img, det, squashed)
    # 本来の比率では高さ 10% の箱は 320 / (16/9) * 0.1 = 18 px、幅は 32 px -> 長い辺は幅
    assert d.radius == pytest.approx(0.05)
