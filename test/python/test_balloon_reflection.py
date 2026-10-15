"""水面の鏡像とつながった検出の後処理（sinsei_umiusi_core.balloon.reflection）の単体テスト。

python3 -m pytest test/python
"""

from sinsei_umiusi_core.balloon.config import DetectorParams
from sinsei_umiusi_core.balloon.detector import Detection
from sinsei_umiusi_core.balloon.reflection import split_reflection


def det(bbox, x=0.0, y=0.0, radius=0.1):
    return Detection('yellow', x, y, radius, 0.0, 1.0, 1.0, bbox)


def test_tall_detection_becomes_the_lower_square():
    p = DetectorParams(work_width=320, source_aspect=4 / 3)  # 作業解像度 320x240
    # 幅 40、高さ 100（風船 + 上の鏡像）。下端は y=200
    out = split_reflection([det((100, 100, 40, 100))], p, (240, 320, 3))
    d = out[0]
    assert d.bbox == (100, 160, 40, 40)
    assert abs(d.y - (180 - 120) / 120) < 1e-9  # 中心は下端から幅の半分上 = 180
    assert abs(d.radius - 20 / 320) < 1e-9


def test_round_detection_is_unchanged():
    p = DetectorParams(work_width=320, source_aspect=None)
    d = det((100, 100, 40, 48))  # 縦横比 1.2 < 1.3
    assert split_reflection([d], p, (240, 320, 3)) == [d]
