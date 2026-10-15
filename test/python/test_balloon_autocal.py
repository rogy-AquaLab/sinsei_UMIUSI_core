"""色合わせの自動提案（sinsei_umiusi_core.balloon.autocal）の単体テスト。ROS・CNN 不要（先生は箱を返す関数）。

python3 -m pytest test/python
"""

import cv2
import numpy as np

from sinsei_umiusi_core.balloon.autocal import autocal
from sinsei_umiusi_core.balloon.config import DetectorParams, HsvRange
from sinsei_umiusi_core.balloon.detector import detect

WATER = (120, 90, 30)  # BGR の青緑っぽい水
YELLOW = (40, 200, 230)


def scene(seed):
    """水の上に黄色い円が 1〜2 個。先生（CNN の代わり）はその箱を返す。"""
    rng = np.random.default_rng(seed)
    img = np.full((240, 320, 3), WATER, np.uint8)
    img = np.clip(img + rng.normal(0, 6, img.shape), 0, 255).astype(np.uint8)
    boxes = []
    for _ in range(1 + seed % 2):
        cx, cy, r = (
            int(rng.integers(60, 260)),
            int(rng.integers(60, 180)),
            int(rng.integers(18, 35)),
        )
        cv2.circle(img, (cx, cy), r, YELLOW, -1)
        boxes.append(('yellow', 0.9, float(cx), float(cy), 2.0 * r, 2.0 * r))
    return img, boxes


def test_fits_yellow_from_teacher_boxes_and_keeps_unseen_colors():
    scenes = [scene(s) for s in range(12)]
    teacher = dict((id(img), boxes) for img, boxes in scenes)
    base = DetectorParams()
    base.colors = {
        'yellow': [HsvRange((0, 0, 0), (179, 255, 255))],  # 何でも黄色（合わせる前）
        'red': [HsvRange((170, 100, 100), (179, 255, 255))],
    }
    result = autocal(base, [img for img, _ in scenes], lambda bgr: teacher[id(bgr)])

    assert 'skipped' not in result.report['yellow']
    assert result.report['yellow']['f1'] >= 0.9
    # 赤と青は先生が見ていないので元のまま
    assert 'skipped' in result.report['red'] and 'skipped' in result.report['blue']
    assert result.params.colors['red'] == base.colors['red']
    # 合わせた範囲で、黄色い円は見つかり、水だけの画像では何も見つからない
    img, boxes = scene(100)
    dets, _ = detect(img, result.params)
    assert any(d.color == 'yellow' for d in dets)
    water = np.full((240, 320, 3), WATER, np.uint8)
    assert not [d for d in detect(water, result.params)[0] if d.color == 'yellow']


def test_skips_when_teacher_sees_nothing():
    img = np.full((240, 320, 3), WATER, np.uint8)
    result = autocal(DetectorParams(), [img] * 3, lambda bgr: [])
    assert all('skipped' in result.report[c] for c in ('red', 'yellow', 'blue'))
