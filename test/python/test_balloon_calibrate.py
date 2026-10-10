import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from sinsei_umiusi_core.balloon.calibrate import hue_interval, in_ranges, propose  # noqa: E402

RNG = np.random.default_rng(0)


def samples(h, s, v, n=500, spread=(3, 15, 15)):
    out = np.stack(
        [
            RNG.normal(h, spread[0], n) % 180,
            np.clip(RNG.normal(s, spread[1], n), 0, 255),
            np.clip(RNG.normal(v, spread[2], n), 0, 255),
        ],
        axis=1,
    )
    return out.astype(np.uint8)


def test_hue_interval_plain():
    a, b = hue_interval(np.array([20, 21, 22, 25, 30]))
    assert a <= 21 and b >= 29 and a < b


def test_hue_interval_wraps_around_red():
    a, b = hue_interval(np.array([175, 177, 179, 0, 2, 4]))
    assert b < a  # 179 をまたぐ
    assert a <= 176 and b >= 3


def test_yellow_range_covers_samples():
    y = samples(22, 220, 230)
    p = propose(y)
    assert len(p.ranges) == 1
    assert p.keep > 0.95
    lo, hi = p.ranges[0].lo, p.ranges[0].hi
    assert lo[0] <= 15 and hi[0] >= 29


def test_red_is_split_into_two_ranges():
    r = samples(178, 230, 200)
    p = propose(r)
    assert len(p.ranges) == 2
    assert {p.ranges[0].lo[0], p.ranges[1].hi[0]} == {0, 179}
    assert p.keep > 0.95


def test_background_raises_saturation_floor():
    # 9/13 の NoIR: 青い風船は淡いピンク。背景（水面近く）も同じ色相・同じくらいの明るさで、S だけ低い
    balloon = samples(170, 70, 235, spread=(3, 10, 8))
    background = samples(172, 35, 232, spread=(3, 10, 8))
    without = propose(balloon)
    with_bg = propose(balloon, background)
    assert in_ranges(background, without.ranges).mean() > with_bg.bg_in
    assert with_bg.bg_in <= 0.05
    assert with_bg.keep >= 0.85


def test_too_few_samples():
    with pytest.raises(ValueError):
        propose(samples(20, 200, 200, n=5))
