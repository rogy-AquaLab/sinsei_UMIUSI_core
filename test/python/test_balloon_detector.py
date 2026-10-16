import dataclasses
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from sinsei_umiusi_core.balloon import detect, load_params  # noqa: E402

CONFIGS = Path(__file__).resolve().parents[2] / 'config' / 'balloon'
# 合成画像はカメラ本来の比率で作るので、比率の補正は外して使う（補正そのものは別のテストで見る）
NOIR = dataclasses.replace(load_params(CONFIGS / 'noir.json'), source_aspect=None)
NORMAL = dataclasses.replace(load_params(CONFIGS / 'normal.json'), source_aspect=None)

# 9/13 の実映像から取った代表値 (HSV)
YELLOW_HSV = (22, 248, 240)
RED_HSV = (176, 255, 225)
TEAL_HSV = (90, 78, 138)  # プールの壁


def bgr(hsv):
    return tuple(
        int(v) for v in cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0]
    )


def canvas(w=320, h=240):
    img = np.zeros((h, w, 3), np.uint8)
    img[:] = bgr(TEAL_HSV)
    return img


def test_finds_yellow_circle_with_normalised_centre():
    img = canvas()
    cv2.circle(img, (240, 60), 25, bgr(YELLOW_HSV), -1)
    dets, _ = detect(img, NOIR)
    assert [d.color for d in dets] == ['yellow']
    d = dets[0]
    assert d.x == pytest.approx(0.5, abs=0.03)
    assert d.y == pytest.approx(-0.5, abs=0.03)
    assert d.fill > 0.8 and d.solidity > 0.9


def test_red_hue_wraps_around_zero():
    img = canvas()
    cv2.circle(img, (80, 120), 20, bgr(RED_HSV), -1)
    cv2.circle(img, (240, 120), 20, bgr((3, 255, 225)), -1)
    dets, _ = detect(img, NOIR)
    assert sorted(d.color for d in dets) == ['red', 'red']


def test_background_only_gives_nothing():
    dets, _ = detect(canvas(), NOIR)
    assert dets == []


def test_thin_line_is_rejected():
    # 赤い紐（9/13 の映像に写っていた）
    img = canvas()
    cv2.line(img, (20, 200), (300, 40), bgr(RED_HSV), 4)
    dets, _ = detect(img, NOIR)
    assert dets == []


def test_jagged_blob_is_rejected():
    # 手や水面の映り込みのような、凹凸の大きい形
    img = canvas()
    pts = []
    for i in range(16):
        r = 50 if i % 2 == 0 else 18
        a = 2 * np.pi * i / 16
        pts.append((160 + r * np.cos(a), 120 + r * np.sin(a)))
    cv2.fillPoly(img, [np.int32(pts)], bgr(RED_HSV))
    dets, _ = detect(img, NOIR)
    assert dets == []


def test_balloon_cut_by_image_edge_is_kept():
    # 突っ込む直前: 風船が画面の下からはみ出している
    img = canvas()
    cv2.circle(img, (160, 260), 90, bgr(YELLOW_HSV), -1)
    dets, _ = detect(img, NOIR)
    assert [d.color for d in dets] == ['yellow']


def test_flat_band_along_edge_is_rejected():
    # 壁の帯のような、端に沿った平たい領域
    img = canvas()
    cv2.rectangle(img, (0, 0), (260, 20), bgr(YELLOW_HSV), -1)
    dets, _ = detect(img, NOIR)
    assert dets == []


def test_ignore_mask_hides_region():
    img = canvas()
    cv2.circle(img, (240, 60), 25, bgr(YELLOW_HSV), -1)
    ignore = np.zeros((240, 320), np.uint8)
    ignore[20:110, 200:290] = 255
    dets, _ = detect(img, NOIR, ignore)
    assert dets == []


def test_input_resolution_does_not_change_normalised_result():
    small = canvas(320, 180)
    cv2.circle(small, (100, 90), 20, bgr(YELLOW_HSV), -1)
    big = cv2.resize(small, (1280, 720), interpolation=cv2.INTER_NEAREST)
    (a,), _ = detect(small, NOIR)
    (b,), _ = detect(big, NOIR)
    assert a.x == pytest.approx(b.x, abs=0.02)
    assert a.y == pytest.approx(b.y, abs=0.02)
    assert a.radius == pytest.approx(b.radius, rel=0.1)


def test_noir_blue_balloon_is_pale_pink_and_separated_by_brightness():
    # 9/13 の実測: NoIR では青い風船が淡いピンク (H≈170 S≈60 V≈235)、
    # 水面付近のピンクがかった背景は H≈175 S≈30 V≈175
    img = np.zeros((240, 320, 3), np.uint8)
    img[:] = bgr((175, 30, 175))
    cv2.circle(img, (160, 120), 30, bgr((170, 60, 235)), -1)
    dets, _ = detect(img, NOIR)
    assert [d.color for d in dets] == ['blue']


def test_normal_profile_sees_dull_red_through_cyan_water():
    # 2019 公式動画の実測 (補正前の BGR 由来の HSV): 水 H≈86 S≈90 V≈255、赤 H≈168 S≈54 V≈158
    # 補正前の HSV では赤の彩度が水と変わらないが、gray world で分かれる
    img = np.zeros((240, 320, 3), np.uint8)
    img[:] = bgr((86, 90, 255))
    cv2.circle(img, (160, 120), 30, bgr((168, 54, 158)), -1)
    assert detect(img, NOIR)[0] == []
    dets, _ = detect(img, NORMAL)
    assert [d.color for d in dets] == ['red']


def test_comment_keys_are_ignored(tmp_path):
    p = tmp_path / 'c.json'
    p.write_text('{"_comment": "x", "colors": {}}')
    assert load_params(p).colors == {}


def test_squashed_input_is_restored_to_camera_aspect():
    # 16:9 のカメラを 320x240 に縮めると円が縦長の楕円になる。source_aspect で戻して円として扱う
    wide = canvas(640, 360)
    cv2.circle(wide, (320, 180), 60, bgr(YELLOW_HSV), -1)
    squashed = cv2.resize(wide, (320, 240), interpolation=cv2.INTER_AREA)
    params = load_params(CONFIGS / 'noir.json')
    assert params.source_aspect == pytest.approx(16 / 9, abs=0.01)
    (d,), _ = detect(squashed, params)
    assert d.fill > 0.85
    (e,), _ = detect(squashed, NOIR)  # 補正なしだと縦長のまま
    assert e.fill < 0.8


def test_unknown_config_key_is_an_error(tmp_path):
    p = tmp_path / 'bad.json'
    p.write_text('{"colors": {}, "min_fil": 0.5}')
    with pytest.raises(ValueError, match='min_fil'):
        load_params(p)
