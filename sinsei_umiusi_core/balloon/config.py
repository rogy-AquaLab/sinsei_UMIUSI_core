"""検出器のパラメータ。JSON から読む（会場で書き換える前提）。

元: sinsei_umiusi_ballon_detector d9e663d（評価ツールと評価セットはそちら）。直すときは両方を揃えること。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union


@dataclass
class HsvRange:
    # OpenCV の HSV: H は 0-179、S と V は 0-255
    lo: Tuple[int, int, int]
    hi: Tuple[int, int, int]


@dataclass
class DetectorParams:
    work_width: int = 320
    # カメラ本来の 幅/高さ。null なら入力画像の比率をそのまま使う
    source_aspect: Optional[float] = None
    # "none" | "grayworld"。通常カメラは水の色かぶりが強いので grayworld を使う
    white_balance: str = 'none'
    wb_max_gain: float = 4.0
    blur_ksize: int = 5
    morph_kernel: int = 5
    min_area_frac: float = 0.0005
    max_area_frac: float = 0.6
    min_fill: float = 0.45
    min_solidity: float = 0.8
    # 画面端に接する輪郭の充填率の下限。壁の帯（0.2〜0.3）を落とし、切れた風船は通す
    border_min_fill: float = 0.3
    colors: Dict[str, List[HsvRange]] = field(default_factory=dict)


def load_params(path: Union[str, Path]) -> DetectorParams:
    # "_" で始まるキーはコメント扱い
    raw = {
        k: v
        for k, v in json.loads(Path(path).read_text()).items()
        if not k.startswith('_')
    }
    colors = {
        name: [HsvRange(tuple(r['lo']), tuple(r['hi'])) for r in ranges]
        for name, ranges in raw.pop('colors').items()
    }
    unknown = set(raw) - set(DetectorParams.__dataclass_fields__)
    if unknown:
        raise ValueError(f'unknown keys in {path}: {sorted(unknown)}')
    return DetectorParams(colors=colors, **raw)
