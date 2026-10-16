"""検出を指令の周期から切り離す（別スレッド）と、hsv+cnn の CNN 補助の判定。ROS に依存しない。"""

from __future__ import annotations

import threading
import time
from typing import List, Optional, Sequence

from sinsei_umiusi_core.balloon.detector import Detection

# cnn_assist_confirm: 前回の補助の検出と「近い」とみなす距離（画面の正規化座標、幅・高さとも -1〜1）
CONFIRM_DIST = 0.15


def confirm_assist(
    dets: Sequence[Detection], prev: Sequence[Detection]
) -> List[Detection]:
    """前回も近くに同じ色があった検出だけを返す（反射など 1 コマだけの誤検出を落とす）。"""
    return [
        d
        for d in dets
        if any(
            p.color == d.color
            and abs(p.x - d.x) <= CONFIRM_DIST
            and abs(p.y - d.y) <= CONFIRM_DIST
            for p in prev
        )
    ]


def fresh_assist(result, now: float, max_age: float) -> Optional[List[Detection]]:
    """DetectionWorker.latest() の結果が max_age 秒以内のコマで、何か見えていればその検出を返す。"""
    if result is None or now - result[1] > max_age or not result[3]:
        return None
    return result[3]


class DetectionWorker:
    """別スレッドで、新しいコマが来るたびに detect_fn を回し、最新の結果だけを持つ。

    検出（特に CNN）が 1 周期より遅くても、指令の周期（_tick）を止めないため。
    実機（2026-10-15）で CNN を周期の中で回すと、指令の更新が 3〜6 Hz に落ちて接近中に左右へ振れた。
    """

    def __init__(self, name: str, get_frame, detect_fn, logger) -> None:
        self._get_frame = get_frame  # () -> (seq, t_frame, frame) か None
        self._detect_fn = detect_fn  # (frame) -> [Detection] か None（この回は休む）
        self._logger = logger
        self._lock = threading.Lock()
        self._result = None  # (seq, t_frame, frame, dets, 検出にかかった秒)
        self._taken = True
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def latest(self):
        with self._lock:
            return self._result

    def take(self):
        """まだ渡していない新しい結果があれば返す（1 回だけ）。"""
        with self._lock:
            if self._taken or self._result is None:
                return None
            self._taken = True
            return self._result

    def _run(self) -> None:
        last_seq = 0
        while not self._stop.is_set():
            got = self._get_frame()
            if got is None or got[0] == last_seq or got[2] is None:
                self._stop.wait(0.005)
                continue
            seq, t_frame, frame = got
            last_seq = seq
            t0 = time.monotonic()
            try:
                dets = self._detect_fn(frame)
            except Exception as e:  # noqa: BLE001 検出が落ちても止めない（次のコマで続ける）
                self._logger.error(f'検出で例外: {e!r}', throttle_duration_sec=5.0)
                continue
            if dets is None:
                self._stop.wait(0.05)
                continue
            with self._lock:
                self._result = (seq, t_frame, frame, dets, time.monotonic() - t0)
                self._taken = False
