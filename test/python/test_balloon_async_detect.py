"""検出の別スレッド化と hsv+cnn の補助の判定（sinsei_umiusi_core.balloon.async_detect）の単体テスト。

python3 -m pytest test/python
"""

import threading
import time

from sinsei_umiusi_core.balloon.async_detect import (
    DetectionWorker,
    confirm_assist,
    fresh_assist,
)
from sinsei_umiusi_core.balloon.detector import Detection


class _Logger:
    def error(self, *a, **k):
        pass


def det(color='red', x=0.0, y=0.0):
    return Detection(color, x, y, 0.1, 0.0, 1.0, 0.9, (0, 0, 10, 10))


def wait_for(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.005)
    return False


def test_confirm_keeps_only_detections_seen_nearby_last_time():
    prev = [det('red', 0.0, 0.0)]
    near, far, other = (
        det('red', 0.1, -0.1),
        det('red', 0.5, 0.0),
        det('yellow', 0.0, 0.0),
    )
    assert confirm_assist([near, far, other], prev) == [near]
    assert confirm_assist([near], []) == []


def test_fresh_assist_rejects_old_or_empty_results():
    d = [det()]
    assert fresh_assist((1, 10.0, None, d, 0.1), now=10.4, max_age=0.5) == d
    assert fresh_assist((1, 10.0, None, d, 0.1), now=10.6, max_age=0.5) is None
    assert fresh_assist((1, 10.0, None, [], 0.1), now=10.1, max_age=0.5) is None
    assert fresh_assist(None, now=10.0, max_age=0.5) is None


def test_worker_detects_each_new_frame_once_and_take_is_one_shot():
    frame = {'v': (0, 0.0, None)}
    calls = []

    def detect_fn(f):
        calls.append(f)
        return [det()]

    w = DetectionWorker('t', lambda: frame['v'], detect_fn, _Logger())
    w.start()
    try:
        assert w.take() is None  # まだ何も無い
        frame['v'] = (1, 1.0, 'f1')
        assert wait_for(lambda: w.latest() is not None)
        r = w.take()
        assert r[0] == 1 and r[2] == 'f1' and r[3] == [det()]
        assert w.take() is None  # 同じ結果は 2 回渡さない
        assert w.latest()[0] == 1  # latest は何度でも見られる
        time.sleep(0.05)
        assert calls == ['f1']  # 同じコマは検出し直さない
        frame['v'] = (2, 2.0, 'f2')
        assert wait_for(lambda: w.latest()[0] == 2)
        assert w.take()[2] == 'f2'
    finally:
        w.stop()


def test_worker_survives_exceptions_and_skips_none():
    frame = {'v': (1, 1.0, 'bad')}
    gate = threading.Event()

    def detect_fn(f):
        if f == 'bad':
            raise RuntimeError('boom')
        if f == 'skip':
            return None  # この回は休む（結果を上書きしない）
        gate.set()
        return [det()]

    w = DetectionWorker('t', lambda: frame['v'], detect_fn, _Logger())
    w.start()
    try:
        time.sleep(0.05)
        assert w.latest() is None
        frame['v'] = (2, 2.0, 'skip')
        time.sleep(0.1)
        assert w.latest() is None
        frame['v'] = (3, 3.0, 'ok')
        assert gate.wait(2.0)
        assert wait_for(lambda: w.latest() is not None and w.latest()[0] == 3)
    finally:
        w.stop()
