"""色の設定ファイルの更新を見て、変わっていたら読み直す（色合わせの結果をノードを止めずに反映する）。

書き込む側（scripts/calibrate_server.py）は一時ファイルに書いてから置き換えるので、書きかけを読むことはない。
読めなかったとき（手で書き換えて壊した等）は、前の設定のまま動き続ける。

最初に読めないとき（current.json をまだ作っていない等）は、fallback の設定で始めて、ファイルができたら切り替える。
ここで起動を失敗させると、ノードが未構成のまま残り、AUTO に入ったときにプロセスごと落ちるため
（rclpy は未構成のノードへの activate を例外にする）。
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional, Tuple

from sinsei_umiusi_core.balloon.config import DetectorParams, load_params


class ConfigWatcher:
    def __init__(self, path: Path, fallback: Optional[Path] = None) -> None:
        self.path = Path(path)
        # 最初に読めなかったときの説明（fallback で動いている間だけ入る）
        self.startup_error: Optional[str] = None
        # 今の設定を読んだファイルと時刻（UI に出す）
        self.loaded_from: Path = self.path
        self.loaded_at: float = time.time()
        try:
            self.params: DetectorParams = load_params(self.path)
            self._stamp: Optional[Tuple[int, int]] = self._stat()
        except Exception as e:  # noqa: BLE001
            if fallback is None:
                raise
            self.params = load_params(fallback)
            self._stamp = None
            self.loaded_from = Path(fallback)
            self.startup_error = f'{self.path} を読めないので {fallback} で動く: {e}'

    def _stat(self) -> Tuple[int, int]:
        st = self.path.stat()
        return st.st_mtime_ns, st.st_size

    def poll(self) -> Tuple[Optional[DetectorParams], Optional[str]]:
        """(読み直した設定 or None, エラーの説明 or None)。変わっていなければ (None, None)。"""
        try:
            stamp = self._stat()
        except OSError as e:
            if self.startup_error:
                return None, self.startup_error
            return None, f'設定ファイルが見えない（前の設定のまま）: {e}'
        if stamp == self._stamp:
            return None, None
        self._stamp = stamp
        try:
            params = load_params(self.path)
        except Exception as e:  # noqa: BLE001 何が壊れていても前の設定を保つ
            return None, f'設定ファイルを読めない（前の設定のまま）: {e}'
        self.params = params
        self.startup_error = None
        self.loaded_from = self.path
        self.loaded_at = time.time()
        return params, None
