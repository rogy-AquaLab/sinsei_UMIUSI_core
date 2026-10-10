"""色の設定ファイルの更新を見て、変わっていたら読み直す（色合わせの結果をノードを止めずに反映する）。

書き込む側（scripts/calibrate_server.py）は一時ファイルに書いてから置き換えるので、書きかけを読むことはない。
読めなかったとき（手で書き換えて壊した等）は、前の設定のまま動き続ける。
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple

from sinsei_umiusi_core.balloon.config import DetectorParams, load_params


class ConfigWatcher:
    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        # 最初に読めなければ例外（起動を失敗させる）
        self.params: DetectorParams = load_params(self.path)
        self._stamp = self._stat()

    def _stat(self) -> Tuple[int, int]:
        st = self.path.stat()
        return st.st_mtime_ns, st.st_size

    def poll(self) -> Tuple[Optional[DetectorParams], Optional[str]]:
        """(読み直した設定 or None, エラーの説明 or None)。変わっていなければ (None, None)。"""
        try:
            stamp = self._stat()
        except OSError as e:
            return None, f'設定ファイルが見えない（前の設定のまま）: {e}'
        if stamp == self._stamp:
            return None, None
        self._stamp = stamp
        try:
            params = load_params(self.path)
        except Exception as e:  # noqa: BLE001 何が壊れていても前の設定を保つ
            return None, f'設定ファイルを読めない（前の設定のまま）: {e}'
        self.params = params
        return params, None
