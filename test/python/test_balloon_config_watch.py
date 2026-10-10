"""色の設定ファイルの読み直し（sinsei_umiusi_core.balloon.config_watch）。"""

import json
import os

import pytest

from sinsei_umiusi_core.balloon.config_watch import ConfigWatcher


def write(path, yellow_lo_h):
    data = {
        'colors': {'yellow': [{'lo': [yellow_lo_h, 100, 100], 'hi': [35, 255, 255]}]}
    }
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data))
    os.replace(tmp, path)  # 色合わせの道具と同じ置き換え方


def test_no_change_gives_nothing(tmp_path):
    p = tmp_path / 'current.json'
    write(p, 15)
    w = ConfigWatcher(p)
    assert w.poll() == (None, None)


def test_change_is_reloaded(tmp_path):
    p = tmp_path / 'current.json'
    write(p, 15)
    w = ConfigWatcher(p)
    write(p, 20)
    os.utime(p, ns=(1, 1))  # 同じ秒内の書き換えでも変化が分かるように時刻をずらす
    params, err = w.poll()
    assert err is None
    assert params.colors['yellow'][0].lo[0] == 20
    assert w.params is params


def test_broken_file_keeps_previous(tmp_path):
    p = tmp_path / 'current.json'
    write(p, 15)
    w = ConfigWatcher(p)
    p.write_text('{ broken')
    params, err = w.poll()
    assert params is None and '前の設定のまま' in err
    assert w.params.colors['yellow'][0].lo[0] == 15


def test_missing_file_keeps_previous(tmp_path):
    p = tmp_path / 'current.json'
    write(p, 15)
    w = ConfigWatcher(p)
    p.unlink()
    params, err = w.poll()
    assert params is None and err


def test_first_load_failure_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        ConfigWatcher(tmp_path / 'nope.json')


def test_missing_at_startup_uses_fallback_then_switches(tmp_path):
    fallback = tmp_path / 'normal.json'
    write(fallback, 12)
    current = tmp_path / 'current.json'
    w = ConfigWatcher(current, fallback=fallback)
    assert w.params.colors['yellow'][0].lo[0] == 12
    assert w.startup_error and 'normal.json' in w.startup_error
    params, err = w.poll()
    assert params is None and err == w.startup_error  # まだ無い
    write(current, 25)  # 色合わせの道具で保存した
    params, err = w.poll()
    assert err is None and params.colors['yellow'][0].lo[0] == 25
    assert w.startup_error is None


def test_broken_at_startup_uses_fallback(tmp_path):
    fallback = tmp_path / 'normal.json'
    write(fallback, 12)
    current = tmp_path / 'current.json'
    current.write_text('{ broken')
    w = ConfigWatcher(current, fallback=fallback)
    assert w.params.colors['yellow'][0].lo[0] == 12
