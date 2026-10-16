"""行動決定（sinsei_umiusi_core.balloon.mission）の単体テスト。ROS 不要。

python3 -m pytest test/python
"""

import pytest

from sinsei_umiusi_core.balloon.detector import Detection
from sinsei_umiusi_core.balloon.mission import (
    APPROACH,
    BACKOFF,
    DIVE,
    END,
    RAM,
    SEARCH,
    WAIT,
    BalloonMission,
    MissionParams,
    params_from_dict,
)


def det(color, x=0.0, y=0.0, radius=0.05, source='hsv'):
    return Detection(
        color=color,
        x=x,
        y=y,
        radius=radius,
        area=0.0,
        fill=1.0,
        solidity=1.0,
        bbox=(0, 0, 0, 0),
        source=source,
    )


def started(params=None):
    """WAIT を抜けて DIVE に入った状態を返す（時刻 t=0 で DIVE に入る）。"""
    p = params or MissionParams()
    m = BalloonMission(p)
    m.reset(-p.start_delay)
    cmd = m.step(0.0, [])
    assert cmd.state == DIVE
    return m


def test_waits_without_thrust_then_dives():
    p = MissionParams(start_delay=10.0)
    m = BalloonMission(p)
    m.reset(0.0)
    cmd = m.step(5.0, [det('yellow')])
    assert cmd.state == WAIT
    assert (cmd.surge, cmd.heave, cmd.yaw_rate) == (0.0, 0.0, 0.0)
    cmd = m.step(10.0, [])
    assert cmd.state == DIVE
    assert cmd.heave < 0  # 潜る


def test_dive_ignores_red_and_switches_to_approach_on_yellow():
    m = started()
    assert m.step(0.5, [det('red')]).state == DIVE  # 最初は黄色だけを狙う
    cmd = m.step(1.0, [det('yellow', y=0.6)])
    assert cmd.state == APPROACH
    assert cmd.target == 'yellow'


def test_approach_steers_toward_the_balloon():
    m = started()
    right_below = m.step(1.0, [det('yellow', x=0.4, y=0.3)])
    assert right_below.yaw_rate < 0  # 右に見える -> 右へ回る（負）
    assert right_below.heave < 0  # 下に見える -> 潜る
    left_above = m.step(1.1, [det('yellow', x=-0.4, y=-0.3)])
    assert left_above.yaw_rate > 0
    assert left_above.heave > 0


def test_does_not_move_forward_while_badly_off_aim():
    m = started()
    cmd = m.step(1.0, [det('yellow', x=0.8)])
    assert cmd.surge == 0.0
    cmd = m.step(1.1, [det('yellow', x=0.1)])
    assert cmd.surge > 0.0


def test_rams_when_close_and_centred_then_backs_off_and_searches():
    p = MissionParams(ram_time=3.0, backoff_time=2.0)
    m = started(p)
    m.step(1.0, [det('yellow', radius=0.05)])
    cmd = m.step(1.1, [det('yellow', radius=0.2)])
    assert cmd.state == RAM
    assert cmd.surge == p.ram_surge and cmd.yaw_rate == 0.0
    assert m.step(2.0, []).state == RAM  # 見えなくなっても突っ込み続ける
    cmd = m.step(4.2, [])
    assert cmd.state == BACKOFF and cmd.surge < 0
    cmd = m.step(6.3, [])
    assert cmd.state == SEARCH and cmd.yaw_rate != 0.0


def test_after_first_yellow_prefers_red_over_yellow():
    m = started()
    m.step(1.0, [det('yellow', radius=0.2)])  # RAM
    m.step(4.1, [])  # BACKOFF
    m.step(6.2, [])  # SEARCH
    cmd = m.step(6.3, [det('yellow', radius=0.1), det('red', radius=0.03)])
    assert cmd.state == APPROACH and cmd.target == 'red'


def test_never_targets_blue():
    m = started(MissionParams(first_only_time=0.0))
    m.first_done = True
    for t in (1.0, 1.1, 1.2):
        cmd = m.step(t, [det('blue', radius=0.2)])
    assert cmd.target is None
    assert cmd.state in (DIVE, SEARCH)


def test_blue_in_front_stops_approach_and_turns_away():
    m = started()
    m.step(1.0, [det('yellow', x=0.1, radius=0.04)])
    cmd = m.step(
        1.1, [det('yellow', x=0.1, radius=0.04), det('blue', x=0.2, radius=0.15)]
    )
    assert cmd.state == APPROACH
    assert cmd.surge == 0.0
    assert cmd.yaw_rate > 0  # 青が右 -> 左へ


def test_loses_target_after_lost_time():
    p = MissionParams(lost_time=1.5)
    m = started(p)
    m.step(1.0, [det('yellow', x=0.1)])
    assert m.step(2.0, []).state == APPROACH  # 少し見えなくても続ける
    assert m.step(2.6, []).state == SEARCH


def test_mission_time_ends_with_zero_thrust():
    p = MissionParams(mission_time=60.0)
    m = started(p)
    m.step(10.0, [det('yellow')])
    cmd = m.step(60.0, [det('yellow')])
    assert cmd.state == END
    assert (cmd.surge, cmd.heave, cmd.yaw_rate, cmd.hold_yaw) == (0.0, 0.0, 0.0, False)


def test_gives_up_on_a_color_after_max_rams():
    p = MissionParams(max_rams=2, ignore_time=10.0, first_only_time=0.0)
    m = started(p)
    t = 1.0
    for _ in range(2):
        m.step(t, [det('red', radius=0.2)])  # APPROACH -> RAM
        t += p.ram_time + 0.1
        m.step(t, [])  # BACKOFF
        t += p.backoff_time + 0.1
        m.step(t, [])  # SEARCH
        t += 0.1
    cmd = m.step(t, [det('red', radius=0.2)])
    assert cmd.state == SEARCH and cmd.target is None  # 赤はしばらく無視
    cmd = m.step(t + p.ignore_time, [det('red', radius=0.05)])
    assert cmd.target == 'red'


def test_falls_back_to_other_colors_if_first_never_found():
    p = MissionParams(first_only_time=30.0)
    m = started(p)
    assert m.step(10.0, [det('red')]).target is None
    assert m.step(31.0, [det('red')]).target == 'red'


def test_yaw_rate_is_clipped():
    p = MissionParams(k_yaw=5.0, max_yaw_rate=0.6)
    m = started(p)
    cmd = m.step(1.0, [det('yellow', x=0.9)])
    assert cmd.yaw_rate == pytest.approx(-0.6)


def test_hold_yaw_only_while_moving():
    p = MissionParams()
    m = BalloonMission(p, use_hold_yaw=True)
    m.reset(0.0)
    assert m.step(1.0, []).hold_yaw is False  # WAIT
    assert m.step(p.start_delay, []).hold_yaw is True  # DIVE


def test_params_from_dict_rejects_typos():
    assert params_from_dict({'ram_time': 2.0, 'priority': ['red']}).priority == ('red',)
    with pytest.raises(ValueError, match='ram_tme'):
        params_from_dict({'ram_tme': 2.0})


def test_mission_time_counts_from_start_not_from_auto():
    p = MissionParams(start_delay=10.0, mission_time=60.0)
    m = BalloonMission(p)
    m.reset(0.0)
    m.step(10.0, [])  # WAIT -> DIVE
    assert m.step(69.9, []).state != END
    assert m.step(70.0, []).state == END


def test_resumed_mission_keeps_the_original_time_limit():
    # ノードが立ち上がり直し、Auto に入った時刻 0 から 100 秒後に再開した
    p = MissionParams(start_delay=10.0, mission_time=60.0)
    m = BalloonMission(p)
    m.reset(0.0)
    assert m.step(100.0, []).state == END  # 10 + 60 を過ぎている
    m.reset(0.0)
    assert m.step(30.0, []).state == DIVE  # 待ちは済んでいるのですぐ動く
    assert m.step(69.9, []).state != END
    assert m.step(70.0, []).state == END


def test_cnn_target_uses_its_own_k_yaw():
    p = MissionParams(k_yaw=0.8, k_yaw_cnn=0.4, max_yaw_rate=10.0)
    m = started(p)
    hsv = m.step(1.0, [det('yellow', x=0.4)])
    assert hsv.yaw_rate == pytest.approx(-0.8 * 0.4)
    cnn = m.step(1.1, [det('yellow', x=0.4, source='cnn')])
    assert cnn.yaw_rate == pytest.approx(-0.4 * 0.4)  # CNN の目標は弱く回す


def test_cnn_ram_radius_is_ram_radius_unless_set():
    m = started(MissionParams(ram_radius=0.16))
    m.step(1.0, [det('yellow', source='cnn')])
    assert m.step(1.1, [det('yellow', radius=0.13, source='cnn')]).state == APPROACH
    m = started(MissionParams(ram_radius=0.16, ram_radius_cnn=0.12))
    m.step(1.0, [det('yellow', source='cnn')])
    assert m.step(1.1, [det('yellow', radius=0.13, source='cnn')]).state == RAM
    m = started(MissionParams(ram_radius=0.16, ram_radius_cnn=0.12))
    m.step(1.0, [det('yellow')])
    assert (
        m.step(1.1, [det('yellow', radius=0.13)]).state == APPROACH
    )  # HSV は 0.16 のまま
