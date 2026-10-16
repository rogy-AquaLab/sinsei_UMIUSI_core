"""風船割りの行動決定。ROS に依存しない（単体テストと、動画での再生確認ができるように）。

1 回の step で「今の時刻」と「新しいフレームの検出結果（無ければ None）」を受け取り、指令を返す。
時間はすべて秒で扱う（フレーム数で数えると、認識の周期が変わったときに全部の時間がずれる）。

指令の向き（この中での約束。ROS の符号への変換はノード側で行う）:
  surge     正 = 前進
  heave     正 = 浮上
  yaw_rate  正 = 左旋回（上から見て反時計回り）[rad/s]
検出の座標は画像中心が (0, 0)、右・下が正、端が ±1。

状態:
  WAIT      Auto に入って（= スタートの合図）から動き出すまでの待ち。推力 0
  DIVE      真下へ潜る。スタート地点の水面からは、正面の黄色は画面の下に外れて見えない
  APPROACH  目標を画面中央（ピンの照準）に保ちながら近づく
  RAM       真っすぐ突っ込む
  BACKOFF   下がる
  SEARCH    その場で回って次の目標を探す
  END       競技時間切れ。推力 0（機体は正浮力で浮く）

「割れたかどうか」は判定しない。突っ込んだあとは下がって探し直し、まだ見えていれば
もう一度狙う。同じ色に続けて max_rams 回突っ込んだら、その色をしばらく無視する。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from sinsei_umiusi_core.balloon.detector import Detection

WAIT, DIVE, APPROACH, RAM, BACKOFF, SEARCH, END = (
    'WAIT',
    'DIVE',
    'APPROACH',
    'RAM',
    'BACKOFF',
    'SEARCH',
    'END',
)


@dataclass
class MissionParams:
    # --- 時間 [s]
    start_delay: float = (
        10.0  # Auto に入ってから動き出すまで（機体を水に入れて手を離す時間）
    )
    mission_time: float = 300.0  # 動き出してからの持ち時間。過ぎたら END
    dive_time: float = 12.0  # 黄色が見えなくても DIVE を打ち切る時間
    ram_time: float = 3.0
    backoff_time: float = 2.0
    lost_time: float = 1.5  # APPROACH で目標をこの時間見失ったら SEARCH
    ignore_time: float = 10.0  # 突っ込み過ぎた色を無視する時間
    first_only_time: float = (
        30.0  # 動き出してからこの時間は最初の色だけを狙う。過ぎたら priority の色も狙う
    )

    # --- 指令の大きさ（正規化値。control 側の上限は別にかかる）
    dive_heave: float = -0.4
    approach_surge_far: float = 0.35
    approach_surge_near: float = 0.15
    ram_surge: float = 0.6
    backoff_surge: float = -0.3
    search_yaw_rate: float = 0.35  # [rad/s]
    max_yaw_rate: float = 0.6  # [rad/s]

    # --- 照準（APPROACH）
    k_yaw: float = 0.8  # 横ずれ 1（画面端）あたりの yaw_rate [rad/s]
    # 目標が CNN の検出のときの k_yaw。CNN は遅れ（周期 + 推論で 0.4〜0.5 s）が大きく、0.8 だと左右に振れ続けた
    # （2026-10-15 の CNN の走行で反転 22 回/分・突進 0。sim で実機の画角・旋回に合わせると 0.4 で振れが消えた）
    k_yaw_cnn: float = 0.4
    k_heave: float = 0.6  # 縦ずれ 1 あたりの heave
    aim_x: float = 0.0  # ピンが当たる位置（画像座標）。カメラとピンの取り付けで決まる
    aim_y: float = 0.0
    near_radius: float = 0.08  # これより大きく見えたら減速（半径 / 画像幅）
    ram_radius: float = 0.16  # これより大きく、照準のずれが ram_tol 以内なら突っ込む
    # 目標が CNN の検出のときの ram_radius。0 なら ram_radius と同じ。
    # CNN の箱は実寸の約 0.86 倍（HSV は約 1.23 倍）に出て、10/15 は 0.16 に一度も届かなかった（未検証。0.12 前後が候補）
    ram_radius_cnn: float = 0.0
    ram_tol: float = 0.15

    # --- 目標の選び方
    first_color: str = 'yellow'  # 最初に狙う色（スタート正面の黄色）
    priority: Tuple[str, ...] = (
        'red',
        'yellow',
    )  # 2 個目以降。左ほど優先。blue は入れない
    max_rams: int = 3  # 同じ色に続けて突っ込む上限

    # --- 青を避ける（APPROACH 中、目標より大きく正面に見えたら前進を止めて向きを変える）
    avoid_color: str = 'blue'
    avoid_radius: float = 0.10
    avoid_tol: float = 0.35


@dataclass
class Command:
    surge: float = 0.0
    heave: float = 0.0
    yaw_rate: float = 0.0
    hold_yaw: bool = False
    state: str = WAIT
    target: Optional[str] = None  # 狙っている色


def _clip(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


@dataclass
class _Track:
    color: str
    x: float
    y: float
    radius: float
    seen_at: float
    cnn: bool = False  # CNN の検出か


def _track(d: Detection, now: float) -> _Track:
    return _Track(
        d.color, d.x, d.y, d.radius, now, getattr(d, 'source', 'hsv') == 'cnn'
    )


class BalloonMission:
    def __init__(self, params: MissionParams, use_hold_yaw: bool = False) -> None:
        self.p = params
        self.use_hold_yaw = use_hold_yaw
        self.reset(0.0)

    def reset(self, now: float) -> None:
        """Auto に入った時刻で初期化する。"""
        self.state = WAIT
        self.t_enter = now  # 今の状態に入った時刻
        self.t_go: Optional[float] = None  # 動き出した時刻（WAIT を抜けた時刻）
        self.track: Optional[_Track] = None
        self.first_done = False  # 最初の黄色に一度突っ込んだか
        self.rams: Dict[str, int] = {}  # 色ごとの、続けて突っ込んだ回数
        self.ignore_until: Dict[str, float] = {}
        self.last_ram_color: Optional[str] = None

    # ------------------------------------------------------------------ 公開
    def step(self, now: float, detections: Optional[Sequence[Detection]]) -> Command:
        """detections は新しいフレームが来たときだけ渡す（来ていなければ None）。"""
        p = self.p
        if (
            self.state != END
            and self.t_go is not None
            and now - self.t_go >= p.mission_time
        ):
            self._go(END, now)

        if detections is not None:
            self._update_track(now, detections)

        handler = {
            WAIT: self._wait,
            DIVE: self._dive,
            APPROACH: self._approach,
            RAM: self._ram,
            BACKOFF: self._backoff,
            SEARCH: self._search,
            END: self._end,
        }[self.state]
        cmd = handler(now, detections)
        cmd.state = self.state
        cmd.target = self.track.color if self.track else None
        cmd.yaw_rate = _clip(cmd.yaw_rate, p.max_yaw_rate)
        cmd.hold_yaw = self.use_hold_yaw and self.state not in (WAIT, END)
        return cmd

    # ------------------------------------------------------------------ 状態
    def _wait(self, now, _dets) -> Command:
        if now - self.t_enter >= self.p.start_delay:
            # 動き出した時刻は予定の時刻にする（立ち上がり直して途中から再開したとき、持ち時間を延ばさない）
            self.t_go = self.t_enter + self.p.start_delay
            if now - self.t_go >= self.p.mission_time:
                self._go(END, now)
                return self._end(now, None)
            self._go(DIVE, now)
            return self._dive(now, None)
        return Command()

    def _dive(self, now, _dets) -> Command:
        p = self.p
        if self.track is not None:  # 狙う色（_wanted）が見えた
            self._go(APPROACH, now)
            return self._approach(now, None)
        if now - self.t_enter >= p.dive_time:
            self._go(SEARCH, now)
            return self._search(now, None)
        return Command(heave=p.dive_heave)

    def _approach(self, now, dets) -> Command:
        p = self.p
        tr = self.track
        if tr is None or now - tr.seen_at > p.lost_time:
            self.track = None
            self._go(SEARCH, now)
            return self._search(now, None)

        ex, ey = tr.x - p.aim_x, tr.y - p.aim_y
        k_yaw = p.k_yaw_cnn if tr.cnn else p.k_yaw
        ram_radius = (
            p.ram_radius_cnn if tr.cnn and p.ram_radius_cnn > 0 else p.ram_radius
        )
        if tr.radius >= ram_radius and abs(ex) <= p.ram_tol and abs(ey) <= p.ram_tol:
            self._go(RAM, now)
            return self._ram(now, None)

        # 画面右（ex > 0）なら右へ回る = 負の yaw_rate。画面下（ey > 0）なら潜る = 負の heave
        yaw_rate = -k_yaw * ex
        heave = _clip(-p.k_heave * ey, 1.0)
        surge = (
            p.approach_surge_near
            if tr.radius >= p.near_radius
            else p.approach_surge_far
        )
        # 照準が大きくずれている間は前に出ない（回り込みながら突っ込むと外す）
        if abs(ex) > 0.5:
            surge = 0.0

        blocker = self._blocking_avoid(dets, tr)
        if blocker is not None:
            surge = 0.0
            yaw_rate = (
                p.search_yaw_rate if blocker.x >= 0 else -p.search_yaw_rate
            )  # 青と逆へ
        return Command(surge=surge, heave=heave, yaw_rate=yaw_rate)

    def _ram(self, now, _dets) -> Command:
        p = self.p
        if now - self.t_enter >= p.ram_time:
            color = self.track.color if self.track else None
            self._count_ram(color, now)
            self.first_done = True
            self.track = None
            self._go(BACKOFF, now)
            return self._backoff(now, None)
        return Command(surge=p.ram_surge)

    def _backoff(self, now, _dets) -> Command:
        if now - self.t_enter >= self.p.backoff_time:
            self._go(SEARCH, now)
            return self._search(now, None)
        return Command(surge=self.p.backoff_surge)

    def _search(self, now, _dets) -> Command:
        if self.track is not None:
            self._go(APPROACH, now)
            return self._approach(now, None)
        return Command(yaw_rate=self.p.search_yaw_rate)

    def _end(self, now, _dets) -> Command:
        return Command()

    # ------------------------------------------------------------------ 補助
    def _go(self, state: str, now: float) -> None:
        self.state = state
        self.t_enter = now

    def _wanted(self, now: float) -> List[str]:
        """今狙う色（優先順）。"""
        p = self.p
        first_only = not self.first_done and (
            self.t_go is None or now - self.t_go < p.first_only_time
        )
        colors = [p.first_color] if first_only else list(p.priority)
        return [
            c
            for c in colors
            if c != p.avoid_color and now >= self.ignore_until.get(c, 0.0)
        ]

    def _update_track(self, now: float, dets: Sequence[Detection]) -> None:
        """目標を 1 つ決める。追っている色が見えていれば、前回の位置に一番近いものを続けて追う。"""
        if self.state in (WAIT, RAM, BACKOFF, END):
            return  # 突っ込み中・下がり中は目標を変えない（見えなくなっても続ける）
        wanted = self._wanted(now)
        if self.track is not None and self.track.color in wanted:
            same = [d for d in dets if d.color == self.track.color]
            if same:
                d = min(
                    same,
                    key=lambda d: (d.x - self.track.x) ** 2 + (d.y - self.track.y) ** 2,
                )
                self.track = _track(d, now)
                return
            return  # 今回は見えていない。lost_time までは前の位置を使う
        for color in wanted:
            cands = [d for d in dets if d.color == color]
            if cands:
                d = max(cands, key=lambda d: d.radius)  # 一番近い（大きい）もの
                self.track = _track(d, now)
                return

    def _blocking_avoid(
        self, dets: Optional[Sequence[Detection]], tr: _Track
    ) -> Optional[Detection]:
        if not dets:
            return None
        p = self.p
        for d in dets:
            if (
                d.color == p.avoid_color
                and d.radius >= p.avoid_radius
                and d.radius > tr.radius
                and abs(d.x - p.aim_x) <= p.avoid_tol
            ):
                return d
        return None

    def _count_ram(self, color: Optional[str], now: float) -> None:
        if color is None:
            return
        if color == self.last_ram_color:
            self.rams[color] = self.rams.get(color, 0) + 1
        else:
            self.rams = {color: 1}
        self.last_ram_color = color
        if self.rams[color] >= self.p.max_rams:
            self.ignore_until[color] = now + self.p.ignore_time
            self.rams[color] = 0


def params_from_dict(d: Dict) -> MissionParams:
    """ROS パラメータなどの dict から作る。知らないキーはエラー（綴り間違いを黙って無視しない）。"""
    fields = MissionParams.__dataclass_fields__
    unknown = set(d) - set(fields)
    if unknown:
        raise ValueError(f'unknown mission params: {sorted(unknown)}')
    d = dict(d)
    if 'priority' in d:
        d['priority'] = tuple(d['priority'])
    return MissionParams(**d)


__all__ = ['BalloonMission', 'Command', 'MissionParams', 'params_from_dict']
