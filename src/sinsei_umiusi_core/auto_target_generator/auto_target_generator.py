#!/usr/bin/env python3

import json
import math
import sys
import threading
import time
from dataclasses import fields
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.node import Publisher
from rclpy.qos import qos_profile_default as QOS_PROFILE_DEFAULT
from rclpy.timer import Timer
from std_msgs.msg import String

from sinsei_umiusi_core.balloon import detect
from sinsei_umiusi_core.balloon.config_watch import ConfigWatcher
from sinsei_umiusi_core.balloon.mission import (
    BalloonMission,
    Command,
    MissionParams,
    params_from_dict,
)
from sinsei_umiusi_msgs.msg import AttitudeTarget, Target

# control の cameras_shm.yaml が書き出す共有メモリ（BGR 320x240）を読む
DEFAULT_CAMERA = (
    'shmsrc socket-path=/tmp/umiusi_cam1.sock is-live=true ! '
    'video/x-raw,format=BGR,width=320,height=240 ! '
    'appsink drop=true max-buffers=1 sync=false'
)


class CameraReader:
    """別スレッドで読み続け、最新の 1 枚だけを持つ。止まったら開き直す。

    source に '!' が含まれていれば GStreamer パイプライン、そうでなければ動画ファイル
    （再生確認用。元の fps で流す）。
    """

    def __init__(self, source: str, logger) -> None:
        self._source = source
        self._logger = logger
        self._lock = threading.Lock()
        self._latest = (0, 0.0, None)  # (通し番号, 受け取った時刻, BGR)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def latest(self):
        with self._lock:
            return self._latest

    def _open(self):
        if '!' in self._source:
            return cv2.VideoCapture(self._source, cv2.CAP_GSTREAMER), 0.0
        cap = cv2.VideoCapture(self._source)
        fps = cap.get(cv2.CAP_PROP_FPS) or 15.0
        return cap, 1.0 / fps

    def _run(self) -> None:
        seq = 0
        while not self._stop.is_set():
            cap, period = self._open()
            if not cap.isOpened():
                self._logger.warning(
                    f'camera を開けない。1 秒後に再試行: {self._source}',
                    throttle_duration_sec=5.0,
                )
                self._stop.wait(1.0)
                continue
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    self._logger.warning(
                        'camera から読めない。開き直す', throttle_duration_sec=5.0
                    )
                    break
                seq += 1
                with self._lock:
                    self._latest = (seq, time.monotonic(), frame)
                if period:
                    self._stop.wait(period)
            cap.release()
            self._stop.wait(0.5)


class AutoTargetGenerator(LifecycleNode):
    """
    # Auto Target Generator Node

    前カメラ -> 風船検出 (sinsei_umiusi_core.balloon.detector)
             -> 行動決定 (sinsei_umiusi_core.balloon.mission)
             -> /cmd/target, /cmd/attitude_target

    Auto モードのときだけ有効になる（robot_strategy の Auto が activate / deactivate する）。
    状態と検出結果は ~/status に JSON で出す（bag に残す用）。
    """

    def __init__(self) -> None:
        super().__init__('auto_target_generator')
        self.declare_parameter('camera_source', DEFAULT_CAMERA)
        # ファイル名だけなら share/sinsei_umiusi_core/config/balloon/ から探す。絶対パスならそのまま
        # 色合わせの道具（scripts/calibrate_server.py）が保存する current.json を既定にする。
        # 無ければ同梱の normal.json で動き、できたら切り替える
        self.declare_parameter('detector_config', '~/umiusi_balloon/current.json')
        # 色の設定ファイルの更新を見る周期 [s]。変わっていたら読み直す（色合わせの結果をそのまま反映する）。0 で見ない
        self.declare_parameter('detector_reload_period', 1.0)
        self.declare_parameter('rate_hz', 10.0)
        # これより古いフレームは使わない [s]
        self.declare_parameter('max_frame_age', 0.5)
        self.declare_parameter('use_hold_yaw', False)
        # 行動決定の向き（surge 正 = 前、heave 正 = 上、yaw_rate 正 = 左旋回）を control の向きへ合わせる係数。
        # 陸上で指令を出して確かめ、逆なら -1.0 にする
        self.declare_parameter('sign_surge', 1.0)
        self.declare_parameter('sign_heave', 1.0)
        self.declare_parameter('sign_yaw', 1.0)
        # 空でなければ、activate ごとにフォルダを作り、生フレームと検出結果を残す（色合わせ用）
        self.declare_parameter('save_dir', '')
        self.declare_parameter('save_every', 0.5)  # [s]
        for f in fields(MissionParams):
            default = getattr(MissionParams(), f.name)
            self.declare_parameter(
                f'mission.{f.name}',
                list(default) if isinstance(default, tuple) else default,
            )

    # ------------------------------------------------------------------ lifecycle
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._target_pub: Publisher = self.create_publisher(
            Target, '/cmd/target', QOS_PROFILE_DEFAULT
        )
        self._attitude_target_pub: Publisher = self.create_publisher(
            AttitudeTarget, '/cmd/attitude_target', QOS_PROFILE_DEFAULT
        )
        self._status_pub: Publisher = self.create_publisher(
            String, '~/status', QOS_PROFILE_DEFAULT
        )
        try:
            share = Path(get_package_share_directory('sinsei_umiusi_core'))
            detector_config = Path(self._param('detector_config')).expanduser()
            if not detector_config.is_absolute():
                detector_config = share / 'config' / 'balloon' / detector_config
            # 読めなくても起動は失敗させない（未構成で残ると AUTO に入った瞬間にプロセスごと落ちる）。
            # 同梱の normal.json で動き、指定のファイルができたら切り替える
            self._watcher = ConfigWatcher(
                detector_config, fallback=share / 'config' / 'balloon' / 'normal.json'
            )
            if self._watcher.startup_error:
                self.get_logger().error(self._watcher.startup_error)
            mission_params = self._mission_params()
        except Exception as e:  # noqa: BLE001 設定ミスは configure の失敗として見せる
            self.get_logger().error(f'設定を読めない: {e}')
            return TransitionCallbackReturn.FAILURE
        self._mission = BalloonMission(
            mission_params, use_hold_yaw=self._param('use_hold_yaw')
        )
        self._timer: Timer = self.create_timer(
            1.0 / self._param('rate_hz'), self._tick, autostart=False
        )
        self._camera = None
        self._save_log = None
        self._reload_timer: Timer | None = None
        if self._param('detector_reload_period') > 0:
            # inactive の間も回す（AUTO に入る前に色合わせを終えておけるように）
            self._reload_timer = self.create_timer(
                self._param('detector_reload_period'), self._reload_detector
            )
        self.get_logger().info(f'detector_config: {detector_config}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._publish(Command())
        self._camera = CameraReader(self._param('camera_source'), self.get_logger())
        self._camera.start()
        self._last_seq = 0
        # AUTO に入るたびに読み直す（ros2 param set で持ち時間などを変えたら、次の AUTO から効く）
        try:
            self._mission = BalloonMission(
                self._mission_params(), use_hold_yaw=self._param('use_hold_yaw')
            )
        except Exception as e:  # noqa: BLE001 読めなければ前の値で走る
            self.get_logger().error(
                f'mission のパラメータを読めない。前の値で走る: {e}'
            )
        self._mission.reset(time.monotonic())
        self._open_save_dir()
        self._timer.reset()
        p = self._mission.p
        self.get_logger().info(
            f'activate: 風船割りを開始（WAIT から）。{p.start_delay:g} 秒後に動き出し、'
            f'{p.mission_time:g} 秒で推力 0'
        )
        return TransitionCallbackReturn.SUCCESS

    def _mission_params(self) -> MissionParams:
        return params_from_dict(
            {f.name: self._param(f'mission.{f.name}') for f in fields(MissionParams)}
        )

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._timer.cancel()
        self._publish(Command())
        if self._camera is not None:
            self._camera.stop()
            self._camera = None
        self._close_save_dir()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        for timer in (self._timer, self._reload_timer):
            if timer is not None and not self.destroy_timer(timer):
                self.get_logger().warning('Failed to destroy timer')
        for pub in (self._target_pub, self._attitude_target_pub, self._status_pub):
            if not self.destroy_publisher(pub):
                self.get_logger().warning('Failed to destroy publisher')
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------ 本体
    def _tick(self) -> None:
        now = time.monotonic()
        try:
            seq, t_frame, frame = self._camera.latest()
            dets = None
            fresh = now - t_frame <= self._param('max_frame_age')
            if seq != self._last_seq and frame is not None and fresh:
                self._last_seq = seq
                dets, _ = detect(frame, self._watcher.params)
            cmd = self._mission.step(now, dets)
        except Exception as e:  # noqa: BLE001 何が起きても推力 0 を出して止まる
            self.get_logger().error(
                f'tick で例外。推力 0 を出す: {e!r}', throttle_duration_sec=1.0
            )
            self._publish(Command(state='ERROR'))
            return
        self._publish(cmd)
        self._publish_status(cmd, dets)
        if dets is not None:
            self._save(now, frame, dets, cmd)

    def _reload_detector(self) -> None:
        params, err = self._watcher.poll()
        if err:
            self.get_logger().error(err, throttle_duration_sec=10.0)
        elif params is not None:
            ranges = {c: [(r.lo, r.hi) for r in rs] for c, rs in params.colors.items()}
            self.get_logger().info(
                f'色の設定を読み直した: {self._watcher.path} {ranges}'
            )

    def _publish(self, cmd: Command) -> None:
        target = Target()
        target.velocity.x = float(self._param('sign_surge') * cmd.surge)
        target.velocity.z = float(self._param('sign_heave') * cmd.heave)
        self._target_pub.publish(target)
        att = AttitudeTarget()
        att.attitude.w = 1.0  # roll / pitch は水平
        att.yaw_rate = float(self._param('sign_yaw') * cmd.yaw_rate)
        att.hold_yaw = bool(cmd.hold_yaw)
        self._attitude_target_pub.publish(att)

    def _publish_status(self, cmd: Command, dets) -> None:
        status = {
            'state': cmd.state,
            'target': cmd.target,
            'cmd': {
                'surge': cmd.surge,
                'heave': cmd.heave,
                'yaw_rate': cmd.yaw_rate,
                'hold_yaw': cmd.hold_yaw,
            },
            'new_frame': dets is not None,
            # 今使っている色の設定（current.json を読めず同梱の設定で動いているなら fallback）
            'config': {
                'file': self._watcher.loaded_from.name,
                'loaded_at': round(self._watcher.loaded_at, 1),
                'fallback': self._watcher.startup_error is not None,
            },
            'detections': [
                {
                    'color': d.color,
                    'x': round(d.x, 3),
                    'y': round(d.y, 3),
                    'r': round(d.radius, 4),
                }
                for d in dets or []
            ],
        }
        self._status_pub.publish(String(data=json.dumps(status)))

    # ------------------------------------------------------------------ 記録（色合わせ用）
    def _open_save_dir(self) -> None:
        self._save_path = None
        self._save_log = None
        self._last_save = -math.inf
        root = self._param('save_dir')
        if not root:
            return
        try:
            self._save_path = Path(root) / datetime.now().strftime('%Y%m%d-%H%M%S')
            self._save_path.mkdir(parents=True, exist_ok=True)
            self._save_log = open(self._save_path / 'log.jsonl', 'a')  # noqa: SIM115 deactivate で閉じる
        except OSError as e:
            self.get_logger().error(f'記録先を作れない（記録しないで続ける）: {e}')
            self._save_path = None

    def _close_save_dir(self) -> None:
        if self._save_log is not None:
            self._save_log.close()
            self._save_log = None

    def _save(self, now: float, frame, dets, cmd: Command) -> None:
        if self._save_path is None or now - self._last_save < self._param('save_every'):
            return
        self._last_save = now
        name = f'{self._last_seq:07d}.jpg'
        try:
            cv2.imwrite(str(self._save_path / name), frame)
            line = {
                't': now,
                'file': name,
                'state': cmd.state,
                'target': cmd.target,
                'detections': [
                    {'color': d.color, 'x': d.x, 'y': d.y, 'r': d.radius} for d in dets
                ],
            }
            self._save_log.write(json.dumps(line) + '\n')
        except OSError as e:
            self.get_logger().error(f'記録に失敗: {e}', throttle_duration_sec=5.0)

    def _param(self, name: str):
        return self.get_parameter(name).value


def main(args: list[str] = sys.argv) -> None:
    rclpy.init(args=args)
    node = AutoTargetGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
