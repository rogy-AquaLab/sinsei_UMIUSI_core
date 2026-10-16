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
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_default as QOS_PROFILE_DEFAULT
from rclpy.timer import Timer
from std_msgs.msg import String

from sinsei_umiusi_core.balloon import detect
from sinsei_umiusi_core.balloon.async_detect import (
    DetectionWorker,
    confirm_assist,
    fresh_assist,
)
from sinsei_umiusi_core.balloon.cnn import CnnDetector, detect_cnn, merge_overlaps
from sinsei_umiusi_core.balloon.config_watch import ConfigWatcher
from sinsei_umiusi_core.balloon.reflection import split_reflection
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
        # 'hsv' = 色のしきい値（detector_config の色）、'cnn' = 学習した CNN（cnn_model）、
        # 'hsv+cnn' = 色のしきい値が主。色で何も見つからないときだけ、別スレッドで回している CNN の新しい結果を使う。
        # 走行中に切り替えてよい。CNN を読めなければ hsv で続ける
        self.declare_parameter('detector', 'hsv')
        # ファイル名だけなら share/sinsei_umiusi_core/config/balloon/models/ から探す。絶対パスならそのまま
        self.declare_parameter('cnn_model', 'balloon_cnn_20261007.onnx')
        # 確信度の下限。濁ったプールでは 0.4 にすると全部消える（10/03）ので上げすぎない
        self.declare_parameter('cnn_conf', 0.3)
        # 推論のスレッド数（onnxruntime / OpenCV）。Pi では 2 が一番速い
        self.declare_parameter('cnn_threads', 2)
        # hsv+cnn で CNN の結果を使ってよい古さ [s]（CNN は遅いので、これより古い位置では狙わない）
        self.declare_parameter('cnn_assist_max_age', 0.5)
        # hsv+cnn で使う CNN の検出の確信度の下限。CNN 単独（cnn_conf）より高くする:
        # 色で何も見えていないときに、反射やタイルに自信を持った CNN を追わないため（2026-10-15 のプール）
        self.declare_parameter('cnn_assist_conf', 0.5)
        # true なら、CNN の直近 2 回の結果で同じ色がほぼ同じ位置（0.15 以内）にあるものだけ使う（一瞬だけの誤検出を落とす）
        self.declare_parameter('cnn_assist_confirm', True)
        # 水面の鏡像とつながった縦長の検出（高さ / 幅 > reflection_aspect）を、下側の本物の風船に直す
        self.declare_parameter('reflection_split', True)
        # CNN が 1 個の風船に重ねて出す小さい箱を、大きい方の 1 個にまとめる（目標の大きさが半分に化けるのを防ぐ）
        self.declare_parameter('cnn_merge', True)
        self.declare_parameter('reflection_aspect', 1.3)
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
        # Auto に入った時刻（time.monotonic の秒）。robot_strategy が activate の前に入れる。
        # ノードが落ちて立ち上がり直したとき、持ち時間をこの時刻から数え続けるため。0 なら activate の時刻
        self.declare_parameter('run_started_at', 0.0)
        # Auto でない間も認識だけ回して ~/status（state PREVIEW）に出す。推力は出さない（UI で認識を確かめる用）
        self.declare_parameter('preview', False)
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
            self._models_dir = share / 'config' / 'balloon' / 'models'
            self._cnn = None  # ((パス, スレッド数), CnnDetector)。読めなかったら ((パス, スレッド数), None)
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
        self._worker = None
        self._assist = None
        self._cnn_lock = (
            threading.Lock()
        )  # CNN は検出スレッドと補助スレッドの両方から使う
        self._assist_used = False
        self._assist_prev = []  # 補助の CNN の前回の検出（続けて見えたかの判定用）
        self._det_ms = None
        self._active = False
        self._last_seq = 0
        self._preview_timer: Timer = self.create_timer(
            1.0 / self._param('rate_hz'), self._preview_tick
        )
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
        self._active = True
        self._ensure_camera()
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
        now = time.monotonic()
        t0 = self._param('run_started_at')
        p = self._mission.p
        if 0.0 < t0 <= now and now - t0 < p.start_delay + p.mission_time:
            if now - t0 > 1.0:
                self.get_logger().warning(
                    f'Auto に入ってから {now - t0:.1f} 秒経っている。その時刻から持ち時間を数え続ける'
                )
        else:
            t0 = now
        self._mission.reset(t0)
        if self._param('detector') in ('cnn', 'hsv+cnn'):
            with self._cnn_lock:
                self._cnn_model()  # 最初のフレームで待たないよう先に読む
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
        self._active = False
        self._publish(Command())
        self._stop_camera()
        # 次の Auto で前の時刻を使わない（robot_strategy が入れ損ねても、その時刻から数える）
        self.set_parameters([Parameter('run_started_at', value=0.0)])
        self._close_save_dir()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._stop_camera()
        for timer in (self._timer, self._reload_timer, self._preview_timer):
            if timer is not None and not self.destroy_timer(timer):
                self.get_logger().warning('Failed to destroy timer')
        for pub in (self._target_pub, self._attitude_target_pub, self._status_pub):
            if not self.destroy_publisher(pub):
                self.get_logger().warning('Failed to destroy publisher')
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------ 本体
    def _take_detection(self, now: float):
        """検出スレッドの新しい結果 (frame, dets)。無い・古いなら (None, None)。"""
        res = self._worker.take() if self._worker is not None else None
        if res is None:
            return None, None
        seq, t_frame, frame, dets, took = res
        self._det_ms = round(took * 1e3)
        if now - t_frame > self._param('max_frame_age'):
            return None, None
        self._last_seq = seq
        return frame, dets

    def _tick(self) -> None:
        now = time.monotonic()
        try:
            frame, dets = self._take_detection(now)
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

    def _ensure_camera(self) -> None:
        if self._camera is None:
            self._camera = CameraReader(self._param('camera_source'), self.get_logger())
            self._camera.start()
            camera = self._camera
            self._worker = DetectionWorker(
                'detect', camera.latest, self._detect, self.get_logger()
            )
            self._assist = DetectionWorker(
                'cnn_assist', camera.latest, self._assist_detect, self.get_logger()
            )
            self._worker.start()
            self._assist.start()

    def _stop_camera(self) -> None:
        for w in (self._worker, self._assist):
            if w is not None:
                w.stop()
        self._worker = self._assist = None
        if self._camera is not None:
            self._camera.stop()
            self._camera = None

    def _preview_tick(self) -> None:
        """Auto でない間の認識だけの表示。指令は出さない。"""
        if self._active:
            return
        if not self._param('preview'):
            self._stop_camera()
            return
        self._ensure_camera()
        _frame, dets = self._take_detection(time.monotonic())
        if dets is None:
            return
        self._publish_status(Command(state='PREVIEW'), dets)

    def _detect(self, frame):
        """検出スレッドで回る。"""
        params = self._watcher.params
        mode = self._param('detector')
        dets = self._run_cnn(frame) if mode == 'cnn' else None
        used_assist = False
        if dets is None:
            dets, _ = detect(frame, params)
            dets = self._split(dets, frame)
            if mode == 'hsv+cnn' and not dets:
                assist = fresh_assist(
                    self._assist.latest() if self._assist is not None else None,
                    time.monotonic(),
                    self._param('cnn_assist_max_age'),
                )
                if assist:
                    dets, used_assist = assist, True
        self._assist_used = used_assist
        return dets

    def _assist_detect(self, frame):
        """補助スレッドで回る。hsv+cnn のときだけ CNN を掛ける（それ以外は休む = None）。

        確信度 cnn_assist_conf 以上で、cnn_assist_confirm なら前回も近くに同じ色があったものだけを返す。
        """
        if self._param('detector') != 'hsv+cnn':
            self._assist_prev = []
            return None
        dets = self._run_cnn(frame)
        if dets is None:
            return None
        dets = [d for d in dets if d.score >= self._param('cnn_assist_conf')]
        prev, self._assist_prev = self._assist_prev, dets
        if not self._param('cnn_assist_confirm'):
            return dets
        return confirm_assist(dets, prev)

    def _run_cnn(self, frame):
        """CNN の検出（鏡像の補正つき）。CNN を読めなければ None。"""
        with self._cnn_lock:
            model = self._cnn_model()
            if model is None:
                return None
            model.conf = self._param('cnn_conf')
            dets = detect_cnn(frame, model, self._watcher.params)
        if self._param('cnn_merge'):
            dets = merge_overlaps(dets)
        return self._split(dets, frame)

    def _split(self, dets, frame):
        if not self._param('reflection_split'):
            return dets
        return split_reflection(
            dets, self._watcher.params, frame.shape, self._param('reflection_aspect')
        )

    def _using_cnn(self) -> bool:
        return (
            self._param('detector') in ('cnn', 'hsv+cnn')
            and self._cnn is not None
            and self._cnn[1] is not None
        )

    def _cnn_model(self):
        """cnn_model を読む（パスが変わったときだけ読み直す）。読めなければ None（色の検出で続ける）。"""
        path = Path(self._param('cnn_model')).expanduser()
        if not path.is_absolute():
            path = self._models_dir / path
        threads = self._param('cnn_threads')
        if self._cnn is None or self._cnn[0] != (path, threads):
            try:
                model = CnnDetector(path, self._param('cnn_conf'), threads=threads)
                self.get_logger().info(
                    f'CNN を読んだ: {path}（{model.backend}、{threads} スレッド、入力 {model.input_size}）'
                )
            except Exception as e:  # noqa: BLE001 読めなくても止まらない
                model = None
                self.get_logger().error(
                    f'CNN を読めない（色の検出で続ける）: {path}: {e}'
                )
            self._cnn = ((path, threads), model)
        return self._cnn[1]

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
            # 追っている目標が CNN の検出か（k_yaw_cnn・ram_radius_cnn が効いている）
            'target_cnn': (
                self._mission.track.cnn if self._mission.track is not None else None
            ),
            'cmd': {
                'surge': cmd.surge,
                'heave': cmd.heave,
                'yaw_rate': cmd.yaw_rate,
                'hold_yaw': cmd.hold_yaw,
            },
            'new_frame': dets is not None,
            # 今使っている検出のしかたとモデル名（CNN を読めず色の検出で続けているなら hsv）
            'detector': (self._param('detector') if self._using_cnn() else 'hsv'),
            # hsv+cnn で、このコマは色で見つからず CNN の結果を使った
            'assist_used': self._assist_used,
            # 直近の検出 1 回にかかった時間 [ms]（検出スレッド）
            'det_ms': self._det_ms,
            'cnn_model': self._cnn[0][0].name if self._using_cnn() else None,
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
                    's': round(d.score, 3),
                    'src': d.source,
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
                    {'color': d.color, 'x': d.x, 'y': d.y, 'r': d.radius, 's': d.score}
                    for d in dets
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
