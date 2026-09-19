#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Publisher, Subscription
from rclpy.timer import Timer
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import qos_profile_default as QOS_PROFILE_DEFAULT

from sinsei_umiusi_msgs.msg import AttitudeTarget, Target


class ManualTargetGenerator(LifecycleNode):
    """
    # Manual Target Generator Node

    Passes target velocity and orientation received from UI
    """

    def __init__(self) -> None:
        super().__init__('manual_target_generator')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._timer: Timer = self.create_timer(
            0.1,
            self._timer_callback,
            autostart=False,
        )
        self._target_pub: Publisher = self.create_publisher(
            Target,
            '/cmd/target',
            QOS_PROFILE_DEFAULT,
        )
        self._attitude_target_pub: Publisher = self.create_publisher(
            AttitudeTarget,
            '/cmd/attitude_target',
            QOS_PROFILE_DEFAULT,
        )
        self._target_sub: Subscription = None
        self._attitude_target_sub: Subscription = None

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._clear_targets()
        self._target_sub: Subscription = self.create_subscription(
            Target,
            '/user_input/target',
            self._target_callback,
            QOS_PROFILE_DEFAULT,
        )
        self._attitude_target_sub: Subscription = self.create_subscription(
            AttitudeTarget,
            '/user_input/attitude_target',
            self._attitude_target_callback,
            QOS_PROFILE_DEFAULT,
        )
        self._timer.reset()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._timer.cancel()
        self._clear_targets()
        if not self.destroy_subscription(self._target_sub):
            self.get_logger().warning('Failed to destroy subscription')
        if not self.destroy_subscription(self._attitude_target_sub):
            self.get_logger().warning('Failed to destroy attitude target subscription')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        if not self.destroy_timer(self._timer):
            self.get_logger().warning('Failed to destroy timer')
        if not self.destroy_publisher(self._target_pub):
            self.get_logger().warning('Failed to destroy publisher')
        if not self.destroy_publisher(self._attitude_target_pub):
            self.get_logger().warning('Failed to destroy attitude target publisher')
        return TransitionCallbackReturn.SUCCESS

    def _timer_callback(self) -> None:
        self.get_logger().warning('Targets are not updated')
        self._clear_targets()

    def _clear_targets(self) -> None:
        # Clear the last command so input loss cannot keep the robot moving.
        self._target_pub.publish(Target())
        attitude_target = AttitudeTarget()
        attitude_target.attitude.w = 1.0
        self._attitude_target_pub.publish(attitude_target)

    def _target_callback(self, msg: Target) -> None:
        self._target_pub.publish(msg)
        self._timer.reset()

    def _attitude_target_callback(self, msg: AttitudeTarget) -> None:
        self._attitude_target_pub.publish(msg)
        self._timer.reset()


def main(args: list[str] = sys.argv) -> None:
    rclpy.init(args=args)
    node = ManualTargetGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
