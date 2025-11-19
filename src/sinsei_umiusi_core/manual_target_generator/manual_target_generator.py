#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Publisher, Subscription
from rclpy.timer import Timer
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import qos_profile_default as QOS_PROFILE_DEFAULT

from sinsei_umiusi_msgs.msg import Target


class ManualTargetGenerator(LifecycleNode):
    """
    # Manual Target Generator Node

    Passes target velocity and orientation received from UI
    """

    def __init__(self) -> None:
        super().__init__('manual_target_generator')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._target_updated: bool = False

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
        self._target_sub: Subscription = None

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._timer.reset()
        self._target_sub: Subscription = self.create_subscription(
            Target,
            '/user_input/target',
            self._target_callback,
            QOS_PROFILE_DEFAULT,
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._timer.cancel()
        if not self.destroy_subscription(self._target_sub):
            self.get_logger().warning('Failed to destroy subscription')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        if not self.destroy_timer(self._timer):
            self.get_logger().warning('Failed to destroy timer')
        if not self.destroy_publisher(self._target_pub):
            self.get_logger().warning('Failed to destroy publisher')
        return TransitionCallbackReturn.SUCCESS

    def _timer_callback(self) -> None:
        if not self._target_updated:
            self.get_logger().warn('Target is not updated')
        self._target_updated = False

    def _target_callback(self, msg: Target) -> None:
        self._target_pub.publish(msg)
        self._target_updated = True


def main(args=sys.argv) -> None:
    rclpy.init(args=args)
    node = ManualTargetGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
