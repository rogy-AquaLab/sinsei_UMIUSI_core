#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Publisher
from rclpy.timer import Timer
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import qos_profile_default as QOS_PROFILE_DEFAULT

from sinsei_umiusi_msgs.msg import Target


# TODO: The implementation is just a placeholder; publishing empty Target messages periodically.
class AutoTargetGenerator(LifecycleNode):
    """
    # Auto Target Generator Node

    Generates target velocity and orientation from feedback data (IMU, Image, etc.)
    """

    def __init__(self) -> None:
        super().__init__('auto_target_generator')

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
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._timer.reset()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        if not self.destroy_timer(self._timer):
            self.get_logger().warning('Failed to destroy timer')
        if not self.destroy_publisher(self._target_pub):
            self.get_logger().warning('Failed to destroy publisher')
        return TransitionCallbackReturn.SUCCESS

    def _timer_callback(self) -> None:
        msg = Target()
        self._target_pub.publish(msg)


def main(args: list[str] = sys.argv) -> None:
    rclpy.init(args=args)
    node = AutoTargetGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
