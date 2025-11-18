#!/usr/bin/env python3

import sys

import rclpy
from rclpy.lifecycle import Node, State, Publisher, TransitionCallbackReturn
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from rclpy.qos import qos_profile_default as QOS_PROFILE_DEFAULT
from sinsei_umiusi_msgs.msg import Target


class ManualTargetGenerator(Node):
    def __init__(self) -> None:
        super().__init__('manual_target_generator')
        self._state: State = 'unconfigured'
        self._target_updated: bool = False

        self._timer = None
        self._target_pub = None
        self._target_sub = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring ManualTargetGenerator...')
        self._state = state

        self._timer: Timer = self.create_timer(1.0, self._timer_callback)
        self._target_sub: Subscription = self.create_subscription(
            Target,
            '/user_input/target',
            self._target_callback,
            QOS_PROFILE_DEFAULT,
        )
        self._target_pub: Publisher = self.create_publisher(
            Target,
            '/cmd/target',
            QOS_PROFILE_DEFAULT,
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Activating ManualTargetGenerator...')
        self._state = state
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating ManualTargetGenerator...')
        self._state = state
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up ManualTargetGenerator...')
        self._state = state

        self.destroy_timer(self._timer)
        self.destroy_subscription(self._target_sub)
        self.destroy_publisher(self._target_pub)

        self._timer = None
        self._target_sub = None
        self._target_pub = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down ManualTargetGenerator...')
        self._state = state
        return TransitionCallbackReturn.SUCCESS

    def _timer_callback(self) -> None:
        # Only check when active
        if self._state.label != 'active':
            return

        if not self._target_updated:
            self.get_logger().warn('Target is not updated')
        self._target_updated = False

    def _target_callback(self, msg: Target) -> None:
        if self._state.label != 'active':
            self.get_logger().warn('Node is not active. Ignoring target.')
            return

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
