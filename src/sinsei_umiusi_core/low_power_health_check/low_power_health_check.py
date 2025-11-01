#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node


class LowPowerHealthCheck(Node):
    def __init__(self):
        super().__init__('low_power_health_check')
        self._timer = self.create_timer(0.05, self._timer_callback)

    def _timer_callback(self):
        self.get_logger().debug('tick')


def main(args=sys.argv):
    rclpy.init(args=args)
    node = LowPowerHealthCheck()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
