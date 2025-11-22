#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node, Publisher, Subscription
from rclpy.timer import Timer
from rclpy.qos import qos_profile_default as QOS_PROFILE_DEFAULT

from sinsei_umiusi_msgs.msg import LowPowerCircuitInfo, HealthCheckResult


def health_check_low_power_circuit(info: LowPowerCircuitInfo) -> HealthCheckResult:
    result: HealthCheckResult = HealthCheckResult()
    result.is_ok = (
        info.can == LowPowerCircuitInfo.OK
        and info.headlights == LowPowerCircuitInfo.OK
        and info.imu == LowPowerCircuitInfo.OK
        and info.indicator_led == LowPowerCircuitInfo.OK
    )
    return result


class LowPowerHealthCheck(Node):
    def __init__(self) -> None:
        super().__init__('low_power_health_check')
        self._health_check_info_updated: bool = False

        self._timer: Timer = self.create_timer(1.0, self._timer_callback)
        self._low_power_circuit_info_sub: Subscription = self.create_subscription(
            LowPowerCircuitInfo,
            '/state/low_power_circuit_info',
            self._low_power_circuit_info_callback,
            QOS_PROFILE_DEFAULT,
        )
        self._health_check_result_pub: Publisher = self.create_publisher(
            HealthCheckResult,
            'low_power_health_check_result',
            QOS_PROFILE_DEFAULT,
        )

    def _timer_callback(self) -> None:
        if not self._health_check_info_updated:
            self.get_logger().warning('Low power circuit info is not updated')
        self._health_check_info_updated = False

    def _low_power_circuit_info_callback(self, msg: LowPowerCircuitInfo) -> None:
        health_check_result = health_check_low_power_circuit(msg)
        self._health_check_result_pub.publish(health_check_result)
        self._health_check_info_updated = True


def main(args: list[str] = sys.argv) -> None:
    rclpy.init(args=args)
    node = LowPowerHealthCheck()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
