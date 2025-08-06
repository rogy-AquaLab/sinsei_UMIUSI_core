#!/usr/bin/env python3

# ref: https://github.com/rogy-AquaLab/2024_umiusi/blob/74f2239/device/joystick/joystick/joystick.py

import sys

import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool


class Joystick(Node):
    def __init__(self):
        super().__init__('joystick')
        self._timer = self.create_timer(0.05, self._timer_callback)

        self._orientation_publisher = self.create_publisher(
            Vector3, '/cmd/target_orientation', 10
        )
        self._velocity_publisher = self.create_publisher(
            Vector3, '/cmd/target_velocity', 10
        )
        self._indicator_led_publisher = self.create_publisher(
            Bool, '/cmd/indicator_led_enabled', 10
        )

        # TODO : ここparameterで変えられるようにしたい
        self._joystick = pygame.joystick.Joystick(0)
        # logging
        name = self._joystick.get_name()
        self.get_logger().info(f'Using controller: {name}')

        self._is_stopped = False

    def _timer_callback(self):
        self.get_logger().debug('tick')
        pygame.event.pump()

        numaxes = self._joystick.get_numaxes()
        numbuttons = self._joystick.get_numbuttons()
        self.get_logger().info(f'Found {numaxes} axes and {numbuttons} buttons')

        # プラマイ0.1以下の値は0とみなす
        l_stick_horizontal = round(self._joystick.get_axis(0), 1)  # 右が正
        l_stick_vertical = round(self._joystick.get_axis(1), 1)  # 下が正
        r_stick_horizontal = round(self._joystick.get_axis(3), 1)
        r_stick_vertical = round(self._joystick.get_axis(4), 1)

        self._is_stopped = self._is_stopped or self._joystick.get_button(0)

        self.get_logger().info(f'_is_stopped: {self._is_stopped}')

        if self._is_stopped:
            circle_pressed = self._joystick.get_button(1)
            if not circle_pressed:
                self._velocity_publisher.publish(Vector3(x=0.0, y=0.0, z=0.0))
                self._orientation_publisher.publish(Vector3(x=0.0, y=0.0, z=0.0))
                self._indicator_led_publisher.publish(Bool(data=False))
                return

            self._is_stopped = False

        # 左スティック
        velocity_msg = Vector3(x=-l_stick_vertical, y=-l_stick_horizontal, z=0.0)
        # 右スティック
        orientation_msg = Vector3(x=-0.0, y=r_stick_vertical, z=r_stick_horizontal)
        # 三角ボタン
        indicator_led_msg = Bool(data=bool(self._joystick.get_button(2)))
        self._velocity_publisher.publish(velocity_msg)
        self._orientation_publisher.publish(orientation_msg)
        self._indicator_led_publisher.publish(indicator_led_msg)


def main(args=sys.argv):
    rclpy.init(args=args)
    pygame.init()
    node = Joystick()
    rclpy.spin(node)
    node.destroy_node()
    pygame.quit()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
