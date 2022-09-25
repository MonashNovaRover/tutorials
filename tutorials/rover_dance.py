#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from core.msg import DriveInput


class Dance(Node):
    def __init__(self):
        super().__init__("node_dance")

        self.publisher = self.create_publisher(DriveInput, '/control/drive_inputs', 10)
        self.timer = self.create_timer(2, self.callback_func)
        self.speed = 0.05

    def callback_func(self):
        msg = DriveInput()
        msg.speed = self.speed
        msg.steer = 0.0
        self.publisher.publish(msg)
        self.speed *= -1


def main(args=None):
    rclpy.init(args=args)
    dance = Dance()
    rclpy.spin(dance)

    dance.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
