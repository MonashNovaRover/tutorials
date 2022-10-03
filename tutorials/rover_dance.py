#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from core.msg import DriveInput

class DancingRobot(Node):
    def __init__(self):
        super().__init__("do_the_robot")

        self.publisher = self.create_publisher(DriveInput, '/control/drive_inputs', 10)
        self.timer = self.create_timer(1, self.callback_func)
        self.go_forward = True
    

    def callback_func(self):
        if self.go_forward:
            self.publisher.publish(DriveInput(speed=0.5, steer=0))
        else:
            self.publisher.publish(DriveInput(speed=-0.5, steer=0))
        
        self.go_forward = not self.go_forward


def main(args = None):
    rclpy.init(args=args)
    dancing_robot_publisher = DancingRobot()
    rclpy.spin(dancing_robot_publisher)
    rclpy.shutdown()


if __name__ == "__main__":
    main()