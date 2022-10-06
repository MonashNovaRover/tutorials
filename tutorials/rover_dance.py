#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from core.msg import DriveInput
from rclpy.duration import Duration
from rclpy.qos import QoSProfile QoSReliabilityProfile

class DancingRobot(Node):
    def __init__(self):
        qos = QoSProfile(
                reliability=QoSReliabilityProfile.BEST_EFFORT,
                depth=1,
                deadline=Duration(nanoseconds=2e8)
            )

        super().__init__("do_the_robot")

        self.publisher = self.create_publisher(DriveInput, '/control/drive_inputs', qos)
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
