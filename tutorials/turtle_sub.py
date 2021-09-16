#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class Subscriber(Node):
    def __init__(self):
        super().__init__("node_sub")
        self.subscription = self.create_subscription(Empty,"/tutorials/turtle", self.callback_func, 10)

    def callback_func(self, msg):
        print("Got data yo")

def main():
    rclpy.init()
    subscriber = Subscriber()
    rclpy.spin(subscriber)

if __name__ == "__main__":
    main()
