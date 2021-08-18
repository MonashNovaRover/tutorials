#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Subscriber (Node):
    
    def __init__(self):
        super().__init__('node_sub')
        self.subscription = self.create_subscription(String, 'my_topic', self.callback_func, 10)
    
    def callback_func (self, msg):
        self.get_logger().info("Received: %s" % msg.data)

def main ():
    rclpy.init()
    subscriber = Subscriber()
    rclpy.spin(subscriber)

if __name__ == "__main__":
    main()