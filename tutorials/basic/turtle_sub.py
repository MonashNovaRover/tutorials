#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Subscriber (Node):

    def __init__(self):
        super().__init__('node_sub')
        self.subscription = self.create_subscription(Float32MultiArray, 'my_topic', self.callback_func, 10)
    
    def callback_func (self, msg):
        val = 0
        for num in msg.data:
            val += num
        self.get_logger().info("Received: %f" % val)

def main ():
    rclpy.init()
    subscriber = Subscriber()
    rclpy.spin(subscriber)

if __name__ == "__main__":
    main()