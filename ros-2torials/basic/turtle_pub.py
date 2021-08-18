#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher (Node):

    def __init__ (self):
        super().__init__('node_pub')
        self.publisher = self.create_publisher(String, 'my_topic', 10)
        self.timer = self.create_timer(0.5, self.callback_func)

    def callback_func (self):
        msg = String()
        msg.data = input("Please enter a word: ")

        self.publisher.publish(msg)
        self.get_logger().info("Publishing: %s" % msg.data)

def main ():
    rclpy.init()
    publisher = Publisher()
    rclpy.spin (publisher)

if __name__ == '__main__':
    main()
