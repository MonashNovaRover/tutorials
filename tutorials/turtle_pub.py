#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class Publisher(Node):
    def __init__(self):
        super().__init__("node_pub")

        self.publisher = self.create_publisher(Empty,"/tutorials/turtle", 10)
        self.timer = self.create_timer(1, self.callback_func)

    def callback_func(self):
        msg = Empty()
        self.publisher.publish(msg)
        
def main():
    rclpy.init()
    publisher = Publisher()
   
    rclpy.spin(publisher)

if __name__ == "__main__":
    main()






