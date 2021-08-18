#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Publisher (Node):

    def __init__ (self):
        super().__init__('node_pub')
        self.publisher = self.create_publisher(Float32MultiArray, 'my_topic', 10)
        self.timer = self.create_timer(0.5, self.callback_func)

    def callback_func (self):
        try:
            msg = Float32MultiArray()
            input_a = input("Enter Number 1: ")
            msg.data.append(float(input_a))
            input_b = input("Eneter Number 2: ")
            msg.data.append(float(input_b))

            self.publisher.publish(msg)
            self.get_logger().info("Publishing: %s" % str(msg.data))
            
        except:
            print("Invalid Number")

def main ():
    rclpy.init()
    publisher = Publisher()
    rclpy.spin (publisher)

if __name__ == '__main__':
    main()
