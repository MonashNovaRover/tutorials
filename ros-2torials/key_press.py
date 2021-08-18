#!/usr/bin/bash
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

class Pub(Node):
    def __init__(self):
        super().__init__("key_presser")
        self.publisher_ = self.create_publisher(Point, "/key_press", 10)

        self.counter = 0
        self.timer = self.create_timer(1, self.timer_callback)
        self.point = None


    def timer_callback(self):
        press = input("Enter keypress: ")
        """
        flatland => z always == 0
        positive x == forward
        positive y == right
        """
        
        if press == "w":
            point = Point(x=1.0, y=0.0, z=0.0)
        if press == "a":
            point = Point(x=0.0, y=-1.0, z=0.0)
        if press == "s":
            point = Point(x=-1.0, y=0.0, z=0.0)
        if press == "d":
            point = Point(x=0.0, y=1.0, z=0.0)
        
        
        self.publisher_.publish(point)
        self.get_logger().info("Publishing: " + str(point))

def main(args=None):
    rclpy.init(args=args)
    pub = Pub()
    rclpy.spin(pub)

if __name__ == '__main__':
    main()
