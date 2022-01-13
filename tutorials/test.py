#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import coms_utils


class Tester (Node):

    def __init__ (self):
        super().__init__('node_test')
        #self.test_obj = Test()
        #print(self.test_obj.get_value(2))


# Main function for setting up the ROS node
def main (args = None):
    rclpy.init(args = args)
    service = Tester()
    rclpy.spin(service)

    service.destroy_node()
    rclpy.shutdown()

# This code is called when 'python3' is used to run the script
if __name__ == '__main__':
    main()