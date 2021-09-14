#!/usr/bin/env python3

# Import all relevant ROS 2 packages
import rclpy
from rclpy.node import Node

# Import the service from the examples
from example_interfaces.srv import AddTwoInts

# This is the main Node class that controls the functions
class ServiceNode (Node):

    # The __init__ function needs to set up the Node.
    # It will create the node name and create the service with the relevant parameters
    def __init__(self):
        super().__init__('node_service')

        # Service Type, Service Name, Callback Handle
        self.service = self.create_service(AddTwoInts, 'add_two_ints', self.callback_func)
    
    # Calculates the sum of the inputs from the system
    def callback_func (self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info("Incoming Request")
        return response

# Main function for setting up the ROS node
def main (args = None):
    rclpy.init(args = args)
    service = ServiceNode()
    rclpy.spin(service)

    service.destroy_node()
    rclpy.shutdown()

# This code is called when 'python3' is used to run the script
if __name__ == '__main__':
    main()
