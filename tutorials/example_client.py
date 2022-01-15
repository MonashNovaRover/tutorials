#!/usr/bin/env python3

# Import all relevant ROS 2 packages
import rclpy
from rclpy.node import Node

# Import the service from the examples
from example_interfaces.srv import AddTwoInts

# This is the main Node class that controls the functions
class ClientNode (Node):

    # The __init__ function needs to set up the Node.
    # It will create the node name and create the client with the relevant parameters
    def __init__(self):
        super().__init__('node_client')

        # Service Type, Service Name
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        # Waits until a service has been set up and running
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not Available, Waiting again...')
        
        # Create the request data-frame
        self.req = AddTwoInts.Request()
    
    # Sends the request from the user input
    def send_request (self):
        self.req.a = int(input("Enter the first number: "))
        self.req.b = int(input("Enter the second number: "))
        self.future = self.client.call_async(self.req)

# Main function for setting up the ROS node    
def main (args = None):
    rclpy.init(args = args)
    client = ClientNode()
    client.send_request()
    
    # Loop until ROS is not okay
    while rclpy.ok():
        rclpy.spin_once(client)
        
        # If the client has completed processing
        if client.future.done():
            # Attempt to get a response
            try:
                response = client.future.result()
            # Handle the error if missing data
            except Exception as e:
                client.get_logger().info("Service call failed %r" % (e,))
            # If success
            else:
                client.get_logger().info(
                    "Calculation Done: %d + %d = %d" %
                    (client.req.a, client.req.b, response.sum)) 
            break
    
    # Shutdown ROS otherwise
    client.destroy_node()
    rclpy.shutdown()

# This code is called when 'python3' is used to run the script
if __name__ == '__main__':
    main()
