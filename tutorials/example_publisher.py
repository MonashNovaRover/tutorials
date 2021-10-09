#!/usr/bin/env python3

# Import all relevant ROS 2 packages
import rclpy
from rclpy.node import Node

# Import the standard message
from std_msgs.msg import String

# This is the main Node class that controls the functions
class PublisherNode (Node):

    # The __init__ function needs to set up the Node.
    # It will create the node name and create the publisher with the relevant parameters
    def __init__ (self):
        super().__init__('node_pub')

        # Message Type, Topic Name, Quality of Service 
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.callback_func)

    # Callback function for every time data needs to be sent
    def callback_func (self):
        msg = String()
        msg.data = "Hello World!"
        self.publisher.publish(msg)
        self.get_logger().info("Publishing: '%s'" % msg.data)

# Main function for setting up the ROS node  
def main (args = None):
    rclpy.init(args = args)
    publisher = PublisherNode()
    rclpy.spin(publisher)
    
    publisher.destroy_node()
    rclpy.shutdown()

# This code is called when 'python3' is used to run the script   
if __name__ == "__main__":
    main()
