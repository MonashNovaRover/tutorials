#!/usr/bin/env python3

# Import all relevant ROS 2 packages
import rclpy
from rclpy.node import Node

# Import the standard message
from std_msgs.msg import String

# This is the main Node class that controls the functions
class SubscriberNode(Node):

    # The __init__ function needs to set up the Node.
    # It will create the node name and create the subscriber with the relevant parameters
    def __init__(self):
        super().__init__('node_pub')

        # Message Type, Topic Name, Callback Handle, Queue Size 
        self.subscription = self.create_subscription(String, 'topic', self.callback_func, 10)

    # Callback function for everytime data is received
    def callback_func(self, msg):
        self.get_logger().info("Received: '%s'" % msg.data)

# Main function for setting up the ROS node
def main (args = None):
    rclpy.init(args = args)
    subscriber = SubscriberNode()
    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()

# This code is called when 'python3' is used to run the script
if __name__ == '__main__':
    main()