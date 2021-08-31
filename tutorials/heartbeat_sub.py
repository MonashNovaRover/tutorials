#!/usr/bin/env python3

"""
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Monash Nova Rover Team

This node listens to heartbeat from base and performs actions if not received
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
NODE: HeartbeatSubscriber
TOPICS:
  - /base/heartbeat [Empty Message]
SERVICES: None
ACTIONS: None
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PACKAGE: 	tutorials
AUTHOR(S):	Jess 
CREATION:	28/08/2021
EDITED:		30/08/2021
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TODO:
 - 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
"""

# Import all ROS packages 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from rclpy.parameter import Parameter

# The time out constant
TIME_OUT: int = 25

# The heartbeat subscriber class
class HeartbeatSubscriber (Node):
    

    # The initialisation function
    def __init__ (self):

        print("Initialising Heartbeat Subscriber class.")
        
        # Stores whether or not a beat has been found
        self.beat: bool = False
        self.beat_count: int = 0
       
        #set up ROS parameters
        super().__init__("heartbeat_subscriber")
        self.subscription = self.create_subscription(Empty, '/base/heartbeat',self.callback_func,10)
    
        #initialising ROS param for the heartbeat
        self.declare_parameter('/rover/heartbeat', False)

        #start timer to check heartbeat regularly
        self.timer = self.create_timer(0.1, self.check_heartbeat)

    # Called when received heartbeat information
    def callback_func (self,msg):
        print("\033[1;31;48mBEAT\033[0;0m")
        self.heartbeat()
        self.beat_count = 0

    # Checks the heartbeat on the loop
    def check_heartbeat (self):

        # If the beat count exceeds the time out
        if self.beat_count > TIME_OUT:
            self.no_heartbeat()

        # Otherwise increase beat count
        else:
            print("beat count: " + str(self.beat_count))
            self.beat_count += 1
            
        # Set the ROS param for the heartbeat
        self.set_parameters([Parameter('/rover/heartbeat', Parameter.Type.BOOL, self.beat_count <= TIME_OUT)])
    
    # When heartbeat times out
    def no_heartbeat (self):
        self.beat = False
        print("\033[1;31;48mNo Heartbeart Detected\033[0;0m")

    # When heartbeat returns
    def heartbeat (self):
        if not self.beat:
            print("\033[1;31;44mHeartbeat Returned\033[0;0m")
        self.beat = True

def main ():
	rclpy.init()
	subscriber = HeartbeatSubscriber()
	rclpy.spin(subscriber)

if __name__ == "__main__":
	main()
