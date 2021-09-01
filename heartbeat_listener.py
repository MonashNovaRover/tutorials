#!/usr/bin/env python3

#--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--
# 
# Currently, it will flash the LED red if no heartbeat found 
#
# Author: Harrison
# Last modified 22/03/21 by: Harrison
# Translated to ROS2 by: Himsara
#
# Subscribed topics:
#    - /base/heartbeat
#--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--**--..--

'''
NOVA ROVER TEAM
This node listens to heartbeat from base and performs actions if not received
Currently, it will flash the LED red if no heartbeat found
Author: Himsara Gallege
'''

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class HeartbeatSub(Node):
    # Store whether or not a beat has been found
    beat = False
    beat_count = 0

    # The time out and loop counts
    LOOP_HERTZ = 50
    TIME_OUT = 25

    # The initialisation function
    def __init__ (self):
        print("Initialising Heartbeat Subscriber class.")

        # ROS initialisation
        super().__init__('HeartbeatSub')
        self.subscription = self.create_subscription(Empty, 'base/heartbeat', self.callback_func, 10)
 
        # Create timer to run checkheartbeat frequently
        self.timer = self.create_timer(0.1, self.check_heartbeat)

    # Called when received heartbeat information
    def received_cb (self, data):
        self.heartbeat()
        self.beat_count = 0

    def check_heartbeat(self):
        if self.beat_count <= TIME_OUT:
            self.beat_count += 1
        else:
            self.no_heartbeat()


    # When heartbeat times out
    def no_heartbeat (self):
	self.beat = False
        print("\033[1;31;48mNo Heartbeart Detected\033[0;0m")


    # When heartbeat returns
    def heartbeat (self):
	if not self.beat:
	    print("\033[1;31;44mHeartbeat Returned\033[0;0m")
	self.beat = True
