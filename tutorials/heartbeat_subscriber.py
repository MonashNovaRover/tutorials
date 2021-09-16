#!/usr/bin/env python3

"""
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Monash Nova Rover Team

This python script sends out the beep boops from base station.
Hopefully rovey picks them up and don't die

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
NODE: heartbeat_subscriber
TOPICS:
  - /tutorials/heartbeat [Empty]
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PACKAGE: 	tutorials
AUTHOR(S):	Emily Kuo
CREATION:	01/09/2021
EDITED:		01/09/2021
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TODO:
 - Have a Sunny day outside
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class HeartbeatSubscriber(Node):
    def __init__(self):
        super().__init__("heartbeat_sub")
        print("Initialising Heartbeat Subscriber class.")
        self.subscription = self.create_subscription(Empty, "/tutorials/heartbeat", self.heartbeat_cb, 10)
        # Stores whether or not a beat has been found
        self.hb_detected = False
        self.timeout_count = 0

        # The time out and loop count constants
        self.LOOP_HERTZ = 50
        self.TIMEOUT = 25
        self.timer = self.create_timer(1/self.LOOP_HERTZ, self.increment_count)

    def heartbeat_cb(self, msg):
        """ 
        Called when receives a message from the /tutorials/heartbeat topic
        """
        self.heartbeat()
        self.timeout_count = 0

    def increment_count(self):
        """
        Called based on the timer, LOOP_HERTZ times a second
        Increments the timeout counter
        """
        if self.timeout_count == self.TIMEOUT:
            #only prints once when heartbeat is lost
            #can change if you prefer the spam
            self.no_heartbeat()
        self.timeout_count +=1

    def no_heartbeat(self):
        """
        Called when the timeout has been exceeded without receiving any heartbeat messages
        """
        self.hb_detected = False
        print("\033[1;31;48mBoop Brrtz :(\033[0;0m")

    def heartbeat(self):
        """
        Called when a heartbeat message is received
        """
        if self.hb_detected == False: 
            #if heartbeat was lost previously but is found again
            print("\033[1;31;44mBeep Boop\033[0;0m")
        self.hb_detected = True
        self.timeout_count = 0

def main():
    rclpy.init()
    hb_subscriber = HeartbeatSubscriber()
    
    rclpy.spin(hb_subscriber)

if __name__ == "__main__":
    main()
