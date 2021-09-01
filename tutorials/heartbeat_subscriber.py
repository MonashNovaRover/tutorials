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








Not done yet!!


"""








import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class HeartbeatSub:
    # Stores whether or not a beat has been found
    beat = False
    beat_count = 0

    # The time out and loop counts
    LOOP_HERTZ = 50
    TIME_OUT = 25

    # The initialisation function
    def __init__ (self):
        print("Initialising Heartbeat Subscriber class.")

        # Set up the ROS params
        rospy.init_node('heartbeat_sub', anonymous=True)
        rospy.Subscriber("/base/heartbeat", Empty, self.received_cb)

        # Run the check node for seeing if heartbeat exists
        self.check_heartbeat()

    # Called when received heartbeat information
    def received_cb (self, data):
        self.heartbeat()
        self.beat_count = 0

    # Checks the heartbeat on the loop
    def check_heartbeat (self):
        rate = rospy.Rate(self.LOOP_HERTZ) # 10hz

        # Call the first heartbeat for the LED controller
        self.heartbeat()

        # While still running ROS
        while not rospy.is_shutdown():
            # If the beat count exceeds the time out
            if self.beat_count > self.TIME_OUT:
                self.no_heartbeat()

            # Otherwise increase beat count
            else:
                self.beat_count += 1

            # Set the ROS param for the heartbeat
            rospy.set_param("/rover/heartbeat", self.beat_count <= self.TIME_OUT)

            rate.sleep()

    # When heartbeat times out
    def no_heartbeat (self):
	self.beat = False
        print("\033[1;31;48mNo Heartbeart Detected\033[0;0m")

    # When heartbeat returns
    def heartbeat (self):
	if not self.beat:
	    print("\033[1;31;44mHeartbeat Returned\033[0;0m")
	self.beat = True

# The main function when run
if __name__ == "__main__":
    sub = HeartbeatSub()
