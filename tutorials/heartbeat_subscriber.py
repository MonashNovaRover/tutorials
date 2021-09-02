#!/usr/bin/python3

"""
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Monash Nova Rover Team

Listen to heartbeat from base and perform actions if not received
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
NODE: node_name
TOPICS:
  - /core/heartbeat [Empty]
ACTIONS:
  - Set or reset param /core/has_heartbeat
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PACKAGE: 	tutorial
AUTHOR(S):	Jory Braun
CREATION:	02/09/2021
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TODO:
  - Add ROS2 action or param to pass on info from self.beat
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
"""

# Import all ROS packages 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


# The heartbeat subscriber class
class HeartbeatSubscriber(Node):
    # Check the heartbeat every LOOP_DELAY seconds
    LOOP_DELAY = 0.02
    # If the heartbeat is not present for TIME_OUT consecutive checks, heartbeat is considered lost
    TIME_OUT = 25

    def __init__(self):
        super().__init__("heartbeat_sub")
        print("Initialising Heartbeat Subscriber class.")

        # Initialise callbacks for heartbeat and for heartbeat checking
        self.subscription = self.create_subscription(Empty, "core/heartbeat", self.received_cb, 10)
        self.timer = self.create_timer(self.LOOP_DELAY, self.check_heartbeat)

        # Stores whether or not a beat has been found
        self.beat = False
        # Store number of checks since last recorded a beat
        self.beat_count = 0

        # Start the heartbeat
        self.heartbeat()

    def received_cb(self, msg):
        """
        Called when received heartbeat information, reset counter
        :param msg: Empty message. Not used
        """
        self.heartbeat()
        self.beat_count = 0

    def heartbeat(self):
        """
        Called when heartbeat starts or returns
        """
        if not self.beat:
            print("\033[1;31;44mHeartbeat Returned\033[0;0m")
        self.beat = True

    def no_heartbeat(self):
        """
        Called when heartbeat is lost
        """
        print("\033[1;31;48mNo Heartbeart Detected\033[0;0m")
        self.beat = False

    def check_heartbeat(self):
        """
        Check heartbeat
        """
        # Increment check counter
        self.beat_count += 1
        # Check if the heartbeat is lost
        if self.beat_count > self.TIME_OUT:
            self.no_heartbeat()


if __name__ == "__main__":
    rclpy.init()
    sub = HeartbeatSubscriber()
    rclpy.spin(sub)
