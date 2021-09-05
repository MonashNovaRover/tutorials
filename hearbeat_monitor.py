#!/usr/bin/env python3
'''
NOVA ROVER TEAM
This node listens to heartbeat from base and performs actions if not received

Author: Marcel Masque
Last Modified: 5/09/2021 by Marcel Masque
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class HeartbeatSubscriber(Node):

    # The time out and loop counts
    LOOP_HERTZ = 50
    TIME_OUT = 25

    def __init__ (self):
        super().__init__('heartbeat_sub')
        self.beat = False
        self.beat_count = 0

        # subscribe to the hearbeat publisher
        self.create_subscription(Empty, "tutorials/heartbeat", self.on_hearbeat, 10)

        # periodically call a callback
        self.timer = self.create_timer(0.5, self.check_heartbeat)

    def check_heartbeat(self):
        if self.beat_count > HeartbeatSubscriber.TIME_OUT:
            self.no_heartbeat()
        else:
            self.beat_count += 1

    def on_hearbeat(self):
        if not self.beat:   # only when heartbeat was lost before
            print("\033[1;31;44mHeartbeat Returned\033[0;0m")
        self.beat = True        
        self.beat_count = 0

    def no_heartbeat(self):
        self.beat = False
        print("\033[1;31;48mNo Heartbeart Detected\033[0;0m")
    
def main():
    rclpy.init()
    heartbeat_subscriber = HeartbeatSubscriber()
    rclpy.spin(heartbeat_subscriber)


if __name__ == '__main__':
    main()