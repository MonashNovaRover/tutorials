#!/usr/bin/env python3
"""
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Monash Nova Rover Team

This class is a class to monitor the heartbeart signal from
the base station to ensure connection has been maintained with
the base station. Where a heart beat is missed an appropriate
message is logged. Where the heart beat is lost, a call is made
to an appropriate service to halt the rover and indicate loss visually (i.e. LED strip)

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
NODE: heartbeat_monitor
TOPICS:
  - None
SERVICES:
  - None
ACTIONS: 
  - None
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PACKAGE: 	tutorials
AUTHOR(S):	Josh Cherubino
CREATION:	31/08/21
EDITED:		31/08/21
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TODO:
    - tune BEAT_MISSED_DEADLINE and BEAT_LOST_DURATION values
    - Instantiate and call client once node has been created
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
"""


import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

#define constants
#max allowed duration between heartbeats.
#failure to meet this deadline indicates a heartbeat missed but
#not a complete failure.
BEAT_MISSED_DEADLINE = rclpy.duration.Duration(seconds=1/50.0)

#max allowed duration before heartbeat is considered lost (heartbeat publisher considered
#to have lost its liveliness from QoS perspective
BEAT_LOST_DURATION = rclpy.duration.Duration(seconds=1/25.0)

class HeartbeatMonitor(Node):
    def __init__(self) -> None:
        """
        Initialise HeartbeatMonitor instance
        """
        #use control namespace for node
        #don't need parameters for this node so do not run.
        super().__init__('heartbeat_monitor', namespace='control',
                start_parameter_services=False)

        #configure custom QoS profile for subscriber to handle timing issues and register
        #events when heartbeat not achieved
        #set history to keep last so depth parameter is used (should be default)
        #depth 1 as only need latest message
        qos_profile = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                depth=1)

        #use best effort to be flexible (allow pub to either be best effort or reliable)
        #https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html#qos-policies
        qos_profile.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT

        #Samples should not be persistent
        #N.B. if publisher chooses to make them consistent then this is still allowed
        qos_profile.durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE

        #automatic liveliness detection so automatically detect when lease duration exceeded
        qos_profile.liveliness = rclpy.qos.QoSLivelinessPolicy.AUTOMATIC
        #N.B. for QoS compatability a publisher must promise a QoS with
        #lease duration shorter than the lease expected here
        qos_profile.liveliness_lease_duration = BEAT_LOST_DURATION

        #set deadline for each beat. if deadline not met log warning
        #N.B. that for QoS compatability a publisher must promise a QoS
        #with deadline shorter than the deadline expected in this subscriber
        qos_profile.deadline = BEAT_MISSED_DEADLINE
        
        #create event callbacks to be triggerred on deadline missed
        #and lease duration execeeded (liveliness lost)
        #this isn't documented for some reason.
        #see source https://github.com/ros2/rclpy/blob/98707238ad2564e28cbc987fc3a31ed9a1c86243/rclpy/rclpy/qos_event.py
        subscription_event_callbacks = \
                rclpy.qos_event.SubscriptionEventCallbacks(deadline=self.beat_missed,
                liveliness=self.check_beat_lost)

        #create subscription with custom QoS profile and QoS callbacks.
        self.subscription = self.create_subscription(Empty, 'heartbeat',
                self.beat_received, qos_profile, event_callbacks=subscription_event_callbacks)

        #flag to track if we have the heart beat.
        #Assume no heartbeat initially
        self.beat = False
        #flag to track if we have halted the rover previously
        self.halted = True

        #create client instance here
        
        #and initially halt rover until we confirm we can receive a heartbeat

        self.get_logger().info('Heartbeat monitor started')

    def beat_received(self, msg: Empty) -> None:
        """
        Callback to update state when a heartbeat is received from topic.
        If rover was previously halted then will resume the rovers behaviour
        Returns None
        :param msg: Empty message received from publisher
        """
        #if heartbeat just re-gained, log that it has been detected
        #otherwise do nothing
        if not self.beat:
            self.get_logger().info('Heartbeat detected')
            self.beat = True
            #heart-beat regained
            if self.halted:
                #call client to resume rover actions
                #self.rover_emergency_client.call_async(Msg(RESUME))
                self.get_logger().info('Resuming rover')
                self.halted = False

    def beat_missed(self, info: rclpy.qos_event.QoSRequestedDeadlineMissedInfo) -> None:
        """
        Event callback run when publisher misses deadline required by QoS profile.
        Sets heartbeat as lost and logs warning
        :param info: Information about the requested deadline miss
        """
        #only log missed beat (to avoid spam where heartbeat completely lost) when 
        #it was previously not lost
        if self.beat:
            self.get_logger().warn('Heartbeat missed. Total misses: {}'.format(info.total_count))
            self.beat = False

    def check_beat_lost(self, info: rclpy.qos_event.QoSLivelinessChangedInfo) -> None:
        """
        Event callback run when publisher liveliness state changes
        Checks changed publisher liveliness, and if 'not_alive', logs error and
        calls halt_rover service to prevent any damage to rover
        :param info: Information about the liveliness change event
        """
        #Beat only lost where our 'not_alive_count' equal to or exceeds 'alive_count'
        #(b/c we are initially in alive state first time we receive message)
        if info.not_alive_count >= info.alive_count:
            #call appropriate service to halt rovers motion here
            #i.e. self.future = self.rover_emergency_client.call_async(Msg(HALT))
            #set state to halted so we can resume when beat re-acquired
            self.halted = True
            self.beat = False
            self.get_logger().error('Heartbeat lost. Halting rover')

def main(args=None):
    rclpy.init(args=args)

    heartbeat_monitor = HeartbeatMonitor()

    rclpy.spin(heartbeat_monitor)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


