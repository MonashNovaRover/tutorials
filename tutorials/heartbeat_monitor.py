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
  - subscribes to /control/heartbeat with msg Empty
SERVICES:
  - calls 'rover_emergency_client' 
ACTIONS: 
  - None
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PACKAGE: 	tutorials
AUTHOR(S):	Josh Cherubino
CREATION:	31/08/21
EDITED:		01/09/21
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TODO:
    - tune BEAT_MISSED_DEADLINE and BEAT_LOST_DURATION values
    - Instantiate and call emergency client once node has been created.
    - Update docstring to match correct client name
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
"""


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
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
        #https://docs.ros.org/en/ros2_documentation/eloquent/Concepts/About-Quality-of-Service-Settings.html                    
        qos_profile = \
                rclpy.qos.QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, #keep only last N samples
                depth = 0, #No need to maintain history
                reliability = QoSReliabilityPolicy.BEST_EFFORT, #will accept a reliable or best effort publisher
                durability = QoSDurabilityPolicy.VOLATILE, #Do not need persistent samples (but will still accept if pub chooses)
                liveliness = QoSLivelinessPolicy.AUTOMATIC, #Automatically detect liveliness change on lease duration exceeded
                liveliness_lease_duration = BEAT_LOST_DURATION, #Max duration between individual messages before beat considered lost
                deadline = BEAT_MISSED_DEADLINE #max duration before single beat considered missed
                )

        #N.B. for QoS compatability a publisher must promise a QoS with
        #lease duration shorter than the lease expected in QoSProfile
        #N.B. that for QoS compatability a publisher must promise a QoS
        #with deadline shorter than the deadline expected in this QoSProfile
        
        #create event callbacks to be triggerred on deadline missed
        #and lease duration execeeded (liveliness lost)
        subscription_event_callbacks = \
                rclpy.qos_event.SubscriptionEventCallbacks(deadline=self.beat_missed,
                liveliness=self.check_beat_lost)

        #create subscription with custom QoS profile and QoS callbacks.
        self.subscription = self.create_subscription(Empty, 'heartbeat',
                self.beat_received, qos_profile, event_callbacks=subscription_event_callbacks)

        #flag to track if we have the heart beat.
        #define possible node states
        self.BEAT_OK = 0
        self.BEAT_MISSED = 1
        self.BEAT_LOST = 2
        #Assume beat lost initially
        self.state = self.BEAT_LOST
        #TODO: create client instance here
        #TODO: initially halt rover until we confirm we can receive a heartbeat
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
        if self.state != self.BEAT_OK:
            self.get_logger().info('Heartbeat detected')
            #check if heart-beat just regained
            if self.state == self.BEAT_LOST:
                #call client to resume rover actions
                #self.rover_emergency_client.call_async(Msg(RESUME))
                self.get_logger().info('Resuming rover')
            #update state
            self.state = self.BEAT_OK

    def beat_missed(self, info: rclpy.qos_event.QoSRequestedDeadlineMissedInfo) -> None:
        """
        Event callback run when publisher misses deadline required by QoS profile.
        Sets heartbeat as lost and logs warning
        :param info: Information about the requested deadline miss
        """
        #log missed beats when heart beat just missed or previously missed
        if self.state == self.BEAT_OK:
            self.get_logger().warn('Heartbeat missed. Total misses: {}'.format(info.total_count))
            self.state = self.BEAT_MISSED

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
            self.state = self.BEAT_LOST
            self.get_logger().error('Heartbeat lost. Halting rover')

def main(args=None):
    rclpy.init(args=args)

    heartbeat_monitor = HeartbeatMonitor()

    rclpy.spin(heartbeat_monitor)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


