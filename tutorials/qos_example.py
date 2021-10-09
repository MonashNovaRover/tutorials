#!/usr/bin/python3

# import rospy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# import qos stuff
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

"""
This python file aims to show how to create and use custom QoS profiles 
Adapted from: https://github.com/ros2/demos/blob/58450594268e1540490239db31dcb413e37288b8/topic_monitor/topic_monitor/scripts/data_publisher.py#L73

Other references:
    https://github.com/ros2/rclpy/blob/master/rclpy/test/test_qos_event.py
    https://docs.ros.org/en/eloquent/Concepts/About-Quality-of-Service-Settings.html
    https://answers.ros.org/question/298594/how-does-ros2-select-the-default-qos-profile/
"""

def generate_qos_profile():
    profile = QoSProfile()
    
    # RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT => UDP (won't keep history)
    # RMW_QOS_POLICY_RELIABILITY_RELIABLE    => TCP (keeps history) 
    profile.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
    from rclpy.qos_event import QoSOfferedDeadlineMissedInfo

    return profile


class Sub(Node):
    def __init__ (self):
        super().__init__('sub_custom_qos')
        self.qos_profile = generate_qos_profile()
        
        # we have to create a "subscription options" object
        # in rclcpp there is a rclcpp::SubscriptionOptions but I'm not sure how to do it here
        self.subscription_options = 

        self.subscriber = self.create_subscription(String, 'drive_commands', self.callback, qos_profile=self.qos_profile)
    
    def callback(self, msg):
        """
        :param msg: String msg data from publisher. 
        """
        print(str(msg.data))
    
    def on_deadline_miss_callback(self):
        print("Missed Deadline!")

def main():
    pass 

if __name__ == "__main__":
    main()    
