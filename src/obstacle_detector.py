#!/usr/bin/env python3

"""
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Monash Nova Rover Team

Subscriber node that finds the average Z value of 
points with 0.3 < Z < 3.0, -0.15 < X < 0.15, and
-0.15 < Y < 0.15, then publishes this average to 
obstacle_proximity topic as a float. Uses a custom 
parser to parse PointCloud2 data object.
Refer cloud_point2.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
NODE: obstacle_detector
TOPICS:
  - /D435/depth/color/points [sensor_msgs.msg.PointCloud2]
  - /T265/odom/sample [nav_msgs.msg.Odometry]
  - /obstacle_proximity [std_msgs.msg.Float32]
SERVICES:
  - 
ACTIONS: None
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PACKAGE: 	induction task
AUTHOR(S):	Amesh
CREATION:	27/09/2021
EDITED:		01/09/2021
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TODO:
 - 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import cloud_point2 as pc2


# To create separate files set this variable to True
CREATE_SEPERATE = False
PC2_FILE = "sub1.csv"
ODO_FILE = "sub2.csv"

class Publisher(Node):
    def __init__(self):
        """
        This class initializes a publisher.
        Publisher topic - obstacle_proximity.
        """
        super().__init__("obstacle_distance_publisher")
        self.publisher = self.create_publisher(Float32, 'obstacle_proximity', 10)

    def publish_message(self, message: float):
        """
        @param message: message to publish
        @return: void
        """
        msg = Float32()
        msg.data = float(message)
        self.publisher.publish(msg)
        self.get_logger().info("Publishing: " + str(msg.data))


class SubscribeAndPublish(Node):
    def __init__(self):
        """
        Initializes the publisher and the subscriber.
        Subscriber topic - /D435/depth/color/points
        """
        super().__init__('obstacle_detector')
        self.publisher = Publisher()
        self.file1 = open(PC2_FILE, "w")
        if CREATE_SEPERATE :
            self.file2 = open(ODO_FILE, "w")
            self.file2.write("time,x\n")
            self.file1.write("time,avg\n")
            self.file1.close()
            self.file2.close()
        else:
            self.file1.write("time,avg,x\n")
            self.file1.close()

        
        self.subscriber = self.create_subscription(PointCloud2, "/D435/depth/color/points", self.callback_func, 10)
        self.subscriber_camera = self.create_subscription(Odometry, "/T265/odom/sample", self.callback_func2, 10)
        self.subscriber_x = 0

    def callback_func(self, msg: PointCloud2):
        """
        This function is called when the publisher receives a message
        @param msg:
        @return:
        """
        self.file1 = open(PC2_FILE, "a")
        counter = 0
        total = 0.0
        # parse the pointcloud2 data set and returns list of lists. The inner list contains the data related to the field names
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):     
            if -0.15 < float(p[0]) < 0.15 and -0.15 < float(p[1]) < 0.15 and 0.3 < float(p[2]) < 3.0:
                total += p[2]
                counter += 1
        if counter > 0:
            avg = total / counter
            self.publisher.publish_message(float(avg))
            self.get_logger().info("Published message to publisher from subscriber: " + str(avg) + "for timestamp - " + str(
                msg.header.stamp.sec) + " seconds")
            # write data to csv file
            if CREATE_SEPERATE:
                self.file1.write(str(msg.header.stamp.sec)+ "," +str(avg) +"\n")
            else:
                self.file1.write(str(msg.header.stamp.sec)+ "," +str(avg)+ "," +str(self.subscriber_x) +"\n")
        self.file1.close()


    def callback_func2(self, msg: Odometry):
        """
        This function is called when the publisher receives a message
        @param msg: msg as a Odometry object
        @return:
        """
        if CREATE_SEPERATE:
            self.file2 = open(ODO_FILE, "a")
            self.file2.write( str(msg.header.stamp.sec)+ "," + str(msg.pose.pose.position.x) + "\n")
            self.file2.close()
        else:
            self.subscriber_x = msg.pose.pose.position.x
        


def main():
    rclpy.init()
    subscriber = SubscribeAndPublish()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
