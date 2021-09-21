#!/usr/bin/env python3
"""
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Monash Nova Rover Team

Subscriber node that finds the average Z value of
 points with 0.3 < Z < 3.0, -0.15 < X < 0.15, and
 -0.15 < Y < 0.15, then publishes this average to 
 obstacle_proximity as a float. Uses floatify_bytes
 to convert byte array to little endian float.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
NODE: Obstacle_Subscriber
TOPICS:
  - /D435/depth/color/points [sensor_msgs.msg.PointCloud2]
SERVICES:
  - None
ACTIONS: None
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PACKAGE: 	induction_task
AUTHOR(S):	Max Tory
CREATION:       20/09/2021
EDITED:		21/09/2021
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
"""
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from std_msgs.msg import String
from typing import List
from time import perf_counter as time


class Obstacle_Subscriber(Node):
    def __init__(self):
        super().__init__('obstacle_subscriber')
        self.subscription = self.create_subscription(PointCloud2, '/D435/depth/color/points', self.parse_points, 10)
        # for logging results
        self.publisher = self.create_publisher(String, 'obstacle_proximity', 10)
        self.counter = 0

    def parse_points(self, msg: PointCloud2):
        output_info = String()
        # timer to check performance
        start_time = time()
        self.counter += 1

        z_sum = 0
        num_points = 0
        data = msg.data
        skip_amount = 1 # in case we want to skip some points for efficiency
        for i in range(0, len(data), skip_amount * 20):
            x = floatify_bytes(data[i : i + 4])
            y = floatify_bytes(data[i + 4 : i + 8])
            z = floatify_bytes(data[i + 8 : i + 12])
        
            if (-0.15 < x < 0.15) and (-0.15 < y < 0.15) and (0.3 < z < 3.0):
                # We consider this point in our average
                z_sum += z
                num_points += 1

        avg = z_sum/num_points
        time_elapsed = time() - start_time
        output_info.data = str(self.counter) + ": Average z: " + str(avg) + " computed in " + str(round(time_elapsed, 4)) + " s."
        
        self.publisher.publish(output_info)


def floatify_bytes(bytes_list: List[int]):
    """
    :param list of 4 bytes as decimal numbers
    :return: float representation of bytes in little endian format
    """
    decimal = 0
    # combine 4 bytes into a single decimal integer
    for i in range(4):
        decimal <<= 8        # bit shifting previous byte over by 8 
        decimal += bytes_list[i]
    # store as unsigned int, then interpret as if it were a little endian float
    return struct.unpack('<f', struct.pack('>I', decimal))[0]

def main():
    rclpy.init(args = None)
    obstacle_subscriber = Obstacle_Subscriber()
    rclpy.spin(obstacle_subscriber)

if __name__ == "__main__":
    main()





