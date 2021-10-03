#!usr/bin/env python3
"""
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Monash Nova Rover Team

Takes in pointcloud data from camera, 
  outputs average distance to obstacle in metres as FLoat32.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
NODE: obstacle_detector
TOPICS:
  - /D435/depth/color/points [sensor_msgs.msg.PointCloud2]
  - /obstacle_proximity [std_msgs.msg.Float32]
SERVICES:
  - /service_name [Service Type]
ACTIONS: None
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PACKAGE: 	obstacle_detector
AUTHOR(S):	Alexander Li
CREATION:	03/10/2021
EDITED:		03/10/2021
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TODO:
 - 
 - parse data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
import struct
from bitarray import bitarray


# use this function for the recruit induction task (or write your own! this seems like a very messy way of doing it)
def floatify_bytes(z_bytes):
    """
    :param lst of four bytes as numbers between 0-255
    :return: the little endian float representation
    """
    bits = bin(z_bytes[0])[2:].ljust(8, "0") + bin(z_bytes[1])[2:].ljust(8, "0") + bin(z_bytes[2])[2:].ljust(8, "0") + bin(z_bytes[3])[2:].ljust(8, "0")
    arr = bitarray(bits[::-1])
    return struct.unpack(">f", arr)[0]


class Obstacle_detector(Node):
    def __init__(self):
        super().__init__("obstacle_detector")
        self.subscription = self.create_subscription(PointCloud2, "/D435/depth/color/points", self.callback, 100) # queue size 100
        self.publisher = self.create_publisher(Float32, "/obstacle_proximity", 10)

    def callback(self, msg: PointCloud2) -> None:
        """
        Publishes the average z-value of points in the point cloud within a 30cm x 30cm window in the x-y plane. 
        :param msg: sensor_msgs.msg.PointCloud2
        :return: None
        """
        sampling_sparseness = 100 # take only one point out of every [this number] of points.
        sum_z = 0.0 # aggregate of z values for qualified points
        num_z = 0.0 # number of qualified points

        for i in range(msg.width//sampling_sparseness):
            start_pos = 20*sampling_sparseness*i # start position within the byte list.

            x = floatify_bytes(msg.data[start_pos:start_pos+4])
            y = floatify_bytes(msg.data[start_pos+4:start_pos+8])
            z = floatify_bytes(msg.data[start_pos+8:start_pos+12])

            # only points within a 30cm x 30cm window are accepted.
            if abs(x) < 0.15 and abs(y) < 0.15 and z > 0.3 and z < 3.0:
                sum_z += z
                num_z += 1

        # create and publish average z value as FLoat32
        z_average = Float32()
        z_average.data = sum_z/num_z
        self.publisher.publish(z_average)


def main():
    rclpy.init(args=None)
    obstacle_detector = Obstacle_detector()
    rclpy.spin(obstacle_detector)


if __name__ == "__main__":
    main()