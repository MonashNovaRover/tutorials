#!/usr/bin/python3
"""
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Monash Nova Rover Team

A proximity sensor that takes in positional data and calculates 
the average "forward" value of points within certain 3D bounds.
Specifically, a forward range of 0.3 - 3.0 and a vertical
and horizontal range of -0.15 - 0.15. 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
NODE: obstacle_detector
TOPICS:
  - /D435/depth/color/points [sensor_msgs.msg.PointCloud2]
  - /obstacle_proximity [Float32]
SERVICES:
  - 
ACTIONS: None
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PACKAGE: 	task
AUTHOR(S):	kelly
CREATION:	27/09/2021
EDITED:		30/09/2021
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TODO:
 - 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32

import struct
from bitarray import bitarray

#---- Subscribes to the topic /D435... and publishes the average Z value
class SubscriberNode(Node):
	def __init__(self):
		super().__init__('obstacle_detector')
		self.subscription = self.create_subscription(
			PointCloud2,
			'/D435/depth/color/points',
			self.sub_callback,
			10)
		self.subscription # Avoid unused variable warning
		self.publisher = self.create_publisher(
			Float32,
			'/obstacle_proximity',
			10)

	def sub_callback(self, msg):
		"""
		Parses positional data, calculates the average value and publishes
		it to the topic /obstacle_proximity.
		"""
		data = msg.data
		y_offset = 4
		z_offset = 8
		z_sum = 0.0
		z_count = 0.0
		factor = 5
		# Interates through the dataset and takes the x,y,z values from every 100th subarray.
		for i in range(0,len(data),factor*20):
			# Deriving the elements corresponding to the x value and converting them into a flaoat.
			x_byte = data[slice(i,i+4)]
			x_float = floatify_bytes(x_byte)

			y_byte = data[slice(i+y_offset,i+4+y_offset)]
			y_float = floatify_bytes(y_byte)

			z_byte = data[slice(i+z_offset,i+4+z_offset)]
			z_float = floatify_bytes(z_byte)
			# Summing the z values of the corresponding x/y/z values that satisfy the conditions.
			if -0.15<x_float<0.15 and -0.15<y_float<0.15 and 0.3<z_float<3.0:
				z_sum += z_float
				z_count += 1
		# Calculating the average and publishing.
		z_avg = Float32()
		z_avg.data = z_sum/z_count
		self.publisher.publish(z_avg)
		self.get_logger().info("Publishing... Average z value = %f" % z_avg.data)

#---- Function that converts the data into floats
def floatify_bytes(z_bytes):
	"""
	:param lst of four bytes as numbers between 0-255
	:return: the little endian float representation
	"""
	bits = bin(z_bytes[0])[2:].ljust(8, "0") + bin(z_bytes[1])[2:].ljust(8, "0") + bin(z_bytes[2])[2:].ljust(8, "0") + bin(z_bytes[3])[2:].ljust(8, "0")
	arr = bitarray(bits[::-1])
	return struct.unpack(">f", arr)[0]

#---- Set up parameters and nodes and start the ROS class
def main(args = None):
	rclpy.init(args = args)
	subscriber = SubscriberNode()
	rclpy.spin(subscriber)

	subscriber.destroy_node()
	rclpy.shutdown

if __name__ == '__main__':
	main()


