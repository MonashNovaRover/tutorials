# !/usr/bin/env python3
"""
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Monash Nova Rover Team

ObstacleDetector subscribes to the /D435/depth/color/points
topic. With these point clouds it gets an average z value from the
points that meet the condition 0.3 < Z < 3.0, -0.15 < X < 0.15
and -0.15 < Y < 0.15. This average is published to the topic
/obstacle_proximity as a Float32.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
NODE: obstacle_detector
TOPICS:
  - /D435/depth/color/points [PointCloud2]
  - /obstacle_proximity [Float32]
ACTIONS: None
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PACKAGE: 	obstacle_detector.py
AUTHOR(S):	Lucas Hellyer
CREATION:	23/09/2021
EDITED:		28/09/2021
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TODO:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from floatify_bytes import floatify_bytes

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(PointCloud2, '/D435/depth/color/points', self.callback_func, 10)
        self.publisher = self.create_publisher(Float32, '/obstacle_proximity', 10)

    def callback_func(self, msg: PointCloud2) -> None:
        """
        Gets an average z value from the points in the point cloud
        that meet the condition 0.3 < Z < 3.0, -0.15 < X < 0.15
        and -0.15 < Y < 0.15. This average is published to the topic
        /obstacle_proximity as a Float32.
        :param msg: a point cloud
        :return: None
        """
        point_data = msg.data
        z_average = 0.0 # the average z value of points that meet a condition
        valid_points = 0 # no of points that meet condition

        uniform_sample = 100 # the number of points we examine to estimate z
        no_points = len(point_data) / 20 # within the point cloud each point is 20 bytes (1 bytes per element)
        if uniform_sample <= no_points:
            skip_interval = round(no_points / uniform_sample)
        else:
            skip_interval = 1

        for point_begin in range(0, len(point_data), 20 * skip_interval):  # iterates through points at interval
            x = floatify_bytes(point_data[point_begin:point_begin + 4]) # list slice to obtain x, then to float
            if -0.15 < x < 0.15:
                y = floatify_bytes(point_data[point_begin + 4: point_begin + 8])
                if -0.15 < y < 0.15:
                    z = floatify_bytes(point_data[point_begin + 8:point_begin + 12])
                    if 0.3 < z < 3.0:
                        valid_points += 1
                        z_average += z

        if valid_points == 0:
            z_average = 0.0
        else:
            z_average = z_average / valid_points

        # publish z_average
        msg = Float32()
        msg.data = z_average
        self.get_logger().info("Publishing: %s" % msg.data)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    detector = ObstacleDetector()
    rclpy.spin(detector)

    # gabage collection
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

