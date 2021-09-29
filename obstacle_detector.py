

#!/usr/bin/env python3

'''
Name : kelvin valani
Nova rover induction task
'''


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from std_msgs.msg import String
import struct
class Subscriber (Node):
    
    def __init__(self):
        '''
        Intialising the subscriber and the publisher  that will publish immeditely after receving data and 
        perofrming necessary calculations
        '''
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(PointCloud2, '/D435/depth/color/points', self.callback_func, 10)
        self.publisher = self.create_publisher(Float32,'obstacle_proximity',10)
    def callback_func (self, msg):
        '''
	receives the data field from the point cloud and calcualtes the average distance (zvalue average)
        from the byte arrays that are given.
        '''
        sumZvals = 0
        nZvals = 0
        avg =  Float32()
        for i in range(0,len(msg.data),20):
                x = floatify_byte(msg.data[i:i+4])
                y = floatify_byte(msg.data[i+4:i+8])
                z = floatify_byte(msg.data[i+8:i+12])
              
                if((-0.15<x<0.15) and (-0.15<y<0.15) and (0.3<z<3.0)):
                        nZvals = nZvals + 1
                        sumZvals = sumZvals + z
        avg.data = sumZvals/nZvals
        output ="current distance from obstacle is " +  str(avg.data)
        self.publisher.publish(avg)
        self.get_logger().info(output)

                

def floatify_byte(arr):
        '''
        converts the array of values into a byte array and then converts it into a small elidian form float
        '''
	
        return struct.unpack('<f', bytearray(arr))[0]



def main ():
    rclpy.init()
    subscriber = Subscriber()
    rclpy.spin(subscriber)

if __name__ == "__main__":
    main()
