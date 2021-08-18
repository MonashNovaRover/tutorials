import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

class Sub(Node):
    def __init__(self):
        super().__init__('pose_node')
        self.subscription = self.create_subscription(Point, '/key_press', self.listener_callback, 10)
        self.pub = self.create_publisher(Point, "/pose", 10)
        self.position = Point() 
         

    def listener_callback(self, msg):
        self.position.x += msg.x
        self.position.y += msg.y
        self.position.z += msg.z
        self.pub.publish(self.position)

def main(args=None):
    rclpy.init(args=args)
    sub = Sub()
    rclpy.spin(sub)

if __name__ == '__main__':
    main()
