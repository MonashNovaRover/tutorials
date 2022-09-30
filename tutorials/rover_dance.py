from typing import Optional, List

from core.msg import DriveInput
from rclpy.node import Node
from rclpy.publisher import Publisher
import rclpy


class RoverDancePublisher(Node):
    def __init__(self):
        super().__init__('rover_dance_publisher_node')
        self.publisher: Publisher = self.create_publisher(DriveInput, '/control/drive_inputs', 10)
        self.create_timer(2, self.spin)

    def spin(self) -> None:
        self.publisher.publish(DriveInput(speed=0.5, steer=1.0))


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    rover_dance_publisher: RoverDancePublisher = RoverDancePublisher()
    rclpy.spin_once(rover_dance_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
