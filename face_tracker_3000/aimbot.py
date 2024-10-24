import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point


class AimBot(Node):

    def __init__(self):
        super().__init__('aimbot')
        self.subscription = self.create_subscription(Point, 'face_offset', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"{msg.x}, {msg.y}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = AimBot()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
