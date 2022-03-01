import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


class Talker(Node):
    def __init__(self):
        super().__init__("talker")
        self.publisher = self.create_publisher(UInt8, "ex1", 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.pub_callback)

    def pub_callback(self):
        msg = UInt8()
        msg.data = random.randint(0, 100)
        self.publisher.publish(msg)
        self.get_logger().info("Publishing: {}".format(msg))


def main(args=None):
    rclpy.init(args=args)

    talker = Talker()
    rclpy.spin(talker)
    talker.destory_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
