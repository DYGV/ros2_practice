import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


class Listener(Node):
    def __init__(self):
        super().__init__("listener")
        self.create_subscription(UInt8, "ex1", self.sub_callback, 10)

    def sub_callback(self, msg):
        self.get_logger().info("{0}*2 = {1}".format(msg.data, msg.data * 2))


def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destory_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
