import rclpy
from rclpy.node import Node

from ex2_msgs.msg import Ex2


class Listener(Node):
    def __init__(self):
        super().__init__("listener")
        # 何倍するかのパラメータの宣言
        self.declare_parameter("N", 1)
        # パラメータにセットされた値をインスタンス変数として保持しておく
        self.n = self.get_parameter("N").value
        self.create_subscription(Ex2, "ex2", self.times, 10)

    def times(self, msg):
        self.get_logger().info("{0}*{1} = {2}".format(msg.x, self.n, msg.x * self.n))
        self.get_logger().info("{0}*{1} = {2}".format(msg.y, self.n, msg.y * self.n))


def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destory_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
