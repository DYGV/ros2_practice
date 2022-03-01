import random

import rclpy
from rclpy.node import Node

from ex2_msgs.msg import Ex2


class Talker(Node):
    def __init__(self):
        super().__init__("talker")
        # Publishする数値の最小値
        self.declare_parameter("rand_min", 0)
        # Publishする数値の最大値
        self.declare_parameter("rand_max", 100)
        # 1秒間に何回Publishしたいか(周波数)
        self.declare_parameter("freq", 1)
        # パラメータにセットされた値をインスタンス変数として保持しておく
        self.rand_min = self.get_parameter("rand_min").value
        self.rand_max = self.get_parameter("rand_max").value
        # タイマーの秒数として使いたいので周期にする
        timer_period = 1.0 / self.get_parameter("freq").value
        # Ex2という型を流すトピックex2
        self.publisher = self.create_publisher(Ex2, "ex2", 10)
        self.create_timer(timer_period, self.publish_callback)

    def publish_callback(self):
        msg = Ex2()
        msg.x = random.randint(self.rand_min, self.rand_max)
        msg.y = random.randint(self.rand_min, self.rand_max)
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
