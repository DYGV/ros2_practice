import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import numpy as np


class Drive(Node):
    def __init__(self):
        super().__init__("drive")
        self.create_subscription(
            LaserScan, "scan", self.scan_callback, qos_profile_sensor_data
        )
        self.pub_twist = self.create_publisher(Twist, "cmd_vel", 10)
        self.twist = Twist()
        self.stop_twist = Twist()

    def __del__(self):
        self.pub_twist.publish(self.stop_twist)

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.where(ranges < msg.range_min, msg.range_max, ranges)
        ahead_object_distance = min(min(ranges[0:30]), min(ranges[330:360]))
        print(ahead_object_distance)
        if ahead_object_distance < 0.5:
            self.twist.linear.x = 0.0
        else:
            self.twist.linear.x = 0.1
        self.twist.angular.z = 0.0
        self.pub_twist.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    drive = Drive()
    rclpy.spin(drive)
    drive.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
