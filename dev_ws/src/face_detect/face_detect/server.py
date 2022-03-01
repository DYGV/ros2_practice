import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

from face_msgs.srv import Face

"""
顔検出パッケージのサーバ側
送られてきた画像を顔検出器にかけて、顔の座標・大きさを返す
"""
class Server(Node):
    def __init__(self):
        super().__init__("service")
        self.declare_parameter("cascade_path", "")
        self.srv = self.create_service(Face, "face_detect", self.callback)
        self.bridge = CvBridge()
        cascade_path = self.get_parameter("cascade_path").value
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        self.boxes = [0, 0, 0, 0]

    def callback(self, msg, pos):
        image = self.bridge.imgmsg_to_cv2(msg.image, "mono8")
        boxes = self.face_cascade.detectMultiScale(image)
        if len(boxes) != 0:
            self.boxes = boxes[0]
        pos.x = int(self.boxes[0])
        pos.y = int(self.boxes[1])
        pos.w = int(self.boxes[2])
        pos.h = int(self.boxes[3])
        return pos


def main(args=None):
    rclpy.init(args=args)
    server = Server()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
