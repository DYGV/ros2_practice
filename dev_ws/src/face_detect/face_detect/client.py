import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Image

from face_msgs.srv import Face

"""
顔検出パッケージのクライアント側
要求された周波数・ビデオデバイスを用いて、
映像取得・グレースケール変換・サーバ側への送信をする。
サーバ側から顔が検出された座標・大きさが返ってきたら、
マーカーを引き、映像確認用のトピックに流す。
"""
class Client(Node):
    def __init__(self):
        super().__init__("client")
        self.declare_parameter("freq", 10)
        self.declare_parameter("video_device", -1)

        freq = self.get_parameter("freq").value
        video_device = self.get_parameter("video_device").value

        self.cam = cv2.VideoCapture(video_device)
        self.bridge = CvBridge()
        cb_group = ReentrantCallbackGroup()
        self.cli = self.create_client(Face, "face_detect", callback_group=cb_group)
        self.img_pub = self.create_publisher(Image, "image_display", 10)
        self.req = Face.Request()
        timer_period = 1.0 / freq
        self.timer = self.create_timer(
            timer_period, self.pub_callback, callback_group=cb_group
        )

    def pub_callback(self):
        ret, self.frame = self.cam.read()
        src_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        self.req.image = self.bridge.cv2_to_imgmsg(src_gray, "mono8")
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.draw)

    def draw(self, future):
        boxes = future.result()
        print(boxes)
        cv2.rectangle(
            self.frame,
            (boxes.x, boxes.y),
            (boxes.x + boxes.w, boxes.y + boxes.h),
            (255, 0, 255),
            2,
        )
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    client = Client()
    rclpy.spin(client)
    client.destroy_node()
    client.cam.release()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
