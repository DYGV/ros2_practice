演習2で独自メッセージを作成したときに参考にした[Creating custom ROS 2 msg and srv files](https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html)で、
`publisher/subscriber通信`ではなく、`service/client通信`という通信方法ががあったため、これを試す。
## publisher/subscriber通信との違い
`publisher/subscriber通信`では、`publisher`は`topic`に対してメッセージを送信し、`subscriber`は`topic`からメッセージをもらってくる通信モデルである。
一方で、`service/client通信`では、`service`から`client`に、`client`から`service`に対して、相互にメッセージを送信することができる。いわゆるソケット通信のクライアント・サーバモデルみたいなもの。

## 作成する題材
`service/client通信`でカメラを用いた人の顔の検出を行う。処理手順は以下のとおりである。
- client 
  1. カメラから画像を入力し、画像をグレースケールに変換
  2. グレースケール画像を`service`に送信
  3. `service`から座標と大きさの受信
  4. 座標と大きさからマーカー描画
  5. Image型のトピックに画像送信(確認用)
- service
  1. グレースケール画像受信
  2. 顔の検出(今回はハールカスケードを使う)
  3. 検出した座標と大きさを`client`に送信


## 作成する手順
はじめに、`service/client通信`で用いる独自メッセージを作成する。その後、`service`と`client`を作成する。  

## `service/client通信`で用いる独自メッセージの作成
1. パッケージの作成  
```bash
$ ros2 pkg create --build-type ament_cmake face_msgs && cd face_msgs/
```  
2. 独自メッセージの定義  
メッセージには、`client`から送信する画像image、`service`から送信する顔が検出された左上のx,y座標とその大きさw,hを含める。
`---`をデリミタとして`service`が`client`に送信するメッセージを上に、`service`が`client`から受信するメッセージを下に記述すれば良い。また、後述する顔検出で、検出する関数が`numpy.int32`を返すので、それにビット幅を合わせておく。
```bash
$ mkdir srv && cd srv
$ echo -e \
"sensor_msgs/Image image\n\
---\n\
int32 x\n\
int32 y\n\
int32 w\n\
int32 h"\
> Face.srv
```

3. `CMakeLists.txt`への追記  
ビルドで独自メッセージを作成するのに`CMakeLists.txt`へその情報を追加する必要がある。23行目あたりに以下を追記する。  
```CMake
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/Face.srv"
        DEPENDENCIES sensor_msgs)
```
4. `package.xml`への追記  
14行目あたりに以下を追記する  
```xml
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```
5. ビルド  
いつもどおり`colcon build`でOK  
ビルドが通ったら、 以下のようにインターフェースができたか確認する。  
  ``` bash
  $ . install/setup.bash
  $ ros2 interface list | grep face_msgs
  face_msgs/srv/Face
  ```
  
## `service/client通信`でのアプリケーションの作成    
`service`と`client`が実装すべき処理は[作成する題材](#作成する題材)で記述したとおりである。  
また、パラメータとして、`service`ではカスケード分類器のファイルパス、`client`ではカメラのデバイス番号と顔検出処理の周波数を指定する。  
1. パッケージの作成  
 ```bash 
 $ ros2 pkg create --build-type ament_python face_detect && cd face_detect/
 ```
2. カスケード分類器のダウンロード  
```bash
$ wget https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml
```
3. 実装  
 service.py、client.py、launch.pyの3つを以下のようにした。  
 
  - face_detect/service.py  
```python:service.py
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

from face_msgs.srv import Face

"""
顔検出パッケージのサービス側
送られてきた画像を顔検出器にかけて、顔の座標・大きさを返す
"""
class Service(Node):
    def __init__(self):
        super().__init__("listener")
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
    service = Service()
    rclpy.spin(service)
    service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```


  - face_detect/client.py
```python:client.py
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
映像取得・グレースケール変換・サービス側への送信をする。
サービス側から顔が検出された座標・大きさが返ってきたら、
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
```


  - launch/face_detect_launch.py
  
```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="face_detect",
                node_executable="client",
                output="screen",
                parameters=[{"video_device": -1, "freq": 30}],
            ),
            Node(
                package="face_detect",
                node_executable="service",
                output="screen",
                parameters=[
                    {
                        "cascade_path": "/home/eisuke/dev_ws/src/face_detect/haarcascade_frontalface_default.xml"
                    }
                ],
            ),
        ]
    )
```


4. `package.xml`への追記  
先程作った独自メッセージを使うために`package.xml`の14行目あたりに以下のように追記する。  
```xml
  <build_depend>face_msgs</build_depend>
  <exec_depend>face_msgs</exec_depend>
```
5. `setup.py`への追記  

`setup.py`に以下を追記する。  
```python
import os
from glob import glob
```
また、data_filesに以下を追記する。  
```python
(os.path.join("share", package_name), glob("launch/*_launch.py")),
```
さらに、entry_pointsに以下を追記する。  
```python
"client=face_detect.client:main", "service=face_detect.service:main",
```
6. ビルド  
```bash
$ colcon build
$ . install/setup.bash
```
もし、ビルド時に`setup.py`関係で警告がうるさいときは`setup.py`に   
```python
import warnings
warnings.filterwarnings("ignore")
```  
とすればよい。  
7. 起動  
 ```bash
$ ros2 launch face_detect face_detect_launch.py 
```
検出してマーカーが引かれているかの確認は、`client`が確認用のトピック`image_display`に画像をpublishしているので、rqtでそれを確認する。
