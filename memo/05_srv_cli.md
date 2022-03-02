# サービス通信で人の顔検出を行う
## 概要  
演習2で独自メッセージを作成したときに参考にした[Creating custom ROS 2 msg and srv files](https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html)で、
`トピック通信`ではなく、`サービス通信`という通信方法ががあったため、これを試す。  

## 目的
- トピック通信との違いを学ぶ
- 独自メッセージの作り方を学ぶ
- 独自メッセージを使ったROSパッケージの作り方を学ぶ
- launchファイルの作り方を学ぶ
- パラメータの作り方を学ぶ

## トピック通信との違い  
　`トピック通信`は、`Publisher`は`Topic`に対してメッセージを送信し、`Subscriber`は`Topic`からメッセージをもらってくる通信モデルである。
一方で、`サービス通信`は、`Client`から`Server`に要求(リクエスト)、`Server`から`Client`に対して応答(レスポンス)し、相互にメッセージを送信することができる。いわゆるソケット通信のクライアント・サーバモデルみたいなもの。  
　考え方の違いを言うなら、`トピック通信`は、トピックを介しているので、`Publisher`から見て`購読者`(購読しているノード)は誰でもいいし、`Subscriber`から見て`出版者`(出版しているノード)は誰でもいいという考え。しかし、`サービス通信`では、`Client`はデータを処理してほしい相手`Server`が決まっていて、`Server`はレスポンスする相手`Client`が決まっているという考え方ができる。  
　つまり、`トピック通信`は、1つの`Publisher`に対し複数の`Subscriber`がいるので一対多の通信で、`サービス通信`は、要求から応答までを1つのトランザクションと見なせば、`サービス通信`は一対一の通信といえる？  

## 作成する題材  
`サービス通信`でカメラを用いた人の顔の検出を行う。処理手順は以下のとおりである。 
  
- Client 
  1. カメラから画像を入力し、画像をグレースケールに変換
  2. グレースケール画像を`Server`に送信
  3. `Server`から座標と大きさの受信
  4. 座標と大きさからマーカー描画
  5. Image型のトピックに画像送信(確認用)
- Server
  1. グレースケール画像受信
  2. 顔の検出(今回は学習済みのカスケード分類器を使う)
  3. 検出した座標と大きさを`Client`に送信


## 作成する手順
はじめに、`サービス通信`で用いる独自メッセージを作成する。その後、`Server`と`Client`を作成する。  

## `サービス通信`で用いる独自メッセージの作成  

### パッケージの作成  
```bash
$ ros2 pkg create --build-type ament_cmake face_msgs && cd face_msgs/
```  
### 独自メッセージの定義  
メッセージには、`Client`から送信する画像image、`Server`から送信する顔が検出された左上のx,y座標とその大きさw,hを含める。
`---`をデリミタとして`Client`が`Server`に送信するメッセージ(リクエスト)を上に、`Client`が`Server`から受信するメッセージ(レスポンス)を下に記述すれば良い。また、後述する顔検出で、検出する関数が`numpy.int32`を返すので、それにビット幅を合わせておく。
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

### `CMakeLists.txt`への追記  
ビルドで独自メッセージを作成するのに`CMakeLists.txt`へその情報を追加する必要がある。23行目あたりに以下を追記する。  
```CMake
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
      "srv/Face.srv"
      DEPENDENCIES sensor_msgs)
```
### `package.xml`への追記  
14行目あたりに以下を追記する  
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
### ビルド
いつもどおり`colcon build`でOK  
ビルドが通ったら、 以下のようにインターフェースができたか確認する。  
``` bash
$ . install/setup.bash
$ ros2 interface list | grep face_msgs
face_msgs/srv/Face
```

## `サービス通信`でのアプリケーションの作成    
`Server`と`Client`が実装すべき処理は`作成する題材`で記述したとおりである。  
また、パラメータとして、`Server`ではカスケード分類器のファイルパス、`Client`ではカメラのデバイス番号と顔検出処理の周波数を指定する。  
### パッケージの作成  
```bash 
$ ros2 pkg create --build-type ament_python face_detect && cd face_detect/
```
### カスケード分類器のダウンロード  
```bash
$ wget https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml
```
  
### face_detect/server.py  

```python
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

    def callback(self, msg, pos):
        boxes = [0, 0, 0, 0]
        image = self.bridge.imgmsg_to_cv2(msg.image, "mono8")
        detection_boxes = self.face_cascade.detectMultiScale(image)
        if len(detection_boxes) != 0:
            boxes = detection_boxes[0]
        pos.x = int(boxes[0])
        pos.y = int(boxes[1])
        pos.w = int(boxes[2])
        pos.h = int(boxes[3])
        return pos


def main(args=None):
    rclpy.init(args=args)
    server = Server()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### face_detect/client.py  

```python
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
```


### launch/face_detect_launch.py

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
                node_executable="server",
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


### `package.xml`への追記  
先程作った独自メッセージを使うために`package.xml`の14行目あたりに以下のように追記する。  

```xml
  <build_depend>face_msgs</build_depend>
  <exec_depend>face_msgs</exec_depend>
```
### `setup.py`への追記
`setup.py`に以下を追記する。  

```python
import os
from glob import glob
```
data_filesに以下を追記する。  

```python
(os.path.join("share", package_name), glob("launch/*_launch.py")),
```
さらに、entry_pointsに以下を追記する。  
```python
"client=face_detect.client:main", "server=face_detect.server:main",
```
### ビルド  
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
  
### 起動  
```bash
$ ros2 launch face_detect face_detect_launch.py 
```
検出してマーカーが引かれているかの確認は、`client`が確認用のトピック`image_display`に画像をpublishしているので、rqtでそれを確認する。  
![rqt_graph](https://user-images.githubusercontent.com/8480644/155850635-4ca410a1-ed2c-4846-96b3-6d2aefecbf0c.png)

動作は以下のようになった  
\*塗りつぶし処理後  

https://user-images.githubusercontent.com/8480644/154873004-270db3d3-d0a7-4fa1-81e6-0e058b4f280c.mp4


## 参考
- [Understanding ROS 2 services](https://docs.ros.org/en/foxy/Tutorials/Services/Understanding-ROS2-Services.html)
- [Writing a simple service and client (Python)](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Service-And-Client.html)
- [Creating custom ROS 2 msg and srv files](https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html)
- [Haar Cascadesを使った顔検出](http://labs.eecs.tottori-u.ac.jp/sd/Member/oyamada/OpenCV/html/py_tutorials/py_objdetect/py_face_detection/py_face_detection.html)

