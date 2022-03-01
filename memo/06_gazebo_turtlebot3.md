# GazeboでTurtleBot3の走行と障害物回避
## 概要  
[Gazebo](http://gazebosim.org/)は、ROSと連携してロボットを動かせるシミュレータである。今回は、`Gazebo`を使って、ロボットを動かす・障害物の前で止まるタスク・障害物を回避するタスクの実装を目指す。  

## 目的
- `Gazebo`の基本的な操作を学ぶ
- `TurtleBot3`を操作する方法を学ぶ
- センサの値から`TurtleBot3`の操作を決定する簡単なプログラムを作れるようにする

## とりあえず動かしてみる  
初回の起動に時間がかかるかもしれないが、とにかく待つ。  
```bash
$ gazebo & # Gazeboのバックグラウンド起動
```
すでにWorldがあって、それを読み込むときは  
```bash
$ gazebo hoge.world &
```
のようにする。  

### Worldを作成する  
ロボットと適当な壁と障害物を配置する。`TurtleBot3`のモデルは`Waffle`にした。Worldファイルは[このよう](https://github.com/DYGV/ros2_practice/blob/master/simple.world)になった。ファイルは適当な場所に保存する。  後述する障害物検知で使う物体`unit_cylinder`も配置した。初めは`coke can`でも良いと思ったが、センサの位置によっては物体にレーザーが照射されないので、物体の高さには注意が必要であることが後からわかった(今回は2Dの距離センサを使うので水平方向にしか飛ばない？)。

![gazebo_world](https://user-images.githubusercontent.com/8480644/154824429-3ad4aa84-55bb-4a99-86a4-589326fc5220.png)

`TurtleBot3`の配置が完了した時点で、以下のようにトピックが作られていることがわかる。  
```bash
$ ros2 topic list 
/camera/camera_info
/camera/image_raw
/clock
/cmd_vel
/imu
/joint_states
/odom
/parameter_events
/rosout
/scan
/tf
```

### キーボード操作で動かしてみる  
[Dockerfile](https://github.com/DYGV/ros2_practice/blob/master/Dockerfile)で立ち上げたコンテナ環境を使っている場合は必要なパッケージは既に入っているはずである。  
ない場合は、  
```bash
$ sudo apt install ros-eloquent-teleop-twist-keyboard
```
で、パッケージをインストールすると、`/opt/ros/eloquent/lib/`に`turtlebot3_teleop`というパッケージが入る。  

```bash
$ ros2 run turtlebot3_teleop teleop_keyboard
```
で`TurtleBot3`をキーボード操作することができる。teleopの操作は以下のとおりである。  
  
| 操作 | キー |
----|----
| 前進 | w |
| 後退 | x |
| 右折 | d |
| 左折 | a |
| 停止 | s |

`teleop`の起動とrqtでカメラ画像を`subscribe`すると以下のような`rqt_graph`となる。  

![rqt_graph](https://user-images.githubusercontent.com/8480644/154825461-09681539-1752-4b54-935a-017789b73737.png)


## 障害物への衝突回避(1)

`TurtleBot3`が運転中に前方にある障害物を検知し、ある距離まで近づいた時点で停止するタスクを考える。今回は、`TurtleBot3`に付属している[2D距離センサ](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/)を用いて障害物を検知する。(「検知」とは、距離が`inf`でない数値とする。)

### 利用するトピック 
このタスクを満たすために、距離の取得と`TurtleBot3`の移動をしたい。この距離センサから距離を取得するには`sensor_msgs/msg/LaserScan`というメッセージ型を`/scan`トピックから`subscribe`すれば良い。また、移動するためには、`/cmd_vel`というトピックに対して`Twist`というメッセージ型で`publish`すれば良い。
```bash
$ ros2 topic info /scan
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Subscriber count: 0
$ ros2 topic info /cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscriber count: 1
```
### `LaserScan`について
メッセージ型の詳細は、`ros2 interface show`で取得できる。以下はLaserScanのメンバである。
```bash
$ ros2 interface show sensor_msgs/msg/LaserScan
(コメントなど省略済み)
std_msgs/Header header
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```
今回使うメンバは
```bash
float32 range_min
float32 range_max
float32[] ranges
```
の3つで、いずれも単位は`メートル`である。  
`range_min`と`range_max`は検出できた距離の最小(最大)値である。`ranges`は360度スキャンした結果(距離)が収められている。配列のインデックスが角度に対応している。ただし、スキャンできる範囲に物体が検出されなかった角度は距離が`inf`となる。  

### `Twist`について
以下は`Twist`のメンバである。  

```bash
$ ros2 interface show geometry_msgs/msg/Twist  
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
```
Vector3はC++などの動的配列のベクターではなく、`X軸周りの回転(Roll), Y軸周りの回転(Pitch), Z軸周りの回転(Yaw)`の値を格納する型である。`linear`は並進方向、`angular`は回転方向で、値の単位は`メートル毎秒`である。
例えば`TurtleBot3`のタイヤをx軸の正の方向に動かしたい(前進したい)なら、  
```python
twist = Twist()
twist.linear.x = 0.1
```
のように使用する。この`twist`を`/cmd_vel`に対して`publish`することでロボットを動かすことができる。  
  
### 実装  
いつもどおりパッケージを作成し、実装する。実装は以下の通りで、`setup.py`への記述を忘れないようにする。ロボットの動作は前進のみで、前方30度のスキャン結果のいずれかに0.5 [m]以下が含まれていたら停止する。  

```python
import numpy
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


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
        ranges = numpy.array(msg.ranges)
        ranges = numpy.where(ranges < msg.range_min, msg.range_max, ranges)
        ahead_object_distance = min(min(ranges[0:15]), min(ranges[345:360]))
        if ahead_object_distance < 0.5:
            self.twist.linear.x = 0.0
            print("stop")
        else:
            self.twist.linear.x = 0.3
            print(
                "{:.3f}[m] to the nearest forward obstacle".format(
                    ahead_object_distance
                )
            )
        self.pub_twist.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    drive = Drive()
    rclpy.spin(drive)
    drive.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```
`Gazebo`を立ち上げたまま、実装したパッケージを立ち上げると`rqt_graph`は以下のようになる。先程実装した`/drive`ノード`/scan`トピックからスキャン結果を`subscribe`し、機体の速度情報(Twist)を`/cmd_vel`トピックへ`publish`している様子がわかる。  
![rosgraph](https://user-images.githubusercontent.com/8480644/154828100-fbe29708-5ac9-4dc8-937e-72ad671fa335.png)

以下のように動作する。  

https://user-images.githubusercontent.com/8480644/154829002-fd8a0812-61fe-4cae-a21b-7565e5335a62.mp4


## 障害物への衝突回避(2)  
次は、障害物を回避する、というタスク。

```python
import time

import numpy
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


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

    def rotate(self, center_index, argmin):
        self.twist.linear.x = 0.1
        # 距離の最小値のインデックスが
        # 前方中央のインデックスより左にあるなら
        # 右へ回転する
        if argmin <= center_index:
            self.twist.angular.z = -0.2
        else:
            self.twist.angular.z = 0.2
        print("rotate")
        self.pub_twist.publish(self.twist)

    def scan_callback(self, msg):
        ranges = numpy.array(msg.ranges)
        ranges = numpy.where(ranges < msg.range_min, msg.range_max, ranges)
        ahead_object_distance_concat = numpy.concatenate(
            (ranges[345:360], ranges[0:15])
        )
        ahead_object_distance = min(ahead_object_distance_concat)

        if ahead_object_distance < 0.5:
            argmin = numpy.argmin(ahead_object_distance_concat)
            center_index = len(ahead_object_distance_concat) // 2
            self.rotate(center_index, argmin)
            time.sleep(0.5)
        else:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0
            self.pub_twist.publish(self.twist)
        print("{:.3f}[m] to the nearest forward obstacle".format(ahead_object_distance))


def main(args=None):
    rclpy.init(args=args)
    drive = Drive()
    rclpy.spin(drive)
    drive.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

回避できたことが確かめられた。しかし、回避する方向を迷っているので、rotate()の処理が甘いのかもしれない？   

https://user-images.githubusercontent.com/8480644/154853387-17f842be-e8e1-496f-b3e0-a3800d37830f.mp4

## 参考
- [Gazebo](http://gazebosim.org/)
- [TurtleBot3 LDS-01](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/)
- [ROSの勉強　第11弾：センシング-LaserScan](https://qiita.com/Yuya-Shimizu/items/413b57b0305597be35be)
