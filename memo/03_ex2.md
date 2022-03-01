# 演習2
## 目的
- 独自メッセージの作り方を学ぶ
- パラメータの作り方、設定の仕方を学ぶ

## 仕様
演習1のようなシンプルなシステムを作る。   

- 2つの数値`x`,`y`を格納できるメッセージ型を作り`Publish/Subscribe通信`をする
- `Publisher`は、2つのランダムな数値`x`,`y`をトピックに流す
- `Subscriber`は、トピックから数値をもらい、`x`,`y`をそれぞれN倍し、画面に出力する
- `Publisher`側のパラメータ
  - `rand_min`: 取りうるランダムな数値の下限値
  - `rand_max`: 取りうるランダムな数値の上限値
  - `freq`: 1秒間に`Publish`する回数
- `Subscriber`側のパラメータ
  - `N`: 何倍するか

## 独自メッセージを定義するパッケージの作成  
### パッケージを作る
```bash
$ cd ~/dev_ws/src/
$ ros2 pkg create --build-type ament_cmake ex2_msgs
$ cd ex2_msgs
$ mkdir msg
```
### メッセージ型を定義する
msg/Ex2.msg
```
int32 x
int32 y
```
### `package.xml`の編集
演習1で説明したとおり、メッセージ型はIDLという記述言語で記述されるため、これを生成するために必要な依存パッケージを追記する。
``` xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
### `CMakeLists.txt`の編集  
ビルド時にメッセージ型を作るように、以下を追記する。
```CMake
# メッセージ型の生成で必要な依存パッケージ
find_package(rosidl_default_generators REQUIRED)
# 作成したメッセージ型の定義ファイル(.msg)からIDLを生成し、実際に使えるようにする
rosidl_generate_interfaces(${PROJECT_NAME} "msg/Ex2.msg")
```
### ビルド  
```bash
$ colcon build
```  
### 確認  
```bash
$ ros2 interface show ex2_msgs/msg/Ex2 
int32 x
int32 y
```
## 作成した独自メッセージでPub/Sub通信をするパッケージの作成  
### パッケージを作る   
```bash
$ ros2 pkg create --build-type ament_python ex2 
```
### ex2/talker.py  
```python
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
```  

### ex2/listener.py  
```python
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
``` 
### setup.py
`setup.py`内のsetup(...)に以下を追記する。
```python
  entry_points={
      'console_scripts': [
          "talker=ex2.talker:main",
          "listener=ex2.listener:main"
      ],
  },
```
talkerノードはex2トピックに対してメッセージを送り、listenerノードはex2トピックからメッセージを受け取っていることがわかる。  
![rqt_graph](https://user-images.githubusercontent.com/8480644/155551716-ce66c577-61b7-448a-a5dc-a1da980ca080.png)

  
## パラメータの値の設定
`talker.py`と`listener.py`で定義したパラメータは、コマンドライン引数として設定する方法とlaunchファイルから設定する方法がある。どちらもパラメータを直接指定する方法と、YAMLファイルに記述されたパラメータ設定を読ませる方法の2通りがある。
### コマンドライン引数から個別に指定する場合  
`ros2 run <パッケージ名> <実行名> --ros-args -p <パラメータ名>:=<値> -p <パラメータ名>:=<値> ...`  
```bash
$ # Listenerの何倍するかというパラメータを3とする例
$ ros2 run ex2 listener --ros-args -p N:=3
$ # ランダムな数値の下限値を100、最大値を200、1秒間に2回Publishする例
$ ros2 run ex2 talker --ros-args -p rand_min:=100 -p rand_max:=200 -p freq:=2
```  
![実行画面](https://user-images.githubusercontent.com/8480644/155811108-ac527607-db4f-42bb-91f3-95d7558598ac.png)  

### コマンドライン引数からYAMLファイルを使って指定する場合  
[ROS2 rcl YAML paramter parser](https://github.com/ros2/rcl/blob/eloquent/rcl_yaml_param_parser/README.md)に記載のフォーマットに従ったYAMLファイルを読ませる。  
`ros2 run <パッケージ名> <実行名> --ros-args --params-file <YAMLファイルのパス>`  

```yaml
talker:
  ros__parameters:
    rand_min: 100
    rand_max: 200
    freq: 2

listener:
  ros__parameters:
    N: 3
```
例えば、上記のようなYAMLファイル(ex2_params.yaml)なら、  
```bash
$ ros2 run ex2 talker --ros-args --params-file ./ex2_params.yaml 
$ ros2 run ex2 listener --ros-args --params-file ./ex2_params.yaml 
```
とすればよい。また、パラメータをYAMLファイル形式で出力したいときは、ノードを立ち上げた状態で、`ros2 param dump /talker`とすればよい。パラメータが多く、手打ちでYAMLの構造を作るのが大変なときに有用だと考えられる。  

### launchファイルから指定する場合  
[launch_rosのnode.pyにあるNodeクラス](https://github.com/ros2/launch_ros/blob/eloquent/launch_ros/launch_ros/actions/node.py#L57)のコンストラクタを見ると、`parameters`という引数がある。  

>    def \_\_init\_\_(  
>        self, \*,  
>        node_executable: SomeSubstitutionsType,  
>        package: Optional[SomeSubstitutionsType] = None,  
>        node_name: Optional[SomeSubstitutionsType] = None,  
>        node_namespace: SomeSubstitutionsType = '',  
>        parameters: Optional[SomeParameters] = None,  
>        remappings: Optional[SomeRemapRules] = None,  
>        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,  
>        \*\*kwargs  
>    ) -> None:     
  
`parameters`の説明には、  

> The parameters are passed as a list, with each element either a yaml file that contains parameter rules (string or pathlib.Path to the full path of the file), or a dictionary that specifies parameter rules.
 
ということなのでリストで、YAMLファイルのパスを渡すか、パラメータが載った辞書形式(key-value)で渡せばよい。  
```python
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ex2",
                node_executable="talker",
                parameters=["/home/eisuke/dev_ws/src/ex2/ex2_params.yaml"],
                # parameters=[{"rand_min": 100, "rand_max": 200, "freq": 2}],
                output="screen",
            ),
            Node(
                package="ex2",
                node_executable="listener",
                parameters=["/home/eisuke/dev_ws/src/ex2/ex2_params.yaml"],
                # parameters=[{"N": 3}],
                output="screen",
            ),
        ]
    )
```

## 参考
- [Creating custom ROS 2 msg and srv files](https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html)
- [Understanding ROS 2 parameters](https://docs.ros.org/en/foxy/Tutorials/Parameters/Understanding-ROS2-Parameters.html)
- [ros2/rclpy at eloquent](https://github.com/ros2/rclpy/tree/eloquent)
- [ros2/rcl at eloquent](https://github.com/ros2/rcl/tree/eloquent)
