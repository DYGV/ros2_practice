# 演習1  
## 目的
- `Topic通信(Publish/Subscribe通信)`の基礎を理解する
- `rclpy`の基本的な関数・メソッドがどんなことをしているのかをふんわりと理解する
- (Pythonの基礎を復習する)

## 実装

### パッケージの作成
```bash
$ cd ~/dev_ws/src/
$ ros2 pkg create --build-type ament_python ex1
$ cd ex1
```

### ex1/talker.py
```python
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

"""
Talker: rclpyというライブラリのnode.pyに実装されているNodeクラスを継承する。
Nodeクラスの実装は
https://github.com/ros2/rclpy/blob/eloquent/rclpy/rclpy/node.py#L92
にある。
継承により、Talkerクラス内からNodeクラスに定義されたメソッドを呼ぶことができる。
"""


class Talker(Node):
    def __init__(self):
        """
        super()は親クラス、つまりNodeを表す。super.__init__()で親クラスのコンストラクタを呼び出す(インスタンス化)。
        Nodeクラスのコンストラクタを見ると、第一引数は"node_name: str"となっている。
        例えば、第一引数に"talker"と渡せば、このクラスは、talkerというノード名が割り当てられる。
        """
        super().__init__("talker")
        
        
        """
        ノードは作れたがトピックにメッセージを流すにはトピックの登録が必要である。
        create_publisherメソッドを使う。
        軽くcreate_publisherの実装を見た感じでは、rclpy.publisher.pyに定義されているPublisherというクラスのインスタンス化をして、
        Nodeクラス内publishersというリストに作ったインスタンスを保持している。
        これによって、1つのノードで複数のトピックに対してpublishできるようになっている。
        create_subscription()も似たようにSubscriptionクラスをリストに持つようにしている。
        Publisherクラスの実装: https://github.com/ros2/rclpy/blob/eloquent/rclpy/rclpy/publisher.py

        create_publisherの
        第1引数は、トピックに流すメッセージの型
        第2引数は、Publishするトピック名(送信先)
        第3引数は、キューサイズ
        (Subscriberの動作が重い場合はメッセージを取りこぼす可能性があるので、それが困る場合は高い値にしておくとよさそう?)

        戻り値はPublishクラスのインスタンスである。
        つまり、self.publisherという変数で、Publisherクラスに定義されているメソッドを呼ぶことができる。
        実装を見ると引数は他にもあるが、いずれもオプションである。
        今回は0-100までのいずれかの値を"numeric"というトピックにPublishし、
        Subscriberでそれを2倍することを考えるので、符号なしの8ビット整数で間に合う。
        """
        self.publisher = self.create_publisher(UInt8, "numeric", 10)
        
        # create_timerで使う変数
        timer_period = 1.0
        
        """
        タイマーを作る
        第1引数は、何秒カウントしてコールバック関数を発火させるか
        第2引数は、タイマーが0になったら発火させるコールバック関数
        実装を見ると引数は他にもあるが、いずれもオプションである。
        今回は1秒カウントごとにpublish_numericというメソッドが呼ばれる
        """
        self.create_timer(timer_period, self.publish_numeric)

    def publish_numeric(self):
        """
        メッセージ型UInt8クラスのインスタンス化
        メッセージ型の定義は/opt/ros/eloquent/share/std_msgs/msg/にあり、
        プログラムとしての実体は、IDL(インターフェース記述言語)で記述されている。(拡張子:.idl)
        UInt8の場合以下のような記述がされていた。
        
        module std_msgs {
            module msg {
                struct UInt8 {
                uint8 data;
                };
            };
        };
        
        std_msgsのモジュールにmsgというモジュールがあり、
        その中の構造体UInt8はdataというメンバ変数を持つ。
        つまり、UInt8というメッセージ型で通信するには
        PublisherがUInt8のdataに値を代入、Subscriberがdataを参照すればよい。
        UInt8のインスタンス化
        """
        msg = UInt8()
        
        # msg(UInt8)のdataというメンバに対してランダムな値を代入する。
        msg.data = random.randint(0, 100)
        # msgを"numeric"というトピックに対してPublishする。
        # publishメソッドの引数は、送信したいメッセージのみ。
        self.publisher.publish(msg)
        # どのノードが出力しているメッセージも合わせて出力してくれるので便利
        self.get_logger().info("Publishing: {}".format(msg))


def main(args=None):
    """
    rclpyパッケージをインポートした時(import rclpy)に使える関数は
    https://github.com/ros2/rclpy/blob/eloquent/rclpy/rclpy/__init__.py
    に定義されている。
    rclpyパッケージ全体の初期化関数initの引数はいずれもオプション
    """
    rclpy.init(args=args)
    
    # 上で定義したtalkerノードのインスタンス化
    talker = Talker()
    
    """
    以下のspin(talker)でtalkerを実際に動かす。
    rclpy.spinではタスクを管理するexecutor(実行機)の初期化処理とノードが立ち上がっているときブロッキングし続ける役割がありそう。
    spin()から軽く追ってみたけど、読み解くのが大変な箇所かもしれない。
    https://github.com/ros2/rclpy/blob/eloquent/rclpy/rclpy/__init__.py#L175
    最終的にTaskというクラスにたどり着き、そこでexecutorの設定を行っているような感じだった。

    読めてないけど、
    http://design.ros2.org/articles/node_lifecycle.html
    あたりを読むとROS全体の内部状態と動作がつかめるかも。
    """
    rclpy.spin(talker)
    
    """
    ノードの破壊
    https://github.com/ros2/rclpy/blob/eloquent/rclpy/rclpy/node.py#L1467
    でdestory_publisher()を呼び、
    destroy_publisher()では、PublisherをNodeクラスで管理しているリストから削除する。
    """
    talker.destory_node()
    
    # rclpyの初期化時に作成されたデータを破棄する
    # https://github.com/ros2/rclpy/blob/eloquent/rclpy/rclpy/__init__.py#L90
    rclpy.shutdown()


# このプログラムがトップレベル(エントリポイント)として実行されたならmainを実行する
if __name__ == "__main__":
    main()
```

### ex1/listener.py
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


class Listener(Node):
    def __init__(self):
        # 継承したNodeクラスの初期化
        # listenerというノード名を割り当てる
        super().__init__("listener")
        
        """
        ノードは作れたがトピックからメッセージを受信するにはトピックの登録が必要である。
        talkerノードの実装で"numeric"というトピックに数字を流すようにしたので、"numeric"から受信する。
        データを受け取るとdoubleというメソッドが呼ばれる。
        create_publisherのときのようにNodeクラスがsubscriptionsというリストでSubscriptionクラスを保持しているので、
        1つのノードで複数のsubscriptionすることもできるようになっている。

        また、諸々を端折って実際にコールバック関数を呼んでいるのは、executors.pyのawait_or_execute関数内で
        return callback(*args)っぽい。
        executor: https://github.com/ros2/rclpy/blob/eloquent/rclpy/rclpy/executors.py#L111
        create_subscription(): https://github.com/ros2/rclpy/blob/eloquent/rclpy/rclpy/node.py#L1120
        """
        self.create_subscription(UInt8, "numeric", self.double, 10)

    def double(self, msg):
        self.get_logger().info("{0}*2 = {1}".format(msg.data, msg.data * 2))


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
        "talker=ex1.talker:main",
        "listener=ex1.listener:main"
    ],
},
```
### launch/ex1_launch.py  
[launch_rosのnode.py](https://github.com/ros2/launch_ros/blob/eloquent/launch_ros/launch_ros/actions/node.py)のNodeクラスをみると、立ち上げるノードの指定やパラメータの設定などができるようになっている。パラメータの詳細は演習2で説明する。

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="ex1", node_executable="talker", output="screen"),
            Node(package="ex1", node_executable="listener", output="screen"),
        ]
    )
```
  
`talker`ノードは`numeric`トピックに対してメッセージを送り、`listener`ノードは`numeric`トピックからメッセージを受け取っていることがわかる。  
  
![rqt_graph](https://user-images.githubusercontent.com/8480644/155382817-d5a20229-99c9-4b3f-bf65-eca8e881cdcf.png)  
    
![実行画面](https://user-images.githubusercontent.com/8480644/155811363-53b16ca6-36b3-431b-ba8f-a9a00782f351.png)  
  
## 参考
- [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ros2/rclpy at eloquent](https://github.com/ros2/rclpy/tree/eloquent)
- [ros2/launch_ros at eloquent](https://github.com/ros2/launch_ros/tree/eloquent)
