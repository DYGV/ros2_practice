# 演習3
## 目的
- 複数の`Subscriber`を動かして動作を理解する
- 同じ実行名のノードを別ノードとして立ち上げる方法を学ぶ  

## 仕様  
`演習2`で作成したパッケージを使い回し、複数の`Subscriber`で処理をする。  ランダムな数値`x`,`y`を`ex2`トピックに流すノードが1つと、`ex2`トピックから受信した数値`x`,`y`を2倍、3倍、4倍するノード3つを立ち上げることを考える。この仕様を満たすには、同じ実行名のノードを別のノード名として設定する(remap: 書き換える)方法と演習2で行ったパラメータの設定を行うことで実現する。  

## コマンドラインからノードを立ち上げる場合
`ros2 run <パッケージ名> <実行名> --ros-args -r __node:=<ノード名>`でノード名を指定した上で、ノードを立ち上げることができる。例えば、talkerを  
```bash
$ ros2 run ex2 talker
```
とし、listenerを  
```bash
$ ros2 run ex2 listener --ros-args -p N:=2 -r __node:=listener_1 &
$ ros2 run ex2 listener --ros-args -p N:=3 -r __node:=listener_2 &
$ ros2 run ex2 listener --ros-args -p N:=4 -r __node:=listener_3 &
```
とすることで、リスナーノードが3つ立ち上がる。  
![rqt_graph](https://user-images.githubusercontent.com/8480644/155809693-9fb40cd5-424f-4441-9f73-738a702718b8.png)  
![実行画面](https://user-images.githubusercontent.com/8480644/155809707-a6f5f8c4-7ad5-4222-8913-eb6b581c3d20.png)  

## launchファイルからノードを立ち上げる場合
[Nodeのコンストラクタ](https://github.com/ros2/launch_ros/blob/eloquent/launch_ros/launch_ros/actions/node.py#L60)の説明によれば、  
  
> If the node_name is not given (or is None) then no name is passed to  
> the node on creation and instead the default name specified within the  
> code of the node is used instead.    

ということなので、`node_name`を指定してやることで、ノード名が違う同じような動作をするノードを複製することができる。  

```python
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ex2",
                node_executable="talker",
                parameters=[{"rand_min": 100, "rand_max": 200, "freq": 2}],
                output="screen",
            ),
            Node(
                package="ex2",
                node_executable="listener",
                node_name="listener_1",
                parameters=[{"N": 2}],
                output="screen",
            ),
            Node(
                package="ex2",
                node_executable="listener",
                node_name="listener_2",
                parameters=[{"N": 3}],
                output="screen",
            ),
            Node(
                package="ex2",
                node_executable="listener",
                node_name="listener_3",
                parameters=[{"N": 4}],
                output="screen",
            ),
        ]
    )
 ```  
コマンドラインから実行したときと同じように別ノードで処理ができる。  
![rqt_graph](https://user-images.githubusercontent.com/8480644/155553873-365304be-3351-48f3-a5a2-bf372c13987a.png)


## 参考
- [ros2/rclpy at eloquent](https://github.com/ros2/rclpy/tree/eloquent)
- [Passing ROS arguments to nodes via the command-line](https://docs.ros.org/en/foxy/How-To-Guides/Node-arguments.html)
