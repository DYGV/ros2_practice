# 独自メッセージを使ったpub/sub通信
 独自メッセージ型を使って通信する。独自メッセージ用のパッケージとpub/sub通信用のパッケージを分けて作成する。
## 独自メッセージ用のパッケージを作成する  
1. パッケージの作成  
 ビルドタイプをament_cmakeにすることに注意  
 `ros2 pkg create --build-type ament_cmake ex2_msgs && cd ex2_msgs/`
2. msgファイルの作成  
 ` mkdir msg`  
 `vi msg/Person.msg`  
 Person.msgの内容は以下のようにした  
 ```
 string name
 string gender
 ```
5.package.xmlの編集
以下を追記
``` xml
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
 ```
4. CMakeLists.txtの編集  
 以下を追記
 ```CMake
 # メッセージの生成で必要な依存パッケージ
find_package(rosidl_default_generators REQUIRED)
# 作成したメッセージ型の定義ファイル(.msg)から実際に使えるようにする
rosidl_generate_interfaces(${PROJECT_NAME} "msg/Person.msg")
```
5. ビルド  
  `colcon build`  
6. 確認  
　`ros2 interface list | grep ex2_msgs`
 で
 `ex2_msgs/msg/Person`と出力されれば、メッセージが作られていることがわかる。
## 作成した独自メッセージでpub/sub通信をするパッケージを作成する
1. パッケージの作成  
`ros2 pkg create --build-type ament_python ex2 && cd ex2/`
2. talker/listenerを作成する
 標準のメッセージ型ではないだけなので、
 ```python
 from std_msgs.msg import Hoge
 ```
 ではなく
 ```python
 from ex2_msgs.msg import Person
 ```
 とし、定義したnameとgenderにアクセスすればよい。
 
 ## 独自メッセージの動作例
 ![custom_msg_simple](https://user-images.githubusercontent.com/8480644/154787654-3e38afb5-bdb1-422d-b930-5fdb1f9bf444.png)
 
 
 ## パラメータを使ってみる
  今回はノード起動時にtalker側でパラメータを初め受け取ることを考える。流れとしては、talker側でパラメータの定義・受け取り、起動時にtalkerへパラメータを投げるようにhoge_launch.pyの変更を行う。
 ### パラメータの定義   

 1. パラメータの定義(例)  
   Talkerのコンストラクタ内で以下のようにする。  
   ```python
    self.declare_parameter("nationality", "hoge")
   ```
 2. 受け取る例(コードを変えればノード上で動的にパラメータ値を変えることもできるはず)  
```python
nationality = self.get_parameter("nationality").get_parameter_value().string_value
self.get_logger().info("got param: {}".format(nationality))
```
 
 パラメータのセットは、コマンドライン引数でもlaunch.pyでも行うことができる。  コマンドライン引数として行うには、ros2 runでパッケージとノード指定後、`--ros-args -p パラメータ名:=値`でセットすることができる。 
 launch.pyから行うには、[ここ](https://github.com/ros2/launch_ros/blob/master/launch_ros/launch_ros/actions/node.py#L126)を見ると大体わかる  
 launch_ros.actionのNodeのコンストラクタの引数を見ると、
 >    def \_\_init\_\_(  
 >       self, *,  
 >       executable: SomeSubstitutionsType,  
 >       package: Optional[SomeSubstitutionsType] = None,  
 >       name: Optional[SomeSubstitutionsType] = None,  
 >       namespace: Optional[SomeSubstitutionsType] = None,  
 >       exec_name: Optional[SomeSubstitutionsType] = None,  
 >       parameters: Optional[SomeParameters] = None,  
 >       remappings: Optional[SomeRemapRules] = None,  
 >       ros_arguments: Optional[Iterable[SomeSubstitutionsType]] = None,  
 >       arguments: Optional[Iterable[SomeSubstitutionsType]] = None,  
 >       \**kwargs  
 >   ) -> None:  

 とあるので、LaunchDescriptionの引数であるNodeの引数に、例えば、  
 talkerノードのNode()に```parameters=[{"nationality":"japan",}]```など追記すればよい。    
 
