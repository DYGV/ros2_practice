
## 学習メモ

- [Service/Client通信で人の顔検出を行う](https://github.com/DYGV/ros2_practice/blob/master/memo/srv_cli.md)
- [gazeboでturtlebot3の走行と障害物回避](https://github.com/DYGV/ros2_practice/blob/master/memo/gazebo_turtlebot3.md)
 
## Docker
動作確認済のホストOS: Ubuntu 20.04     

**[Warning]** コンテナから出たら`xhost -local:docker`すること  
作成したものはdev_ws.zipにまとめている  
(Dockerfileでワークスペースのディレクトリをコピーするのにファイルサイズを減らしたいのでdev_ws.zipに圧縮済み)  
 
### 初回起動
```
$ DOCKER_BUILDKIT=1 docker build . -t ubuntu1804_ros2
$ . x_setup.sh
$ docker run -d -m 8GB -it --name container_ros2 \
  -e DISPLAY=$DISPLAY -v $XSOCK:$XSOCK -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH \
  -v /dev:/dev --device-cgroup-rule='c *:* rmw' \
  ubuntu1804_ros2
$ docker attach container_ros2
```  

### 2回目以降の起動
```
$ . x_setup.sh
$ docker start container_ros2
$ docker attach container_ros2
```  

### 新規コマンド  
```
$ docker exec -it container_ros2 /bin/bash
```

うまくコンテナに入れるとコンテナで起動したアプリケーションのGUIがホスト側で表示される。  
`python3 cam_test.py`

![Screenshot from 2022-02-17 13-34-03](https://user-images.githubusercontent.com/8480644/154786230-f24fbaa8-9aa2-46aa-b94c-8b15d7bc34fb.png)

