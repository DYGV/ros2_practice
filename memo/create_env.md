# 環境構築  
環境構築にはDockerを利用する。GUIアプリケーションも使いたいので設定が煩雑となった。  

## 動作確認済のホスト環境
- ホストOS: Ubuntu 20.04
- Dockerバージョン: 20.10.12
  
## 構築される環境
- Ubuntu 18.04
- ROS eloquent  

## Dockerfile
```dockerfile
FROM ubuntu:18.04
RUN --mount=type=cache,target=/var/cache/apt/archives
RUN --mount=type=cache,target=/var/cache/apt/lists
# gitを入れようとするとタイムゾーンの選択が止まってしまうのでtzdataを入れる
RUN apt-get update && apt-get install tzdata sudo

# ユーザーの追加
ARG username=eisuke
RUN echo "root:pass" | chpasswd && \
    adduser --disabled-password --gecos "" "${username}" && \
    echo "${username}:${username}" | chpasswd && \
    echo "Defaults:${username} !env_reset" >> /etc/sudoers.d/${username} && \
    echo "%${username}    ALL=(ALL)   NOPASSWD:    ALL" >> /etc/sudoers.d/${username} && \
    chmod 0440 /etc/sudoers.d/${username}

# rootから作成したユーザに切り替える
USER ${username}
# ROS周りの構築に必要なパッケージ
RUN sudo apt-get install -y --no-install-recommends \
    ca-certificates \
    git \
    curl \
    gnupg2 \
    vim \
    lsb-release \
    python3-pip

# ROS周りの構築
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN sudo apt-get update
RUN sudo apt-get install -y ros-eloquent-desktop
RUN sudo apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-eloquent-v4l2-camera \
    ros-eloquent-gazebo-ros-pkgs \
    ros-eloquent-dynamixel-sdk \
    ros-eloquent-turtlebot3 \
    ros-eloquent-turtlebot3-msgs \
    ros-eloquent-nav2-bringup \
    ros-eloquent-navigation2 \
    ros-eloquent-cartographer-ros \
    ros-eloquent-cartographer

# 必須ではない
RUN pip3 install --upgrade pip setuptools
RUN pip3 install scikit-build
RUN pip3 install opencv-python

# ワークスペースのコピー
COPY . /home/${username}/
RUN sudo chown -R ${username}:${username} /home/${username}/
# .bashrcに最低限必要なパスを追加しておく
RUN echo '. /opt/ros/eloquent/setup.bash' >> ~/.bashrc
RUN echo 'export PATH="$PATH:/usr/bin"'>> ~/.bashrc
RUN echo 'export PATH="$PATH:/home/'${username}'/.local/bin"' >> ~/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:/home/'${username}'/dev_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models"' >> ~/.bashrc

WORKDIR /home/${username}/dev_ws/src
# Gazeboのworldのlaunchファイルやモデルとかが含まれている
RUN git clone -b eloquent-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
WORKDIR face_detect
RUN curl -OL https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml
WORKDIR /home/${username}
RUN curl -OL https://gist.githubusercontent.com/DYGV/575673d2857dbb19c33cc6f75753fd6f/raw/fa56670adbbc7b6ce5a6265100ba499eb731582f/.vimrc

CMD ["/bin/bash"]
```  
## 初回起動  
__Dockerイメージの作成__  
```bash
$ DOCKER_BUILDKIT=1 docker build . -t ubuntu1804_ros2
```
__コンテナからXサーバへ接続するための認証情報の作成__
```bash
$ # dockerに対してディスレプレイの使用を許可する
$ xhost +local:docker
$ # X11のソケットファイルの環境変数
$ XSOCK=/tmp/.X11-unix
$ # Xサーバと接続するための認証情報(xauthファイル)のパス
$ XAUTH=/tmp/.docker.xauth
$ # xauthファイルを作成する
$ xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
```
__コンテナの作成__  
```bash
$ docker create --interactive --tty \
--name=container_ros2 \
--env=DISPLAY=$DISPLAY \
--volume=$XSOCK:$XSOCK \
--volume=/dev:/dev \
--device-cgroup-rule='c *:* rw' \
ubuntu1804_ros2 /bin/bash
```  
- -- name: 作成するコンテナの名前
- -- interactive: コンテナの標準入力にアタッチ
- -- tty: 疑似ターミナルを割り当てる
- --env: 環境変数の設定
  - 接続するXサーバの指定
- --volume: ホスト側のディレクトリをとコンテナ側でマウントする
  - Xサーバのソケットファイルのディレクトリをマウント
  - ビデオデバイスを使いたいので、/dev/をマウント
- --device-cgroup-rule: デバイスリストのルール追加
  - プロセスをグループ化、リソースの利用を制限・監視するLinuxカーネルの機能[cgroup](https://man7.org/linux/man-pages/man7/cgroups.7.html)に、マウントしたデバイスのうちキャラクタデバイスの読み書きを可能にするルールを追加する。

  
## コンテナの起動と接続  
Xサーバ関係のファイルを/tmp/に置いているので、PCの再起動後などは、`コンテナからXサーバへ接続するための認証情報の作成`を行ってからコンテナを起動すること。  
  
```bash
$ docker start container_ros2
$ docker exec -it container_ros2 /bin/bash
```  

コンテナに入ると起動したアプリケーションのGUIがホスト側で表示できるようになる。  

![GUIアプリケーションを起動した様子](https://user-images.githubusercontent.com/8480644/154786230-f24fbaa8-9aa2-46aa-b94c-8b15d7bc34fb.png)

## 参考  
- [Docker-docs-ja](http://docs.docker.jp/v19.03/engine/reference/commandline/run.html) 
- [ubuntu:20.04 dockerコンテナ内でホスト側にGUIを出力（xeyesを出力）する検討メモ](https://qiita.com/seigot/items/83bc1bb32c04ca36c97f)
- [Docker Ubuntu18.04でtzdataをinstallするときにtimezoneの選択をしないでinstallする](https://qiita.com/yagince/items/deba267f789604643bab)
- [cgroupのDevice Whitelist Controllerについて](https://kamatama41.hatenablog.com/entry/2021/08/10/044822)
- [cgroups(7) - Linux manual page](https://man7.org/linux/man-pages/man7/cgroups.7.html)

