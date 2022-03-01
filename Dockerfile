FROM ubuntu:18.04
RUN --mount=type=cache,target=/var/cache/apt/archives
RUN --mount=type=cache,target=/var/cache/apt/lists
# gitを入れようとするとタイムゾーンの選択が止まってしまうのでtzdataを入れる
# [参考: Docker Ubuntu18.04でtzdataをinstallするときにtimezoneの選択をしないでinstallする](https://qiita.com/yagince/items/deba267f789604643bab)
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
