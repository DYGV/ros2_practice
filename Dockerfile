FROM ubuntu:18.04
RUN --mount=type=cache,target=/root/.cache/enpit-build

RUN apt-get update && apt-get install tzdata

RUN apt-get install -y --no-install-recommends \
      bzip2 \
      g++ \
      git \
      sudo \
      graphviz \
      unzip \
      libgl1-mesa-glx \
      libhdf5-dev \
      openmpi-bin \
      wget \
      python3-pip \
      gcc \
      build-essential \
      python3-tk \
      eog \
      cheese \
      vim && \
      rm -rf /var/lib/apt/lists/*

RUN (apt-get autoremove -y; \
     apt-get autoclean -y)  


RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN apt-get update
RUN apt-get install -y ros-eloquent-desktop ros-eloquent-v4l2-camera
RUN apt-get install -y python3-colcon-common-extensions python-rosdep ros-eloquent-gazebo-ros-pkgs python3-vcstool ros-eloquent-dynamixel-sdk cmake
RUN pip3 install --upgrade pip setuptools
RUN pip3 install scikit-build
RUN pip3 install opencv-python

ENV QT_X11_NO_MITSHM=1

ARG username=eisuke

RUN echo "root:pass" | chpasswd && \
    adduser --disabled-password --gecos "" "${username}" && \
    echo "${username}:${username}" | chpasswd && \
    echo "%${username}    ALL=(ALL)   NOPASSWD:    ALL" >> /etc/sudoers.d/${username} && \
    chmod 0440 /etc/sudoers.d/${username} 

USER ${username}
WORKDIR /home/${username}
COPY ./cam_test.py ./cam_test.py
COPY ./dev_ws.zip ./dev_ws.zip
RUN unzip ./dev_ws.zip
RUN rm ./dev_ws.zip
RUN echo ". /opt/ros/eloquent/setup.bash" >> ~/.bashrc
RUN echo "export PATH=$PATH:/usr/bin" >> ~/.bashrc
RUN echo "export PATH=$PATH:/home/${username}/.local/bin" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/${username}/dev_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models" >> ~/.bashrc

RUN sudo apt-get install -y ros-eloquent-turtlebot3 ros-eloquent-turtlebot3-msgs ros-eloquent-nav2-bringup ros-eloquent-navigation2 ros-eloquent-cartographer-ros ros-eloquent-cartographer
WORKDIR dev_ws/src
RUN git clone -b eloquent-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
WORKDIR /home/${username}

CMD ["bash"]
