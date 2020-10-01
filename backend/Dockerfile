FROM osrf/ros:kinetic-desktop-xenial as dev
SHELL [ "/bin/bash" ]
LABEL key="ros-kinetic-dev"
USER $USER
RUN apt-get update && apt-get install -y \
  tmux \
  locales \
  curl \
  wget \
  gdb \
  vim \
  sudo \
  libgl1-mesa-glx \
  libgl1-mesa-dri \
  mesa-utils \
  unzip \
  && rm -rf /var/likb/apt/lists/*

RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
RUN curl -sL https://deb.nodesource.com/setup_current.x | sudo -E bash - \
  && sudo apt-get install -y nodejs \
  && npm install -g nodemon

RUN apt-get install -y  \
  ros-$ROS_DISTRO-turtle-tf2 \
  ros-$ROS_DISTRO-tf2-tools \
  ros-$ROS_DISTRO-tf \
  ros-$ROS_DISTRO-catkin-pip \
  ros-$ROS_DISTRO-catkin-virtualenv \
  ros-$ROS_DISTRO-rosbridge-suite \
  ros-$ROS_DISTRO-map-server
RUN apt-get  install -y python3-empy 
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash">> ~/.bashrc
RUN echo "source source ~/catkin_make/devel/setup.bash">> ~/.bashrc
CMD ["bash"]

FROM dev as prod 
# RUN ln -snf /bin/bash /bin/sh
# RUN mkdir -p /ros_workspace/src/backend
# WORKDIR /ros_workspace
# COPY [".", "/ros_workspace/src/test.md"]
# RUN source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
EXPOSE 9090 11311 33690 8080
# CMD source /ros_workspace/devel/setup.bash; roslaunch backend run_with_bridge.launch
# RUN ln -snf /bin/bash /bin/sh
RUN mkdir -p /ros_workspace/src/backend
WORKDIR /ros_workspace
COPY [".", "/ros_workspace/src/backend"]
# ENTRYPOINT [ "/bin/bash","src/backend/entrypoint.sh" ]
ENTRYPOINT [ "/bin/bash","src/backend/entrypoint.sh" ]