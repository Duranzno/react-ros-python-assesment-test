#!/bin/bash
if [[ "$(docker images -q ros-kinetic-dev 2> /dev/null)" == "" ]]; then
  docker build . -f Dockerfile --target dev -t ros-kinetic-dev ;
else
  echo "Using existing build image"
fi
if [ ! -d "/home/$USER/catkin_ws_kinetic" ]; then
  echo "Creating folder"
  mkdir -p ~/catkin_ws_kinetic
  chmod +rwx -R ~/catkin_ws_kinetic 
else
  echo "Already Created"
fi


xhost +local:
docker run -it \
  --net=host \
  --name=ros-kinetic-dev-runned \
  --user "$(id -u):$(id -g)" \
  --workdir=/home/$USER/catkin_ws \
  --device=/dev/dri:/dev/dri \
  --rm \
  -e DISPLAY=$DISPLAY \
  -e QT_GRAPHICSSYSTEM=native \
  -e CONTAINER_NAME=ros-kinetic-dev \
  -e USER=$USER \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/group:/etc/group:ro" \
  -v "/etc/passwd:/etc/passwd:ro" \
  -v "/etc/shadow:/etc/shadow:ro" \
  -v "/etc/sudoers.d:/etc/sudoers.d:ro" \
  -v "/home/$USER/catkin_ws_kinetic:/home/$USER/catkin_ws" \
  -v "/home/$USER/.ros_kinetic:/home/$USER/.ros" \
  -v "$(pwd)":/home/$USER/catkin_ws/src/backend:z \
  -p 8080 \
  -p 9000 \
  ros-kinetic-dev
  bash
  # -v "$(pwd)":home/$USER/catkin_ws/src/backend
