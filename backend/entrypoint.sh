#!/bin/bash
if [ "$CONTAINER_NAME" == "ros-kinetic-dev" ]; then
    sudo chown $USER:$USER src
else
    echo "Will prepare environment"
fi
source /opt/ros/$ROS_DISTRO/setup.bash 
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash 
if [ "$CONTAINER_NAME" == "ros-kinetic-dev" ]; then
    echo "To launch the entire project use the command:"
    echo "$ roslaunch backend run_with_bridge.launch"
else
    roslaunch backend run_with_bridge.launch
fi
