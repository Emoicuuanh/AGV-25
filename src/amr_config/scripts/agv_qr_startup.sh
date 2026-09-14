#!/bin/bash
cd  ~/catkin_ws
source /opt/ros/noetic/setup.zsh
source devel/setup.zsh
roslaunch amr_app amr_startup.launch use_joystick:=true simulation:=false use_rviz:=false
