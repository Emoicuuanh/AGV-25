ros_master=192.168.XXX.XXX
export ROS_WORKSPACE=$HOME/XXX_WS
export ROS_ENTRY_REAL="roslaunch pkg_name file.launch simulation:=false use_rviz:=false use_joystick:=true"
export ROS_ENTRY_SIMULATION="roslaunch pkg_name file.lauch simulation:=true use_rviz:=true use_joystick:=false"
export ROS_MASTER_URI=http://$ros_master:11311
export ROS_HOSTNAME=$ros_master
