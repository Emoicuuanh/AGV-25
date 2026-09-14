#!/bin/zsh

echo ROS_HOSTNAME: $ROS_HOSTNAME
echo WS: $ROS_WORKSPACE
source /opt/ros/noetic/setup.zsh
source $ROS_WORKSPACE/devel/setup.zsh
# head -1 $HOME/ros_config.sh # Echo first line in file
# source $HOME/robot_config/ros_config.sh # Already source in desktop file, use if run not by desktop file
export ROSCONSOLE_FORMAT='[${severity}] [${walltime}] [${node}] [${line}]: ${message}'

# cd /opt/ros
# # cd $(ls -d */|head -n 1) # cd to first directory to get ROS_DISTRO
# cd $(ls |grep -e noetic -e melodic)

# # If #!/bin/zsh, change *.bash to *.zsh
# source setup.zsh
# source $ROS_WORKSPACE/devel/setup.zsh
