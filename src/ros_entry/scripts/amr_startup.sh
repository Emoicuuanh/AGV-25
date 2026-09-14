#!/bin/zsh

time=`date +%F_%H%M%S`
file="$HOME/log/ros/$time.log"
mkdir -p "$HOME/log/ros"
echo $file | tee $file

cd "$(dirname -- "$0")" # cd to script's directory
source env.sh
source /opt/ros/noetic/setup.zsh
source $ROS_WORKSPACE/devel/setup.zsh
source $HOME/install_isolated/setup.zsh --extend
T=20 # Time must be differ with other roslauch cmd
if [ $# -ne 0 ]
then
T=$1
fi

echo "Launching application after $T (s), please wait..."

for i in {1..$T}; # For zsh
# for((i=1;i<=$T;i++)) # For bash
do
  echo -n '.'
  sleep 1
done

echo ""
echo "Kill all nodes"
rosnode kill -a

echo ""
echo "Killing rosmaster"
killall -9 rosmaster

echo ""
echo "Waiting before start"
for i in {1..5};
# for((i=1;i<=5;i++)) # For bash
do
  echo -n '.'
  sleep 1
done
echo ""

# Start point
simulation=false
# if [[ $2 -eq 1 ]]; then
#   simulation=true
#   echo ROS_ENTRY_SIMULATION: $ROS_ENTRY_SIMULATION
#   $ROS_ENTRY_SIMULATION 2>&1 | tee -a $file
# else
#  simulation=false
#   echo "ROS_ENTRY_REAL: $ROS_ENTRY_REAL"
#   if [[ -z "$ROS_ENTRY_REAL" ]]; then
#     echo "ROS_ENTRY_REAL not set! Falling back..."
#     ROS_ENTRY_REAL="roslaunch amr_app amr_startup.launch"
#   fi
#   $ROS_ENTRY_REAL 2>&1 | tee -a $file
# fi
roslaunch amr_app amr_startup.launch