# Ubuntu 18.04 and Ros-Melodic 
# INSTALL THE ARDUINO PACKAGE FOR ROS
sudo apt-get install ros-$ROS_DISTRO-rosserial-arduino
sudo apt-get install ros-$ROS_DISTRO-rosserial
sudo apt-get update && sudo apt-get upgrade -y
cd ~/catkin_ws/src/
# change when use other Ros 
git clone -b melodic-devel https://github.com/ros-drivers/rosserial.git 
cd ..
catkin_make --only-pkg-with-deps rosserial
catkin_make install

# GIT 
cd ~/Arduino/libraries && git clone https://github.com/sparkfun/SparkFun_VL53L5CX_Arduino_Library.git
