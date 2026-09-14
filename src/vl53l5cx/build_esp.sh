# INSTALL THE ROS LIBRARIES IN YOUR ARDUINO IDE
cd ~/Arduino/libraries/
rm -rf ros_lib
source ~/catkin_ws/devel/setup.bash 
rosrun rosserial_arduino make_libraries.py .
sudo cp ros.h ~/Arduino/libraries/ros_lib 

# COPY THE LIBRARY FOR ESP
cd ~/Arduino/libraries/
mkdir espros
cd espros
git clone https://github.com/agnunez/espros.git
mv espros/includes/* .
rm -rf espros

