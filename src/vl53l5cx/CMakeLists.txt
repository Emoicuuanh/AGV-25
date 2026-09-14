cmake_minimum_required(VERSION 3.0.2)
project(vl53l5cx)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  pcl_conversions
  pcl_ros
)

# # Generate messages in the 'msg' folder
add_message_files(
  FILES
  Vl53l5cxRanges.msg
  Safety_Esp.msg
  vl53l5cx_safety.msg
  ultra_vl53l5_safety.msg
)

# # Generate services in the 'srv' folder
# add_service_files(
# FILES
# Service1.srv
# Service2.srv
# )

# # Generate actions in the 'action' folder
# add_action_files(
# FILES
# Action1.action
# Action2.action
# )

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

# ##########
# # Build ##
# ##########

# Regular catkin package follows.
find_package(console_bridge REQUIRED)
include_directories(include ${console_bridge_INCLUDE_DIRS})
catkin_package(CATKIN_DEPENDS roscpp  message_runtime)

## include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(vl53l5cx_ros_node src/vl53l5cx_ros.cpp)
add_dependencies(vl53l5cx_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vl53l5cx_ros_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})
