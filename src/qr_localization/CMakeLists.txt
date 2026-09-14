cmake_minimum_required(VERSION 3.0.2)
project(qr_localization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nav_msgs
  agv_msgs
  roscpp
  std_stamped_msgs
  rospy
  tf2
  tf2_ros
)

find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
    CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    std_msgs
    roscpp
    rospy
    agv_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(qr_code_localization src/qr_code_localization.cpp include/qr_localization/json.hpp include/qr_localization/qr_code_localization.h)
target_link_libraries(qr_code_localization ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(qr_code_localization ${PROJECT_NAME}_generate_messages_cpp)
