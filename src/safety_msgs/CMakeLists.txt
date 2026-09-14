cmake_minimum_required(VERSION 3.0.2)
project(safety_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)


add_message_files(
  FILES
  SafetyStatus.msg
  SafetyJob.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs  # Or other packages containing msgs
  )

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  message_runtime
)
install(TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# message(STATUS "CATKIN_PACKAGE_LIB_DESTINATION : ${CATKIN_PACKAGE_LIB_DESTINATION}")
# message(STATUS "CATKIN_PACKAGE_LIB_DESTINATION : ${CATKIN_PACKAGE_LIB_DESTINATION}")
# message(STATUS "CATKIN_PACKAGE_BIN_DESTINATION : ${CATKIN_PACKAGE_BIN_DESTINATION}")