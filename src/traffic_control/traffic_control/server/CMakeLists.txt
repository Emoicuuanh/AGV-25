cmake_minimum_required(VERSION 3.0.2)
project(server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  nav_msgs
  std_stamped_msgs
  tuw_multi_robot_msgs
)

find_package(jsoncpp CONFIG REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tuw_multi_robot_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(server_traffic_control src/server_traffic_control.cpp)
add_dependencies(server_traffic_control ${catkin_EXPORTED_TARGETS} ${tuw_multi_robot_msgs_EXPORTED_TARGETS})
add_executable(client_agv src/client_agv.cpp)
add_dependencies(client_agv ${catkin_EXPORTED_TARGETS} ${tuw_multi_robot_msgs_EXPORTED_TARGETS})
target_link_libraries(server_traffic_control
  ${catkin_LIBRARIES}
  jsoncpp_lib
)
target_link_libraries(client_agv
  ${catkin_LIBRARIES}
  jsoncpp_lib
)