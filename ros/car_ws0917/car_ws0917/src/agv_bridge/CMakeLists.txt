cmake_minimum_required(VERSION 3.0.2)
project(agv_bridge)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  serial
  std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs serial std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(agv_bridge_node src/agv_bridge_node.cpp)
target_link_libraries(agv_bridge_node ${catkin_LIBRARIES})

install(TARGETS agv_bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
