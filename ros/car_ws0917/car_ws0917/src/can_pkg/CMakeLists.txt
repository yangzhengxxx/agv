cmake_minimum_required(VERSION 3.0.2)
project(can_pkg)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(can_node src/can_node.cpp)
target_link_libraries(can_node ${catkin_LIBRARIES})

install(TARGETS can_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
