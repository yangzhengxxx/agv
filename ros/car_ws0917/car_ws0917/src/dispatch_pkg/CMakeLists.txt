cmake_minimum_required(VERSION 3.0.2)
project(dispatch_pkg)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)

add_executable(dispatch_node
  src/agv002/dispatch_node.cpp
  src/agv002/http_utils.cpp
  src/agv002/task_handlers.cpp
  src/agv002/status_handlers.cpp
)
target_link_libraries(dispatch_node
  ${catkin_LIBRARIES}
  ${CURL_LIBRARIES}
  nlohmann_json::nlohmann_json
)

install(TARGETS dispatch_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
