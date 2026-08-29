cmake_minimum_required(VERSION 3.0.2)
project(ndt_mapping)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)

add_message_files(FILES PointsDownsamplerInfo.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime pcl_conversions pcl_ros roscpp sensor_msgs std_msgs tf
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(ndt_matching_node src/ndt_matching.cpp)
target_link_libraries(ndt_matching_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(map_loader_node src/point_load.cpp)
target_link_libraries(map_loader_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(voxel_filter_node src/voxel_grid_filter.cpp)
target_link_libraries(voxel_filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(voxel_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ndt_matching_node map_loader_node voxel_filter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
