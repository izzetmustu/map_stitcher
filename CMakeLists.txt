cmake_minimum_required(VERSION 3.0.2)
project(map_stitcher)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
)

find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} 
# LIBRARIES stitcher
 CATKIN_DEPENDS 
 cv_bridge
 roscpp
 sensor_msgs
 std_msgs
 grid_map_core
 grid_map_ros
 grid_map_cv
 grid_map_filters
 grid_map_loader
 grid_map_msgs
 grid_map_octomap
 grid_map_rviz_plugin
 grid_map_visualization
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_stitcher_node src/MapStitcher.cpp)

target_link_libraries(map_stitcher_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

