cmake_minimum_required(VERSION 3.8.2)
project(dynamic_view_planning)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_sensor_msgs
  ufomap
  ufomap_msgs
  ufomap_ros
  geometry_msgs
  rosbag
  dynamic_view_planning_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES dynamic_view_planning
  #CATKIN_DEPENDS roscpp ufomap ufomap_msgs
  #DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(view_planning_node
  src/ros_nodes/dynamic_view_planning_node.cpp
  src/code_base/view_planner.cpp
)

add_executable(create_view_space_node 
  src/ros_nodes/create_view_space.cpp
)

add_executable(generate_reference_map_node 
  src/ros_nodes/generate_reference_map.cpp
)

add_executable(world_representation_node 
  src/ros_nodes/world_representation_node.cpp
  src/code_base/ufomap_world_representation.cpp
)

add_executable(evaluation_node 
  src/ros_nodes/evaluation_node.cpp
  src/code_base/evaluation_tools.cpp
)

add_executable(test_node
  src/ros_nodes/test_node.cpp
)

add_executable(split_bag_node
  src/ros_nodes/split_bag_node.cpp
)

add_executable(ig_server_node
  src/ros_nodes/ig_server_node.cpp
  src/code_base/ig_server.cpp
)

target_link_libraries(view_planning_node
  ${catkin_LIBRARIES}
)

target_link_libraries(create_view_space_node
  ${catkin_LIBRARIES}
)

target_link_libraries(generate_reference_map_node
  ${catkin_LIBRARIES}
)

target_link_libraries(world_representation_node
  ${catkin_LIBRARIES}
)

target_link_libraries(evaluation_node
  ${catkin_LIBRARIES}
)

target_link_libraries(test_node
  ${catkin_LIBRARIES}
)

target_link_libraries(split_bag_node
  ${catkin_LIBRARIES}
)

target_link_libraries(ig_server_node
  ${catkin_LIBRARIES}
)


install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
