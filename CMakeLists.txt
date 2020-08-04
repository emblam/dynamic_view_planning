cmake_minimum_required(VERSION 3.8.2)
project(dynamic_view_planning)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic)

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
  movements
  ig_active_reconstruction
  ig_active_reconstruction_ros
  message_generation
  geometry_msgs
  rosbag
)

add_service_files(
  FILES
  ChangeCamera.srv
  AnalyseFile.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
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

add_executable(ig_calculator_node 
  src/ros_nodes/ig_calculator_node.cpp
  src/code_base/ufomap_ig_calculator.cpp
)

add_executable(robot_node 
  src/ros_nodes/robot_node.cpp
  src/code_base/virtual_robot.cpp
)

add_executable(create_view_space_node 
  src/ros_nodes/create_view_space.cpp
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
  src/code_base/test.cpp
)

add_executable(talker_node
  src/ros_nodes/talker_node.cpp
)

target_link_libraries(ig_calculator_node
  ${catkin_LIBRARIES}
)

target_link_libraries(robot_node
  ${catkin_LIBRARIES}
)

target_link_libraries(create_view_space_node
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

target_link_libraries(talker_node
  ${catkin_LIBRARIES}
)


install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
