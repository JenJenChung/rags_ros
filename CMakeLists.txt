cmake_minimum_required(VERSION 2.8.3)

project(rags_ros)
set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base
  amcl
  roscpp
  rospy
  message_generation
  std_msgs
)

# Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  FILES
  EdgeCosts.msg
  PathCost.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rags_ros
  CATKIN_DEPENDS geometry_msgs move_base amcl roscpp rospy message_runtime std_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rags_planner src/rags_planner.cpp)
target_link_libraries(rags_planner ${catkin_LIBRARIES})
add_dependencies(rags_planner ${catkin_EXPORTED_TARGETS})

add_executable(edge_cost_publisher src/edge_cost_publisher.cpp)
target_link_libraries(edge_cost_publisher ${catkin_LIBRARIES})
add_dependencies(edge_cost_publisher ${catkin_EXPORTED_TARGETS})

add_executable(speed_regulator src/speed_regulator.cpp)
target_link_libraries(speed_regulator ${catkin_LIBRARIES})
add_dependencies(speed_regulator ${catkin_EXPORTED_TARGETS})
