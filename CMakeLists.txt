cmake_minimum_required(VERSION 3.13)

project(rova_bridge)
set(CMAKE_BUILD_TYPE Release)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -Wall -Wextra -Wno-switch)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  serial
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

catkin_package(
    CATKIN_DEPENDS std_msgs sensor_msgs serial
)



add_executable(rova_bridge_node src/rova_bridge_node.cpp)
add_dependencies(rova_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(rova_bridge_node ${catkin_LIBRARIES})