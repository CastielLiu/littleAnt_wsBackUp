cmake_minimum_required(VERSION 2.8.3)
project(ant_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ant_math
  can2serial
  little_ant_msgs
  roscpp
  rospy
  serial
  std_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ant_base_control
#  CATKIN_DEPENDS ant_math can2serial little_ant_msgs roscpp rospy serial std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ant_driver  src/base_control.cpp)
add_dependencies(ant_driver  ${catkin_EXPORTED_TARGETS})
target_link_libraries(ant_driver ${catkin_LIBRARIES})
