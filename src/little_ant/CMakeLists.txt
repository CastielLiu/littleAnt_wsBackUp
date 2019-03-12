cmake_minimum_required(VERSION 2.8.3)
project(little_ant)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  little_ant_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  INCLUDE_DIRS include
  
#  LIBRARIES ant_base_control
  CATKIN_DEPENDS roscpp rospy std_msgs little_ant_msgs
#  DEPENDS system_lib
)


include_directories(
 include
  src/base_control/USB2CAN
  ${catkin_INCLUDE_DIRS}
)


AUX_SOURCE_DIRECTORY(src/base_control/. SRC_BASE_CONTROL_1)
AUX_SOURCE_DIRECTORY(src/base_control/USB2CAN/. SRC_BASE_CONTROL_2)
add_executable(base_control    ${SRC_BASE_CONTROL_1} ${SRC_BASE_CONTROL_2})
add_dependencies(base_control  ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_control ${catkin_LIBRARIES})




























