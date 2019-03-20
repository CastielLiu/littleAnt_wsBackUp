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


catkin_package(
#  INCLUDE_DIRS include
  
#  LIBRARIES ant_base_control
  CATKIN_DEPENDS roscpp rospy std_msgs little_ant_msgs
#  DEPENDS system_lib
)


include_directories(
 include/little_ant/base_control
 include/little_ant/ESR_radar
  src/base_control/USB2CAN
  ${catkin_INCLUDE_DIRS}
)


AUX_SOURCE_DIRECTORY(src/base_control/. SRC_BASE_CONTROL_1)
AUX_SOURCE_DIRECTORY(src/base_control/USB2CAN/. SRC_BASE_CONTROL_2)
add_executable(base_control    ${SRC_BASE_CONTROL_1} ${SRC_BASE_CONTROL_2})
add_dependencies(base_control  ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_control ${catkin_LIBRARIES})

AUX_SOURCE_DIRECTORY(src/ESR_radar/.  SRC_ESR_RADAR_1)
AUX_SOURCE_DIRECTORY(src/base_control/USB2CAN/. SRC_ESR_RADAR_2)
add_executable(esr_radar ${SRC_ESR_RADAR_1} ${SRC_ESR_RADAR_2})
add_dependencies(esr_radar  ${catkin_EXPORTED_TARGETS})
target_link_libraries(esr_radar ${catkin_LIBRARIES})

add_executable(key_control src/keyControl/keyControl.cpp)
add_dependencies(key_control  ${catkin_EXPORTED_TARGETS})
target_link_libraries(key_control ${catkin_LIBRARIES})

add_executable(telecontrol src/telecontrol/telecontrol.cpp src/base_control/USB2CAN/serial.cpp)
add_dependencies(telecontrol  ${catkin_EXPORTED_TARGETS})
target_link_libraries(telecontrol ${catkin_LIBRARIES})


























