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
  jsk_recognition_msgs
)


catkin_package(
#  INCLUDE_DIRS include
  
#  LIBRARIES ant_base_control
  CATKIN_DEPENDS roscpp rospy std_msgs little_ant_msgs jsk_recognition_msgs
#  DEPENDS system_lib
)


include_directories(
 include/little_ant/base_control
 include/little_ant/ESR_radar
  src/base_control/USB2CAN
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/lib)


AUX_SOURCE_DIRECTORY(src/base_control/. SRC_BASE_CONTROL_1)
add_executable(base_control    ${SRC_BASE_CONTROL_1} )
add_dependencies(base_control  ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_control ${catkin_LIBRARIES} littleAnt_utils)

AUX_SOURCE_DIRECTORY(src/ESR_radar/.  SRC_ESR_RADAR_1)
add_executable(esr_radar ${SRC_ESR_RADAR_1})
add_dependencies(esr_radar  ${catkin_EXPORTED_TARGETS})
target_link_libraries(esr_radar ${catkin_LIBRARIES} littleAnt_utils)

add_executable(key_control src/keyControl/keyControl.cpp)
add_dependencies(key_control  ${catkin_EXPORTED_TARGETS})
target_link_libraries(key_control ${catkin_LIBRARIES})

add_executable(telecontrol src/telecontrol/telecontrol.cpp )
add_dependencies(telecontrol  ${catkin_EXPORTED_TARGETS})
target_link_libraries(telecontrol ${catkin_LIBRARIES} littleAnt_utils)


add_executable(telecontrol_stm32 src/telecontrol/telecontrol_stm32.cpp)
add_dependencies(telecontrol_stm32  ${catkin_EXPORTED_TARGETS})
target_link_libraries(telecontrol_stm32 ${catkin_LIBRARIES} littleAnt_utils)


























