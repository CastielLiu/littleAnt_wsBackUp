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
  serial
  can2serial
  jsk_recognition_msgs
  utils
)

include_directories(
	include/little_ant
	src/lib/USB2CAN
	${catkin_INCLUDE_DIRS}
)

catkin_package(
 
  CATKIN_DEPENDS roscpp rospy std_msgs little_ant_msgs jsk_recognition_msgs
   INCLUDE_DIRS include
   LIBRARIES littleAnt_utils
#  DEPENDS system_lib
)


add_subdirectory(src/lib)


add_executable(base_control  src/base_control/base_control.cpp)
add_dependencies(base_control  ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_control ${catkin_LIBRARIES})


#add_executable(key_control src/keyControl/keyControl.cpp)
#add_dependencies(key_control  ${catkin_EXPORTED_TARGETS})
#target_link_libraries(key_control ${catkin_LIBRARIES})

#add_executable(telecontrol src/telecontrol/telecontrol.cpp )
#add_dependencies(telecontrol  ${catkin_EXPORTED_TARGETS})
#target_link_libraries(telecontrol ${catkin_LIBRARIES} littleAnt_utils)


#add_executable(telecontrol_stm32 src/telecontrol/telecontrol_stm32.cpp)
#add_dependencies(telecontrol_stm32  ${catkin_EXPORTED_TARGETS})
#target_link_libraries(telecontrol_stm32 ${catkin_LIBRARIES} littleAnt_utils)


add_executable(calculate_deceleration_node src/test/test.cpp)
add_dependencies(calculate_deceleration_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(calculate_deceleration_node ${catkin_LIBRARIES})







