cmake_minimum_required(VERSION 2.8.3)
project(utils)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  little_ant_msgs
)

add_message_files(
   FILES
   Debug.msg
 )
 
generate_messages(
	DEPENDENCIES
	sensor_msgs
	std_msgs
)


catkin_package(
	CATKIN_DEPENDS roscpp rospy message_runtime
	LIBRARIES ${PROJECT_NAME}
	INCLUDE_DIRS include
	#CATKIN_DEPENDS serial #
)


## Sources
set(utils_SRCS
    src/utils.cpp
    include/utils/utils.h
)
 

## Add utils library
add_library(${PROJECT_NAME} ${utils_SRCS})

#target_link_libraries(${PROJECT_NAME} boost)

## Include headers
include_directories(include ${catkin_INCLUDE_DIRS})

## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install headers
install(FILES include/utils.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/utils)


