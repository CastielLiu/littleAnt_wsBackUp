cmake_minimum_required(VERSION 2.8.3)
project(state_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  little_ant_msgs
  message_generation
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
set(${PROJECT_NAME}_SRCS
    src/${PROJECT_NAME}.cpp
    include/${PROJECT_NAME}/${PROJECT_NAME}.h
)
 

## Add ${PROJECT_NAME} library
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

#target_link_libraries(${PROJECT_NAME} boost)

## Include headers
include_directories(include ${catkin_INCLUDE_DIRS})

## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install headers
install(FILES include/${PROJECT_NAME}.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME})


