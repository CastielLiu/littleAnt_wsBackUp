cmake_minimum_required(VERSION 2.8.3)
project(can2serial)

# Find catkin
find_package(catkin REQUIRED COMPONENTS 
	serial
)

include_directories(${serial_INCLUDE_DIRS})


catkin_package(
	LIBRARIES ${PROJECT_NAME}
	INCLUDE_DIRS include
	CATKIN_DEPENDS serial #
)

## Sources
set(can2serial_SRCS
    src/can2serial.cpp
    include/can2serial/can2serial.h
)

## Add can2serial library
add_library(${PROJECT_NAME} ${can2serial_SRCS})

#target_link_libraries(${PROJECT_NAME} boost)

## Include headers
include_directories(include)

## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install headers
install(FILES include/can2serial.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/can2serial)

