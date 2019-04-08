cmake_minimum_required(VERSION 2.8.3)
project(ant_math)

# Find catkin
find_package(catkin REQUIRED COMPONENTS 
	#serial
)


catkin_package(
	LIBRARIES ${PROJECT_NAME}
	INCLUDE_DIRS include
	#CATKIN_DEPENDS serial #
)


## Sources
set(ant_math_SRCS
    src/ant_math.cpp
    include/ant_math/ant_math.h
)
 

## Add ant_math library
add_library(${PROJECT_NAME} ${ant_math_SRCS})

#target_link_libraries(${PROJECT_NAME} boost)

## Include headers
include_directories(include)

## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install headers
install(FILES include/ant_math.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ant_math)


