cmake_minimum_required(VERSION 2.8.3)
project(planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  little_ant_msgs
  roscpp
  rospy
  std_msgs
  gps_msgs
  serial
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning
#  CATKIN_DEPENDS little_ant_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  src/path_tracking
  src/can2serial
  src/decision_making
  
)


AUX_SOURCE_DIRECTORY(src/can2serial/. TEST_SRC)

add_executable(test_ ${TEST_SRC})
add_dependencies(test_ ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_   ${catkin_LIBRARIES})

AUX_SOURCE_DIRECTORY(src/path_tracking/.  SRC_path_tracking_node)

add_executable( path_tracking_node ${SRC_path_tracking_node} )
add_dependencies(path_tracking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_tracking_node   ${catkin_LIBRARIES})

add_executable(decision_making_node src/decision_making/decision_making.cpp)
add_dependencies(decision_making_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(decision_making_node   ${catkin_LIBRARIES})

add_executable(record_gps_data src/record_gps_data/record_gps_data.cpp)
add_dependencies(record_gps_data ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_gps_data   ${catkin_LIBRARIES})



