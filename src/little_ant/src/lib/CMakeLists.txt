add_library(littleAnt_utils
            USB2CAN/can2serial.cpp
            USB2CAN/serial.cpp
            getPkgFromDev.h)
target_link_libraries(littleAnt_utils ${catkin_LIBRARIES})


