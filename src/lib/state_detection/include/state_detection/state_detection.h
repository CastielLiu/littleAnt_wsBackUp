#ifndef DEBUG_H_
#define DEBUG_H_
#include<ros/ros.h>
#include<string>
#include<iostream>

#include<state_detection/Debug.h>

namespace state_detection
{

void publishDebugMsg(uint8_t level , const std::string& str);
void debugSystemInitial();



}//end state_detection namespace
#endif
