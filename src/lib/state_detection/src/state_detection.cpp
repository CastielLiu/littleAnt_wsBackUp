#include"state_detection/state_detection.h"

namespace state_detection
{
static bool is_initial = false;
static ros::Publisher pub;
static state_detection::Debug debug;

void debugSystemInitial()
{
	if(is_initial == false)
	{
		is_initial = true;
		ros::NodeHandle nh;
		pub = nh.advertise<state_detection::Debug>("/debug",10);
	}
}

void publishDebugMsg(uint8_t level , const std::string& str)
{
	if(is_initial)
	{
		debug.level = level;
		debug.info = str;
		pub.publish(debug);
	}
	else
	{
		ROS_ERROR("please initial debug system before use it!");
		ROS_ERROR("utils::debugSystemInitial();");
	}
}
















}//end utils namespace
