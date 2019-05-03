#include"utils/utils.h"
#include<utils/Debug.h>

namespace utils
{
static bool is_initial = false;
static ros::Publisher pub;
static utils::Debug debug;

void debugSystemInitial()
{
	if(is_initial == false)
	{
		is_initial = true;
		ros::NodeHandle nh;
		pub = nh.advertise<utils::Debug>("/debug",10);
	}
}

void publishDebugMsg(const std::string& str)
{
	if(is_initial)
	{
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
