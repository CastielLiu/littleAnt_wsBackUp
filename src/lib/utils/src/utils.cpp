#include"utils/utils.h"


namespace utils
{

void publishDebugMsg(const std::string& str)
{
	static bool is_initial = false;
	static ros::Publisher pub;
	
	static little_ant_msgs::Debug debug;
	
	if(is_initial == false)
	{
		is_initial = true;
		ros::NodeHandle nh;
		pub = nh.advertise<little_ant_msgs::Debug>("/debug",10);
	}

	debug.info = str;
	pub.publish(debug);
}
















}//end utils namespace
