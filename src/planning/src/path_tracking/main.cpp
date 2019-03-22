#include"path_tracking.h"

int main(int argc,char**argv)
{
	ros::init(argc,argv,"path_tracking");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	
	
	ros::spin();
	return 0;
}
