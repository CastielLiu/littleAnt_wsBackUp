#include "ESR_radar.h"
#include "unistd.h"
#include <string>
#include <boost/bind.hpp>
#include <boost/thread.hpp>


using namespace std;
 

int main(int argc,char **argv)
{
	
	ESR_RADAR radar(argc,argv);
	
	if(!radar.init())
		return 1;
		
	radar.run();
	
	ros::spin();
	
	return 0;
}
