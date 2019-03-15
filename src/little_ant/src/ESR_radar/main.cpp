#include "ESR_radar.h"
#include "unistd.h"
#include <string>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

 

int main(int argc,char **argv)
{
	using std::string;
	using namespace std;
	

	
	ESR_RADAR radar;
	//ESR_RADAR radar_send("/dev/ttyUSB1" ,SEND__);
	if(!radar.init("/dev/ttyUSB1" ,RECEIVE__))
		return 1;
	
	radar.run();
	
	while(1)
	{
		//radar_send.pub_installHeight(10);
		//usleep(10000);
		//radar_send.pub_vehicleSpeed(0.0);
		usleep(500000);
		cout <<"running...."<<endl;
	}
	
	return 0;
}
