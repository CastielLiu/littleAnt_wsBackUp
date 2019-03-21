#include<iostream>
#include "getPkgFromDev.h"
#include"little_ant_msgs/ControlCmd1.h"
#include"little_ant_msgs/ControlCmd2.h"
#include<boost/thread.hpp>
#include<boost/bind.hpp>

#include<ros/ros.h>

using namespace std;


int main(int argc,char** argv)
{
	ros::init(argc,argv,"key_control");
	
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	std::string port_name_;
	
	nh_private.param<std::string>("port_name",port_name_,"/dev/ttyUSB0");
	
	ros::Publisher pub1 = nh.advertise<little_ant_msgs::ControlCmd1>("/controlCmd1",10);
	ros::Publisher pub2 = nh.advertise<little_ant_msgs::ControlCmd2>("/controlCmd2",10);
	
	usleep(100000);
	
	little_ant_msgs::ControlCmd1 cmd1;
	little_ant_msgs::ControlCmd2 cmd2;
	
	GetPkgFromDev serial(port_name_,9600,12);
	boost::mutex mutex_;
	
	if(!serial.init())
		return 1;
	serial.startTread();
		
	const uint8_t *data = NULL;
	
	while(ros::ok())
	{
		usleep(10000);
		
		boost::mutex::scoped_lock lock(mutex_); 
		data = serial.getPkgPtr();
		
		for(int i=0;i<12;i++)
			printf("%x\t",data[i]);
		printf("\r\n");
		/////
		/////
	
	}
	serial.closeThread();
	printf("closed\r\n");
	return 0;
}


