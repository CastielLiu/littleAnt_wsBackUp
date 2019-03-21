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
	
	nh_private.param<std::string>("port_name",port_name_,"/dev/stm32");
	
	ros::Publisher pub1 = nh.advertise<little_ant_msgs::ControlCmd1>("/controlCmd1",10);
	ros::Publisher pub2 = nh.advertise<little_ant_msgs::ControlCmd2>("/controlCmd2",10);
	
	usleep(100000);
	
	little_ant_msgs::ControlCmd1 cmd1;
	little_ant_msgs::ControlCmd2 cmd2;
	
	GetPkgFromDev serial(port_name_,115200,12);
	boost::mutex mutex_;
	
	if(!serial.init())
		return 1;
	serial.startTread();
		
	const uint8_t *data = NULL;
	
	uint16_t left_U_D ;
	uint16_t left_L_R ;
	uint16_t right_U_D;
	uint16_t right_L_R;
	
	
	while(ros::ok())
	{
		usleep(20000);
		
		boost::mutex::scoped_lock lock(mutex_); 
		data = serial.getPkgPtr();
		
		bool mode = data[8]&0x01;
		if(mode==0)
			continue;
		
		left_U_D = data[0]*256+data[1];
		left_L_R = data[6]*256+data[7];
		right_U_D = data[2]*256+data[3];
		right_L_R = data[4]*256+data[5];
		
		
		
		printf("%d\t%d\t%d\t%d\n",left_U_D,left_L_R,right_U_D,right_L_R);
		/*
		for(int i=0;i<9;i++)
		{
			printf("%x\t",data[i]);
		}
		printf("\n");*/
		/////
		/////
	
	}
	serial.closeThread();
	printf("closed\r\n");
	return 0;
}


