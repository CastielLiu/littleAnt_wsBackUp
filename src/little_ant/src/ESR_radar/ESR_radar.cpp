#include "ESR_radar.h"
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>


using std::cout;
using std::endl;
using std::hex;



ESR_RADAR::ESR_RADAR(int argc,char ** argv):
	argc_(argc),
	argv_(argv) 
{
	can2serial = new CAN_2_SERIAL();
}

bool ESR_RADAR::init()
{

	ros::init(argc_,argv_,"esr_radar");
	ros::NodeHandle nh_private("~");
	ros::NodeHandle nh;
	nh_private.param<std::string>("port_name", port_name_, "/dev/ttyUSB0");
	esr_pub = nh.advertise<esr_radar_msgs::Objects>("/esr_radar",10);
	
	
	if(!can2serial->openPort(port_name_.c_str()))
	{
		cout << "open "<< port_name_ <<" failed!! "<<endl;
		return 0;
	}
	
	can2serial->clearCanFilter();
	
	can2serial->configBaudrate(500);
	
	can2serial->run();
	
	ROS_INFO("esr radar initialization complete");

	return 1;
}

ESR_RADAR::~ESR_RADAR()
{
	can2serial->closePort();
	delete can2serial;
}

void ESR_RADAR::run()
{
	boost::thread parse_thread(boost::bind(&ESR_RADAR::handleCanMsg,this));
}

void ESR_RADAR::handleCanMsg()
{
	STD_CAN_MSG can_msg;
	while(1)
	{
		usleep(10000);
		if(!can2serial->getMsg(can_msg)) continue;
		
		parse_msg(can_msg);
	}
}


void ESR_RADAR::parse_msg(STD_CAN_MSG &can_msg)
{
	//cout << "ID:" << hex << can_msg.ID <<endl;
	uint16_t scan_index;
	if(can_msg.ID == 0x4E0)
	{
		scan_index = can_msg.data[3]*256 + can_msg.data[4];
		objects.sequence = scan_index;
		objects.count = objects.objects.size();
		
		esr_pub.publish(objects);
		
		objects.objects.clear();
	}
	else if(can_msg.ID <= 0x53f && can_msg.ID >=0x500)
	{
				
		if(objects.objects.size() > 62)
		{
			objects.objects.clear();
			return ;
		}
		
		int measurementStatus = (can_msg.data[1]&0xE0) >> 5;
		
		//if(measurementStatus !=1 && measurementStatus !=3 && measurementStatus !=4)
		//	return;
			
		uint8_t g_trackStatus = ((can_msg.data[1]&0xE0) >> 5);
		
		uint16_t u16_tempAngle = (can_msg.data[1] &0x1F);
		u16_tempAngle <<=5;
		u16_tempAngle += ((can_msg.data[2] &0xf8)>>3) ;
		
		int16_t s16_angle = (int16_t)u16_tempAngle;
		
		if((can_msg.data[1]&0x10) !=0 )
			s16_angle -=1024;
		
		uint16_t u16_distance = (can_msg.data[2] &0x07);
		u16_distance <<=8;
		u16_distance += can_msg.data[3];
		
		uint16_t u16_tempTargetSpeed = can_msg.data[6] & 0x1f; 
		u16_tempTargetSpeed <<= 8;
		u16_tempTargetSpeed += can_msg.data[7];
		
		int16_t s16_targetSpeed =(int16_t)u16_tempTargetSpeed;
		
		if((can_msg.data[6]&0x20) !=0)
			s16_targetSpeed -= 8192;
		
		esr_radar_msgs::Object object;
		
		object.azimuth = s16_angle*0.1;

		object.distance = u16_distance*0.1;

		object.speed = s16_targetSpeed*0.01;
		
		object.status = measurementStatus;
		
		objects.objects.push_back(object);
		
		/*
		switch(measurementStatus)
		{	
			case 1: //new target
				cout <<"ID :" << hex << can_msg.ID << "   new     target ----> angle:" <<angle << "\t range:" <<distance <<"\tspeed:" <<speed << endl;
				break;
			case 3: //update target
				cout <<"ID :" << hex << can_msg.ID << "   update  target ----> angle:" <<angle << "\t range:" <<distance <<"\tspeed:" <<speed << endl;
				break;
			case 4: //coasted target
				cout <<"ID :" << hex << can_msg.ID << "   coasted target ----> angle:" <<angle << "\t range:" <<distance <<"\tspeed:" <<speed << endl;
				break;
					
			default:
				break;
		}*/
	}
}

void ESR_RADAR::pub_vehicleSpeed(float speed,bool dir)
{
	uint16_t vehicleSpeed = speed/0.0625; // m/s

	
	
	STD_CAN_MSG canMsg4F0 ={0x4F0,8}; //
	STD_CAN_MSG canMsg4F1 ={0x4F1,8};
	
	canMsg4F0.data[0] |= (uint8_t)(vehicleSpeed / 8);
	canMsg4F0.data[1] |= (uint8_t)(vehicleSpeed % 8) << 5;
	
	can2serial->sendStdCanMsg(canMsg4F0);
	

}

void ESR_RADAR::pub_installHeight(uint8_t installHeight)
{
	STD_CAN_MSG canMsg5F2 = {0x4F0,8};
	 
	canMsg5F2.data[4] |= installHeight & 0x7f;//install_height
	
	can2serial->sendStdCanMsg(canMsg5F2);
}
