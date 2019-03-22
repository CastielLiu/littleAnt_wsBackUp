#include<iostream>
#include "getPkgFromDev.h"
#include"little_ant_msgs/ControlCmd1.h"
#include"little_ant_msgs/ControlCmd2.h"
#include<std_msgs/Bool.h>
#include<boost/thread.hpp>
#include<boost/bind.hpp>

#include<ros/ros.h>

using namespace std;

std::string port_name_;

const uint8_t *data = NULL;
	
uint16_t left_U_D ;
uint16_t left_L_R ;
uint16_t right_U_D;
uint16_t right_L_R;
bool is_telecontrol_mode;

boost::mutex mutex_;
ros::Publisher pub_is_tele;
	
void parse();
 

void send_is_telecontrol(const ros::TimerEvent&)
{
	ROS_ERROR("stm32   is_telecontrol_mode:%d",is_telecontrol_mode);
	/*boost::mutex::scoped_lock lock(mutex_); 
	std_msgs::Bool _mode_ ;
	_mode_.data = is_telecontrol_mode;
	
	pub_is_tele.publish(_mode_);*/
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"key_control");
	
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	nh_private.param<std::string>("port_name",port_name_,"/dev/ttyUSB0");
	
	ros::Timer timer =nh.createTimer(ros::Duration(0.03),send_is_telecontrol);
	
	ros::Publisher pub_cmd1 = nh.advertise<little_ant_msgs::ControlCmd1>("/controlCmd1",10);
	ros::Publisher pub_cmd2 = nh.advertise<little_ant_msgs::ControlCmd2>("/controlCmd2",10);
	
	pub_is_tele =nh.advertise<std_msgs::Bool>("/is_telecontrol",5);
	
	
	
	little_ant_msgs::ControlCmd1 cmd1;
	little_ant_msgs::ControlCmd2 cmd2;
	
	//boost::thread newThread(boost::bind(&parse));
	

	float speed = 0, steer = 0, hand_brake = 0, D_change = 0, left_light = 0, right_light = 0 , brake =0.0;
	
	
	while(ros::ok())
	{
		
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		ROS_ERROR("runing...");
		if(is_telecontrol_mode==false) 
		{
			usleep(10000);
			continue;
		}
		usleep(10000);
		
		if (right_U_D > 3500)
		{
			while (right_U_D > 2500)
			{
				usleep(1000);
			}
			if (hand_brake == 0)
				hand_brake = 1;
			else
				hand_brake = 0;
		}
		if (right_U_D < 500)
		{
			while (right_U_D<1500)
			{ 
				usleep(1000);
			}
			if (D_change == 0)
				D_change = 1;
			else
				D_change = 0;
		}
		if(right_L_R<2200&&right_L_R>1900)
		{
			steer = 0;
		}
		else 
		{
			steer = (1.0*(2048 - right_L_R) / 2048) * 200;
		}
		if (hand_brake == 0 && D_change == 1)
		{
			if (left_U_D < 2200 && left_U_D>1900)
			{
				brake =0;
				speed =0;
			}
			else if (left_U_D >= 2200)
			{
				speed = (1.0*(left_U_D - 2048) / 2048)*14.9;
				brake = 0;
			}
			else if (left_U_D <= 1900)
			{
				brake = (1.0*(2048 - left_U_D) / 2048) * 40;
				speed = 0;
			}
		}
		cmd1.set_driverlessMode = true;
		cmd1.set_handBrake = hand_brake;
		cmd1.set_turnLight_R = right_light;
		cmd1.set_turnLight_L = left_light;
		cmd2.set_gear = D_change;
		cmd2.set_brake = brake;
		cmd2.set_speed = speed;
		cmd2.set_steeringAngle = steer;

		pub_cmd1.publish(cmd1);
		pub_cmd2.publish(cmd2);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////		
		
		
		/*
		for(int i=0;i<9;i++)
		{
			printf("%x\t",data[i]);
		}
		printf("\n");*/
		/////
		/////
	
	}
	
	return 0;
}

void parse()
{
	GetPkgFromDev serial(port_name_,115200,12);
	
	if(!serial.init())
	{
		ros::shutdown();
		return ;
	}
	serial.startTread();
	while(ros::ok)
	{
		usleep(20000);
		//cout << "running...."<<endl;
		boost::mutex::scoped_lock lock(mutex_); 
		data = serial.getPkgPtr();
		
		is_telecontrol_mode = data[8]&0x01;
		if(is_telecontrol_mode==0)
			continue;
		
		left_U_D = data[0]*256+data[1];
		left_L_R = data[6]*256+data[7];
		right_U_D = data[2]*256+data[3];
		right_L_R = data[4]*256+data[5];
		//printf("%d\t%d\t%d\t%d\n",left_U_D,left_L_R,right_U_D,right_L_R);
	}
	serial.closeThread();
	printf("closed\r\n");
}


