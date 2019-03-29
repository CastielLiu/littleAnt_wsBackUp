#include<iostream>
#include "getPkgFromDev.h"
#include"little_ant_msgs/ControlCmd1.h"
#include"little_ant_msgs/ControlCmd2.h"
#include<little_ant_msgs/ControlCmd.h>
#include<std_msgs/Bool.h>
#include<boost/thread.hpp>
#include<boost/bind.hpp>

#include<ros/ros.h>

using namespace std;

class Telecontrol
{
public:
	
	Telecontrol()
	{
		telecontrol_cmd_.origin = little_ant_msgs::ControlCmd::_TELECONTROL;  //_TELECONTROL  _LIDAR
	}
	~Telecontrol(){}
	
	void init(ros::NodeHandle nh,ros::NodeHandle nh_private)
	{
		nh_private.param<std::string>("port_name",port_name_,"/dev/U3");
	
		pub_telecontrol_cmd_ = nh.advertise<little_ant_msgs::ControlCmd>("/sensor_decision",2);
	
	}
	void run()
	{
		boost::thread newThread(boost::bind(&Telecontrol::parse,this));
		
		usleep(10000);
		
		while(ros::ok())
		{
			usleep(20000);
			
			if(telecontrol_cmd_.status == false) 
			{
				pub_telecontrol_cmd_.publish(telecontrol_cmd_);
				continue;
			}
		
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
			
			if((D_change==0) || (right_L_R<2200&&right_L_R>1900))
			{
				steer = 0;
			}
			else 
			{
				steer = (1.0*(2048 - right_L_R) / 2048) * 200;
			}
			
			telecontrol_cmd_.cmd1.set_driverlessMode = true;
			telecontrol_cmd_.cmd1.set_handBrake = hand_brake;
			telecontrol_cmd_.cmd1.set_turnLight_R = right_light;
			telecontrol_cmd_.cmd1.set_turnLight_L = left_light;
			telecontrol_cmd_.cmd2.set_gear = D_change;
			telecontrol_cmd_.cmd2.set_brake = brake;
			telecontrol_cmd_.cmd2.set_speed = speed;
			telecontrol_cmd_.cmd2.set_steeringAngle = steer;

			pub_telecontrol_cmd_.publish(telecontrol_cmd_);
		}
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
			cout << "running...."<<endl;
			boost::mutex::scoped_lock lock(mutex_); 
			data = serial.getPkgPtr();
		
			telecontrol_cmd_.status = data[8]&0x01; //遥控器状态
			if(telecontrol_cmd_.status ==0)
				continue;
		
			left_U_D = data[0]*256+data[1];
			left_L_R = data[6]*256+data[7];
			right_U_D = data[2]*256+data[3];
			right_L_R = data[4]*256+data[5];
			//printf("%d\t%d\t%d\t%d\n",left_U_D,left_L_R,right_U_D,right_L_R);
		}
		serial.closeThread();
		printf("closed telecontrol serial\r\n");
	}
	
private:
	std::string port_name_;

	const uint8_t *data = NULL;
	
	uint16_t left_U_D ;
	uint16_t left_L_R ;
	uint16_t right_U_D;
	uint16_t right_L_R;

	boost::mutex mutex_;
	
	float speed = 0, steer = 0, hand_brake = 0, D_change = 0, left_light = 0, right_light = 0 , brake =0.0;
	
	little_ant_msgs::ControlCmd telecontrol_cmd_;
	
	ros::Publisher pub_telecontrol_cmd_;
	
};



int main(int argc,char** argv)
{
	ros::init(argc,argv,"key_control");
	
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	Telecontrol telecontrol;
	telecontrol.init(nh,nh_private);
	telecontrol.run();
	
	return 0;
}




