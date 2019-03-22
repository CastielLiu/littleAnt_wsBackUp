#include<iostream>
#include "getPkgFromDev.h"
#include"little_ant_msgs/ControlCmd1.h"
#include"little_ant_msgs/ControlCmd2.h"
#include<std_msgs/Bool.h>
#include<boost/thread.hpp>
#include<boost/bind.hpp>

#include<ros/ros.h>

using namespace std;


class Telecontrol
{
public:
	
	Telecontrol(){}
	~Telecontrol(){}
	
	void init(ros::NodeHandle nh,ros::NodeHandle nh_private)
	{
		nh_private.param<std::string>("port_name",port_name_,"/dev/ttyUSB0");
	
		timer =nh.createTimer(ros::Duration(0.03),&Telecontrol::send_is_telecontrol,this);
	
		pub_cmd1 = nh.advertise<little_ant_msgs::ControlCmd1>("/controlCmd1",10);
		pub_cmd2 = nh.advertise<little_ant_msgs::ControlCmd2>("/controlCmd2",10);
	
		pub_is_tele =nh.advertise<std_msgs::Bool>("/is_telecontrol",5);
	}
	void run()
	{
		boost::thread newThread(boost::bind(&Telecontrol::parse,this));
		boost::thread spinThread(boost::bind(&Telecontrol::ros_spin,this));
		usleep(10000);
		while(ros::ok())
		{
			//ROS_DEBUG("runing...");
			if(is_telecontrol_mode==false) 
			{
				usleep(10000);
				continue;
			}
			//ROS_ERROR("stm32   is_telecontrol_mode:%d",is_telecontrol_mode);
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
		}
	}
	void send_is_telecontrol(const ros::TimerEvent&)
	{
		//ROS_DEBUG("stm32   is_telecontrol_mode:%d",is_telecontrol_mode);
		boost::mutex::scoped_lock lock(mutex_); 
		std_msgs::Bool _mode_ ;
		_mode_.data = is_telecontrol_mode;

		pub_is_tele.publish(_mode_);
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
		printf("closed telecontrol serial\r\n");
	}
	void ros_spin()
	{
		ros::spin();
	}

private:
	std::string port_name_;

	const uint8_t *data = NULL;
	
	uint16_t left_U_D ;
	uint16_t left_L_R ;
	uint16_t right_U_D;
	uint16_t right_L_R;
	bool is_telecontrol_mode;

	boost::mutex mutex_;
	
	float speed = 0, steer = 0, hand_brake = 0, D_change = 0, left_light = 0, right_light = 0 , brake =0.0;
	little_ant_msgs::ControlCmd1 cmd1;
	little_ant_msgs::ControlCmd2 cmd2;
	ros::Timer timer;
	ros::Publisher pub_cmd1 ;
	ros::Publisher pub_cmd2 ;
	ros::Publisher pub_is_tele;
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




