#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

#include <little_ant_msgs/State1.h>
#include <little_ant_msgs/State2.h>
#include <little_ant_msgs/State3.h>
#include <little_ant_msgs/State4.h>


#include <little_ant_msgs/ControlCmd1.h>
#include <little_ant_msgs/ControlCmd2.h>


float g_set_brake;
float g_get_speed;
float g_origin_speed;
bool g_start_calculate =false;

double g_timer_begin;
double g_timer_end;


void state_callBack(const little_ant_msgs::State2::ConstPtr msg)
{
	size_t i =0;
	float speed = 0.0;
	if(msg->wheel_speed_FL_valid==true) {i++; speed+=msg->wheel_speed_FL;}
	if(msg->wheel_speed_FR_valid==true) {i++; speed+=msg->wheel_speed_FR;}
	if(msg->wheel_speed_RL_valid==true) {i++; speed+=msg->wheel_speed_RL;}
	if(msg->wheel_speed_RR_valid==true) {i++; speed+=msg->wheel_speed_RR;}
	speed = speed/i /3.6; //m/s
	if(speed ==0 &&g_start_calculate==true )
	{
		g_timer_end = ros::Time::now().toSec(); //end timing
		
		float deceleration = g_origin_speed / (g_timer_end - g_timer_begin);
		
		ROS_INFO("originSpeed=%f\t brake=%f\t time=%f deceleration=%f\t deceleration_cofficient=%f",
					g_origin_speed,g_set_brake,g_timer_end - g_timer_begin,deceleration,g_set_brake/deceleration);
		
		g_start_calculate = false;
	}
	else
		g_get_speed = speed;
	
}


void cmd_callBack(const little_ant_msgs::ControlCmd2::ConstPtr msg)
{
	float set_brake = msg->set_brake;
	if(set_brake!=0)
	{
		g_timer_begin = ros::Time::now().toSec(); //start timing
		g_origin_speed = g_get_speed ;
		g_set_brake = set_brake;
		g_start_calculate = true;
	}
	
}
	


int main(int argc,char**argv)
{
	ros::init(argc,argv,"calculate_deceleration_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	
	ros::Subscriber sub1 = nh.subscribe("/vehicleState2",1,&state_callBack);
	ros::Subscriber sub2 = nh.subscribe("/ControlCmd2",1,&cmd_callBack);
	
	//ROS_INFO("System initialization completed");
	
	
	ros::spin();
	
	return 1;
}



