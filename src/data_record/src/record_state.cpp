#include<iostream>
#include<cstdio>
#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<little_ant_msgs/State4.h>  //steeringAngle  hz 35
#include<little_ant_msgs/State2.h>  //vehicle_speed  hz 19
#include<gps_msgs/Inspvax.h>

using namespace std;

FILE * fp = NULL;
float steeringAngle = 0.0;

bool is_state4_ok =false;

void state2_callback(const little_ant_msgs::State2::ConstPtr& state2)
{
	static size_t i=0;
	ROS_INFO("vehicleState:%d",i);
	i++;
	fprintf(fp,"%.2f,%.2f\r\n",state2->vehicle_speed,steeringAngle);
	fflush(fp);
}
void state4_callback(const little_ant_msgs::State4::ConstPtr& state4)
{
	if(!is_state4_ok)
		is_state4_ok = true;
	steeringAngle = state4->steeringAngle;
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"state_record_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	ros::Subscriber sub_steering = nh.subscribe("/vehicleState4",0,&state4_callback);
	
	std::string _file_name;
	nh_private.param<std::string>("file_name",_file_name,"data");
	_file_name = _file_name+"_vehicleState.txt";
	
	fp = fopen(_file_name.c_str(),"w");
	if(fp==NULL)
	{
		ROS_ERROR("open file failed!");
		return 1;
	}
	else
		ROS_INFO("open %s ok",_file_name.c_str());
		
	fprintf(fp,"speed,steeringAngle\r\n");
	while(ros::ok() && (!is_state4_ok))
	{
		static size_t count=0;
		if(count%10==0)
			ROS_INFO("wait for vehicle state init....");
		++count;
		ros::spinOnce();
		usleep(20000);
	}
	ros::Subscriber sub_speed=nh.subscribe("/vehicleState2",0,&state2_callback);
	ros::spin(); 
	return 0;
}

