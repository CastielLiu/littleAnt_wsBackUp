#include<iostream>
#include<cstdio>
#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<little_ant_msgs/State4.h>  //steeringAngle
#include<little_ant_msgs/State2.h>  //vehicle_speed
#include<gps_msgs/Inspvax.h>

using namespace std;

FILE * fp = NULL;


void gps_callback(const gps_msgs::Inspvax::ConstPtr& gps)
{
	static size_t i=0;
	ROS_INFO("gps:%d",i);
	i++;
	
	//gps
	fprintf(fp,"%.8f,%.8f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
			gps->longitude,gps->latitude,gps->height,
			gps->roll,gps->azimuth,gps->pitch,
			gps->north_velocity,gps->east_velocity,gps->up_velocity);
	fflush(fp);
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"gps_record_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	ros::Subscriber sub_gps = nh.subscribe("/gps",0,&gps_callback);
	std::string _file_name;
	nh_private.param<std::string>("file_name",_file_name,"data");
	
	_file_name = _file_name+"_gps.txt";
	
	fp = fopen(_file_name.c_str(),"w");
	
	if(fp==NULL)
	{
		ROS_ERROR("open %s failed",_file_name.c_str());
		return 0;
	}
	else
		ROS_INFO("open %s ok",_file_name.c_str());
	
	fprintf(fp,"  longitude ,  latitude ,high;roll,yaw,pitch;n_vel,e_vel,u_vel\r\n");
	
	ros::spin(); 

	return 0;
}

