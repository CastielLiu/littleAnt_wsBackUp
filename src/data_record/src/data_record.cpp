#include<iostream>
#include<cstdio>
#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<little_ant_msgs/State4.h>  //steeringAngle
#include<little_ant_msgs/State2.h>  //vehicle_speed
#include<gps_msgs/Inspvax.h>

FILE * fp = NULL;
float vehicleSpeed = 0.0;
float steeringAngle = 0.0;
sensor_msgs::Imu imuData;
gps_msgs::Inspvax gpsMsg;

bool is_imu_ok = false,
	is_gps_ok = false,
	is_state2_ok = false,
	is_state4_ok =false;

void imu_callback(const sensor_msgs::Imu::ConstPtr& imu)
{
	is_imu_ok = true;
	imuData = *imu;
}
void gps_callback(const gps_msgs::Inspvax::ConstPtr& gps)
{
	is_gps_ok =true;
	gpsMsg = *gps;
}
void state2_callback(const little_ant_msgs::State2::ConstPtr& state2)
{
	is_state2_ok = true;
	vehicleSpeed = state2->vehicle_speed;
}
void state4_callback(const little_ant_msgs::State4::ConstPtr& state4)
{
	is_state4_ok = true;
	steeringAngle = state4->steeringAngle;
}

void timerCallback(const ros::TimerEvent&)
{
	static size_t i=1;
	
	if(is_gps_ok && is_imu_ok && is_state2_ok && is_state4_ok )
	{
		//gps
		fprintf(fp,"%.8f,%.8f,%.2f;%.2f,%.2f,%.2f;%.2f,%.2f,%.2f;",
				gpsMsg.longitude,gpsMsg.latitude,gpsMsg.height,
				gpsMsg.roll,gpsMsg.azimuth,gpsMsg.pitch,
				gpsMsg.north_velocity,gpsMsg.east_velocity,gpsMsg.up_velocity);
		//imu
		fprintf(fp,"%.2f,%.2f,%.2f;%.2f,%.2f,%.2f;",
				imuData.angular_velocity.x,imuData.angular_velocity.y,imuData.angular_velocity.z,
				imuData.linear_acceleration.x,imuData.linear_acceleration.y,imuData.linear_acceleration.z);
		//vehicle
		fprintf(fp,"%.2f,%.2f\r\n",vehicleSpeed,steeringAngle);
		
		fflush(fp);
		
		ROS_INFO("%4d pieces of data have been recorded",i);
		
		++i;
	}
	else
	{
		static size_t count=0;
		if(count%10==0)
			ROS_INFO("wait for sensors init....");
		++count;
	}
}

int main(int argc,char **argv)
{
	if(argc <=1)
	{
		ROS_ERROR("please input data file path!");
		return 1;
	}
	ros::init(argc,argv,"data_record_node");
	ros::NodeHandle nh;
	ros::Subscriber sub_imu= nh.subscribe("/imu",1,&imu_callback);
	ros::Subscriber sub_speed=nh.subscribe("/state2",1,&state2_callback);
	ros::Subscriber sub_steering = nh.subscribe("/state4",1,&state4_callback);
	ros::Subscriber sub_gps = nh.subscribe("/gps",1,&gps_callback);
	fp = fopen(argv[1],"w");
	if(fp==NULL)
	{
		ROS_ERROR("open file failed!");
		return 1;
	}
	fprintf(fp,"  longitude ,  latitude ,high;roll,yaw,pitch;n_vel,e_vel,u_vel;angleSpeedx_y_z;lineAccx_y_z;speed,steeringAngle\r\n");
	ros::Timer timer = nh.createTimer(ros::Duration(0.05), &timerCallback);
	ros::spin();
	return 0;
}

