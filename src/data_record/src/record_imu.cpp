#include<iostream>
#include<cstdio>
#include<ros/ros.h>
#include<sensor_msgs/Imu.h>

using namespace std;

FILE * fp = NULL;

void imu_callback(const sensor_msgs::Imu::ConstPtr& imu)
{
	//imu
	static size_t i=0;
	ROS_INFO("imu:%d",i++);
	
	fprintf(fp,"%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\r\n", imu->header.stamp.toSec(),
			imu->angular_velocity.x,imu->angular_velocity.y,imu->angular_velocity.z,
			imu->linear_acceleration.x,imu->linear_acceleration.y,imu->linear_acceleration.z);
	
	fflush(fp);
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"imu_record_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	ros::Subscriber sub_imu= nh.subscribe("/raw_imu",0,&imu_callback);
	std::string _file_name;
	nh_private.param<std::string>("file_name",_file_name,"data");
	
	_file_name = _file_name+"_imu.txt";
	fp = fopen(_file_name.c_str(),"w");
	if(fp==NULL)
	{
		ROS_ERROR("open %s failed",_file_name.c_str());
		return 0;
	}
	else
		ROS_INFO("open %s ok",_file_name.c_str());
	
	fprintf(fp,"stamp    angle_speed_x_y_z \t line_acc_x_y_z\r\n");
	ros::spin();

	return 0;
}

