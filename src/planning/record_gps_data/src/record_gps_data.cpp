#include<ros/ros.h>
#include "gps_msgs/Inspvax.h"
#include <unistd.h>
#include<cmath>
#include<nav_msgs/Odometry.h> 
#include<ant_math/ant_math.h>
#include<little_ant_msgs/PathInfo.h>

#ifndef PI_
#define PI_ 3.141592653589
#endif

class Record
{
	private:
		void gps_callback(const gps_msgs::Inspvax::ConstPtr& gpsMsg);
#if IS_POLAR_COORDINATE_GPS==0
		void cartesian_gps_callback(const nav_msgs::Odometry::ConstPtr& msg);
#endif
		float calculate_dis2(gpsMsg_t & point1,gpsMsg_t& point2);
		
		std::string file_path_;
		std::string	file_name_;
		
		FILE *fp;
		FILE *fp_lat_lon;
		
		gpsMsg_t last_point , current_point;
		
		float sample_distance_;
		ros::Subscriber sub_gps_;
		
		ros::Subscriber sub_cartesian_gps_ ;
		
		bool gps_status_;
		
	public:
		Record();
		~Record();
		bool init();
		void recordToFile();
};

Record::Record():
	gps_status_(0)
{
	last_point = {0.0,0.0,0.0,0.0,0.0};
	current_point = last_point;
}

Record::~Record()
{
	if(fp != NULL)
		fclose(fp);
	if(fp_lat_lon != NULL)
		fclose(fp_lat_lon);
}

bool Record::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	
	private_nh.param<std::string>("file_path",file_path_,"");
	private_nh.param<std::string>("file_name",file_name_,"");
	
	if(file_path_.empty() || file_name_.empty())
	{
		ROS_ERROR("please input record file path and file name in launch file!!!");
		return false;
	}
	
	private_nh.param<float>("sample_distance",sample_distance_,0.1);

	sub_gps_= nh.subscribe("/gps",1,&Record::gps_callback,this);
	
#if IS_POLAR_COORDINATE_GPS==0
	sub_cartesian_gps_ = nh.subscribe("/best_utm",2,&Record::cartesian_gps_callback,this);
#endif    
    
	fp = fopen((file_path_+file_name_).c_str(),"w");
	
	fp_lat_lon = fopen((file_path_+"0"+file_name_).c_str(),"w");
	
	if(fp == NULL)
	{
		ROS_INFO("open record data file %s failed !!!",(file_path_+file_name_).c_str());
		return false;
	}
	else if(fp_lat_lon == NULL)
	{
		ROS_INFO("open record data file %s failed !!!",(file_path_+"0"+file_name_).c_str());
		return false;
	}
	return true;
	
	
}

#if IS_POLAR_COORDINATE_GPS ==1
void Record::gps_callback(const gps_msgs::Inspvax::ConstPtr& gps)
{
	current_point.latitude = gps->latitude;
	current_point.longitude = gps->longitude;
	current_point.yaw = gps->azimuth;
	if(sample_distance_*sample_distance_ <= calculate_dis2(current_point,last_point))
	{
		fprintf(fp,"%.8f\t%.8f\t%.3f\r\n",current_point.longitude,current_point.latitude,current_point.azimuth);
		fflush(fp);
		
		ROS_INFO("%.8f\t%.8f\r\n",current_point.longitude,current_point.latitude);
		last_point = current_point;
	}
}

float Record::calculate_dis2(gpsMsg_t & point1,gpsMsg_t& point2)
{
	float x = (point1.longitude -point2.longitude)*111000*cos(point1.latitude * M_PI/180.);
	float y = (point1.latitude - point2.latitude) *111000;
	return x*x+y*y;
}

#else
void Record::gps_callback(const gps_msgs::Inspvax::ConstPtr& gps)
{
	gps_status_ = true;
	current_point.latitude = gps->latitude;
	current_point.longitude = gps->longitude;
	current_point.yaw = gps->azimuth*M_PI/180.0;
}

float Record::calculate_dis2(gpsMsg_t & point1,gpsMsg_t& point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	return x*x+y*y;
}

void Record::cartesian_gps_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	static size_t  row_num = 0;
	current_point.x = msg->pose.pose.position.x;
	current_point.y = msg->pose.pose.position.y;
	if(gps_status_ == true && sample_distance_*sample_distance_ <= calculate_dis2(current_point,last_point))
	{
		gps_status_ = false;
		//x,y,theta,offset_l,offset_r,traffic_sign
		fprintf(fp,"%.3f\t%.3f\t%.3f\r\n",current_point.x,current_point.y,current_point.yaw);
		fflush(fp);
		
		fprintf(fp_lat_lon,"%.8f\t%.8f\t%.3f\n",current_point.latitude,current_point.longitude,current_point.yaw);
		
		fflush(fp_lat_lon);
		
		row_num++;
		
		ROS_INFO("row:%d\t%.3f\t%.3f\t%.3f\r\n",row_num,current_point.x,current_point.y,current_point.yaw);
		last_point = current_point;
	}
}

#endif

int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_data_node");
	
	Record record;
	
	if(!record.init())
		return 1;

	ros::spin();
	
	return 0;
}


