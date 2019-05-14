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
		
		void path_info_callback(const little_ant_msgs::PathInfo::ConstPtr& info);

		std::string file_path_;
		FILE *fp;
		FILE *fp_lat_lon;
		
		gpsMsg_t last_point , current_point;
		
		float sample_distance_;
		ros::Subscriber sub_gps_;
		
		ros::Subscriber sub_cartesian_gps_ ;
		
		ros::Subscriber sub_pathInfo_;
		
		bool gps_status_;
		
		little_ant_msgs::PathInfo path_info_;
		
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
	path_info_.other_info = 0;
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
	
	private_nh.param<std::string>("file_path",file_path_,"/home/wendao/projects/littleAnt_ws/a.txt");
	private_nh.param<float>("sample_distance",sample_distance_,0.1);
	
	private_nh.param<float>("initial_maxOffset_left",path_info_.maxOffset_left,0.0);
	private_nh.param<float>("initial_maxOffset_right",path_info_.maxOffset_right,0.0);
	
	int _temp;
	private_nh.param<int>("traffic_sign",_temp,0);
	path_info_.traffic_sign = _temp;
	

	sub_gps_= nh.subscribe("/gps",1,&Record::gps_callback,this);
	sub_pathInfo_= nh.subscribe("/path_info",1,&Record::path_info_callback,this);
	
#if IS_POLAR_COORDINATE_GPS==0
	sub_cartesian_gps_ = nh.subscribe("/gps_odom",2,&Record::cartesian_gps_callback,this);
#endif    
    if(file_path_.empty())
    {
    	ROS_ERROR("no input file...");
    	return false;
    }
    
	fp = fopen(file_path_.c_str(),"w");
	fp_lat_lon = fopen((file_path_+"_lat_lon").c_str(),"w");
	
	if(fp == NULL)
	{
		ROS_INFO("open record data file %s failed !!!",file_path_.c_str());
		return 0;
	}
	else if(fp_lat_lon == NULL)
	{
		ROS_INFO("open record data file %s failed !!!",(file_path_+"_lat_lon").c_str());
		return 0;
	}
	return 1;
	
	
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
	current_point.x = msg->pose.pose.position.x;
	current_point.y = msg->pose.pose.position.y;
	if(gps_status_ == true && sample_distance_*sample_distance_ <= calculate_dis2(current_point,last_point))
	{
		gps_status_ = false;
		//x,y,theta,offset_l,offset_r,traffic_sign
		fprintf(fp,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d\t%d\r\n",current_point.x,current_point.y,current_point.yaw,
													path_info_.maxOffset_left,path_info_.maxOffset_right,
													path_info_.traffic_sign,path_info_.other_info);
		fflush(fp);
		
		fprintf(fp_lat_lon,"%.8f\t%.8f\n",current_point.latitude,current_point.longitude);
		
		fflush(fp_lat_lon);
		
		
		ROS_INFO("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d\t%d\r\n",current_point.x,current_point.y,current_point.yaw,
													path_info_.maxOffset_left,path_info_.maxOffset_right,
													path_info_.traffic_sign,path_info_.other_info);
		last_point = current_point;
	}
}

#endif

void Record::path_info_callback(const little_ant_msgs::PathInfo::ConstPtr& info)
{
	path_info_ = *info;
}


int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_data_node");
	
	Record record;
	
	if(!record.init())
		return 1;

	ros::spin();
	
	return 0;
}


