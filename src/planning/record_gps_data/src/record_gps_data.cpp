#include<ros/ros.h>
#include "gps_msgs/Inspvax.h"
#include <unistd.h>
#include<cmath>
#include<nav_msgs/Odometry.h> 

#ifndef PI_
#define PI_ 3.141592653589
#endif


#define GPS_POLAR_COORDINATE 0

typedef struct
{
	double latitude;
	double longitude;
	
	double x;
	double y;
	
}location_t;

class Record
{
	private:
#if GPS_POLAR_COORDINATE==1
		void gps_callback(const gps_msgs::Inspvax::ConstPtr& gpsMsg);
#else
		void cartesian_gps_callback(const nav_msgs::Odometry::ConstPtr& msg);
#endif		
		void timerCallback(const ros::TimerEvent&);
		float calculate_dis2(location_t & point1,location_t& point2);
		
		std::string file_path_;
		FILE *fp;
		
		location_t last_point , current_point;
		
		float sample_distance_;
		ros::Subscriber gps_sub;
		
		ros::Subscriber sub_cartesian_gps_ ;
		
		ros::Timer timer;
		
		
	public:
		Record();
		~Record();
		bool init();
		void recordToFile();
};

Record::Record()
{
	last_point.longitude = 0.0;
	last_point.latitude = 0.0;
}

Record::~Record()
{
	fclose(fp);
}

bool Record::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	
	private_nh.param<std::string>("file_path",file_path_,"/home/wendao/projects/littleAnt_ws/a.txt");
	private_nh.param<float>("sample_distance",sample_distance_,0.1);

#if GPS_POLAR_COORDINATE==1	
	gps_sub= nh.subscribe("/gps",1,&Record::gps_callback,this);
#else	
	sub_cartesian_gps_ = nh.subscribe("/gps_odom",2,&Record::cartesian_gps_callback,this);
#endif    
    if(file_path_.empty())
    {
    	ROS_ERROR("no input file...");
    	return false;
    }
    
	fp = fopen(file_path_.c_str(),"w");
	
	if(fp == NULL)
	{
		ROS_INFO("open record data file failed !!!!!");
		return 0;
	}
	return 1;
	
	//timer = nh.createTimer(ros::Duration(0.05),&Record::timerCallback,this);
}

#if GPS_POLAR_COORDINATE ==1
float Record::calculate_dis2(location_t & point1,location_t& point2)
{
	float x = (point1.longitude -point2.longitude)*111000*cos(point1.latitude * M_PI/180.);
	float y = (point1.latitude - point2.latitude) *111000;
	return x*x+y*y;
}

void Record::gps_callback(const gps_msgs::Inspvax::ConstPtr & gps)
{
	current_point.latitude = gps->latitude;
	current_point.longitude = gps->longitude;
		
	if(sample_distance_*sample_distance_ <= calculate_dis2(current_point,last_point))
	{
		fprintf(fp,"%.8f\t%.8f\r\n",current_point.longitude,current_point.latitude);
		fflush(fp);
		
		ROS_INFO("%.8f\t%.8f\r\n",current_point.longitude,current_point.latitude);
		last_point = current_point;
	}
}

#else
float Record::calculate_dis2(location_t & point1,location_t& point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	return x*x+y+y;
}

void Record::cartesian_gps_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	current_point.x = msg->pose.pose.position.x;
	current_point.y = msg->pose.pose.position.y;
		
	if(sample_distance_*sample_distance_ <= calculate_dis2(current_point,last_point))
	{
		fprintf(fp,"%.3f\t%.3f\r\n",current_point.x,current_point.y);
		fflush(fp);
		
		ROS_INFO("%.3f\t%.3f\r\n",current_point.x,current_point.y);
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


