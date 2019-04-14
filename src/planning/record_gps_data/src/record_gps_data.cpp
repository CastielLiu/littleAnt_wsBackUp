#include<ros/ros.h>
#include "gps_msgs/Inspvax.h"
#include <unistd.h>
#include<cmath>

#ifndef PI_
#define PI_ 3.141592653589
#endif

typedef struct
{
	double latitude;
	double longitude;
}location_t;

class Record
{
	private:
		void gps_callback(const gps_msgs::Inspvax::ConstPtr& gpsMsg);
		void timerCallback(const ros::TimerEvent&);
		float calculate_dis2(location_t & point1,location_t& point2);
		
		std::string file_path_;
		FILE *fp;
		
		location_t last_point , current_point;
		
		float sample_distance_;
		ros::Subscriber gps_sub;
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
	gps_sub= nh.subscribe("/gps",1,&Record::gps_callback,this);
    
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

int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_data_node");
	
	Record record;
	
	if(!record.init())
		return 1;

	ros::spin();
	
	return 0;
}


