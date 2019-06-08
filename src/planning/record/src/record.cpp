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

		float calculate_dis2(gpsMsg_t & point1,gpsMsg_t& point2);
		

		std::string file_path_;
		FILE *fp;
		FILE *fp_lat_lon;
		
		gpsMsg_t last_point , current_point;
		
		float sample_distance_;
		ros::Subscriber sub_gps_;
		
		ros::Subscriber sub_cartesian_gps_ ;
		
		ros::Subscriber sub_pathInfo_;
		
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
	
	private_nh.param<std::string>("file_path",file_path_,"a.txt");
	
	int _temp;
	private_nh.param<int>("traffic_sign",_temp,0);
	

	sub_gps_= nh.subscribe("/gps",1,&Record::gps_callback,this);
	
    if(file_path_.empty())
    {
    	ROS_ERROR("no input file...");
    	return false;
    }
    
	fp_lat_lon = fopen((file_path_).c_str(),"w");
	
	if(fp_lat_lon == NULL)
	{
		ROS_INFO("open record data file %s failed !!!",(file_path_+"_lat_lon").c_str());
		return 0;
	}
	return 1;
}

void Record::gps_callback(const gps_msgs::Inspvax::ConstPtr& gps)
{
		fprintf(fp_lat_lon,"%.8f\t%.8f\t%.3f\t%.5f\t%.5f\r\n",
				gps->longitude,gps->latitude,gps->azimuth,gps->roll,gps->pitch);
		fflush(fp_lat_lon);
}

float Record::calculate_dis2(gpsMsg_t & point1,gpsMsg_t& point2)
{
	float x = (point1.longitude -point2.longitude)*111000*cos(point1.latitude * M_PI/180.);
	float y = (point1.latitude - point2.latitude) *111000;
	return x*x+y*y;
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


