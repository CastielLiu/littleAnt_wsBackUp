#include<ros/ros.h>
#include<gps_msgs/Inspvax.h>
#include <unistd.h>
#include<cmath>
#include<nav_msgs/Odometry.h> 
#include<ant_math/ant_math.h>
#include<little_ant_msgs/PathInfo.h>
#include<little_ant_msgs/State2.h> //speed
#include<little_ant_msgs/State4.h> //steeringAngle
#include<message_filters/subscriber.h>
#include<message_filters/time_synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>

using namespace std;

class Record
{
public:
	typedef message_filters::sync_policies::ApproximateTime<gps_msgs::Inspvax, nav_msgs::Odometry, little_ant_msgs::State2, little_ant_msgs::State4> MySyncPolicy;
	Record();
	~Record();
	bool init();
	void recordToFile();
	
private:
	float dis2points(gpsMsg_t & point1,gpsMsg_t& point2);
	void record_callback(const gps_msgs::Inspvax::ConstPtr& gps, 
					 const nav_msgs::Odometry::ConstPtr& utm,
					 const little_ant_msgs::State2::ConstPtr& state2,
					 const little_ant_msgs::State4::ConstPtr& state4);
	FILE *fp;
	gpsMsg_t last_point_ , current_point_;
	float sample_distance_;
	
	unique_ptr<message_filters::Subscriber<gps_msgs::Inspvax>> sub_gps_;
	unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> sub_utm_;
	unique_ptr<message_filters::Subscriber<little_ant_msgs::State2>> sub_state2_;
	unique_ptr<message_filters::Subscriber<little_ant_msgs::State4>> sub_state4_;
	unique_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
	
};

Record::Record()
{
	last_point_ = {0.0,0.0,0.0,0.0,0.0};
	current_point_ = last_point_;
}

Record::~Record()
{
	if(fp != NULL)
		fclose(fp);
}

bool Record::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	
	std::string file_path = private_nh.param<std::string>("file_path","");
	std::string file_name = private_nh.param<std::string>("file_name","");

	if(file_path.empty() || file_name.empty())
	{
		ROS_ERROR("please input record file path and file name in launch file!!!");
		return false;
	}
	if(file_name[file_name.length()-1] != '/')
		file_name += '/';
	fp = fopen((file_path + file_name).c_str(),"w");
	
	if(fp == NULL)
	{
		ROS_INFO("open record data file %s failed !!!",(file_path+file_name).c_str());
		return false;
	}
	
	
	
	private_nh.param<float>("sample_distance",sample_distance_,0.1);
	
	sub_gps_.reset(new message_filters::Subscriber<gps_msgs::Inspvax>(nh,"/gps",10));
	sub_utm_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh,"/gps_odom",10));
	sub_state2_.reset(new message_filters::Subscriber<little_ant_msgs::State2>(nh,"/vehicleState2",10));
	sub_state4_.reset(new message_filters::Subscriber<little_ant_msgs::State4>(nh,"/vehicleState4",10));
	
	sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),*sub_gps_,*sub_utm_,*sub_state2_,*sub_state4_));
	sync_->registerCallback(boost::bind(&Record::record_callback, this, _1, _2,_3,_4));
			
	return true;
}

void Record::record_callback(const gps_msgs::Inspvax::ConstPtr& gps, 
					 const nav_msgs::Odometry::ConstPtr& utm,
					 const little_ant_msgs::State2::ConstPtr& state2,
					 const little_ant_msgs::State4::ConstPtr& state4)
{
	static int row_num = 0;
	current_point_.latitude = gps->latitude;
	current_point_.longitude = gps->longitude;
	current_point_.yaw = gps->azimuth*M_PI/180.0;
	
	current_point_.x = utm->pose.pose.position.x;
	current_point_.y = utm->pose.pose.position.y;
	
	float speed = state2->vehicle_speed;
	float roadwheelAngle = state4->roadwheelAngle;
	
	if(sample_distance_*sample_distance_ <= dis2points(current_point_,last_point_))
	{
		fprintf(fp,"%.3f\t%.3f\t%.3f\t%.7f\t%.7f\t%.2f\t%.2f\r\n",
					current_point_.x,current_point_.y,current_point_.yaw,current_point_.longitude,current_point_.latitude,
					speed,roadwheelAngle);
		fflush(fp);
		
		row_num++;
		
		ROS_INFO("row:%3d recording...", row_num);
		last_point_ = current_point_;
	}
	
	
}

float Record::dis2points(gpsMsg_t & point1,gpsMsg_t& point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
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


