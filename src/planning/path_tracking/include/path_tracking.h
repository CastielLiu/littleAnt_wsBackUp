#ifndef PATH_TRACKING_H_
#define PATH_TRACKING_H_
#include<ros/ros.h>
#include<little_ant_msgs/ControlCmd.h>
#include<std_msgs/Int8.h>

#include<little_ant_msgs/State2.h>  //speed
#include"gps_msgs/Inspvax.h"
#include<nav_msgs/Odometry.h> 
#include<geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

#include<std_msgs/Bool.h>
#include<ant_math/ant_math.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <thread>

#define IS_POLAR_COORDINATE_GPS 0

typedef struct
{
	double longitude;
	double latitude;
	double yaw;
	
	double x;
	double y;
	
}gpsMsg_t;


class PathTracking
{
public:
	PathTracking();
	~PathTracking();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	void run();
	float deg2rad(float deg);
	void limitRoadWheelAngle(float& angle);
	
	void read_a_point_from_pathFile(gpsMsg_t& point);
	float point2point_dis(gpsMsg_t &point1,gpsMsg_t &point2);
	std::pair<float, float>  get_dis_yaw(gpsMsg_t &point1,gpsMsg_t &point2);
	
	void pub_gps_cmd_callback(const ros::TimerEvent&);
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg);
	
	void cartesian_gps_callback(const nav_msgs::Odometry::ConstPtr& msg);
	
	void vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg);
	void avoiding_flag_callback(const std_msgs::Int8::ConstPtr& msg);
	bool is_gps_data_valid(gpsMsg_t& point);
	void rosSpinThread();
	
private:
	ros::Subscriber sub_gps_;
	
	ros::Subscriber sub_cartesian_gps_;
	
	ros::Subscriber sub_vehicleState2_;
	
	ros::Subscriber sub_avoiding_from_lidar_;
	
	ros::Timer timer_;
	
	ros::Publisher pub_gps_cmd_;
	
	FILE * fp;
	
	boost::shared_ptr<boost::thread> rosSpin_thread_ptr_;
	
	std::string path_points_file_;
	
	gpsMsg_t current_point_;
	gpsMsg_t target_point_;
	
	float disThreshold_;
	float vehicle_axis_dis_;
	float avoiding_disThreshold_;
	
	little_ant_msgs::ControlCmd gps_controlCmd_;
	
	float path_tracking_speed_;
	
	uint8_t gps_status_;

};


#endif

