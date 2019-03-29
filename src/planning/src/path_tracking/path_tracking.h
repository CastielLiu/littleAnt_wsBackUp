#ifndef PATH_TRACKING_H_
#define PATH_TRACKING_H_
#include<ros/ros.h>
#include<little_ant_msgs/ControlCmd.h>
#include<std_msgs/Int8.h>

#include<little_ant_msgs/State2.h>  //speed
#include"gps_msgs/Inspvax.h"
#include<std_msgs/Bool.h>


typedef struct
{
	double longitude;
	double latitude;
	float yaw;
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
	
	float point2point_dis(gpsMsg_t &point1,gpsMsg_t &point2);
	std::pair<float, float>  get_dis_yaw(gpsMsg_t &point1,gpsMsg_t &point2);
	
	void pub_gps_cmd_callback(const ros::TimerEvent&);
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg);
	void vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg);
	void avoiding_flag_callback(const std_msgs::Int8::ConstPtr& msg);

	
	
private:
	ros::Subscriber sub_gps_;
	ros::Subscriber sub_vehicleState2_;
	ros::Subscriber sub_avoiding_from_lidar_;
	
	ros::Timer timer_;
	
	ros::Publisher pub_gps_cmd_;
	FILE * fp;
	
	
	std::string path_points_file_;
	
	gpsMsg_t current_point_;
	gpsMsg_t target_point_;
	
	float disThreshold_;
	float vehicle_axis_dis_;
	float avoiding_disThreshold_;
	
	little_ant_msgs::ControlCmd gps_controlCmd_;
	
	float speed_;

};


 




#endif

