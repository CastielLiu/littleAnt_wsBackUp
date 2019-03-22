#ifndef PATH_TRACKING_H_
#define PATH_TRACKING_H_
#include<ros/ros.h>
#include"little_ant_msgs/ControlCmd1.h"
#include"little_ant_msgs/ControlCmd2.h"

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
	void limitSteeringAngle(float& angle);
	
	float point2point_dis(gpsMsg_t &point1,gpsMsg_t &point2);
	std::pair<float, float>  get_dis_yaw(gpsMsg_t &point1,gpsMsg_t &point2);
	
	void pub_cmd2_10ms(const ros::TimerEvent&);
	void pub_cmd1_20ms(const ros::TimerEvent&);
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg);
	void vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg);
	void telecontrolState_callback(const std_msgs::Bool::ConstPtr& msg) ;

	
	
private:
	ros::Subscriber sub_gps;
	ros::Subscriber sub_telecontrol;
	ros::Subscriber sub_vehicleState2;
	
	ros::Timer timer1;
	ros::Timer timer2;
	
	ros::Publisher pub_cmd1;
	ros::Publisher pub_cmd2;
	FILE * fp;
	
	
	std::string path_points_file_;
	
	gpsMsg_t current_point;
	gpsMsg_t target_point;
	
	float disThreshold_;
	float vehicle_axis_dis;
	
	little_ant_msgs::ControlCmd1 controlCmd1;
	little_ant_msgs::ControlCmd2 controlCmd2;
	
	bool is_telecontrol;
	

};


 




#endif

