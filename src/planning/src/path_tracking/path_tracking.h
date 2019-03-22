#ifndef PATH_TRACKING_H_
#define PATH_TRACKING_H_
#include<ros/ros.h>
#include"little_ant_msgs/ControlCmd1.h"
#include"little_ant_msgs/ControlCmd2.h"

#include<little_ant_msgs/State2.h>  //speed
#include"gps_msgs/Inspvax.h"
#include<std_msgs/Bool.h>


class Pursuit
{
public:
	Pursuit();
	~Pursuit();
	


	
	
	
};

class PathTracking
{
public:
	PathTracking();
	~PathTracking();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg);
	void vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg);
	void telecontrolState_callback(const std_msgs::Bool::ConstPtr& msg) ;
	
	
private:
	ros::Subscriber sub_gps;
	ros::Subscriber sub_telecontrol;
	ros::Subscriber sub_vehicleState2;
	
	ros::Publisher pub_cmd1;
	ros::Publisher pub_cmd2;

};















#endif

