#ifndef LANE_KEEPING_H_
#define LANE_KEEPING_H_
#include<ros/ros.h>
#include<little_ant_msgs/Lane.h>
#include<little_ant_msgs/State2.h>
#include<little_ant_msgs/ControlCmd.h>
#include<little_ant_msgs/ControlCmd1.h>
#include<little_ant_msgs/ControlCmd2.h>
#include<ant_math/ant_math.h>

#include<nav_msgs/Odometry.h>
#include<gps_msgs/Inspvax.h>

#include<geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

#include<vector>


class LaneKeeping
{
public:
	LaneKeeping();
	~LaneKeeping();
	
	bool init();
	void run();
	void pub_cmd_callback(const ros::TimerEvent&);
	void laneDetect_callback(const little_ant_msgs::Lane::ConstPtr& msg);
	void vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg);
	
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg);
	void cartesian_gps_callback(const nav_msgs::Odometry::ConstPtr& msg);
	
	
	
private:
	typedef struct
	{
		little_ant_msgs::Lane lane_msg;
		float vehicle_speed;
		
	}status_msgs_t;
	
	int get_steeringDir(float err,float theta,float alpha);
	void changeLane(int dir);
	void generate_laneChange_points(int dir);
	
	typedef enum
	{
		ChangeLine_None = 0,
		ChangeLine_Left = 1,
		ChangeLine_Right = 2,
		ChangeLine_Ok = 3
		
	} change_lane_status_t;
	
private:
	ros::Subscriber sub_laneMsg_;
	ros::Subscriber sub_vehicleSpeed_;
	ros::Subscriber sub_cartesian_gps_;
	ros::Subscriber sub_polar_gps_;
	
	ros::Timer pub_cmd_20ms_;
	
	ros::Publisher pub_controlCmd_;
	
	little_ant_msgs::ControlCmd cmd_;
	little_ant_msgs::ControlCmd1 cmd1_;
	little_ant_msgs::ControlCmd1 cmd2_;
	
	float foresight_distance_;
	float lane_keeping_Speed_;
	
	float vehicle_speed_;
	float mean_vehicleSpeed_;
	
	enum{Max_history_size=10};
	
	little_ant_msgs::Lane lane_msg_;
	status_msgs_t history_status_msgs_[Max_history_size];
	int current_lane_msg_index;
	
	int system_delay_;
	
	std::vector<gpsMsg_t> temp_targetArray_;
	
	gpsMsg_t current_point_;
	
	uint8_t gps_status_;
	
	change_lane_status_t changeLane_status_;

};




#endif
