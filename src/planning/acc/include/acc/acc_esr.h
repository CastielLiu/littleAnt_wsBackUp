#ifndef ACC_ESR_H_
#define ACC_ESR_H_
#include<ros/ros.h>
#include <little_ant_msgs/ControlCmd.h>
#include<little_ant_msgs/ControlCmd1.h>
#include<little_ant_msgs/ControlCmd2.h>

#include<little_ant_msgs/State1.h>
#include<little_ant_msgs/State2.h>
#include<esr_radar_msgs/Object.h>
#include<esr_radar_msgs/Objects.h>
#include<std_msgs/Bool.h>

class Acc_esr
{
public:
	Acc_esr(ros::NodeHandle nh,ros::NodeHandle nh_private);
	~Acc_esr(){};
	
	void vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg);
	void object_callback(const esr_radar_msgs::Objects::ConstPtr& objects);
	void is_acc_callback(const std_msgs::Bool::ConstPtr& state);
	void updateTargetStatus_callback(const ros::TimerEvent&);
	
	
	bool init();
	void run();

private:
	float brakingAperture_2_deceleration(const float & brakingAperture);
	
private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	
	ros::Subscriber sub_esrObjects_;
	ros::Subscriber sub_vehicleSpeed_;
	ros::Subscriber sub_start_acc_;
	ros::Publisher pub_cmd_;
	ros::Timer updateTargetStatus_300ms_;
	
	little_ant_msgs::ControlCmd cmd_;
	
	float vehicleSpeed_;
	float tracking_distance_;
	bool is_acc_;
	uint8_t acc_targetId_;
	uint8_t acc_targetIndex_;
	esr_radar_msgs::Object trackTargetMsg_;
	
	uint8_t potentialTarget_index_[64];
	uint8_t potentialTarget_num_;
	
	float trackTargetAngle_range_;
	
	double lastTime_of_seekTarget_;
	
	float deceleration_cofficient_;
	

};



#endif






