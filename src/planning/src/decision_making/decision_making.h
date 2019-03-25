#ifndef DECISION_MAKING_H_
#define DECISION_MAKING_H_
#include<ros/ros.h>
#include<little_ant_msgs/ControlCmd.h>
#include<little_ant_msgs/ControlCmd1.h>
#include<little_ant_msgs/ControlCmd2.h>

class DecisionMaking
{
public:
	DecisionMaking();
	~DecisionMaking();
	
	void init(ros::NodeHandle nh,ros::NodeHandle nh_private);

private:
	void sennsor_decision_callback(const little_ant_msgs::ControlCmd1::ConstPtr& msg);

	void sendCmd1Timer_20ms_(const ros::TimerEvent&);

	void sendCmd2Timer_10ms_(const ros::TimerEvent&);


private:
	ros::Subscriber sub_sensors_decision_;
	
	ros::Publisher pub_final_decision1_;
	ros::Publisher pub_final_decision2_;
	
	ros::Timer sendCmd1Timer_20ms_;
	ros::Timer sendCmd2Timer_10ms_;
	
	std::string sensors_decision_topic_;
	std::string final_decision_topic1_;
	std::string final_decision_topic2_;
	
	little_ant_msgs::ControlCmd1 cmd1_;
	little_ant_msgs::ControlCmd2 cmd2_;
	
	bool gps_cmd_status;
	bool lidar_cmd_status;
	bool telecontrol_cmd_status;
	
};






#endif
