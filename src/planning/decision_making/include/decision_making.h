#ifndef DECISION_MAKING_H_
#define DECISION_MAKING_H_
#include<ros/ros.h>
#include<little_ant_msgs/ControlCmd.h>
#include<little_ant_msgs/ControlCmd1.h>
#include<little_ant_msgs/ControlCmd2.h>


#define SENSOR_NUM 3

#define _LIDAR        little_ant_msgs::ControlCmd::_LIDAR
#define _GPS          little_ant_msgs::ControlCmd::_GPS
#define _ESR_RADAR    little_ant_msgs::ControlCmd::_ESR_RADAR


class DecisionMaking
{
public:
	DecisionMaking();
	~DecisionMaking();
	
	void init(ros::NodeHandle nh,ros::NodeHandle nh_private);

private:
	void sensor_decision_callback(const little_ant_msgs::ControlCmd::ConstPtr& msg);

	void sendCmd1_callback(const ros::TimerEvent&);

	void sendCmd2_callback(const ros::TimerEvent&);
	
	void updateCmdStatus_callback(const ros::TimerEvent&);


private:
	ros::Subscriber sub_sensors_decision_;
	
	ros::Publisher pub_final_decision1_;
	ros::Publisher pub_final_decision2_;
	
	ros::Timer sendCmd1Timer_20ms_;
	ros::Timer sendCmd2Timer_10ms_;
	ros::Timer updateCmdStatus_300ms_;
	
	std::string sensors_decision_topic_;
	std::string final_decision_topic1_;
	std::string final_decision_topic2_;
	
	little_ant_msgs::ControlCmd1 cmd1_;
	little_ant_msgs::ControlCmd2 cmd2_;
	
	struct cmd_msg_t
	{
		bool status;
		double time;
		little_ant_msgs::ControlCmd cmd;
	}cmdMsg_[SENSOR_NUM];
	
	float gps_cmd_speed_;
	
};






#endif
