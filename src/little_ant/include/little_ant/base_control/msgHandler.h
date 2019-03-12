#ifndef MSG_HANDLER_H_
#define MSG_HANDLER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include"can2serial.h"
#include <little_ant_msgs/State1.h>
#include <little_ant_msgs/State2.h>
#include <little_ant_msgs/State3.h>
#include <little_ant_msgs/State4.h>


#include <little_ant_msgs/ControlCmd1.h>
#include <little_ant_msgs/ControlCmd2.h>

#define ID_CMD_1 0x2C5
#define	ID_CMD_2 0x1C5

#define ID_STATE1 0x151
#define ID_STATE2 0x300
#define ID_STATE3 0x4D1
#define ID_STATE4 0x1D5


class MsgHandler
{
public:
	MsgHandler();
	~MsgHandler();
	bool init(int ,char**);
	void run();
	
	void parse();

	void callBack1(const little_ant_msgs::ControlCmd1::ConstPtr msg);
	void callBack2(const little_ant_msgs::ControlCmd2::ConstPtr msg);
	
private:
	CAN_2_SERIAL can2serial;
	
	ros::Subscriber cmd1_sub;
	ros::Subscriber cmd2_sub;
	
	ros::Publisher state1_pub;
	ros::Publisher state2_pub;
	ros::Publisher state3_pub;
	ros::Publisher state4_pub;
	
	std::string port_name_;
	
	
	STD_CAN_MSG canMsg_cmd1;
	STD_CAN_MSG canMsg_cmd2;
	
	little_ant_msgs::State1 state1;
	little_ant_msgs::State2 state2;
	little_ant_msgs::State3 state3;
	little_ant_msgs::State4 state4;

};


#endif
