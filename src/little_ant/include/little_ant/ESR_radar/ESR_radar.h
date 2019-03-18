#ifndef ESR_RADAR_H_
#define ESR_RADAR_H_
#include "can2serial.h"
#include <ros/ros.h>
#include <string>
#include <esr_radar_msgs/Objects.h>
#include <esr_radar_msgs/Object.h>
 
#define RECEIVE__ 0
#define SEND__  1


class ESR_RADAR 
{
	public:
		 ESR_RADAR(int argc,char ** argv);
		~ESR_RADAR();
		
		bool init();
		
		void run();
		void handleCanMsg();
		
		void pub_installHeight(uint8_t installHeight=10);
		void pub_vehicleSpeed(float speed, bool dir=0);
		
	private:
		CAN_2_SERIAL *can2serial;
		
		void parse_msg(STD_CAN_MSG &can_msg);
		
		ros::Publisher esr_pub;
		
		std::string port_name_;
		
		int argc_;
		char **argv_;
		
		esr_radar_msgs::Objects objects;

};

#endif
