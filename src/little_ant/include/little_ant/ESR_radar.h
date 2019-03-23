#ifndef ESR_RADAR_H_
#define ESR_RADAR_H_
#include "can2serial.h"
#include <ros/ros.h>
#include <string>
#include <esr_radar_msgs/Objects.h>
#include <esr_radar_msgs/Object.h>
#include<jsk_recognition_msgs/BoundingBox.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<cmath>
 
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
		void start_publishBoundingBoxArray_thread();
		
		
	private:
		CAN_2_SERIAL *in_can2serial;
		CAN_2_SERIAL *out_can2serial;
		
		void send_installHeight(uint8_t installHeight);
		void send_installHeight_callback(const ros::TimerEvent&);
		void parse_msg(STD_CAN_MSG &can_msg);
		void pubBoundingBoxArray();
		
		ros::Publisher esr_pub;
		ros::Publisher boundingBox_pub;
		
		std::string in_port_name_;
		std::string out_port_name_;
		
		bool is_pubBoundingBox_;
		bool is_sendMsgToEsr_;
		
		ros::Timer timer_50ms;
		
		int argc_;
		char **argv_;
		
		esr_radar_msgs::Objects objects;
		esr_radar_msgs::Objects last_frame_objects;
		jsk_recognition_msgs::BoundingBoxArray  boxes;
		boost::mutex mutex_;

};

#endif
