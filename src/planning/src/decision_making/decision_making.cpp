#include"decision_making.h"

DecisionMaking::DecisionMaking()
{
	gps_cmd_status = false;
	lidar_cmd_status = false;
	telecontrol_cmd_status = false;
}

DecisionMaking::~DecisionMaking()
{
	
}


void DecisionMaking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	nh_private.param<std::string>("sensors_decision_topic",sensors_decision_topic_,"/sensor_decision");
	
	nh_private.param<std::string>("final_decision_topic1",final_decision_topic1_,"/controlCmd1");
	nh_private.param<std::string>("final_decision_topic2",final_decision_topic2_,"/controlCmd2");
	
	sub_gps_decision = nh.subscribe(sensors_decision_topic_,2,&DecisionMaking::sennsor_decision_callback,this);
	
	sendCmd1Timer_20ms_ = nh.createTimer(ros::Duration(0.02), &DecisionMaking::sendCmd1_callback,this);
	sendCmd2Timer_10ms_ = nh.createTimer(ros::Duration(0.02), &DecisionMaking::sendCmd2_callback,this);
	
	pub_final_decision1 = nh.advertise<little_ant_msgs::ControlCmd1>(final_decision_topic1_,2);
	pub_final_decision2 = nh.advertise<little_ant_msgs::ControlCmd2>(final_decision_topic2_,2);
}

void DecisionMaking::sennsor_decision_callback(const little_ant_msgs::ControlCmd1::ConstPtr& msg)
{
	switch(msg->origin)
	{
		case FROM_GPS:
			gps_cmd_status = msg->status;
			if(telecontrol_cmd_status || lidar_cmd_status || (!gps_cmd_status))
				break;
				
			cmd1_ = msg->cmd1;
			cmd2_ = msg->cmd2;
			break;
			
		
		case FROM_LIDAR:
			lidar_cmd_status = msg->status;
			if(telecontrol_cmd_status ||(!lidar_cmd_status))
				break;
				
			cmd1_ = msg->cmd1;
			cmd2_ = msg->cmd2;
			break;	
		
		case FROM_TELECONTROL:
			telecontrol_cmd_status =msg->status;
			if(!telecontrol_cmd_status) break;
			if(msg->just_decelerate)
			{
				cmd2_.set_speed = msg->cmd2.set_speed;
				cmd2_.set_brake = msg->cmd2.set_brake;
			}
			else
			{
				cmd1_ = msg->cmd1;
				cmd2_ = msg->cmd2;
			}
			break;
		
		default:
			break;
	}
}

void DecisionMaking::sendCmd1Timer_20ms_(const ros::TimerEvent&)
{
	pub_final_decision1_.publish(cmd1_);
}

void DecisionMaking::sendCmd2Timer_10ms_(const ros::TimerEvent&)
{
	pub_final_decision2_.publish(cmd2);
}




int main(int argc,char ** argv)
{
	ros::init(argc,argv,"decision_making_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private;
	
	DecisionMaking decision_making;
	decision_making.init(nh,nh_private);
	
	ros::spin();
	
	return 0;
	
}
