#include"decision_making.h"

DecisionMaking::DecisionMaking()
{
	gps_cmd_status_ = false;
	lidar_cmd_status_ = false;
	telecontrol_cmd_status_ = false;
	
	gps_cmd_speed_ = 0.0;
}

DecisionMaking::~DecisionMaking()
{
	
}


void DecisionMaking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	nh_private.param<std::string>("sensors_decision_topic",sensors_decision_topic_,"/sensor_decision");
	
	nh_private.param<std::string>("final_decision_topic1",final_decision_topic1_,"/controlCmd1");
	nh_private.param<std::string>("final_decision_topic2",final_decision_topic2_,"/controlCmd2");
	
	sub_sensors_decision_ = nh.subscribe(sensors_decision_topic_,2,&DecisionMaking::sensor_decision_callback,this);
	
	sendCmd1Timer_20ms_ = nh.createTimer(ros::Duration(0.02), &DecisionMaking::sendCmd1_callback,this);
	sendCmd2Timer_10ms_ = nh.createTimer(ros::Duration(0.02), &DecisionMaking::sendCmd2_callback,this);
	
	pub_final_decision1_ = nh.advertise<little_ant_msgs::ControlCmd1>(final_decision_topic1_,2);
	pub_final_decision2_ = nh.advertise<little_ant_msgs::ControlCmd2>(final_decision_topic2_,2);
}

void DecisionMaking::sensor_decision_callback(const little_ant_msgs::ControlCmd::ConstPtr& msg)
{
	//printf("msg->origin:%d\n",msg->origin);
	switch(msg->origin)
	{
		case little_ant_msgs::ControlCmd::_GPS:
			//ROS_INFO("GPS: status:%d\t speed:%f\t brake:%f",msg->status,msg->cmd2.set_speed,msg->cmd2.set_brake);
			gps_cmd_status_ = msg->status;
			gps_cmd_speed_ = msg->cmd2.set_speed;
			
			if(telecontrol_cmd_status_ || lidar_cmd_status_ || (!gps_cmd_status_))
				break;
				
			cmd1_ = msg->cmd1;
			cmd2_ = msg->cmd2;
			break;
			
		
		case little_ant_msgs::ControlCmd::_LIDAR:
			//ROS_INFO("_LIDAR: status:%d\t speed:%f\t brake:%f",msg->status,msg->cmd2.set_speed,msg->cmd2.set_brake);
			lidar_cmd_status_ = msg->status;
			if(telecontrol_cmd_status_ ||(!lidar_cmd_status_))
				break;
				
			cmd1_ = msg->cmd1;
			cmd2_ = msg->cmd2;
			
			if(msg->cmd2.set_speed > gps_cmd_speed_) //雷达避障速度超过gps跟踪速度时，以跟踪速度为准
				cmd2_.set_speed = gps_cmd_speed_;
			break;	
		
		case little_ant_msgs::ControlCmd::_TELECONTROL:
			//ROS_INFO("_TELECONTROL: status:%d\t speed:%f\t brake:%f",msg->status,msg->cmd2.set_speed,msg->cmd2.set_brake);
			telecontrol_cmd_status_ =msg->status;
			if(!telecontrol_cmd_status_) break;
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

void DecisionMaking::sendCmd1_callback(const ros::TimerEvent&)
{
	pub_final_decision1_.publish(cmd1_);
}

void DecisionMaking::sendCmd2_callback(const ros::TimerEvent&)
{
	pub_final_decision2_.publish(cmd2_);
}




int main(int argc,char ** argv)
{
	ros::init(argc,argv,"decision_making_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	DecisionMaking decision_making;
	decision_making.init(nh,nh_private);
	
	ros::spin();
	
	return 0;
	
}
