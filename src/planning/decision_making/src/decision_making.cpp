#include"decision_making.h"

DecisionMaking::DecisionMaking():
	traffic_light_status_(TrafficLight_go),
	speed_limit_sign_(-1)
{
	for(size_t i=0;i<SENSOR_NUM;i++)
	{
		cmdMsg_[i].status = false;
	}
	
	//gps_cmd_speed_ = 5.0;
}

DecisionMaking::~DecisionMaking()
{
	
}


bool DecisionMaking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	nh_private.param<std::string>("sensors_decision_topic",sensors_decision_topic_,"/sensor_decision");
	
	nh_private.param<std::string>("final_decision_topic1",final_decision_topic1_,"/controlCmd1");
	nh_private.param<std::string>("final_decision_topic2",final_decision_topic2_,"/controlCmd2");
	
	nh_private.param<std::string>("traffic_mark_topic",traffic_mark_topic_,"");
	
	if(traffic_mark_topic_.empty())
	{
		ROS_ERROR("please input the traffic_mark_topic");
		return false;
	}
	
	
	sub_sensors_decision_ = nh.subscribe(sensors_decision_topic_,2,&DecisionMaking::sensor_decision_callback,this);
	sub_traffic_mark_  =  nh.subscribe(traffic_mark_topic_,1,&DecisionMaking::traffic_mark_callback,this);
	
	sendCmd1Timer_20ms_ = nh.createTimer(ros::Duration(0.02), &DecisionMaking::sendCmd1_callback,this);
	sendCmd2Timer_10ms_ = nh.createTimer(ros::Duration(0.01), &DecisionMaking::sendCmd2_callback,this);
	updateCmdStatus_300ms_ = nh.createTimer(ros::Duration(0.30),&DecisionMaking::updateCmdStatus_callback,this);
	
	pub_final_decision1_ = nh.advertise<little_ant_msgs::ControlCmd1>(final_decision_topic1_,2);
	pub_final_decision2_ = nh.advertise<little_ant_msgs::ControlCmd2>(final_decision_topic2_,2);
	
	return true;
}

void DecisionMaking::sensor_decision_callback(const little_ant_msgs::ControlCmd::ConstPtr& cmd)
{
	if(!cmd->status) return;
	
	cmdMsg_[cmd->origin].status =cmd->status;
	
	cmdMsg_[cmd->origin].time = ros::Time::now().toSec();
	
	cmdMsg_[cmd->origin].cmd = *cmd;

}

//包含无人驾驶模式以及附件开关
//
void DecisionMaking::sendCmd1_callback(const ros::TimerEvent&)
{
	
	pub_final_decision1_.publish(cmdMsg_[_GPS].cmd.cmd1);

}

void DecisionMaking::sendCmd2_callback(const ros::TimerEvent&)
{
	for(size_t i=0;i<SENSOR_NUM;i++)
	{
		if(cmdMsg_[i].status == true)
		{
			if((_LIDAR == i)||(_ESR_RADAR == i))
			{
				cmd2_.set_brake = cmdMsg_[i].cmd.cmd2.set_brake;
				cmd2_.set_speed = cmdMsg_[i].cmd.cmd2.set_speed;
				cmd2_.set_steeringAngle = cmdMsg_[_GPS].cmd.cmd2.set_steeringAngle;
			}
			else
			{
				cmd2_ = cmdMsg_[i].cmd.cmd2;
			}
			break;
		}
	}
	//traffic mark
	if(traffic_light_status_ == TrafficLight_stop)
	{
		cmd2_.set_speed = 5.0;   //waiting debug
		cmd2_.set_brake = 40.0;  //waiting debug
	}
	else if(speed_limit_sign_ >0 && cmd2_.set_speed > speed_limit_sign_)
		cmd2_.set_speed = speed_limit_sign_;
	pub_final_decision2_.publish(cmd2_);
}

void DecisionMaking::updateCmdStatus_callback(const ros::TimerEvent&)
{
	double current_time = ros::Time::now().toSec();
	for(size_t i=0;i<SENSOR_NUM;i++)
	{
		if((cmdMsg_[i].status==true)&& (current_time-cmdMsg_[i].time > 0.1)) //over 100ms;
			cmdMsg_[i].status = false;
	}
	
	//限速超时，退出限速
	if(current_time - speed_limit_time_ > 60.0)
		speed_limit_sign_ = -1;
}

void DecisionMaking::traffic_mark_callback(const little_ant_msgs::DetectedObjectArray::ConstPtr& msg)
{
	for(int i=0;i<msg->objects.size(); i++)
	{
		if(msg->objects[i].label == "Traffic_Light_go")
			traffic_light_status_ = TrafficLight_go;
		else if(msg->objects[i].label == "Traffic_Light_stop")
			traffic_light_status_ = TrafficLight_stop;
		else if(msg->objects[i].label == "limit_speed_r")
		{
			speed_limit_sign_ = 20.0;  //固定限速值，修改此处
			speed_limit_time_ = ros::Time::now().toSec();
		}
	}
}

int main(int argc,char ** argv)
{
	ros::init(argc,argv,"decision_making_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	DecisionMaking decision_making;
	if(decision_making.init(nh,nh_private))
		ros::spin();
	
	return 0;
	
}
