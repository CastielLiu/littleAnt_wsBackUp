#include"decision_making.h"

DecisionMaking::DecisionMaking()
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


void DecisionMaking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	nh_private.param<std::string>("sensors_decision_topic",sensors_decision_topic_,"/sensor_decision");
	
	nh_private.param<std::string>("final_decision_topic1",final_decision_topic1_,"/controlCmd1");
	nh_private.param<std::string>("final_decision_topic2",final_decision_topic2_,"/controlCmd2");
	
	sub_sensors_decision_ = nh.subscribe(sensors_decision_topic_,2,&DecisionMaking::sensor_decision_callback,this);
	
	sendCmd1Timer_20ms_ = nh.createTimer(ros::Duration(0.02), &DecisionMaking::sendCmd1_callback,this);
	sendCmd2Timer_10ms_ = nh.createTimer(ros::Duration(0.01), &DecisionMaking::sendCmd2_callback,this);
	updateCmdStatus_300ms_ = nh.createTimer(ros::Duration(0.30),&DecisionMaking::updateCmdStatus_callback,this);
	
	pub_final_decision1_ = nh.advertise<little_ant_msgs::ControlCmd1>(final_decision_topic1_,2);
	pub_final_decision2_ = nh.advertise<little_ant_msgs::ControlCmd2>(final_decision_topic2_,2);
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
	for(size_t i=0;i<SENSOR_NUM;i++)
	{
		if(cmdMsg_[i].status == true)
		{
			pub_final_decision1_.publish(cmdMsg_[i].cmd.cmd1);
			break;
		}
		
	}
}

void DecisionMaking::sendCmd2_callback(const ros::TimerEvent&)
{
	for(size_t i=0;i<SENSOR_NUM;i++)
	{
		if(cmdMsg_[i].status == true)
		{
			//when the lidar send just_decelerate cmd, the obstacle is in vehicle's dangerous area now,
			//according to lidar's speed cmd and other sensor's steer angle cmd to control vehicle
			if((_LIDAR==i)&& cmdMsg_[i].cmd.just_decelerate)
			{
				cmd2_.set_brake = cmdMsg_[i].cmd.cmd2.set_brake;
				cmd2_.set_speed = cmdMsg_[i].cmd.cmd2.set_speed;
				for(size_t j=i+1;j<SENSOR_NUM;j++)
				{
					if(cmdMsg_[j].status == true)
					{
						cmd2_.set_steeringAngle = cmdMsg_[j].cmd.cmd2.set_steeringAngle;
						break;
					}
				}
			}
			//when the vehicle is in accMode ,close the avoiding function!
			else if((_LIDAR==i) && cmdMsg_[_ESR_RADAR].status)
			{
				cmd2_ = cmdMsg_[_ESR_RADAR].cmd.cmd2;
			}
			else
			{
				cmd2_ = cmdMsg_[i].cmd.cmd2;
			}
			pub_final_decision2_.publish(cmd2_);
			break;
		}
	}
}

void DecisionMaking::updateCmdStatus_callback(const ros::TimerEvent&)
{
	double current_time = ros::Time::now().toSec();
	for(size_t i=0;i<SENSOR_NUM;i++)
	{
		if((cmdMsg_[i].status==true)&& (current_time-cmdMsg_[i].time > 0.1)) //over 100ms;
			cmdMsg_[i].status = false;
	}
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
