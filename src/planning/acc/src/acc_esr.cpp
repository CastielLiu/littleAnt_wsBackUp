#include "acc/acc_esr.h"



Acc_esr::Acc_esr(ros::NodeHandle nh,ros::NodeHandle nh_private) :
	nh_(nh),
	nh_private_(nh_private)
{
	cmd_.cmd2.set_gear =1;
	cmd_.cmd1.set_driverlessMode =true;
	
	
	vehicleSpeed_ = 0.0;
	is_acc_ = false;
	
	acc_targetId_ = 0xff; //no target
	lastTime_of_seekTarget_ = 0.0;
	
}

bool Acc_esr::init()
{
	sub_esrObjects_ = nh_.subscribe("/esr_objects",2,&Acc_esr::object_callback,this);
	sub_vehicleSpeed_ = nh_.subscribe("/vehicleState2",2,&Acc_esr::vehicleSpeed_callback,this);
	sub_start_acc_ = nh_.subscribe("/is_acc",2,&Acc_esr::is_acc_callback,this);
	
	pub_cmd_ = nh_.advertise<little_ant_msgs::ControlCmd>("/sensor_decision",2);
	
	nh_private_.param<float>("trackTargetAngle_range",trackTargetAngle_range_,1.0);
	nh_private_.param<float>("deceleration_cofficient",deceleration_cofficient_,50.0);
	
	updateTargetStatus_300ms_ = nh_.createTimer(ros::Duration(0.30),&Acc_esr::updateTargetStatus_callback,this);
}

void Acc_esr::run()
{
	ros::spin();
}


void Acc_esr::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	static int i=0;
	vehicleSpeed_ = (msg->wheel_speed_FL + msg->wheel_speed_RR)/2*5.0/18; //m/s

	i++;
	if(i%20==0)
		ROS_INFO("vehicle speed:%f ",vehicleSpeed_);
		
	tracking_distance_ = 0.5* vehicleSpeed_ * vehicleSpeed_ /brakingAperture_2_deceleration(40.0)  + 5.0;
	tracking_distance_ *= 3.0;  //wait debug..
}

float Acc_esr::brakingAperture_2_deceleration(const float & brakingAperture)
{
	return brakingAperture / deceleration_cofficient_;
}

void Acc_esr::object_callback(const esr_radar_msgs::Objects::ConstPtr& objects)
{
	if(!is_acc_) return;
	
	float min_distance=500.0; //a big num
	
	if(acc_targetId_==0xff) //no track target
	{
		for(size_t i=0,potentialTarget_num_=0;i<objects->size;i++)
		{
			if((objects->objects[i].azimuth <trackTargetAngle_range_) &&
				(objects->objects[i].azimuth >-trackTargetAngle_range_))
			{
				potentialTarget_index_[potentialTarget_num_++] = i; 
			}
		}
		for(size_t i=0;i<potentialTarget_num_;i++)
		{
			if(objects->objects[potentialTarget_index_[i]].distance < min_distance)
			{
				min_distance = objects->objects[potentialTarget_index_[i]].distance;
				acc_targetId_ = objects->objects[potentialTarget_index_[i]].id;
			}
				
		}
		//acc_targetId_ = objects->
	}
	else
	{
		ROS_INFO("target id:0x%x",acc_targetId_);
		for(size_t i=0;i<objects->size;i++)
		{
			if(objects->objects[i].id == acc_targetId_)
			{
				trackTargetMsg_ = objects->objects[i];
				lastTime_of_seekTarget_ = ros::Time::now().toSec();
				break;
			}
		}
		float speed = trackTargetMsg_.speed + (trackTargetMsg_.distance -tracking_distance_) *1.0;
		if(speed >=0.0)
			cmd_.cmd2.set_speed = speed;
		else
		{
			cmd_.cmd2.set_speed = 0.0;
			cmd_.cmd2.set_brake = -speed * 1.0;
		}
			
	}
}

void Acc_esr::is_acc_callback(const std_msgs::Bool::ConstPtr& state)
{
	is_acc_ = state->data;
}

void Acc_esr::updateTargetStatus_callback(const ros::TimerEvent&)
{
	if(is_acc_ && (acc_targetId_!=0xff) && (ros::Time::now().toSec()-lastTime_of_seekTarget_)>0.2) //overtime
	{
		is_acc_ = false;
		acc_targetId_ = 0xff;
		ROS_ERROR("acc target losed! acc has to exited !!");
	}
		
}


int main(int argc,char ** argv)
{
	ros::init(argc,argv,"acc_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	Acc_esr acc(nh,nh_private);
	if(acc.init())
		acc.run();
	
	return 0;
}


