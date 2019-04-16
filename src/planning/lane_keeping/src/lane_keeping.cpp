#include "lane_keeping/lane_keeping.h"
#include<assert.h>
#include<cmath>

LaneKeeping::LaneKeeping():
	current_lane_msg_index(0),
	system_delay_(20),
	mean_vehicleSpeed_(0)
{
	cmd_.origin = little_ant_msgs::ControlCmd::_LANE_KEEPING;
	cmd_.status = true;
	cmd_.cmd2.set_gear = 1;
	cmd_.cmd1.set_driverlessMode = true;
}

LaneKeeping::~LaneKeeping()
{

}

bool LaneKeeping::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	nh_private.param<float>("foresight_distance",foresight_distance_,10.0);
	nh_private.param<float>("lane_keeping_Speed",lane_keeping_Speed_,10.0);
	
	sub_laneMsg_ = nh.subscribe("/lane",1,&LaneKeeping::laneDetect_callback,this);
	
	sub_vehicleSpeed_ = nh.subscribe("/State2",1,&LaneKeeping::vehicleSpeed_callback,this);
	
	pub_controlCmd_ = nh.advertise<little_ant_msgs::ControlCmd>("/sensor_decision",2);
	
	pub_cmd_20ms_ = nh.createTimer(ros::Duration(0.02), &LaneKeeping::pub_cmd_callback,this);
	
	sub_cartesian_gps_ = nh.subscribe("/gps_odom",1,&LaneKeeping::cartesian_gps_callback,this);
}


void LaneKeeping::pub_cmd_callback(const ros::TimerEvent&)
{
	if(system_delay_ >0)
		return;
		
	//float alpha = asin(lane_msg_.distance_from_center / foresight_distance_) ;//distance_from_center =OA
	
	//distance_from_center =OB
	float alpha = asin(lane_msg_.distance_from_center *cos(lane_msg_.included_angle) / foresight_distance_) ;
	
	float delta = fabs(lane_msg_.included_angle) - fabs(alpha);
	
	//ROS_INFO("alpha:%f\t theta:%f \t delta:%f",alpha*180.0/M_PI,lane_msg_.included_angle*180.0/M_PI,delta*180.0/M_PI);
	
	float steering_radius = 0.5*foresight_distance_ / sin(delta) *sign(lane_msg_.included_angle);
	
	float angle = limit_steeringAngle(generate_steeringAngle_by_steeringRadius(steering_radius),15.0) * g_steering_gearRatio;
		
	cmd_.cmd2.set_steeringAngle = fabs(angle) * get_steeringDir(lane_msg_.distance_from_center, lane_msg_.included_angle,alpha);
	
	cmd_.cmd2.set_speed = lane_keeping_Speed_;
	
	/*
	ROS_INFO("vehicle_speed:%f\t steerAngle:%f\t included_angle:%f\t err:%f",
				mean_vehicleSpeed_,
				cmd_.cmd2.set_steeringAngle,
				lane_msg_.included_angle*180.0/M_PI,
				lane_msg_.distance_from_center);
	*/
	pub_controlCmd_.publish(cmd_);
}


void LaneKeeping::laneDetect_callback(const little_ant_msgs::Lane::ConstPtr& msg)
{
	lane_msg_ = *msg;
	history_status_msgs_[current_lane_msg_index].lane_msg = *msg;
	history_status_msgs_[current_lane_msg_index].vehicle_speed = vehicle_speed_;
	
	if(system_delay_ > 0)
		system_delay_ --;
	else
	{
		int lastIndex = (current_lane_msg_index + 1 == Max_history_size) ? 0 : current_lane_msg_index+1;
		float time_interval = history_status_msgs_[current_lane_msg_index].lane_msg.header.stamp.toSec() -
							  history_status_msgs_[lastIndex].lane_msg.header.stamp.toSec();
							   
		float diff_of_lateralErr = history_status_msgs_[current_lane_msg_index].lane_msg.distance_from_center - 
										history_status_msgs_[lastIndex].lane_msg.distance_from_center;
		
		mean_vehicleSpeed_ = 0.0;
		for(size_t i=0; i<Max_history_size; i++)
			mean_vehicleSpeed_ += history_status_msgs_[i].vehicle_speed;
			
		mean_vehicleSpeed_ /= Max_history_size;
			
		//mean_vehicleSpeed_ += (history_status_msgs_[current_lane_msg_index].vehicle_speed - 
		//					  history_status_msgs_[lastIndex].vehicle_speed)/(Max_history_size-1);
		
		//ROS_INFO("lastIndex:%d\t currentIndex:%d",lastIndex,current_lane_msg_index);				  
		//for(int i=0;i<Max_history_size;i++)
		//{
		//	std::cout << history_status_msgs_[i].vehicle_speed <<"\t";
		//}
		//std::cout <<std::endl;
		
		float distance_between_twoTime ;
		
		if( mean_vehicleSpeed_ >0.0)
		{
			distance_between_twoTime = mean_vehicleSpeed_ * time_interval;
			
			//lane_msg_.included_angle = asin(diff_of_lateralErr/distance_between_twoTime) ; //distance_from_center = OA
			 lane_msg_.included_angle = atan(diff_of_lateralErr/distance_between_twoTime) ;  //distance_from_center = OB
		}
	}
	
	current_lane_msg_index = (current_lane_msg_index+1 == Max_history_size) ? 0: current_lane_msg_index+1;
}


void LaneKeeping::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	vehicle_speed_ = msg->vehicle_speed;
}

//left -1 ; right 1
int LaneKeeping::get_steeringDir(float err,float theta,float alpha)
{
	if(err<= 0 && theta<0)
		return -1;
	else if(err >=0 && theta >0)
		return 1;
	else if(theta==0 && err >0)
		return 1;
	else if(theta ==0 && err <0)
		return -1;
	else if(err <0) // err<0 && theta >0
	{
		return fabs(theta) > fabs(alpha) ? 1:-1;
	}
	else if(err >0)
	{
		return fabs(theta) > fabs(alpha) ? -1:1;
	}
	else
		return 0;
}

void LaneKeeping::generate_laneChange_points()
{
	float widthOfLane = 3.0;
	//车辆离目标车道的距离
	float dis2targetLane = lane_msg_.distance_from_center*cos(lane_msg_.included_angle) + widthOfLane;
	float yawOfLane = current_point_.yaw - lane_msg_.included_angle;
	
	//目标车道中心点相对车辆的坐标
	float x0 = dis2targetLane * cos(lane_msg_.included_angle);
	float y0 = dis2targetLane * sin(lan2_msg_.included_angle);
	
	//目标车道中心点绝对坐标
	float x = x0 * cos(current_point_.yaw) - y0 * sin(current_point_.yaw);
	float y = x0 * sin(current_point_.yaw) + y0 * cos(current_point_.yaw);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	
	
}

void LaneKeeping::cartesian_gps_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	current_point_.x = msg->pose.pose.position.x;
	current_point_.y = msg->pose.pose.position.y;
	
	tf::Quaternion quat;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
	
	double roll,pitch;
	
	tf::Matrix3x3(quat).getRPY(roll, pitch, current_point_.yaw);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"lane_keeping_node");
	
	LaneKeeping lane_keeping;
	
	lane_keeping.init();
	
	
	ros::spin();
	
	return 0;

}




