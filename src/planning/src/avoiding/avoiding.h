#ifndef AVOIDING_H_
#define AVOIDING_H_
#include<ros/ros.h>
#include<little_ant_msgs/ControlCmd.h>
#include<little_ant_msgs/State2.h>
#include<jsk_recongnition_msgs/BoundingBoxArray.h>
#include<jsk_recongnition_msgs/BoundingBox.h>

class Avoiding
{

public:
	Avoiding();
	~Avoiding();


private:
	ros::Subscriber sub_objects_msg_;
	ros::Subscriber sub_vehicle_speed_;
	ros::Publisher pub_avoid_cmd_;
	
	little_ant_msgs::ControlCmd avoid_cmd_;
	
	float avoid_speed_;
	
	std::string objects_topic_;
	
	float safety_distance_front_;
	float safety_distance_side_ ;
	
	float danger_distance_front_;
	float danger_distance_side_;
	
	float vehicleSpeed_; //m/s
	
	enum whatArea_t
	{
		SafetyArea = 0,
		AvoidingArea =1,
		DangerArea =2
	};
};

Avoiding::Avoiding()
{
	avoid_cmd_.origin = little_ant_msgs::ControlCmd::_LIDAR;
	avoid_cmd_.status = false;
	avoid_cmd_.just_decelerate = false;
	avoid_cmd_.cmd1.set_driverlessMode = true;
	avoid_cmd_.cmd2.set_gear = 1;
	avoid_cmd_.cmd2.set_speed = avoid_speed_;
	
	danger_distance_side_ = 0.9 + 0.3;
	safety_distance_side_ = 0.9 + 0.8;
	
}

void Avoiding::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	nh_private.param<float>("avoid_speed",avoid_speed_,10.0);
	nh_private.param<std::string>("objects_topic",objects_topic_,"/detected_bounding_boxs");
	
	nh_private.param<float>("max_deceleration",max_deceleration_,1.25);

	
	sub_objects_msg_ = nh.subscribe(objects_topic_,2,&Avoiding::objects_callback,this);
	sub_vehicle_speed_ = nh.subscribe("/State2",2,&Avoiding::vehicleSpeed_callback,this);
	pub_avoid_cmd_ = nh.publish<little_ant_msgs::ControlCmd>("/sensor_decision",2);
	
}

void Avoiding::objects_callback(const jsk_recongnition_msgs::BoundingBoxArray::ConstPtr& objects)
{
	uint8_t area;
	
	for(size_t i=0;i<objects->boxes.size();i++)
	{
		area = which_area(objects->boxes[i]);
		if(area == DangerArea)
		{
			avoid_cmd_.status = true;
			avoid_cmd_.just_decelerate =true;
			avoid_cmd_.cmd1.set_speed =0.0;
			avoid_cmd_.
		}
	}

}

whatArea_t Avoiding::which_area(const jsk_recongnition_msgs::BoundingBox& object)
{
	float x = object.pose.position.x;
	float y = object.pose.position.y;
	
	float delta_x = object.dimensions.x/2;
	float delta_y = object.dimensions.y/2;
	
	if(x>0&&y>0) //left front
	{
		x -= delta_x;
		y -= delta_y;
	}
	else if(x>0&&y<0) //right front
	{
		x -= delta_x;
		y += delta_y;
	}
	else if(x<0&&y>0) //left rear
	{
		x += delta_x;
		y -= delta_y;
	}
	else //right rear
	{
		x += delta_x;
		y += delta_y;
	}
	
	return which_area(x,y);
}

whatArea_t Avoiding::which_area(float& x,float& y)
{
	if(y>safety_distance_side_ || y< -safety_distance_side_ || x>safety_distance_front_ || x< 0.0)
		return 0; //safety
	else if(y>danger_distance_side_ || y<-danger_distance_side_ || x>danger_distance_front_)
		return 1; //avoiding
	else
		return 2; //danger!
}


void Avoiding::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	vehicleSpeed_ = (msg->wheel_speed_FL + msg->wheel_speed_RR)*5.0/18; //m/s
	danger_distance_front_ = max_deceleration_ * vehicleSpeed_ * vehicleSpeed_ /2 + 3.0;
	safety_distance_front_ = danger_distance_front_ + 20.0;
}


int main(int argc,char **argv)
{
	ros::init(argc,argv,"avoiding_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	Avoiding avoiding;
	avoiding.init();
	
	ros::spin();

	return 0;
}


#endif
