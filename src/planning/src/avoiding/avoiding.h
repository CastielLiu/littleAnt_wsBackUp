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
	enum object_type_t
	{
		Unknown = 0,
		Person = 1,
		Vehicle = 2
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
	
	nh_private.param<float>("deceleration_cofficient",deceleration_cofficient_,32);

	
	sub_objects_msg_ = nh.subscribe(objects_topic_,2,&Avoiding::objects_callback,this);
	sub_vehicle_speed_ = nh.subscribe("/State2",2,&Avoiding::vehicleSpeed_callback,this);
	pub_avoid_cmd_ = nh.publish<little_ant_msgs::ControlCmd>("/sensor_decision",2);
	
}

void Avoiding::objects_callback(const jsk_recongnition_msgs::BoundingBoxArray::ConstPtr& objects)
{
	size_t n_object = objects->boxes.size();
	if(n_object==0)
		return;
	
	float *obstacleDistance = new float[n_object]; //存放障碍物的距离
	
	size_t *obstacleIndex = new size_t[n_object]; //存放障碍物在目标中的索引
	
	whatArea_t *obstacleArea = new whatArea_t[n_object]; //存放障碍物所在区域
	
	size_t obstacleSequence = 0; //记录障碍物序号,用于按远近排序
	
	for(size_t i=0;i<n_object;i++)
	{
		this->get_obstacle_msg(objects,i,obstacleArea,obstacleDistance,obstacleIndex,obstacleSequence);
	}
	
	bubbleSort(obstacleDistance,obstacleIndex,obstacleSequence); //障碍物由近到远排序
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	for(auto object:objects->boxes)
	{
		area = which_area(object);
		object_type = object_type_t(object.label);
		
		if(area == DangerArea)
		{
			avoid_cmd_.status = true;
			avoid_cmd_.just_decelerate =true;
			avoid_cmd_.cmd1.set_speed =0.0;
			avoid_cmd_.cmd1.set_brake = 40;
			break;
		}
		else if(area == AvoidingArea )
		{
			float dis_x = object.pose.x;
			float dis_y = object.pose.y;
			if(object_type == Person)
			{
				avoid_cmd_.status = true;
				avoid_cmd_.just_decelerate =true;
				avoid_cmd_.set_speed = 0.0;
				avoid_cmd_.set_brake = deceleration_2_brakingAperture(0.5*vehicleSpeed_*vehicleSpeed_/(dis_x-5));
			}
			else //Vehicle  or Unknown
			{
				///////////	///???
			
			}
			
		
		}
	}
}

void Avoiding::get_obstacle_msg(const jsk_recongnition_msgs::BoundingBoxArray& objects,size_t objectIndex,whatArea_t *obstacleArea,
									float *obstacleDistance, size_t *obstacleIndex,size_t &obstacleSequence)
{
	float x = objects.boxes[objectIndex].pose.position.x;
	float y = objects.boxes[objectIndex].pose.position.y;
	
	float delta_x = objects.boxes[objectIndex].dimensions.x/2;
	float delta_y = objects.boxes[objectIndex].dimensions.y/2;
	
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
	
	whatArea_t area = which_area(x,y);
	if( area != SafetyArea)
	{
		obstacleDistance[objectIndex] = sqrt(x*x+y*y);
		obstacleIndex[obstacleSequence] = objectIndex;
		obstacleArea[objectIndex] = area;
		obstacleSequence ++;
	}
	
}

whatArea_t Avoiding::which_area(float& x,float& y)
{
	if(y>safety_distance_side_ || y< -safety_distance_side_ || x>safety_distance_front_ || x< 0.0)
		return SafetyArea; //safety
	else if(y>danger_distance_side_ || y<-danger_distance_side_ || x>danger_distance_front_)
		return AvoidingArea; //avoiding
	else
		return DangerArea; //danger!
}


void Avoiding::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	vehicleSpeed_ = (msg->wheel_speed_FL + msg->wheel_speed_RR)*5.0/18; //m/s
	danger_distance_front_ = brakingAperture_2_deceleration(40.0) * vehicleSpeed_ * vehicleSpeed_ /2 + 3.0;
	safety_distance_front_ = danger_distance_front_ + 20.0;
}

float Avoiding::deceleration_2_brakingAperture(const float & deceleration)
{
	return deceleration * deceleration_cofficient_;
}

float Avoiding::brakingAperture_2_deceleration(const float & brakingAperture)
{
	return brakingAperture / deceleration_cofficient_;
}

void Avoiding::bubbleSort(float * const distance, size_t * index, size_t length)
{
	for (size_t i = 0; i < length; i++)
	{
		for (size_t j = 0; j < length- i - 1; j++)
		{
			if (distance[j] > distance[j + 1])
			{
				size_t ind_temp = index[j];
				index[j] = index[j + 1];
				index[j + 1] = ind_temp;
			}
		}
	}
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
