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
	
	float pedestrian_detection_area_side_; //行人检测区两侧距离
	
	float vehicleSpeed_; //m/s
	
	enum whatArea_t
	{
		SafetyArea = 0,
		AvoidingArea =1,
		DangerArea =2,
		PedestrianDetectionArea = 3
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
	nh_private.param<float>("pedestrian_detection_area_side",pedestrian_detection_area_side_,safety_distance_side_+1.0);

	
	sub_objects_msg_ = nh.subscribe(objects_topic_,2,&Avoiding::objects_callback,this);
	sub_vehicle_speed_ = nh.subscribe("/State2",2,&Avoiding::vehicleSpeed_callback,this);
	pub_avoid_cmd_ = nh.publish<little_ant_msgs::ControlCmd>("/sensor_decision",2);
	
}

void Avoiding::objects_callback(const jsk_recongnition_msgs::BoundingBoxArray::ConstPtr& objects)
{
	size_t n_object = objects->boxes.size();
	if(n_object==0)
		return;
	
	float *obstacleDistances = new float[n_object]; //存放障碍物的距离
	
	float **obstacleVertex_x_y = new float[n_object][2];
	
	size_t *obstacleIndices = new size_t[n_object]; //存放障碍物在目标中的索引
	
	whatArea_t *obstacleAreas = new whatArea_t[n_object]; //存放障碍物所在区域
	
//	bool *is_vetex_and_core_sameSide_ = new bool[n_object]; //障碍物顶点与其中心是否在同一侧
	
	size_t obstacleSequence = 0; //记录障碍物序号,用于按远近排序
	
	for(size_t i=0;i<n_object;i++)
	{
		//this->get_obstacle_msg(objects,i,obstacleAreas,obstacleVertex_x_y,is_vetex_and_core_sameSide_,obstacleDistances,obstacleIndices,obstacleSequence);
		this->get_obstacle_msg(objects,i,obstacleAreas,obstacleVertex_x_y,obstacleDistances,obstacleIndices,obstacleSequence);
	}
	
	bubbleSort(obstacleDistances,obstacleIndices,obstacleSequence); //障碍物由近到远排序
	
	if(obstacleSequence==0)  //所有目标物均在安全区
	{
		avoid_cmd_.status = false;
		avoid_cmd_.just_decelerate = false;
		return ;
	}
		
	
	//size_t nearest_obstacleIndex = obstacleIndices[0];
	
	//object_type_t nearest_obstacleType = object_type_t(objects->boxes[nearest_obstacleIndex].label);
	
	for(size_t i =0;i<obstacleSequence;i++)
	{
		whatArea_t area = obstacleAreas[i];
		if(area == DangerArea)  //dangerous!! brake right now!!
		{
			avoid_cmd_.status = true;
			avoid_cmd_.just_decelerate =true;
			avoid_cmd_.cmd2.set_speed =0.0;
			avoid_cmd_.cmd2.set_brake = 40;
			break; 
		}
		else if(area == AvoidingArea)//AvoidingArea
		{
			object_type_t objectType = object_type_t(objects->boxes[obstacleIndices[i]].label);
			if(objectType==Person)//行人进入避障区  危险!
			{
				avoid_cmd_.status = true;
				avoid_cmd_.just_decelerate =true;
				avoid_cmd_.cmd2.set_speed =0.0;
				avoid_cmd_.cmd2.set_brake = 40;  //deceleration_2_brakingAperture(0.5*vehicleSpeed_*vehicleSpeed_/(dis_x-5));
				break;
			}
			else //other obstacle ,start to avoiding
			{
				avoid_cmd_.status = true;
				avoid_cmd_.just_decelerate  = false;
				avoid_cmd_.cmd2.set_speed = avoid_speed_;
				avoid_cmd_.cmd2.set_brake = 0.0;
				float t_roadWheelAngle ;  // >0!!
				if(objects->boxes[obstacleIndices[i]].pose.y > 0.2) //障碍物在左侧
				{
					t_roadWheelAngle  == ??//
					avoid_cmd_.cmd2.set_steeringAngle = limitRoadWheelAngle(t_roadWheelAngle)*500.0/20.0;
				}
				else  /right
				{
					t_roadWheelAngle == ??//
					avoid_cmd_.cmd2.set_steeringAngle = -limitRoadWheelAngle(t_roadWheelAngle)*500.0/20.0;
					
				}
				break;
			}
		}
		else if(area == PedestrianDetectionArea) 
		{
			float dis_x = objects->boxes.pose.x;
			
			avoid_cmd_.status = true;
			avoid_cmd_.just_decelerate =true;
			avoid_cmd_.set_speed = 0.0;
			avoid_cmd_.set_brake = deceleration_2_brakingAperture(0.5*vehicleSpeed_*vehicleSpeed_/(dis_x-5)); // -5 is safty distance!
		}
		else // other area ! 
		{
			continue;
		}
	
	}
	
	pub_avoid_cmd_.publish(avoid_cmd_);
	
	
}

//void Avoiding::get_obstacle_msg(const jsk_recongnition_msgs::BoundingBoxArray& objects,size_t objectIndex,whatArea_t *obstacleArea,
//									float ** obstacleVertex_x_y,bool* is_vetex_and_core_sameSide_,float *obstacleDistance, size_t *obstacleIndex,size_t &obstacleSequence)
void Avoiding::get_obstacle_msg(const jsk_recongnition_msgs::BoundingBoxArray& objects,size_t objectIndex,whatArea_t *obstacleArea,
									float ** obstacleVertex_x_y,float *obstacleDistance, size_t *obstacleIndex,size_t &obstacleSequence)
{
	
	float core_x = objects.boxes[objectIndex].pose.position.x;
	float core_y = objects.boxes[objectIndex].pose.position.y;
	
	float x = core_x;
	float y = core_y;
	
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
	/*
		if((core_x>0&&x>0)||(core_x<0&&x<0))
			is_vetex_and_core_sameSide_[obstacleSequence] = true;
		else
			is_vetex_and_core_sameSide_[obstacleSequence] =false;
	*/		
		obstacleVertex_x_y[obstacleSequence][0] = x;
		obstacleVertex_x_y[obstacleSequence][1] = y;
		obstacleArea[obstacleSequence] = area;
		
		obstacleDistance[obstacleSequence] = sqrt(x*x+y*y);
		obstacleIndex[obstacleSequence] = objectIndex;
		obstacleSequence ++;
	}
	
}

whatArea_t Avoiding::which_area(float& x,float& y)
{
	if(y>safety_distance_side_ || y< -safety_distance_side_ || x>safety_distance_front_ || x< 0.0) 
	{
		if(x<safety_distance_front_ && x> safety_distance_front_/2.0 && 
			y < pedestrian_detection_area_side_ && y> -pedestrian_detection_area_side_)) //行人检测区
			return PedestrianDetectionArea;
		else
			return SafetyArea;//safety
	}
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
 
float Avoiding::deg2rad(float deg)
{
	return  (deg/180.0)*M_PI;
}

void Avoiding::limitRoadWheelAngle(float& angle)
{
	if(angle>30.0) angle =30.0;
	else if(angle<-30.0) angle =-30.0;
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
