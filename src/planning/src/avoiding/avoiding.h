#ifndef AVOIDING_H_
#define AVOIDING_H_
#include<ros/ros.h>
#include<little_ant_msgs/ControlCmd.h>
#include<little_ant_msgs/State2.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<jsk_recognition_msgs/BoundingBox.h>
#include<iostream>

typedef enum
{
	SafetyArea = 0,
	AvoidingArea =1,
	DangerArea =2,
	PedestrianDetectionArea = 3
}whatArea_t;

class Avoiding
{

public:
	Avoiding();
	~Avoiding(){}
	void init(ros::NodeHandle nh,ros::NodeHandle nh_private);

private:
	
	
	enum object_type_t
	{
		Unknown = 0,
		Person = 1,
		Vehicle = 2
	};
	
	void objects_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects);

	void get_obstacle_msg(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects,size_t objectIndex,whatArea_t *obstacleArea,
										float ** obstacleVertex_x_y,float *obstacleDistance, size_t *obstacleIndex,size_t &obstacleSequence);

	whatArea_t which_area(float& x,float& y);

	void vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg);

	float deceleration_2_brakingAperture(const float & deceleration);

	float brakingAperture_2_deceleration(const float & brakingAperture);

	void bubbleSort(float * const distance, size_t * index, size_t length);
	 
	float deg2rad(float deg);

	void limitRoadWheelAngle(float& angle);


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
	float vehicle_axis_dis_ ;
	float deceleration_cofficient_;
	

};




#endif
