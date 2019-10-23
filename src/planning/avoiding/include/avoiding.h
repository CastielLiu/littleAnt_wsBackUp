#ifndef AVOIDING_H_
#define AVOIDING_H_
#include<ros/ros.h>
#include<little_ant_msgs/ControlCmd.h>
#include<little_ant_msgs/State2.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<jsk_recognition_msgs/BoundingBox.h>
#include<nav_msgs/Odometry.h>
#include<iostream>
#include<std_msgs/Float32.h>
#include<ant_math/ant_math.h>
#include<std_msgs/UInt32.h>
#include<vector>
#include<assert.h>

#include<gps_msgs/Utm.h>

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
	
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);

private:
	enum object_type_t
	{
		Unknown = 0,
		//Person = 1,
		Person = 2,
		Vehicle = 3
	};
	void utm_gps_callback(const nav_msgs::Odometry::ConstPtr& utm);
	
	void objects_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects);
	
	void target_point_index_callback(const std_msgs::UInt32::ConstPtr& msg);

	void get_obstacle_msg(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects,
						  size_t objectIndex,
						  whatArea_t *obstacleArea,
						  float ** obstacleVertex_x_y,
						  float *obstacleDistance, 
						  size_t *obstacleIndex,
						  size_t &obstacleSequence);
	
	whatArea_t which_area(float& x,float& y);

	void vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg);

	float deceleration_2_brakingAperture(const float & deceleration);

	float brakingAperture_2_deceleration(const float & brakingAperture);

	void bubbleSort(const float * distance, size_t * index, size_t length);
	
	float calculate_dis2path(const double& X_,const double& Y_);
	float dis2path2(const double& X_,const double& Y_);
	std::pair<double,double> vehicleToWorldCoordination(float x,float y);
	void decision(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
				const float dis2vehicleArray[],const size_t indexArray[],const float dis2pathArray[],const int n_object);
	bool is_backToOriginalLane(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
						const float dis2vehicleArray[],const size_t indexArray[],const float dis2pathArray[],const int& n_object);
						
	bool is_dangerous(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
					const float dis2vehicleArray[],const size_t indexArray[],const float dis2pathArray[],const int& n_object);
	void showErrorSystemStatus();
	void emergencyBrake();
	void backToOriginalLane();
	
private:
	ros::Subscriber sub_objects_msg_;
	ros::Subscriber sub_vehicle_speed_;
	ros::Subscriber sub_target_point_index_;
	ros::Subscriber sub_utm_gps_;
	
	ros::Publisher pub_avoid_cmd_;
	ros::Publisher pub_avoid_msg_to_gps_; 
	
	little_ant_msgs::ControlCmd avoid_cmd_;
	
	std_msgs::Float32 start_avoidingFlag_;
	
	float avoid_speed_;
	
	float max_deceleration_;
	
	std::string objects_topic_;
	
	float safety_distance_front_;
	float safety_distance_side_ ;
	
	float danger_distance_front_;
	float danger_distance_side_;
	
	float pedestrian_detection_area_side_; //行人检测区两侧距离
	
	float vehicle_speed_; //m/s
	float vehicle_axis_dis_ ;
	float deceleration_cofficient_;
	
	std::vector<gpsMsg_t> path_points_;
	
	std::string path_points_file_;
	
	uint32_t target_point_index_;
	
	gpsMsg_t current_point_,target_point_;
	
	
	
	float avoiding_offest_;
	std_msgs::Float32 offset_msg_;
	
	float maxOffset_right_,maxOffset_left_;
	
	bool gps_status_;
	bool target_point_index_status_;
	bool vehicle_speed_status_;
	
	bool is_systemOk_;
};




#endif
