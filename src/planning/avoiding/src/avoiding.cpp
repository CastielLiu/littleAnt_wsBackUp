#include"avoiding.h"

using namespace state_detection;

Avoiding::Avoiding():
	gps_status_(false),
	target_point_index_status_(false),
	vehicle_speed_status_(false),
	avoiding_offest_(0.0)
{
	avoid_cmd_.origin = little_ant_msgs::ControlCmd::_LIDAR;
	avoid_cmd_.status = false;
	avoid_cmd_.just_decelerate = false;
	avoid_cmd_.cmd1.set_driverlessMode = true;
	avoid_cmd_.cmd2.set_gear = 1;
	avoid_cmd_.cmd2.set_speed = avoid_speed_;
	
	vehicle_axis_dis_ = 1.5;

	danger_distance_front_ = 3.0;
	safety_distance_front_=20.0;
	vehicle_speed_ = 5.0;
}

bool Avoiding::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	nh_private.param<float>("avoid_speed",avoid_speed_,10.0);
	
	nh_private.param<std::string>("objects_topic",objects_topic_,"/detected_bounding_boxs");
	
	nh_private.param<float>("deceleration_cofficient",deceleration_cofficient_,50);
	
	nh_private.param<float>("safety_distance_side",safety_distance_side_,0.3);
	
	nh_private.param<float>("pedestrian_detection_area_side",pedestrian_detection_area_side_,safety_distance_side_+1.0);
	
	nh_private.param<float>("max_deceleration",max_deceleration_,5.0); ///////////
	
	nh_private.param<float>("danger_distance_side",danger_distance_side_,1.2);
	
	nh_private.param<std::string>("path_points_file",path_points_file_,"");
	
	nh_private.param<float>("maxOffset_left",maxOffset_left_,-0.5);
	nh_private.param<float>("maxOffset_right",maxOffset_right_,0.5);
	
	assert(maxOffset_left_ < 0 && maxOffset_right_ >=0);
	
	if(path_points_file_.empty())
	{
		ROS_ERROR("no input path points file !!");
		return false;
	}

	sub_objects_msg_ = nh.subscribe(objects_topic_,1,&Avoiding::objects_callback,this);
	sub_vehicle_speed_ = nh.subscribe("/vehicleState2",1,&Avoiding::vehicleSpeed_callback,this);
	sub_target_point_index_ = nh.subscribe("/track_target_index",1,&Avoiding::target_point_index_callback,this);
	sub_utm_gps_ = nh.subscribe("/gps_utm",1,&Avoiding::utm_gps_callback,this);
	
	pub_avoid_cmd_ = nh.advertise<little_ant_msgs::ControlCmd>("/sensor_decision",1);
	pub_avoid_msg_to_gps_ = nh.advertise<std_msgs::Float32>("/start_avoiding",1);
	
	if(!loadPathPoints(path_points_file_, path_points_))
		return false;
	return true;
}

void Avoiding::target_point_index_callback(const std_msgs::UInt32::ConstPtr& msg)
{
	target_point_index_status_ = true;
	target_point_index_ = msg->data;
}


void Avoiding::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)   
{
	vehicle_speed_status_ = true;
	
	vehicle_speed_ = msg->vehicle_speed; //m/s
	
	danger_distance_front_ = generateDangerDistanceBySpeed(vehicle_speed_);  
	safety_distance_front_ = generateSafetyDisByDangerDis(danger_distance_front_);

	/*
	static int i=0;
	i++;
	if(i%20==0)
		ROS_INFO("callback speed:%f\t danger_distance_front_:%f\t safety_distance_front_:%f",
				 vehicle_speed_,danger_distance_front_,safety_distance_front_);
	*/
}

void Avoiding::utm_gps_callback(const gps_msgs::Utm::ConstPtr& msg)
{
	gps_status_ = true;
	current_point_.x = msg->x;
	current_point_.y = msg->y;
	current_point_.yaw = msg->yaw;
}

void Avoiding::objects_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects)
{
	if(!gps_status_ || !vehicle_speed_status_ || !target_point_index_status_)
	{
		showErrorSystemStatus();
		sleep(1);
		return;
	}
	
	size_t n_object = objects->boxes.size();
	
	//ROS_INFO("n_object:%d",n_object);
	
	if(n_object==0)
	{
		if(offset_msg_.data != 0.0)
		{
			offset_msg_.data = 0.0;
			pub_avoid_msg_to_gps_.publish(offset_msg_);
		}
		avoid_cmd_.status = false;
		avoid_cmd_.just_decelerate = false;
		pub_avoid_cmd_.publish(avoid_cmd_);
		return;
	}
	
	size_t *indexArray = new size_t[n_object];
	float * dis2vehicleArray = new float[n_object];
	float * dis2pathArray = new float[n_object];
	jsk_recognition_msgs::BoundingBox object;
	
	//object position in vehicle coordination (x,y)
	//object position in  world  coordination (X,Y)
	float x,y;  double X,Y;
	
	for(size_t i=0; i< n_object; i++)
	{
		object =  objects->boxes[i];
		
		x = - object.pose.position.y;
		y =   object.pose.position.x;
		
		X =  x * cos(current_point_.yaw) + y * sin(current_point_.yaw) + current_point_.x;
		Y = -x * sin(current_point_.yaw) + y * cos(current_point_.yaw) + current_point_.y;
		
		indexArray[i] = i;
		dis2vehicleArray[i] = sqrt(x * x + y * y);
		//dis2pathArray[i] = calculate_dis2path(X,Y);
		dis2pathArray[i] = calculateDis2path(X,Y,path_points_,target_point_index_);
		
		//printf("x:%f\ty:%f\tyaw:%f\t X:%f\tY:%f\t\n",current_point_.x,current_point_.y,current_point_.yaw*180.0/M_PI,X,Y);
		
		
		//ROS_INFO("dis2path:%f\t dis2vehicle:%f\t x:%f  y:%f",dis2pathArray[i],dis2vehicleArray[i],x,y);
	}
	bubbleSort(dis2vehicleArray,indexArray,n_object);
	if(is_dangerous(objects, dis2vehicleArray,indexArray, dis2pathArray,n_object))
	{
		this->emergencyBrake();
		return ;
	}
		
	//dis2vehicleArray was sorted but dis2pathArray not!
	decision(objects, dis2vehicleArray,indexArray, dis2pathArray,n_object);
	delete [] indexArray;
	delete [] dis2vehicleArray;
	delete [] dis2pathArray;
}

inline void Avoiding::decision(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
						float dis2vehicleArray[], size_t indexArray[], float dis2pathArray[], int n_object)
{
	jsk_recognition_msgs::BoundingBox object; 
	float avoiding_offest[2] ={0.0,0.0};
	float safety_distance_front;
	float safety_center_distance_x;
	float dis2path;
	float dis2vehicle;
	
	for(size_t i=0; i<n_object; i++)
	{
		dis2vehicle = dis2vehicleArray[i];
		object = objects->boxes[indexArray[i]];
		safety_center_distance_x = g_vehicle_width/2 + object.dimensions.y/2 + safety_distance_side_ +0.2;
		//dis2path:    distance from the object to the current path 
		//dis2pathArray[indexArray[i]] : distance from the object to the origin path
		//obstacle avoidance offset has been set in last time
		dis2path = dis2pathArray[indexArray[i]] - avoiding_offest_;
		
		if(avoiding_offest[0] != 0.0)
		{
			safety_distance_front = safety_distance_front_ + 2*danger_distance_front_ ;
			dis2path -= avoiding_offest[0];
		}
		else
			safety_distance_front =safety_distance_front_;
		
		//object is outside the avoding area
		if((fabs(dis2path) >= safety_center_distance_x) || (dis2vehicle >= safety_distance_front))
			continue;
		
		
		//object is inside the avoding area!
		//object is person, slow down(in the true avoid area) or pass(just in the false avoid area) 
		if(object.label == Person)
		{
			if(dis2vehicle <= danger_distance_front_*2)
			{
				avoid_cmd_.status = true;
				avoid_cmd_.just_decelerate = true;
				avoid_cmd_.cmd2.set_brake = 25.0;  //waiting test
				avoid_cmd_.cmd2.set_speed = 0.0;
				pub_avoid_cmd_.publish(avoid_cmd_);
				return;
			}
			//the person is inside the true avoiding area
			else if(dis2vehicle <= safety_distance_front_)
			{
				avoid_cmd_.status = true;
				avoid_cmd_.just_decelerate = true;
				avoid_cmd_.cmd2.set_brake = 0.0;  //waiting test
				avoid_cmd_.cmd2.set_speed = 10.0;
				pub_avoid_cmd_.publish(avoid_cmd_);
				return;
			}
			//the person is just in the false avoid area, so pass
			// when test_offset not zero: safety_distance_front = safety_distance_front_ + 2*danger_distance_front_ ;
			continue;
		}
		//object is other type, start to avoid
		avoiding_offest[0] += dis2path - safety_center_distance_x; //try avoid in the left	
	}
	
	for(size_t i=0; i<n_object; i++)
	{
		dis2vehicle = dis2vehicleArray[i];
		object = objects->boxes[indexArray[i]];
		safety_center_distance_x = g_vehicle_width/2 + object.dimensions.y/2 + safety_distance_side_ +0.2;
		dis2path = dis2pathArray[indexArray[i]];
		
		if(avoiding_offest[1] != 0.0)
		{
			safety_distance_front = safety_distance_front_ + 2*danger_distance_front_ ;
			dis2path -= avoiding_offest[1];
		}
		else
			safety_distance_front = safety_distance_front_;
		
		
		
		//object is outside the avoding area
		if((fabs(dis2path) >= safety_center_distance_x) || (dis2vehicle >= safety_distance_front))
			continue;
			
		//object is inside the avoding area!
		//object is person, slow down(in the true avoid area) or pass(just in the false avoid area) 
		if(object.label == Person)
		{
			if(dis2vehicle <= danger_distance_front_*2)
			{
				avoid_cmd_.status = true;
				avoid_cmd_.just_decelerate = true;
				avoid_cmd_.cmd2.set_brake = 25.0;  //waiting test
				avoid_cmd_.cmd2.set_speed = 0.0;
				pub_avoid_cmd_.publish(avoid_cmd_);
				return;
			}
			//the person is inside the true avoiding area
			else if(dis2vehicle <= safety_distance_front_)
			{
				avoid_cmd_.status = true;
				avoid_cmd_.just_decelerate = true;
				avoid_cmd_.cmd2.set_brake = 0.0;  //waiting test
				avoid_cmd_.cmd2.set_speed = 10.0;
				pub_avoid_cmd_.publish(avoid_cmd_);
				return;
			}
			//the person is just in the false avoid area, so pass
			continue;
		}
		//object is other type, start to avoid
		avoiding_offest[1] += dis2path + safety_center_distance_x; //try avoid in the left	
	}
	
	avoiding_offest[0] += avoiding_offest_;
	avoiding_offest[1] += avoiding_offest_;
	
	//ROS_INFO("offest0:%f\t offset1:%f",avoiding_offest[0],avoiding_offest[1]);
	
	//no avoid message
	if(avoiding_offest[0]==0.0 && avoiding_offest[1] ==0.0) 
	{
		avoid_cmd_.status = false;
		pub_avoid_cmd_.publish(avoid_cmd_);
	}
	//avoid message is invalid ,must slow down ,perhaps not brake!
	else if(avoiding_offest[0] < maxOffset_left_ && avoiding_offest[1] > maxOffset_right_)
	{
		std::stringstream debug_msg("Unable to avoid obstacle! slow down ! \n\t");
		debug_msg << "t_offset_L: " <<  avoiding_offest[0] << "maxOffset_L: "<< maxOffset_left_ << "\n";
		debug_msg << "t_offset_R: " <<  avoiding_offest[1] << "maxOffset_R: "<< maxOffset_right_ ;
		publishDebugMsg(state_detection::Debug::INFO,debug_msg.str());
		avoid_cmd_.status = true;
		avoid_cmd_.just_decelerate = true;
		avoid_cmd_.cmd2.set_brake = 40.0;  //waiting test
		avoid_cmd_.cmd2.set_speed = 0.0;
		pub_avoid_cmd_.publish(avoid_cmd_);
	}
	//avoid message is valid
	//left offest is smaller,so avoid from left side
	else if(( -avoiding_offest[0] <= avoiding_offest[1] && avoiding_offest[0] > maxOffset_left_) ||
			(-avoiding_offest[0] > avoiding_offest[1] && avoiding_offest[0] > maxOffset_left_ && 
			avoiding_offest[1] >maxOffset_right_))
	{
		//assuming that no deceleration is required for avoidance
		avoid_cmd_.status = false;
		pub_avoid_cmd_.publish(avoid_cmd_);
		
		offset_msg_.data = avoiding_offest[0];
		pub_avoid_msg_to_gps_.publish(offset_msg_);
	}
	//right offest is smaller,so avoid from right side
	else
	{
		//assuming that no deceleration is required for avoidance
		avoid_cmd_.status = false;
		pub_avoid_cmd_.publish(avoid_cmd_);
		
		offset_msg_.data = avoiding_offest[1];
		pub_avoid_msg_to_gps_.publish(offset_msg_);
	}
	ROS_INFO("offset:%f",offset_msg_.data);
	
	if(offset_msg_.data!=0.0 && is_backToOriginalLane(objects,dis2vehicleArray,indexArray,dis2pathArray,n_object))
	{
		offset_msg_.data = 0.0;
		publishDebugMsg(state_detection::Debug::INFO,"return to original path..");
		pub_avoid_msg_to_gps_.publish(offset_msg_);
	}
}

inline bool Avoiding::is_backToOriginalLane(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
						float dis2vehicleArray[], size_t indexArray[], float dis2pathArray[], int n_object)
{
	float safety_center_distance_x; //the safety distance along x axis
	
	bool is_ok = true;
	
	for(size_t i=0; i<n_object; i++)
	{
		safety_center_distance_x = g_vehicle_width/2 + objects->boxes[indexArray[i]].dimensions.y/2 + safety_distance_side_;
		
		if(dis2vehicleArray[i] < safety_distance_front_*1.5 && fabs(dis2pathArray[indexArray[i]]) <= safety_center_distance_x)
		{
			is_ok = false;
			break;
		}
	}
	return is_ok;
}

inline bool Avoiding::is_dangerous(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
						float dis2vehicleArray[], size_t indexArray[], float dis2pathArray[], int n_object)
{
	float safety_center_distance_x; //the safety distance along x axis
	float safety_center_distance_y; //the safety distance along y axis
	float x,y;
	for(size_t i=0; i<n_object; i++)
	{
		safety_center_distance_x = g_vehicle_width/2 + objects->boxes[indexArray[i]].dimensions.y/2 + safety_distance_side_;
		safety_center_distance_y = g_vehicle_length/2+ objects->boxes[indexArray[i]].dimensions.x/2 + 1.5;
		x = -objects->boxes[indexArray[i]].pose.position.y;
		y = objects->boxes[indexArray[i]].pose.position.x;
		
		//ROS_INFO("safety_x: %f\t safety_y:%f\t x:%f\t y:%f",safety_center_distance_x,safety_center_distance_y,x,y);
		
		if(fabs(y) <= safety_center_distance_y && 
		   fabs(x) <= safety_center_distance_x)
		   return true;
	}
	return false;
}

std::pair<double,double> Avoiding::vehicleToWorldCoordination(float x,float y)
{
	//object position in world coordination
	double X =  x * cos(current_point_.yaw) + y * sin(current_point_.yaw) + current_point_.x;
	double Y = -x * sin(current_point_.yaw) + y * cos(current_point_.yaw) + current_point_.y;
	return std::pair<double,double>(X,Y);
}


float Avoiding::calculate_dis2path(const double& X_,const double& Y_)
{
	//ROS_INFO("path_points_.size:%d\t target_point_index_:%d",path_points_.size(),target_point_index_);
	
	//this target is tracking target,
	//let the target points as the starting point of index
	//Judging whether to index downward or upward
	float dis2target = pow(path_points_[target_point_index_].x - X_, 2) + 
					   pow(path_points_[target_point_index_].y - Y_, 2) ;
	
	float dis2next_target = pow(path_points_[target_point_index_+1].x - X_, 2) + 
							pow(path_points_[target_point_index_+1].y - Y_, 2) ;
							
	float dis2last_target = pow(path_points_[target_point_index_-1].x - X_, 2) + 
					        pow(path_points_[target_point_index_-1].y - Y_, 2) ;
	
	//std::cout << sqrt(dis2target)<<"\t"<< sqrt(dis2next_target) <<"\t"<< sqrt(dis2last_target) << std::endl;
	
	float first_dis ,second_dis ,third_dis;  //a^2 b^2 c^2
	size_t first_point_index,second_point_index;
	
	first_dis = dis2target;
	first_point_index = target_point_index_;
	
	size_t direction = 1;
	
	if(dis2last_target <dis2target && dis2next_target > dis2target) //downward
	{
		direction = -1;
		for(size_t i=1;true;i++)
		{
			second_point_index = target_point_index_-i;
			
			second_dis = pow(path_points_[second_point_index].x - X_, 2) + 
						 pow(path_points_[second_point_index].y - Y_, 2) ;
			
			if(second_dis < first_dis) //continue 
			{
				first_dis = second_dis;
				first_point_index = second_point_index;
			}
			else  //end
				break;
		}
	}
	else if(dis2next_target < dis2target && dis2last_target > dis2target) //upward
	{
		for(size_t i=1;true;i++)
		{
			second_point_index = target_point_index_ + i;
			second_dis = pow(path_points_[second_point_index].x - X_, 2) + 
						 pow(path_points_[second_point_index].y - Y_, 2) ;

			if(second_dis < first_dis) //continue
			{
				first_dis = second_dis;
				first_point_index = second_point_index;
			}
			else  //end
				break;
		}
	}
	else //midile
	{
		first_point_index = target_point_index_-1;
		second_point_index = target_point_index_ +1;
	}
	
	//the direction of side c 
	//float yaw_of_c = (path_points_[first_point_index].yaw + path_points_[second_point_index].yaw)/2;
	float yaw_of_c = direction * atan2(path_points_[second_point_index].x-path_points_[first_point_index].x,
									   path_points_[second_point_index].y-path_points_[first_point_index].y);
						   
	//object : world coordination to local coordination
	float x = (X_-path_points_[first_point_index].x) * cos(yaw_of_c) - (Y_-path_points_[first_point_index].y) * sin(yaw_of_c);
	//float y = (X_-path_points_[first_point_index].x) * sin(yaw_of_c) + (Y_-path_points_[first_point_index].y) * cos(yaw_of_c);
	
	return x;
}

void Avoiding::get_obstacle_msg(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects,
								size_t objectIndex,whatArea_t *obstacleArea,
								float ** obstacleVertex_x_y,
								float *obstacleDistance, 
								size_t *obstacleIndex,
								size_t &obstacleSequence)
{
	
	float core_x = objects->boxes[objectIndex].pose.position.x;
	float core_y = objects->boxes[objectIndex].pose.position.y;
	
	float x = core_x;
	float y = core_y;
	
	float delta_x = objects->boxes[objectIndex].dimensions.x/2;
	float delta_y = objects->boxes[objectIndex].dimensions.y/2;
	
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
	if(y>safety_distance_side_ || y< -safety_distance_side_ || x>safety_distance_front_ || x< 1.0)
	{
		if(x<safety_distance_front_ && x> safety_distance_front_/4 &&  //行人检测区域限定
			y < pedestrian_detection_area_side_ && y> -pedestrian_detection_area_side_) //行人检测区
			return PedestrianDetectionArea;
		else
			return SafetyArea;//safety
	}
	else if(y>danger_distance_side_ || y<-danger_distance_side_ || x>danger_distance_front_)
		return AvoidingArea; //avoiding
	else
		return DangerArea; //danger!
}


float Avoiding::deceleration_2_brakingAperture(const float & deceleration)
{
	float brakingAperture = (deceleration * deceleration_cofficient_);
	return brakingAperture>40? 40:brakingAperture;
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

inline void Avoiding::showErrorSystemStatus()
{
	ROS_INFO("gps status:%d\t targetIndex status:%d\t vehicleSpeed status:%d",
			gps_status_,target_point_index_status_,vehicle_speed_status_);
	ROS_INFO("waiting for all messages is availble....");
}

inline void Avoiding::emergencyBrake()
{
	avoid_cmd_.status = true;
	avoid_cmd_.just_decelerate = true;
	avoid_cmd_.cmd2.set_brake = 100.0;  //waiting test
	avoid_cmd_.cmd2.set_speed = 0.0;
	pub_avoid_cmd_.publish(avoid_cmd_);
	publishDebugMsg(state_detection::Debug::INFO,"dangerous! emergency brake!");
}


int main(int argc,char **argv)
{
	ros::init(argc,argv,"avoiding_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	state_detection::debugSystemInitial();
	
	Avoiding avoiding;
	if(avoiding.init(nh,nh_private))
		ros::spin();

	return 0;
}


