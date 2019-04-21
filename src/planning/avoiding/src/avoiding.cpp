#include"avoiding.h"


Avoiding::Avoiding()
{
	avoid_cmd_.origin = little_ant_msgs::ControlCmd::_LIDAR; //_TELECONTROL  //_LIDAR
	avoid_cmd_.status = false;
	avoid_cmd_.just_decelerate = false;
	avoid_cmd_.cmd1.set_driverlessMode = true;
	avoid_cmd_.cmd2.set_gear = 1;
	avoid_cmd_.cmd2.set_speed = avoid_speed_;
	
	danger_distance_side_ = 0.9 + 0.3;
	safety_distance_side_ = 0.9 + 1.0;
	vehicle_axis_dis_ = 1.5;

	danger_distance_front_ = 5.0;
	safety_distance_front_=20.0;
	vehicleSpeed_ = 5.0;
}

bool Avoiding::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	nh_private.param<float>("avoid_speed",avoid_speed_,10.0);
	nh_private.param<std::string>("objects_topic",objects_topic_,"/detected_bounding_boxs");
	
	nh_private.param<float>("deceleration_cofficient",deceleration_cofficient_,50);
	
	nh_private.param<float>("safety_distance_side",safety_distance_side_,1.7);
	
	nh_private.param<float>("pedestrian_detection_area_side",pedestrian_detection_area_side_,safety_distance_side_+1.0);
	
	nh_private.param<float>("danger_distance_side",danger_distance_side_,1.2);
	
	nh_private.param<std::string>("path_points_file",path_points_file_,"");
	
	if(path_points_file_.empty())
	{
		ROS_ERROR("no input path points file !!");
		return false;
	}

	sub_objects_msg_ = nh.subscribe(objects_topic_,2,&Avoiding::objects_callback,this);
	sub_vehicle_speed_ = nh.subscribe("/vehicleState2",2,&Avoiding::vehicleSpeed_callback,this);
	sub_target_point_index_ = nh.subscribe("/track_target_index",1,&Avoiding::target_point_index_callback,this);
	
	pub_avoid_cmd_ = nh.advertise<little_ant_msgs::ControlCmd>("/sensor_decision",1);
	pub_avoid_msg_to_gps_ = nh.advertise<std_msgs::Float32>("/start_avoiding",1);
	
	if(!load_path_points(path_points_file_, path_points_))
		return false;
	return true;
}

void Avoiding::target_point_index_callback(const std_msgs::UInt32::ConstPtr& msg)
{
	target_point_index_ = msg->data;
}

void Avoiding::objects_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects)
{
	//printf("danger_distance_front_:%f\tsafety_distance_front_:%f\t",danger_distance_front_,safety_distance_front_);
	//printf("danger_distance_side_:%f\tsafety_distance_side:%f\n\n",danger_distance_side_,safety_distance_side_);
	
	size_t n_object = objects->boxes.size();
	
	//ROS_INFO("n_object:%d",n_object);
	
	if(n_object==0)
	{
		avoid_cmd_.status = false;
		avoid_cmd_.just_decelerate = false;
		pub_avoid_cmd_.publish(avoid_cmd_);
		return;
	}
	
	float *obstacleDistances = new float[n_object]; //存放障碍物的距离
	
	float **obstacleVertex_x_y = new float *[n_object];
	for(size_t i=0;i<n_object;i++)
		obstacleVertex_x_y[i] = new float[2];
	
	size_t *obstacleIndices = new size_t[n_object]; //存放障碍物在目标中的索引
	
	whatArea_t *obstacleAreas = new whatArea_t[n_object]; //存放障碍物所在区域
	
//	bool *is_vetex_and_core_sameSide_ = new bool[n_object]; //障碍物顶点与其中心是否在同一侧
	
	size_t obstacleSequence = 0; //记录障碍物序号,用于按远近排序
	
	for(size_t i=0;i<n_object;i++)
	{
		this->get_obstacle_msg(objects,i,obstacleAreas,obstacleVertex_x_y,obstacleDistances,obstacleIndices,obstacleSequence);
		
	}
	
	//std::cout << "obstacleSequence:" <<obstacleSequence <<std::endl;
	bubbleSort(obstacleDistances,obstacleIndices,obstacleSequence); //障碍物由近到远排序
	
	if(obstacleSequence==0)  //所有目标物均在安全区
	{
		avoid_cmd_.status = false;
		avoid_cmd_.just_decelerate = false;
//ROS_ERROR("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
		goto Delete_memory;
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
			ROS_INFO("Danger!! x: %f\ty: %f",obstacleVertex_x_y[i][0],obstacleVertex_x_y[i][1]);
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
				ROS_INFO("Person  just_decelerate  x: %f\ty: %f",obstacleVertex_x_y[i][0],obstacleVertex_x_y[i][1]);
				break;
			}
			else //other obstacle ,start to avoiding
			{
				start_avoidingFlag_.data = 1;
				///////////////pub_avoid_msg_to_gps_.publish(start_avoidingFlag_);
				avoid_cmd_.status = true;
				avoid_cmd_.just_decelerate  = false;
		//ROS_ERROR("avoid_speed_:%f",avoid_speed_);
				avoid_cmd_.cmd2.set_brake = 0.0;
				
				float t_roadWheelAngle;
				if(obstacleVertex_x_y[i][0] > danger_distance_front_)
				{
					//float _x = objects->boxes[obstacleIndices[i]].pose.position.x/2;// 型心水平点
					float _x = obstacleVertex_x_y[i][0]/2; //避障区内_x > 0
				
					float _y = (fabs(obstacleVertex_x_y[i][1]) + safety_distance_side_)/2;
					float _theta = 2*atan(2*_x/_y);
					float turning_radius = _x / sin(_theta);
					t_roadWheelAngle = asin(vehicle_axis_dis_/turning_radius)*180/M_PI *5;
				}
				else
				{
					t_roadWheelAngle = 5.0;
				}
				
				t_roadWheelAngle = limit_steeringAngle(t_roadWheelAngle,15.0);
				
				avoid_cmd_.cmd2.set_speed = avoid_speed_;
				if(objects->boxes[obstacleIndices[i]].pose.position.y > 0.5) //障碍物在左侧
				{
					avoid_cmd_.cmd2.set_steeringAngle = -t_roadWheelAngle * 200.0/15;
ROS_INFO("car  avoiding  x: %f\ty: %f\t t_roadWheelAngle:%f",obstacleVertex_x_y[i][0],obstacleVertex_x_y[i][1],-t_roadWheelAngle);
				}
				else  //right
				{
					avoid_cmd_.cmd2.set_steeringAngle = t_roadWheelAngle *200.0/15;
ROS_INFO("car  avoiding  x: %f\ty: %f\t t_roadWheelAngle:%f",obstacleVertex_x_y[i][0],obstacleVertex_x_y[i][1],t_roadWheelAngle);
				}
				
				break;
			}
		}
		else if(area == PedestrianDetectionArea && objects->boxes[obstacleIndices[i]].label ==Person) 
		{
			float dis_x = objects->boxes[obstacleIndices[i]].pose.position.x;
			
			avoid_cmd_.status = true;
			avoid_cmd_.just_decelerate =true;
			avoid_cmd_.cmd2.set_speed = 0.0;
			if(dis_x>5)
				avoid_cmd_.cmd2.set_brake = deceleration_2_brakingAperture(0.5*vehicleSpeed_*vehicleSpeed_/(dis_x-5)); // -5 is safty distance!
			else
				avoid_cmd_.cmd2.set_brake = 40;
				
			ROS_INFO("Pedestrian....x: %f\ty: %f \tbrake:%f ",obstacleVertex_x_y[i][0],obstacleVertex_x_y[i][1],avoid_cmd_.cmd2.set_brake);
			//ROS_ERROR("speed: %f\tbrake: %f",avoid_cmd_.cmd2.set_speed,avoid_cmd_.cmd2.set_brake);
			break;
		}
		
	
	}
Delete_memory:
	//printf("set_brake:%f\t set_speed:%f\t speed:%f\t safety_distance_front_:%f\t danger_distance_front_:%f\n",
	//			avoid_cmd_.cmd2.set_brake,avoid_cmd_.cmd2.set_speed,vehicleSpeed_,safety_distance_front_,danger_distance_front_);
	pub_avoid_cmd_.publish(avoid_cmd_);
	
	
	delete [] obstacleDistances;
	for(size_t i=0;i<n_object;i++)
	{
		delete [] obstacleVertex_x_y[i];
	}
	delete [] obstacleVertex_x_y;
	delete [] obstacleIndices;
	delete [] obstacleAreas;
	//ROS_INFO("end");
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


void Avoiding::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)   
{
	static int i=0;
	vehicleSpeed_ = (msg->wheel_speed_FL + msg->wheel_speed_RR)/2*5.0/18; //m/s
	//最大减速度->最短制动距离 + 防撞距离
	danger_distance_front_ = 0.5* vehicleSpeed_ * vehicleSpeed_ /brakingAperture_2_deceleration(40.0)  + 5.0;  
	
	//safety_distance_front_ = danger_distance_front_ + 10.0;
	
	safety_distance_front_ = danger_distance_front_ *(3.2);  // 安全距离 随危险距离变化

	i++;
	if(i%20==0)
		ROS_INFO("callback speed:%f\t danger_distance_front_:%f\t safety_distance_front_:%f",
						vehicleSpeed_,danger_distance_front_,safety_distance_front_);
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

void Avoiding::sds(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects)
{
	float x,y;
	double X,Y;
	for(size_t i=0; i<objects.boxes.size(); i++)
	{
		//object position in vehicle coordination
		x = objects.boxes[i].pose.position.x;
		y = objects.boxes[i].pose.position.y;
		
		//object position in world coordination
		X = x * cos()
		
		float x =  x0 * cos(current_point_.yaw) + y0 * sin(current_point_.yaw) + current_point_.x;
	float y = -x0 * sin(current_point_.yaw) + y0 * cos(current_point_.yaw) + current_point_.y;
	}
	

}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"avoiding_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	Avoiding avoiding;
	if(avoiding.init(nh,nh_private))
		ros::spin();

	return 0;
}


