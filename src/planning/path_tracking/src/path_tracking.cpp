#include"path_tracking.h"


PathTracking::PathTracking():
	gps_status_(0x00),
	vehicle_speed_status_(false),
	target_point_index_(0),
	nearest_point_index_(0),
	avoiding_offset_(0.0),
	max_roadwheelAngle_(25.0),
	is_avoiding_(false),
	is_laneChanging_(false),
	is_trafficLight_green_(true)
{
	gps_controlCmd_.origin = little_ant_msgs::ControlCmd::_GPS;
	gps_controlCmd_.status = true;
	
	gps_controlCmd_.cmd2.set_gear =1;
	gps_controlCmd_.cmd2.set_speed =0.0;
	gps_controlCmd_.cmd2.set_brake=0.0;
	gps_controlCmd_.cmd2.set_accelerate =0.0;
	gps_controlCmd_.cmd2.set_steeringAngle=0.0;
	gps_controlCmd_.cmd2.set_emergencyBrake =0;
	
	gps_controlCmd_.cmd1.set_driverlessMode =true;
}

PathTracking::~PathTracking()
{
}

bool PathTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	sub_gps_ = nh.subscribe("/gps",5,&PathTracking::gps_callback,this);

	sub_cartesian_gps_ = nh.subscribe("/gps_odom",2,&PathTracking::cartesian_gps_callback,this);

	sub_vehicleState2_ = nh.subscribe("/vehicleState2",1,&PathTracking::vehicleSpeed_callback,this);
	
	sub_vehicleState4_ = nh.subscribe("/vehicleState4",1,&PathTracking::vehicleState4_callback,this);
	
	sub_avoiding_from_lidar_ = nh.subscribe("/start_avoiding",1,&PathTracking::avoiding_flag_callback,this);
	
	sub_trafficLight_ = nh.subscribe("/traffic_light",1,&PathTracking::trafficLight_callback,this);
	
	pub_gps_cmd_ = nh.advertise<little_ant_msgs::ControlCmd>("/sensor_decision",1);
	pub_max_tolerate_speed_ = nh.advertise<std_msgs::Float32>("/max_tolerate_speed",1);
	
	timer_ = nh.createTimer(ros::Duration(0.01),&PathTracking::pub_gps_cmd_callback,this);
	
	pub_related_index_ = nh.advertise<array_msgs::UInt32Array>("/target_and_nearest_points_index",1);
	
	nh_private.param<std::string>("path_points_file",path_points_file_,"");

	nh_private.param<float>("speed",path_tracking_speed_,3.0);

	nh_private.param<float>("foreSightDis_speedCoefficient", foreSightDis_speedCoefficient_,1.8);
	nh_private.param<float>("foreSightDis_latErrCoefficient", foreSightDis_latErrCoefficient_,0.3);
	
	
	nh_private.param<float>("min_foresight_distance",min_foresight_distance_,5.0);
	
	if(path_points_file_.empty())
	{
		ROS_ERROR("no input path points file !!");
		return false;
	}
	
	//start the ros::spin() thread
	rosSpin_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&PathTracking::rosSpinThread, this)));
	
	if(!loadPathPoints(path_points_file_, path_points_))
		return false;
	
	while(ros::ok() && !is_gps_data_valid(current_point_))
	{
		ROS_INFO("gps data is invalid, please check the gps topic or waiting...");
		sleep(1);
	}
	
	target_point_index_ = findNearestPoint(path_points_,current_point_);
	
	if(target_point_index_ > path_points_.size() - 10)
	{
		ROS_ERROR("file read over, No target point was found !!!");
		return false;
	}
	
	while(!vehicle_speed_status_ && ros::ok())
	{
		ROS_INFO("waiting for vehicle speed data ok ...");
		usleep(200000);
	}
	
	target_point_ = path_points_[target_point_index_];
	return true;
}

void PathTracking::rosSpinThread()
{
	ros::spin();
}

void PathTracking::run()
{
	size_t i =0;
	
	ros::Rate loop_rate(20);
	
	while(ros::ok() && target_point_index_ < path_points_.size()-2)
	{
		if( avoiding_offset_ != 0.0)
		{
		//target point offset
			target_point_.x =  avoiding_offset_ * cos(target_point_.yaw) + path_points_[target_point_index_].x;
			target_point_.y = -avoiding_offset_ * sin(target_point_.yaw) + path_points_[target_point_index_].y;
			//printf("new__ x:%lf \ty:%lf \t yaw:%f\n",target_point_.x,target_point_.y,target_point_.yaw);
		}
		
		try{
			lateral_err_ = calculateDis2path(current_point_.x,current_point_.y,path_points_,
											 target_point_index_,&nearest_point_index_) - avoiding_offset_;
		}catch(const char* str){
			ROS_INFO("%s",str);
			break;
		}	
									 
		std::pair<float, float> dis_yaw = get_dis_yaw(target_point_, current_point_);

		if( dis_yaw.first < disThreshold_)
		{
			target_point_ = path_points_[target_point_index_++];
			continue;
		}
		
		//publish the current target and nearest point index
		this->publishRelatedIndex();
		
		float yaw_err = dis_yaw.second - current_point_.yaw;
		
		if(yaw_err==0.0) continue;
		
		float turning_radius = (-0.5 * dis_yaw.first)/sin(yaw_err);

		float t_roadWheelAngle = generateRoadwheelAngleByRadius(turning_radius);
		
		t_roadWheelAngle = limitRoadwheelAngleBySpeed(t_roadWheelAngle,vehicle_speed_);
		
		//ROS_INFO("t_roadWheelAngle :%f\n",t_roadWheelAngle);
		
		if(is_laneChanging_)
		{
			if(fabs(lateral_err_ - avoiding_offset_) < 0.15 && fabs(yaw_err) < 10.0*M_PI/180.0)
			{
				is_laneChanging_  = false;
				gps_controlCmd_.cmd1.set_turnLight_R = false;
				gps_controlCmd_.cmd1.set_turnLight_L = false;
			}
			
			if(lane_width_ >= 1.0)
				gps_controlCmd_.cmd1.set_turnLight_R = true;
			else if(lane_width_ < -1.0)
				gps_controlCmd_.cmd1.set_turnLight_L = true;
		}
		
		float _temp_limit_speed = 30.0;
		static bool is_stop = false;
		static double temp_stop_time;
		
		switch(path_points_[nearest_point_index_].traffic_sign)
		{
			case TrafficSign_TrafficLight:
				if(!is_trafficLight_green_)
					_temp_limit_speed = 0.0;
				break;
				
			case TrafficSign_None:
			case TrafficSign_CarFollow:
				break;
			
			case TrafficSign_TurnLeft:
				gps_controlCmd_.cmd1.set_turnLight_L = true;
				_temp_limit_speed = 20.0;
				break;
				
			case TrafficSign_UTurn:
				gps_controlCmd_.cmd1.set_turnLight_L = true;
				_temp_limit_speed = 5.0;
				break;
			case TrafficSign_TurnRight:
				_temp_limit_speed = 20.0;
				gps_controlCmd_.cmd1.set_turnLight_R = true;
				break;
				
			case TrafficSign_PickUp:
			case TrafficSign_TempStop:
				gps_controlCmd_.cmd1.set_turnLight_R = true;
				_temp_limit_speed = 10.0;
				break;
				
			case TrafficSign_Stop:
				if(is_stop == false)
				{
					temp_stop_time = ros::Time::now().toSec();
					is_stop = true;
					_temp_limit_speed = 0.0;
					gps_controlCmd_.cmd1.set_turnLight_R = false;
				}
				else if(ros::Time::now().toSec() - temp_stop_time > 30)
				{
					_temp_limit_speed = 20.0;
					gps_controlCmd_.cmd1.set_turnLight_L = true;
				}
				else
					_temp_limit_speed = 0.0;
					
				break;
			case TrafficSign_StopArea:
				_temp_limit_speed = 0.0;
				break;
				
			case TrafficSign_CloseTurnLight:
				gps_controlCmd_.cmd1.set_turnLight_L = false;
				gps_controlCmd_.cmd1.set_turnLight_R = false;
				_temp_limit_speed = 20.0;
				is_stop = false;
				break;
			case TrafficSign_AccidentArea:
				gps_controlCmd_.cmd1.set_turnLight_L = false;
				gps_controlCmd_.cmd1.set_turnLight_R = false;
				_temp_limit_speed = 20.0;
				break;
			
			case TrafficSign_Railway :
				_temp_limit_speed = 15.0;
				break;
			
			default :
				_temp_limit_speed = 20.0;
				break;
		}
		
		//ROS_DEBUG("status:%d",path_points_[nearest_point_index_].traffic_sign);
		
		gps_controlCmd_.cmd2.set_speed = 
				limitSpeedByPathCurvature(path_tracking_speed_,path_points_[target_point_index_+10].curvature);
		
		if(_temp_limit_speed < 29.0)
			gps_controlCmd_.cmd2.set_speed = 
				saturationEqual(gps_controlCmd_.cmd2.set_speed,_temp_limit_speed);
		
		this->publishMaxTolerateSpeed();
		
		//gps_controlCmd_.cmd2.set_speed =  limitSpeedByCurrentRoadwheelAngle(path_tracking_speed_,current_roadwheelAngle_);
		
		gps_controlCmd_.cmd2.set_steeringAngle = t_roadWheelAngle * g_steering_gearRatio;
		
		
		if(i%20==0)
		{
			ROS_INFO("curvature:%f",target_point_.curvature);
			ROS_INFO("set_speed:%f\t speed:%f",gps_controlCmd_.cmd2.set_speed ,vehicle_speed_*3.6);
			ROS_INFO("dis2target:%.2f\t yaw_err:%.2f\t lat_err:%.2f",dis_yaw.first,yaw_err*180.0/M_PI,lateral_err_);
			ROS_INFO("disThreshold:%f\t expect roadwheel angle:%.2f\n",disThreshold_,t_roadWheelAngle);
		}
		i++;
		
		loop_rate.sleep();
	}
}

size_t PathTracking::findNearestPoint(const std::vector<gpsMsg_t>& path_points,
									 const gpsMsg_t& current_point)
{
	size_t index = 0;
	float min_dis = FLT_MAX;
	float dis;
	for(size_t i=0; i<path_points.size(); )
	{
		dis = dis2Points(path_points[i],current_point);
		if(dis < min_dis)
		{
			min_dis = dis;
			index = i;
		}
		if(dis>1000)
			i += 1000;
		else if(dis > 500)
			i += 500;
		else if(dis > 250)
			i += 250;
		else if(dis > 125)
			i += 125;
		else if(dis > 63)
			i += 63;
		else if(dis > 42)
			i += 42;
		else if(dis > 21)
			i += 21;
		else
			i += 2;
	}
	if(min_dis >50)
		return path_points.size();
	
	return index;
}

void PathTracking::publishRelatedIndex()
{
	array_msgs::UInt32Array msg;
	msg.data.push_back(target_point_index_);
	msg.data.push_back(nearest_point_index_) ;
	pub_related_index_.publish(msg);
}

void PathTracking::publishMaxTolerateSpeed()
{
	std_msgs::Float32 msg;
	msg.data = generateMaxTolarateSpeedByCurvature(path_points_[target_point_index_+10].curvature);
	pub_max_tolerate_speed_.publish(msg);
}

float PathTracking::point2point_dis(gpsMsg_t &point1,gpsMsg_t &point2)
{
	float x = (point1.longitude -point2.longitude)*111000*cos(point1.latitude*M_PI/180.0);
	float y = (point1.latitude - point2.latitude ) *111000;
	return  sqrt(x * x + y * y);
}


void PathTracking::pub_gps_cmd_callback(const ros::TimerEvent&)
{
	pub_gps_cmd_.publish(gps_controlCmd_);
}

void PathTracking::gps_callback(const gps_msgs::Inspvax::ConstPtr &msg)
{
	if(gps_status_!=0x03)
		gps_status_ |= 0x01;
	
	current_point_.longitude = msg->longitude;
	current_point_.latitude = msg->latitude;
	current_point_.yaw = deg2rad(msg->azimuth);
	//std::cout << "call_back ID: "<< std::this_thread::get_id()  << std::endl;
}

void PathTracking::cartesian_gps_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	if(gps_status_!=0x03)
		gps_status_ |= 0x02;
	
	current_point_.x = msg->pose.pose.position.x;
	current_point_.y = msg->pose.pose.position.y;
	/*
	tf::Quaternion quat;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
	
	double roll,pitch;
	
	tf::Matrix3x3(quat).getRPY(roll, pitch, current_point_.yaw);
*/
}

void PathTracking::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	vehicle_speed_status_ = true;
	vehicle_speed_ = msg->vehicle_speed; //  m/s
	
	if(vehicle_speed_ >20.0)
		return;
	
	disThreshold_ = foreSightDis_speedCoefficient_ * vehicle_speed_ ;
	if(disThreshold_ < min_foresight_distance_) 
		disThreshold_  = min_foresight_distance_;
	
	disThreshold_ = disThreshold_ * (1.0 + foreSightDis_latErrCoefficient_ * fabs(lateral_err_) );
	
	//ROS_INFO("disThreshold:%f\t lateral_err:%f",disThreshold_,lateral_err_);
	
	danger_distance_front_ = generateDangerDistanceBySpeed(vehicle_speed_);  
	safety_distance_front_ = generateSafetyDisByDangerDis(danger_distance_front_);
}

void PathTracking::vehicleState4_callback(const little_ant_msgs::State4::ConstPtr& msg)
{
	current_roadwheelAngle_ = msg->steeringAngle/g_steering_gearRatio;
}

void PathTracking::avoiding_flag_callback(const std_msgs::Float32::ConstPtr& msg)
{
	if(avoiding_offset_ != msg->data && is_laneChanging_ == false)
	{
		is_laneChanging_ = true;
		//line change width  left(-)  right(+)
		lane_width_ = avoiding_offset_ - lateral_err_ ;
	}
	
	//avoid to left(-) or right(+) the value presents the offset
	avoiding_offset_ = msg->data;
}

void PathTracking::trafficLight_callback(const std_msgs::Bool::ConstPtr& msg)
{
	is_trafficLight_green_ = msg->data;
}

#if IS_POLAR_COORDINATE_GPS ==1
bool PathTracking::is_gps_data_valid(gpsMsg_t& point)
{
	if(	gps_status_ > 0 &&
		point.longitude >10.0 && point.longitude < 170.0 && 
		point.latitude >15.0 && point.latitude <70.0)
		return true;
	return false;
}

std::pair<float, float> PathTracking::get_dis_yaw(gpsMsg_t &point1,gpsMsg_t &point2)
{
	float x = (point1.longitude -point2.longitude)*111000*cos(point1.latitude);
	float y = (point1.latitude - point2.latitude ) *111000;
	
	std::pair<float, float> dis_yaw;
	dis_yaw.first = sqrt(x * x + y * y);
	dis_yaw.second = atan2(x,y);
	
	//ROS_INFO("second:%f")
	
	if(dis_yaw.second <0)
		dis_yaw.second += 2*M_PI;
	return dis_yaw;
}
float PathTracking::dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
{
	float x = (point1.longitude -point2.longitude)*111000*cos(point1.latitude);
	float y = (point1.latitude - point2.latitude ) *111000;
	
	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}

#else
bool PathTracking::is_gps_data_valid(gpsMsg_t& point)
{
	if(gps_status_ == 0x03 && point.x !=0 && point.y !=0)
		return true;
	return false;
}


std::pair<float, float> PathTracking::get_dis_yaw(gpsMsg_t &point1,gpsMsg_t &point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	std::pair<float, float> dis_yaw;
	dis_yaw.first = sqrt(x * x + y * y);
	dis_yaw.second = atan2(x,y);
	
	if(dis_yaw.second <0)
		dis_yaw.second += 2*M_PI;
	return dis_yaw;
}

float PathTracking::dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}

#endif


int main(int argc,char**argv)
{
	ros::init(argc,argv,"path_tracking");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	state_detection::debugSystemInitial();
	
	PathTracking path_tracking;
	if(!path_tracking.init(nh,nh_private))
		return 1;
	path_tracking.run();
	
	ROS_INFO("path tracking completed.");
	ros::shutdown();
	

	return 0;
}
