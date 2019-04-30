#include"path_tracking.h"


PathTracking::PathTracking():
	gps_status_(0x00),
	target_point_index_(0),
	avoiding_offset_(0.0),
	disThreshold_(6.0),
	max_roadwheelAngle_(25.0),
	is_avoiding_(false)
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
	
	
	pub_gps_cmd_ = nh.advertise<little_ant_msgs::ControlCmd>("/sensor_decision",5);
	
	timer_ = nh.createTimer(ros::Duration(0.01),&PathTracking::pub_gps_cmd_callback,this);
	
	pub_tracking_target_index_ = nh.advertise<std_msgs::UInt32>("/track_target_index",1);
	
	nh_private.param<std::string>("path_points_file",path_points_file_,"");
									
	nh_private.param<float>("speed",path_tracking_speed_,3.0);
	
	nh_private.param<float>("maxOffset_left",maxOffset_left_,-0.5);
	nh_private.param<float>("maxOffset_right",maxOffset_right_,0.5);
	
	if(path_points_file_.empty())
	{
		ROS_ERROR("no input path points file !!");
		return false;
	}
	
	//start the ros::spin() thread
	rosSpin_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&PathTracking::rosSpinThread, this)));
	
	if(!loadPathPoints(path_points_file_, path_points_))
		return false;

	
	float last_distance = 999999999999;
	float current_distance = 0;
	
	for(target_point_index_ =0; target_point_index_<path_points_.size()&& ros::ok(); )
	{
		if(!is_gps_data_valid(current_point_))
		{
			ROS_INFO("gps data is invalid, please check the gps topic or waiting...");
			usleep(100000);
			continue;
		}
		
		target_point_ = path_points_[target_point_index_];
		
		current_distance = get_dis_yaw(current_point_,target_point_).first;
		
		ROS_INFO("current_distance:%f\t last_distance:%f",current_distance,last_distance);
		
		if(current_distance - last_distance > 0)
			break;
		
		last_distance = current_distance;
		
		target_point_index_++;
	}
	
	ROS_INFO("first target index:%d   total index:%d",target_point_index_,path_points_.size());
	
	if(target_point_index_ == path_points_.size())
	{
		ROS_ERROR("file read over, No target was found");
		return false;
	}
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
	
	while(ros::ok() && target_point_index_ < path_points_.size()-1)
	{
		//publish the current target index
		std_msgs::UInt32 index;   index.data = target_point_index_;
		pub_tracking_target_index_.publish(index);
		
		/*printf("x:%lf \ty:%lf \t yaw:%f\n",path_points_[target_point_index_].x,
										  path_points_[target_point_index_].y,
										  path_points_[target_point_index_].yaw);*/
		if( avoiding_offset_ != 0.0)
		{
		//target point offset
			target_point_.x =  avoiding_offset_ * cos(target_point_.yaw) + path_points_[target_point_index_].x;
			target_point_.y = -avoiding_offset_ * sin(target_point_.yaw) + path_points_[target_point_index_].y;
			//printf("new__ x:%lf \ty:%lf \t yaw:%f\n",target_point_.x,target_point_.y,target_point_.yaw);
		}
		lateral_err_ = calculateDis2path(current_point_.x,current_point_.y,path_points_,target_point_index_)
					   -avoiding_offset_;
		float dis_threshold = disThreshold_;  /*(1 + sqrt(fabs(lateral_err_)));*/
		std::pair<float, float> dis_yaw = get_dis_yaw(current_point_,target_point_);
		if( dis_yaw.first < dis_threshold)
		{
			target_point_ = path_points_[target_point_index_++];
			continue;
		}
		
		float yaw_err = dis_yaw.second - current_point_.yaw;
		
		if(yaw_err==0.0) continue;
		
		float turning_radius = (-0.5 * dis_yaw.first)/sin(yaw_err);

		float t_roadWheelAngle = generateRoadwheelAngleByRadius(turning_radius);
		
		//ROS_INFO("0 t_roadWheelAngle :%f",t_roadWheelAngle);
		
		t_roadWheelAngle = limitRoadwheelAngleBySpeed(t_roadWheelAngle,vehicle_speed_);
		
		//ROS_INFO("1 t_roadWheelAngle :%f",t_roadWheelAngle);
		
		gps_controlCmd_.cmd2.set_speed = path_tracking_speed_;/* limitSpeedByCurrentRoadwheelAngle(path_tracking_speed_,current_roadwheelAngle_);*/
		gps_controlCmd_.cmd2.set_steeringAngle = -t_roadWheelAngle * g_steering_gearRatio;
		
		if(i%50==0)
		{
			ROS_INFO("dis2target:%.2f\t yaw_err:%.2f\t lat_err:%.2f",dis_yaw.first,yaw_err*180.0/M_PI,lateral_err_);
			ROS_INFO("disThreshold:%f\t expect roadwheel angle:%.2f",dis_threshold,t_roadWheelAngle);
		}
		i++;
		
		loop_rate.sleep();
	}
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
	gps_status_ |= 0x01;
	
	current_point_.longitude = msg->longitude;
	current_point_.latitude = msg->latitude;
	current_point_.yaw = deg2rad(msg->azimuth);
	//std::cout << "call_back ID: "<< std::this_thread::get_id()  << std::endl;
}

void PathTracking::cartesian_gps_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
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
	vehicle_speed_ = msg->vehicle_speed; //  m/s
	
	if(vehicle_speed_ >20.0)
		return;
	
	disThreshold_ = 2.0*vehicle_speed_ ;
	if(disThreshold_ < 3.0) 
		disThreshold_  = 3.0;
}

void PathTracking::vehicleState4_callback(const little_ant_msgs::State4::ConstPtr& msg)
{
	current_roadwheelAngle_ = msg->steeringAngle/g_steering_gearRatio;
}

void PathTracking::avoiding_flag_callback(const std_msgs::Float32::ConstPtr& msg)
{
	//avoid to left(-) or right(+) the value presents the offset
	avoiding_offset_ = msg->data;
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

#endif


int main(int argc,char**argv)
{
	ros::init(argc,argv,"path_tracking");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	PathTracking path_tracking;
	if(!path_tracking.init(nh,nh_private))
		return 1;
	path_tracking.run();

	return 0;
}
