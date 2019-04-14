#include"path_tracking.h"


PathTracking::PathTracking()
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
	fclose(fp);
}

bool PathTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	sub_gps_ = nh.subscribe("/gps",5,&PathTracking::gps_callback,this);
	sub_vehicleState2_ = nh.subscribe("/vehicleState2",5,&PathTracking::vehicleSpeed_callback,this);
	sub_avoiding_from_lidar_ = nh.subscribe("/start_avoiding",2,&PathTracking::avoiding_flag_callback,this);
	
	timer_ = nh.createTimer(ros::Duration(0.01),&PathTracking::pub_gps_cmd_callback,this);
	
	pub_gps_cmd_ = nh.advertise<little_ant_msgs::ControlCmd>("/sensor_decision",5);
	
	nh_private.param<float>("vehicle_axis_dis",vehicle_axis_dis_,1.50);
	nh_private.param<std::string>("path_points_file",path_points_file_,std::string("/home/wendao/projects/littleAnt_ws/src/data/data/2.txt"));
	nh_private.param<float>("disThreshold",disThreshold_,5.0);
	nh_private.param<float>("speed",speed_,3.0);
	
	nh_private.param<float>("avoiding_disThreshold",avoiding_disThreshold_,15.0);
	
	//start the ros::spin() thread
	rosSpin_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&PathTracking::rosSpinThread, this)));
	
	fp = fopen(path_points_file_.c_str(),"r");
	
	if(fp==NULL)
	{
		ROS_ERROR("open %s failed",path_points_file_.c_str());
		return false;
	}
	
	float last_distance = 99999;
	float current_distance = 0;
	
	while(ros::ok() && !feof(fp))
	{
		if(!is_lon_lat_valid(current_point_))
		{
			ROS_INFO("gps data is invalid, please check the gps topic or waiting...");
			usleep(20000);
			continue;
		}
		
		//std::cout << "main_Id: "<< std::this_thread::get_id()  << std::endl;
		
		fscanf(fp,"%lf\t%lf\n",&target_point_.longitude,&target_point_.latitude);
		std::pair<float, float> dis_yaw = get_dis_yaw(current_point_,target_point_);
		current_distance = dis_yaw.first;
		
		ROS_INFO("current_distance:%f\t last_distance:%f",current_distance,last_distance);
		
		if(current_distance - last_distance > 0)
			break;
		
		//ROS_INFO("current_distance:%f\t last_distance:%f",current_distance,last_distance);	
		last_distance = current_distance;
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
	while(ros::ok() && !feof(fp))
	{
		std::pair<float, float> dis_yaw = get_dis_yaw(current_point_,target_point_);
		
		if( dis_yaw.first < disThreshold_)
		{
			fscanf(fp,"%lf\t%lf\n",&target_point_.longitude,&target_point_.latitude);
			continue;
		}
			
		
		float yaw_err = dis_yaw.second - current_point_.yaw;
		
		if(yaw_err==0.0) continue;
		
		float turning_radius = (-0.5 * dis_yaw.first)/sin(deg2rad(yaw_err));
		
		float t_roadWheelAngle = asin(vehicle_axis_dis_/turning_radius)*180/M_PI;
		
		if(i%50==0)
		{
			printf("%.7f,%.7f,%.2f\t%.7f,%.7f\t t_yaw:%f\n",
					current_point_.longitude,current_point_.latitude,current_point_.yaw,
					target_point_.longitude,target_point_.latitude,dis_yaw.second);
			printf("dis:%f\tyaw_err:%f\t Radius:%f\t t_roadWheelAngle:%f\n",
					dis_yaw.first,yaw_err,turning_radius,t_roadWheelAngle);
		}
		i++;
		
		limitRoadWheelAngle(t_roadWheelAngle);
		gps_controlCmd_.cmd2.set_speed = speed_;
		gps_controlCmd_.cmd2.set_steeringAngle = -t_roadWheelAngle *350.0/20.0;
		
		usleep(15000);
		
	}
}


float PathTracking::deg2rad(float deg)
{
	return  (deg/180.0)*M_PI;
}

void PathTracking::limitRoadWheelAngle(float& angle)
{
	if(angle>20.0) angle =20.0;
	else if(angle<-20.0) angle =-20.0;
}



float PathTracking::point2point_dis(gpsMsg_t &point1,gpsMsg_t &point2)
{
	float x = (point1.longitude -point2.longitude)*111000*cos(point1.latitude*M_PI/180.0);
	float y = (point1.latitude - point2.latitude ) *111000;
	return  sqrt(x * x + y * y);
}

std::pair<float, float> PathTracking::get_dis_yaw(gpsMsg_t &point1,gpsMsg_t &point2)
{
	float x = (point1.longitude -point2.longitude)*111000*cos(point1.latitude*M_PI/180.0);
	float y = (point1.latitude - point2.latitude ) *111000;
	
	std::pair<float, float> dis_yaw;
	dis_yaw.first = sqrt(x * x + y * y);
	dis_yaw.second = atan2(x,y) *180.0/M_PI;
	
	//ROS_INFO("second:%f")
	
	if(dis_yaw.second <0)
		dis_yaw.second += 360.0;
	return dis_yaw;
}

void PathTracking::pub_gps_cmd_callback(const ros::TimerEvent&)
{
	pub_gps_cmd_.publish(gps_controlCmd_);
}

void PathTracking::gps_callback(const gps_msgs::Inspvax::ConstPtr &msg)
{
	current_point_.longitude = msg->longitude;
	current_point_.latitude = msg->latitude;
	current_point_.yaw = msg->azimuth;
	//std::cout << "call_back ID: "<< std::this_thread::get_id()  << std::endl;
}

void PathTracking::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	
}

void PathTracking::avoiding_flag_callback(const std_msgs::Int8::ConstPtr& msg)
{
	if(msg->data== 1)
	{
		while(ros::ok() && !feof(fp))
		{
			fscanf(fp,"%lf\t%lf\n",&target_point_.longitude,&target_point_.latitude);
			std::pair<float, float> dis_yaw = get_dis_yaw(current_point_,target_point_);
			if( dis_yaw.first >= avoiding_disThreshold_)
				break;
		}
	}
}

bool PathTracking::is_lon_lat_valid(gpsMsg_t& point)
{
	if(point.longitude >10.0 && point.longitude < 170.0 && 
		point.latitude >15.0 && point.latitude <70.0)
		return true;
	return false;
}

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
