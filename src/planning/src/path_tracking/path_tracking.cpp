#include"path_tracking.h"

#define AXIS_DIS 1.5

PathTracking::PathTracking()
{
	controlCmd2.set_gear =1;
	controlCmd2.set_speed =10.0;
	controlCmd2.set_brake=0.0;
	controlCmd2.set_accelerate =0.0;
	controlCmd2.set_steeringAngle=0.0;
	controlCmd2.set_emergencyBrake =0;
	
	controlCmd1.set_driverlessMode =true;
	is_telecontrol = true;
}

PathTracking::~PathTracking()
{

}

bool PathTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	sub_gps = nh.subscribe("/gps",5,&PathTracking::gps_callback,this);
	sub_vehicleState2 = nh.subscribe("/vehicleState2",5,&PathTracking::vehicleSpeed_callback,this);
	sub_telecontrol = nh.subscribe("/is_telecontrol" ,2,&PathTracking::telecontrolState_callback,this);
	
	timer1 = nh.createTimer(ros::Duration(0.01),&PathTracking::pub_cmd2_10ms,this);
	timer2 = nh.createTimer(ros::Duration(0.02),&PathTracking::pub_cmd1_20ms,this);
	
	pub_cmd1 = nh.advertise<little_ant_msgs::ControlCmd1>("/controlCmd1",5);
	pub_cmd2 =nh.advertise<little_ant_msgs::ControlCmd2>("/controlCmd2",5);
	
	nh.param<std::string>("path_points_file",path_points_file_,std::string("/home/wendao/gps_data.txt"));
	nh.param<float>("disThreshold",disThreshold_,5.0);
	
	
	fp = fopen(path_points_file_.c_str(),"r");
	
	if(fp==NULL)
	{
		ROS_ERROR("open path_points_file failed");
		return false;
	}
	return true;
}

void PathTracking::run()
{
	while(ros::ok())
	{
		
		ros::spinOnce();
		if(current_point.longitude <1.0 && current_point.latitude <1.0)//初始状态或者数据异常
			continue;
			
		if(feof(fp)) break; //file complete
		float dis2target = point2point_dis(current_point,target_point);
		
		if( dis2target < disThreshold_ || dis2target>1000.0)//初始状态下 target(0,0)-> dis2target 将会很大
			fscanf(fp,"%lf\t%lf\n",&target_point.longitude,&target_point.latitude);
		
		
		
		std::pair<float, float> dis_yaw = get_dis_yaw(current_point,target_point);
		
		float yaw_err = dis_yaw.second - current_point.yaw;
		
		if(yaw_err==0.0) continue;
		
		float turning_radius = (-0.5 * dis_yaw.first)/sin(deg2rad(yaw_err));
		
		float t_roadWheelAngle = asin(AXIS_DIS/turning_radius)*180/M_PI;
		
		printf("%.7f,%.7f,%.2f\t%.7f,%.7f\n",
				current_point.longitude,current_point.latitude,current_point.yaw,
				target_point.longitude,target_point.latitude);
		printf("dis:%f\tyaw_err:%f\t Radius:%f\t t_roadWheelAngle:%f\n",dis_yaw.first,yaw_err,turning_radius,t_roadWheelAngle);
		
		limitSteeringAngle(t_roadWheelAngle);
		
		controlCmd2.set_steeringAngle = t_roadWheelAngle *500.0/30.0;
		
		usleep(8000);
	}
}


float PathTracking::deg2rad(float deg)
{
	return  (deg/180.0)*M_PI;
}

void PathTracking::limitSteeringAngle(float& angle)
{
	if(angle>30.0) angle =30.0;
	else if(angle<-30.0) angle =-30.0;
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
	dis_yaw.second = atan2(x,y);
	if(dis_yaw.second <0)
		dis_yaw.second += 2*M_PI;
	return dis_yaw;
}

void PathTracking::pub_cmd2_10ms(const ros::TimerEvent&)
{
	//ROS_ERROR("is_telecontrol:%d",is_telecontrol);
	if(is_telecontrol==false)
		pub_cmd2.publish(controlCmd2);
}

void PathTracking::pub_cmd1_20ms(const ros::TimerEvent&)
{
	if(is_telecontrol ==false)
		pub_cmd1.publish(controlCmd1);
}

void PathTracking::gps_callback(const gps_msgs::Inspvax::ConstPtr &msg)
{
	current_point.longitude = msg->longitude;
	current_point.latitude = msg->latitude;
	current_point.yaw = msg->azimuth;
}

void PathTracking::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	
}

void PathTracking::telecontrolState_callback(const std_msgs::Bool::ConstPtr& msg)
{
	is_telecontrol = msg->data;
}
