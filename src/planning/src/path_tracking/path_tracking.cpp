#include"path_tracking.h"


PathTracking::PathTracking()
{

}

PathTracking::~PathTracking()
{

}

bool PathTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	sub_gps = nh.subscribe("/gps",5,&PathTracking::gps_callback,this);
	sub_vehicleState2 = nh.subscribe("/vehicleState2",5,&PathTracking::vehicleSpeed_callback,this);
	sub_telecontrol = nh.subscribe("/telecontrolState" ,2,&PathTracking::telecontrolState_callback,this);
	
	pub_cmd1 = nh.advertise<little_ant_msgs::ControlCmd1>("/controlCmd1",5);
	pub_cmd2 =nh.advertise<little_ant_msgs::ControlCmd2>("/controlCmd2",5);
	
	
}

void PathTracking::gps_callback(const gps_msgs::Inspvax::ConstPtr &msg)
{

}

void PathTracking::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{

}

void PathTracking::telecontrolState_callback(const std_msgs::Bool::ConstPtr& msg)
{
	
}
