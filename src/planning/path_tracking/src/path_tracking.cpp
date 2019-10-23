#include<ros/ros.h>
#include<little_ant_msgs/ControlCmd.h>
#include<little_ant_msgs/State2.h>  //speed
#include<little_ant_msgs/State4.h>  //steerAngle
#include<std_msgs/Bool.h>
#include<std_msgs/UInt32.h>
#include<std_msgs/UInt8.h>
#include<vector>

#include<nav_msgs/Odometry.h> 
#include<geometry_msgs/Quaternion.h>
#include<tf/transform_datatypes.h>
#include<std_msgs/Float32.h>

#include<ant_math/ant_math.h>
#include<path_tracking/State.h>
#include<climits>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <thread>

#define LowSpeed 15.0

class PathTracking
{
public:
	PathTracking();
	~PathTracking();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	void run();
	
	void pub_gps_cmd_callback(const ros::TimerEvent&);
	void gps_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
	
	void vehicleState4_callback(const little_ant_msgs::State4::ConstPtr& msg);
	void vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg);
	void avoiding_flag_callback(const std_msgs::Float32::ConstPtr& msg);

	bool is_gps_data_valid(gpsMsg_t& point);
	void rosSpinThread(){ros::spin();}

private:
	gpsMsg_t pointOffset(const gpsMsg_t& point,float offset);
	void publishPathTrackingState();
private:
	ros::Subscriber sub_utm_odom_;
	ros::Subscriber sub_vehicleState2_;
	ros::Subscriber sub_vehicleState4_;
	ros::Subscriber sub_avoiding_from_lidar_;
	ros::Timer timer_;
	
	ros::Publisher pub_gps_cmd_;
	little_ant_msgs::ControlCmd gps_controlCmd_;
	
	ros::Publisher pub_tracking_state_;
	path_tracking::State tracking_state_;
	
	ros::Publisher pub_acc_;
	bool is_acc_;
	
	boost::shared_ptr<boost::thread> rosSpin_thread_ptr_;
	
	std::string path_points_file_;
	std::vector<gpsMsg_t> path_points_;
	
	gpsMsg_t current_point_, target_point_;
	
	float min_foresight_distance_;
	float disThreshold_;
	float avoiding_offset_;
	
	float path_tracking_speed_;
	
	bool vehicle_speed_status_;
	
	float vehicle_speed_;
	float speed_limit_;
	float current_roadwheelAngle_;
	
	float safety_distance_front_;
	float danger_distance_front_;
	
	float max_roadwheelAngle_;
	bool is_avoiding_;
	float lateral_err_;
	float yaw_err_;
	
	size_t target_point_index_;
	size_t nearest_point_index_;
	
	float foreSightDis_speedCoefficient_;
	float foreSightDis_latErrCoefficient_;
	bool key_;
	
};

PathTracking::PathTracking():
	vehicle_speed_status_(false),
	target_point_index_(0),
	nearest_point_index_(0),
	avoiding_offset_(0.0),
	max_roadwheelAngle_(25.0),
	is_avoiding_(false),
	is_acc_(false)
{
	gps_controlCmd_.origin = little_ant_msgs::ControlCmd::_GPS;
	gps_controlCmd_.status = true;
	
	gps_controlCmd_.cmd2.set_gear =1;
	gps_controlCmd_.cmd2.set_speed =0.0;
	gps_controlCmd_.cmd2.set_brake=0.0;
	gps_controlCmd_.cmd2.set_accelerate =0.0;
	gps_controlCmd_.cmd2.set_roadWheelAngle =0.0;
	gps_controlCmd_.cmd2.set_emergencyBrake =0;
	
	gps_controlCmd_.cmd1.set_driverlessMode =true;
}

PathTracking::~PathTracking()
{
}

bool PathTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	std::string utm_odom_topic = nh_private.param<std::string>("utm_odom_topic","/ll2utm_odom");
	std::string tracking_info_topic = nh_private.param<std::string>("tracking_info_topic","/tracking_state");
	
	sub_utm_odom_ = nh.subscribe(utm_odom_topic, 5,&PathTracking::gps_odom_callback,this);

	sub_vehicleState2_ = nh.subscribe("/vehicleState2",1,&PathTracking::vehicleSpeed_callback,this);
	
	sub_vehicleState4_ = nh.subscribe("/vehicleState4",1,&PathTracking::vehicleState4_callback,this);
	
	sub_avoiding_from_lidar_ = nh.subscribe("/start_avoiding",1,&PathTracking::avoiding_flag_callback,this);
	
	pub_gps_cmd_ = nh.advertise<little_ant_msgs::ControlCmd>("/sensor_decision",1);
	
	timer_ = nh.createTimer(ros::Duration(0.01),&PathTracking::pub_gps_cmd_callback,this);
	
	pub_tracking_state_ = nh.advertise<path_tracking::State>(tracking_info_topic,1);
	pub_acc_ = nh.advertise<std_msgs::Bool>("/is_acc",1);
	
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
	
	ROS_INFO("pathPoints size:%d",path_points_.size());
	
	while(ros::ok() && !is_gps_data_valid(current_point_))
	{
		ROS_INFO("gps data is invalid, please check the gps topic or waiting...");
		sleep(1);
	}
	
	target_point_index_ = findNearestPoint(path_points_,current_point_);
	
	if(target_point_index_ > path_points_.size() - 10)
	{
		ROS_ERROR("target index:%d ?? file read over, No target point was found !!!",target_point_index_);
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

gpsMsg_t PathTracking::pointOffset(const gpsMsg_t& point,float offset)
{
	gpsMsg_t result = point;
	result.x =  offset * cos(point.yaw) + point.x;
	result.y = -offset * sin(point.yaw) + point.y;
	return result;
}

void PathTracking::run()
{
	size_t i =0;
	
	ros::Rate loop_rate(30);
	
	while(ros::ok() && target_point_index_ < path_points_.size()-2)
	{
		speed_limit_ = path_tracking_speed_;
		is_acc_ = false;
		if(nearest_point_index_ < 700) //satrt
			speed_limit_ = 10.0;
		else if(nearest_point_index_ > 2026 && nearest_point_index_ < 2746) //acc
		{
			is_acc_ = true;
			std_msgs::Bool acc; acc.data = true;
			pub_acc_.publish(acc);
			speed_limit_ = LowSpeed;
		}
		else if(nearest_point_index_ >2746 && nearest_point_index_ < 3800) //path narrow
			speed_limit_ = LowSpeed;
		else if(nearest_point_index_ >4460 && nearest_point_index_ < 5238) //person
			speed_limit_ = LowSpeed;
		else if(nearest_point_index_ >8443 && nearest_point_index_ < 9615) //working
			speed_limit_ = LowSpeed;
		else if(nearest_point_index_ >10644 && nearest_point_index_ < 11168) //phone
			speed_limit_ = LowSpeed;
		size_t A = 10854, B = 10967; //phone
		static int stoped1 = 0 ; //0: unstop 1: stop 2: stoped
		static double stop_time;
		float stop_duration = 5.0;
		if(nearest_point_index_ > A && nearest_point_index_ < B)
		{
//			ROS_ERROR("stoped1:%d",stoped1);
			if(stoped1 == 0)
			{
				speed_limit_ = (B-nearest_point_index_)/(B-A)*LowSpeed;
				if(speed_limit_ < 0) speed_limit_ = 0.0;
				if(vehicle_speed_ < 1.0) //stoped
				{
//					ROS_ERROR("stop+++++++++++++++++++++++++++++++++++");
					stop_time = ros::Time::now().toSec();
					stoped1 = 1;
				}
			}
			else if(stoped1 == 1)
			{
				if(ros::Time::now().toSec()-stop_time > stop_duration)
				{
//					ROS_ERROR("duration: %f",ros::Time::now().toSec()-stop_time);
					speed_limit_ = path_tracking_speed_;
					stoped1 = 2;
				}
				else
					speed_limit_ = 0.0;
			}
		}
		
		if(nearest_point_index_ >12429 && nearest_point_index_ < 12665) //up and down
			speed_limit_ = LowSpeed;
			
		static int stoped2 = 0 ; //0: unstop 1: stop 2: stoped
		A = 12665; B = 12765;
		if(nearest_point_index_ > A && nearest_point_index_ < B)
		{
//			ROS_ERROR("stoped:%d",stoped2);
			if(stoped2 == 0)
			{
				speed_limit_ = (B-nearest_point_index_)/(B-A)*LowSpeed;
				if(speed_limit_ < 0) speed_limit_ = 0.0;
				if(vehicle_speed_ < 1.0) //stoped
				{
//					ROS_ERROR("stop+++++++++++++++++++++++++++++++++++");
					stop_time = ros::Time::now().toSec();
					stoped2 = 1;
				}
			}
			else if(stoped2 == 1)
			{
				if(ros::Time::now().toSec()-stop_time > stop_duration)
				{
					speed_limit_ = path_tracking_speed_;
					stoped2 = 2;
//					ROS_ERROR("duration: %f",ros::Time::now().toSec()-stop_time);
				}
				else
					speed_limit_ = 0.0;
			}
		}
		if(nearest_point_index_ > 14035 && nearest_point_index_ < 14699) //turn rignt
			speed_limit_ = LowSpeed;
		
		A = 16385; B = 16435;
		if(nearest_point_index_ > 16079 && nearest_point_index_ < A)
			speed_limit_ = LowSpeed;
		else if(nearest_point_index_ > A && nearest_point_index_ < B)
		{
			speed_limit_ = (B-nearest_point_index_)/(B-A)*LowSpeed;
			ROS_ERROR("speed_limit_:%f",speed_limit_);
			if(speed_limit_ < 0) speed_limit_ = 0.0;
		}
		else if(nearest_point_index_ > B)
			speed_limit_ = 0;
		
		
		if(key_)
			target_point_ = pointOffset(path_points_[target_point_index_],-2.5);
		if(!key_ && avoiding_offset_ != 0.0 && !is_acc_)
			target_point_ = pointOffset(path_points_[target_point_index_],avoiding_offset_);
		
		try
		{
			lateral_err_ = calculateDis2path(current_point_.x,current_point_.y,path_points_,
											 target_point_index_,&nearest_point_index_) - avoiding_offset_;
		}catch(const char* str)
		{
			ROS_INFO("%s",str);
			break;
		}
		
		disThreshold_ = foreSightDis_speedCoefficient_ * vehicle_speed_ + foreSightDis_latErrCoefficient_ * fabs(lateral_err_);
	
		if(disThreshold_ < min_foresight_distance_) 
			disThreshold_  = min_foresight_distance_;
			
		// disThreshold_ = min_foresight_distance_ + foreSightDis_speedCoefficient_ * vehicle_speed_ + foreSightDis_latErrCoefficient_ * fabs(lateral_err_);
	
		//ROS_INFO("disThreshold:%f\t lateral_err:%f",disThreshold_,lateral_err_);
									 
		std::pair<float, float> dis_yaw = get_dis_yaw(target_point_, current_point_);

		if( dis_yaw.first < disThreshold_)
		{
			target_point_ = path_points_[++target_point_index_];
			continue;
		}
		
		yaw_err_ = dis_yaw.second - current_point_.yaw;
		
		if(yaw_err_==0.0) continue;
		
		float turning_radius = (-0.5 * dis_yaw.first)/sin(yaw_err_);

		float t_roadWheelAngle = generateRoadwheelAngleByRadius(turning_radius);
		
		t_roadWheelAngle = limitRoadwheelAngleBySpeed(t_roadWheelAngle,vehicle_speed_);
		
		//ROS_INFO("t_roadWheelAngle :%f\n",t_roadWheelAngle);
		
		//find the index of a path point 8.0meters from the current point
		size_t index = findIndexForGivenDis(path_points_,nearest_point_index_,disThreshold_ + 20.0); 
		if(index ==0)
		{
			ROS_INFO("findIndexForGivenDis faild!");
			break;
		}
		float max_curvature = maxCurvatureInRange(path_points_, nearest_point_index_, index);
		
		float set_speed = limitSpeedByPathCurvature(path_tracking_speed_,max_curvature);
		
		gps_controlCmd_.cmd2.set_speed =  set_speed > speed_limit_ ? speed_limit_ : set_speed;
		
		this->publishPathTrackingState();
		
		gps_controlCmd_.cmd2.set_roadWheelAngle = t_roadWheelAngle;
		
		if(i%30==0)
		{
			ROS_INFO("tar_cur:%f\t max_cur:%f",target_point_.curvature, max_curvature);
			ROS_INFO("nearest_point_index:%d \t target_point_index:%d end_index:%d",
					nearest_point_index_,target_point_index_,index);
			ROS_INFO("set_speed:%f\t speed:%f",gps_controlCmd_.cmd2.set_speed ,vehicle_speed_*3.6);
			ROS_INFO("dis2target:%.2f\t yaw_err:%.2f\t lat_err:%.2f",dis_yaw.first,yaw_err_*180.0/M_PI,lateral_err_);
			ROS_INFO("disThreshold:%f\t expect roadwheel angle:%.2f",disThreshold_,t_roadWheelAngle);
			ROS_INFO("avoiding_offset_:%f\n",avoiding_offset_);
		}
		i++;
		
		loop_rate.sleep();
	}
	
	ROS_INFO("driverless completed...");
	
	gps_controlCmd_.cmd2.set_roadWheelAngle = 0.0;
	gps_controlCmd_.cmd2.set_speed = 0.0;
	
	while(ros::ok())
	{
		sleep(1);
	}
}

void PathTracking::publishPathTrackingState()
{
	tracking_state_.header.stamp = ros::Time::now();
	tracking_state_.position_x = current_point_.x;
	tracking_state_.position_y = current_point_.y;
	tracking_state_.yaw = current_point_.yaw;
	tracking_state_.vehicle_speed =  vehicle_speed_;
	tracking_state_.roadwheel_angle = current_roadwheelAngle_;
	tracking_state_.lateral_error = lateral_err_;
	tracking_state_.yaw_error = yaw_err_;
	
	tracking_state_.target_index = target_point_index_;
	tracking_state_.current_index = nearest_point_index_;
	pub_tracking_state_.publish(tracking_state_);
	
}

void PathTracking::pub_gps_cmd_callback(const ros::TimerEvent&)
{
	pub_gps_cmd_.publish(gps_controlCmd_);
}


void PathTracking::gps_odom_callback(const nav_msgs::Odometry::ConstPtr& utm)
{
	current_point_.x = utm->pose.pose.position.x;
	current_point_.y = utm->pose.pose.position.y;
	current_point_.yaw = utm->pose.covariance[0];
}

void PathTracking::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	if(vehicle_speed_ >20.0)
		return;
	vehicle_speed_status_ = true;
	vehicle_speed_ = msg->vehicle_speed; //  m/s
}

void PathTracking::vehicleState4_callback(const little_ant_msgs::State4::ConstPtr& msg)
{
	current_roadwheelAngle_ = msg->roadwheelAngle;
	key_ = msg->key;
}

void PathTracking::avoiding_flag_callback(const std_msgs::Float32::ConstPtr& msg)
{
	//avoid to left(-) or right(+) the value presents the offset
	avoiding_offset_ = msg->data;
}

bool PathTracking::is_gps_data_valid(gpsMsg_t& point)
{
	if(point.x >1000 && point.y >1000)
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
	
	ROS_INFO("path tracking completed.");
	ros::shutdown();

	return 0;
}
