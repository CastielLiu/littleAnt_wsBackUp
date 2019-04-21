#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<gps_msgs/Inspvax.h>
#include<gps_msgs/Utm.h>
#include<ant_math/ant_math.h>


gps_msgs::Utm utm_gps_msg;

ros::Publisher pub_utm_gps;


uint8_t gps_status_ = 0;

void polar_gps_callback(const gps_msgs::Inspvax::ConstPtr &msg)
{
	gps_status_ |= 0x01;
	
	utm_gps_msg.yaw = deg2rad(msg->azimuth);
}

void cartesian_gps_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	gps_status_ |= 0x02;
	
	utm_gps_msg.x = msg->pose.pose.position.x;
	utm_gps_msg.y = msg->pose.pose.position.y;

}

void pub_utm_gps_callback(const ros::TimerEvent&)
{
	if(gps_status_ == 0x03)
	{
		gps_status_ = 0;
		pub_utm_gps.publish(utm_gps_msg);
	}

}



int main(int argc,char** argv)
{
	ros::init(argc,argv,"utm_gps");
	ros::NodeHandle nh;
	
	ros::Subscriber sub_polar_gps = nh.subscribe("/gps",1,&polar_gps_callback);
	ros::Subscriber sub_cartesian_gps = nh.subscribe("/gps_odom",1,&cartesian_gps_callback);
	ros::Timer timer = nh.createTimer(ros::Duration(0.05),&pub_utm_gps_callback);
	
	pub_utm_gps = nh.advertise<gps_msgs::Utm>("/gps_utm",1);
	
	ros::spin();
}
