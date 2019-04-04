#ifndef ACC_ESR_H_
#define ACC_ESR_H_
#include<ros/ros.h>
#include <little_ant_msgs/ControlCmd.h>
#include<little_ant_msgs/ControlCmd1.h>
#include<little_ant_msgs/ControlCmd2.h>

#include<little_ant_msgs/State1.h>
#include<little_ant_msgs/State2.h>
#include<esr_radar_msgs/Object.h>
#include<esr_radar_msgs/Objects.h>

class Acc_esr
{
public:
	Acc_esr(ros::NodeHandle nh,ros::NodeHandle nh_private);
	~Acc_esr();
	bool init()
	void run();
	
	
private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	
	ros::Subscriber sub_esrObjects_;
	ros::Subscriber sub_vehicleSpeed_;
	ros::Publisher pub_cmd_;
	
	little_ant_msgs::ControlCmd cmd_;
};



#endif
Acc_esr::Acc_esr(ros::NodeHandle nh,ros::NodeHandle nh_private)
	nh_(nh),
	nh_private_(nh_private)
{
	cmd.cmd2.set_gear =1;
	
	cmd.cmd1.set_driverlessMode =1;
	
}

bool Acc_esr::init()
{
	nh_private_.param<>();
	
	sub_esrObjects_ = nh_.subscribe("/esr_objects",2,&Acc_esr::object_callback,this);
	sub_vehicleSpeed_ = nh_.subscribe("/vehicleState2",2,&Acc_esr::vehicleSpeed_callback,this);
	pub_cmd_ = nh.advertise<little_ant_msgs::ControlCmd>("/sensor_decision",2);
}

void Acc_esr::run()
{


	ros::spin();
}

int main(int argc,char ** argv)
{
	ros::init(argc,argv,"acc_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	Acc_esr acc(nh,nh_private);
	if(acc.init())
		acc.run();
	
	return 0;
}

void Acc_esr::vehicleSpeed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	static int i=0;
	vehicleSpeed_ = (msg->wheel_speed_FL + msg->wheel_speed_RR)/2*5.0/18; //m/s

	i++;
	if(i%20==0)
		ROS_INFO("callback speed:%f ",vehicleSpeed_);
}

void Acc_esr::object_callback(const esr_radar_msgs::Objects::ConstPtr& msg)
{
	
}









